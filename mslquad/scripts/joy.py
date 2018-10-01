#!/usr/bin/env python

import sys
import argparse
import rospy
import tf 
import numpy as np
from tf.transformations import quaternion_from_euler

#messages and services
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, Float64

from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3, Quaternion, Point

from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import HomePosition, State, AttitudeTarget

class Pad(object):
    def __init__(self):
        rospy.init_node('pad', anonymous=True)
        self.threshold= .1 #because the controler is a little sticky 

        #self.offset=rospy.get_param('~offset')
         
        self.axes_map = {
            'roll': 3,
            'pitch': 4,
            'yaw': 0,
            'throttle': 1,
            'mode_set1': 6,
            'mode_set2': 7
        }

        self.velocity_scale= {
          'angular': 1.5, 
          'linear' : 2.0
        }

        self.button_map = {
            'arm' : 0, #a
            'disarm' : 1,#b
            'takeoff': 2, #x
            'land': 3, #y
            'enable': 4 #rb
        }

        
        self.mode="POS" # three modes: POS, VEL, ATD
        self.joy_sub=rospy.Subscriber("joy", Joy, self.joy_cb)


        #state info
        self.pose=Pose()
        self.agentPose_sub = rospy.Subscriber('mavros/local_position/pose', 
            PoseStamped, self.poseCB)

        #pose control
        self.cmdPose=Pose()
        self.cmdPose.orientation.w=-1
        self.cmdPose_pub = rospy.Publisher('mavros/setpoint_position/local',
            PoseStamped,  queue_size=10)

        #atd control
        self.cmdAtd=AttitudeTarget()
        self.cmdAtd_pub = rospy.Publisher('mavros/setpoint_raw/attitude',
            AttitudeTarget,  queue_size=10)


        #velocity control
        self.cmdVel=Twist()
        self.cmdVel_pub = self.agentVel_cmd = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped',
            Twist, queue_size=10)


    def arm(self, mode):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armResponse = armService(mode)
            rospy.loginfo(armResponse)
        except rospy.ServiceException as e:
            rospy.loginfo("Clearance: Arming failed: %s" %e)
        rospy.sleep(.1) #debounce 

    def enable(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            modeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            modeResponse = modeService(0, 'OFFBOARD')
            rospy.loginfo(modeResponse)
        except rospy.ServiceException as e:
            rospy.loginfo("Clearance: mode switch failed: %s" %e)
        rospy.sleep(.1) #debounce


    def modeSet(self, joy):
        #Logic for mode switch using D pad
        modeVal= [self.axisState(joy, 'mode_set1'), self.axisState(joy, 'mode_set2')]
        if modeVal== [0, 1]:
            self.mode='POS'
            rospy.loginfo("Pad: position mode")
            self.cmdPose=self.pose #reset position controller 
        elif modeVal== [1, 0]:
            rospy.loginfo("Pad: attitude mode")
            self.mode='ATD'
        elif modeVal== [-1, 0]:
            rospy.loginfo("Pad: velocity mode")
            self.mode='VEL'



    def axisState(self, joy, name):
        #returns value of stick parsed from joy message
        axisValue= joy.axes[self.axes_map[name]] 
        if abs(axisValue) > self.threshold:
            return axisValue 
        else:
            return 0

    def buttonState(self, joy, name):
        #returns value of button parsed from joy message
        return joy.buttons[self.button_map[name]]


    def poseCB(self, msg):
        self.pose=msg.pose

    def frameTransform(self, vector, pose, flip=False):
        #transforms vector into the world frame 
        #pose is relative to world, vector is relative to pose
        v_ = list(vector)
        v_.append(0.0) # "real" component of quaternions in ros is w, which is the 4th element. turn vector into a pure (no real part) quaternion

        q=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        if flip:
            q=tf.transformations.quaternion_inverse(q)

        # "conjugate" vector v_ by quaternion q (i.e. apply the rotation to v_) - https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
        v_ = tf.transformations.quaternion_multiply(
                     tf.transformations.quaternion_multiply(q, v_),
                     tf.transformations.quaternion_inverse(q)) # should be same as tf.transformations.quaternion_conjugate(q1), assuming q is a unit quaternion
        return v_[:3]

    def joy_cb(self, joy):
        print(joy)
        self.modeSet(joy)

        if self.buttonState(joy,'arm') == 1:
            self.arm(True)
        if self.buttonState(joy, 'enable')==1:
            self.enable()

        if self.mode=="POS":
            #get deltas 
            d_y = self.axisState(joy, 'roll')
            d_x = self.axisState(joy, 'pitch')
            d_p = self.axisState(joy, 'yaw')
            d_z = self.axisState(joy, 'throttle')

            #apply changes 
            #postion
            self.cmdPose.position.x += d_x
            self.cmdPose.position.y += d_y
            self.cmdPose.position.z += d_z

            #orientation
            q= [self.cmdPose.orientation.x, self.cmdPose.orientation.y, self.cmdPose.orientation.z, self.cmdPose.orientation.w]
            d_q = quaternion_from_euler(0, 0, d_p)

            self.cmdPose.orientation=Quaternion(*tf.transformations.quaternion_multiply(q, d_q))
            print(self.cmdPose)
            #PUBLISH
            outPose = PoseStamped(header=Header(stamp=rospy.get_rostime()), pose=self.cmdPose)
            self.cmdPose_pub.publish(outPose)

        if self.mode=="VEL":
            d_y = self.axisState(joy, 'roll')
            d_x = self.axisState(joy, 'pitch')
            d_p = self.axisState(joy, 'yaw')
            d_z = self.axisState(joy, 'throttle')

            #transform velocity
            localVel= np.array([d_x, d_y, d_z]) 
            globalVel= self.frameTransform(localVel, self.pose, flip=False)


            #fill message
            self.cmdVel.linear.x= globalVel[0] * self.velocity_scale['linear'] 
            self.cmdVel.linear.y= globalVel[1] * self.velocity_scale['linear']
            self.cmdVel.linear.z= globalVel[2] * self.velocity_scale['linear']

            self.cmdVel.angular.z=d_p * self.velocity_scale['angular']

            #publish
            print(self.cmdVel)
            self.cmdVel_pub.publish(self.cmdVel)

        if self.mode=='ATD':
            
            #not working right now?
            #get deltas 
            roll = self.axisState(joy, 'roll')
            pitch = self.axisState(joy, 'pitch')
            yaw = self.axisState(joy, 'yaw')
            thrust = self.axisState(joy, 'throttle')

            #get attitude
            Qrpy = quaternion_from_euler(roll, pitch, yaw)

            #
            self.cmdAtd=AttitudeTarget(header=Header(stamp=rospy.get_rostime()))
            self.cmdAtd.orientation=Quaternion(*Qrpy)
            self.cmdAtd.thrust=thrust*20
            print(self.cmdAtd)

            #PUBLISH
            print(self.cmdAtd)
            self.cmdAtd_pub.publish(self.cmdAtd)

if __name__ == '__main__':
    pad=Pad()
    rospy.spin()

