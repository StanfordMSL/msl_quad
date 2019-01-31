#!/usr/bin/env python

#ros node that simulates a external camera system in order to align all assest to the same frame

import rospy
import tf
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from gazebo_msgs.msg import ModelStates
from mavros_msgs.msg import HomePosition

import numpy as np

class VRNP_sim:

    def __init__(self):
        rospy.init_node('vrpn_sim', anonymous=True)
        
        #get model name
        self.modelName = rospy.get_param('~modelname')
        self.visionPose = PoseStamped()
        self.visionTwist = TwistStamped()
        
        rospy.loginfo("Vision: Publishing pose and odom of gazebo model"+self.modelName)

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.modeStates_CB)

        while self.visionPose==None:
            rospy.sleep(.5)
            #sleep till pose is found

        self.visionPose_pub=rospy.Publisher('esimate/pose',
            PoseStamped, queue_size=10)

        self.visionTwist_pub=rospy.Publisher('esimate/veloctiy',
            TwistStamped, queue_size=10)

    def run(self):
        r=rospy.Rate(100)
        while not rospy.is_shutdown():
            #publish pose
            self.visionPose.header.stamp=rospy.Time.now()
            self.visionPose_pub.publish(self.visionPose)
            #publish twist
            self.visionTwist.header.stamp=rospy.Time.now()
            self.visionTwist_pub.publish(self.visionTwist)
            r.sleep()
       
    def modeStates_CB(self, msg):
        modelIdx=msg.name.index(self.modelName)
        self.visionPose.pose=msg.pose[modelIdx]
        self.visionTwist.twist=msg.twist[modelIdx]






if __name__ == '__main__':
    vrpn_sim = VRNP_sim()
    vrpn_sim.run()