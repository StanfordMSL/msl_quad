#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

class Pilot:
    def __init__(self):
        #Tell ROS this is a node
        rospy.init_node('posePilot', anonymous=True)
        #Sub to '/quad0/mavros/local_position/pose' and Pub to '/quad0/command/pose' 
        self.pathSub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.updatePoseCB) 
        self.pathPub = rospy.Publisher('command/pose', Pose, queue_size = 10)
        #Static Variables
        self.l, self.w, self.h = 7.0, 10.0, 2.0
        self.distTol = 0.15
        self.waypoints =   [(0, 0, self.h, 0),
                            (self.l/2, 0, self.h, np.pi/2),
                            (self.l/2, self.w/2, self.h, np.pi),
                            (0, self.w/2, self.h, 3*np.pi/2),
                            (-self.l/2, self.w/2, self.h, 0),
                            (-self.l/2, 0, self.h, np.pi/2),
                            (-self.l/2, -self.w/2, self.h, np.pi),
                            (0, -self.w/2, self.h, 3*np.pi/2),
                            (self.l/2, -self.w/2, self.h, 0),
                            (self.l/2, 0, self.h, np.pi/2),
                            (0, 0, self.h, np.pi),
                            (0, 0, self.h, 0)]
        rospy.sleep(2)

    #Update current pose and orientation
    def updatePoseCB(self, msg):
        self.pose = msg.pose
        quat = [msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,]
        self.euler = euler_from_quaternion(quat)

    #Update the distance from next point to current pose
    def poseDistance(self, pose):
        currentPt = np.array((self.pose.position.x, 
                              self.pose.position.y, 
                              self.pose.position.z))
        pt = np.array((pose.position.x, 
                       pose.position.y,
                       pose.position.z))
        return np.linalg.norm(currentPt - pt)

    #Main function that runs the path test
    def run_path(self):
    	while not rospy.is_shutdown():
            for chpt, point in enumerate(self.waypoints):           
                self.checkpoint = chpt
                targetPose = Pose()          
                # set position
                targetPose.position.x = point[0]
                targetPose.position.y = point[1]
                targetPose.position.z = point[2]
                # set orientation
                self.yaw = point[3]
                quat_target = quaternion_from_euler(0,0,self.yaw)
                targetPose.orientation.x = quat_target[0]
                targetPose.orientation.y = quat_target[1]
                targetPose.orientation.z = quat_target[2]
                targetPose.orientation.w = quat_target[3]           
                self.setPoseTarget(targetPose)
        return 0

    #Wait for UAV to get to destination 
    def setPoseTarget(self, pose):
        #publish target pose
        self.pathPub.publish(pose)
        #wait for pose and orientation
        dist = self.poseDistance(pose)
        while (dist > self.distTol):
            dist = self.poseDistance(pose)
            print("Goal: %d | Pose: (%.1f,%.1f,%.1f) (%.3f) | Yaw: (%.3f) (%.3f)" % (self.checkpoint,pose.position.x,pose.position.y,pose.position.z,dist,np.degrees(self.yaw),np.degrees(self.euler[2])))
            rospy.sleep(0.1)
        print("\n\t\t***** Here! *****\n")
        rospy.sleep(2)


if __name__ == '__main__':
    #Enter 'x' to begin
    userEntry = " "
    while userEntry != "x":
        print
        userEntry = raw_input("Enter 'x' to begin path: ")
    #Run path test
    pilot = Pilot()
    pilot.run_path()