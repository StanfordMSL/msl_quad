#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped

import numpy as np

class Captian:

    def __init__(self):
        rospy.init_node('Captian', anonymous=True)

        #goal topic
        self.quadnName="postman/"
        self.goalPub = rospy.Publisher(self.quadnName+'command/pose', PoseStamped, queue_size=10)

        #agent (postman) parameters
        self.agentPose=Pose()


        #ros topics for postman
        self.agentPose_sub = rospy.Subscriber('mavros/local_position/pose', 
            PoseStamped, self.agentPoseCB)
        
        #test goal
        self.goal=[[7, 7, 7.],
                [0.0, 0.0, 0],
                [0.0, 0.0, 0.0]]
        rospy.sleep(2)


    def agentPoseCB(self, msg):
        #msg is PoseStamped
        self.agentPose=msg.pose

    def run(self):
        while True:
            rate = rospy.Rate(10) # 10hz
            goalMsg=PoseStamped()
            #lazy unpack
            goalMsg.pose.position.x=self.goal[0][0]
            goalMsg.pose.position.y=self.goal[0][1]
            goalMsg.pose.position.z=self.goal[0][2]

            goalMsg.pose.orientation.x = self.agentPose.orientation.x;
            goalMsg.pose.orientation.y = self.agentPose.orientation.y;
            goalMsg.pose.orientation.z = self.agentPose.orientation.z;
            goalMsg.pose.orientation.w = self.agentPose.orientation.w;  


            #fill header
            goalMsg.header.stamp = rospy.Time.now()
            goalMsg.header.frame_id = "1"
            #publish
            #print goalMsg
            self.goalPub.publish(goalMsg)
            #rospy.loginfo("published goal traj")
            rate.sleep()
            


if __name__ == '__main__':
    captian = Captian()
    captian.run()