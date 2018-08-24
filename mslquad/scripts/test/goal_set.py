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
        self.goalPub = rospy.Publisher('command/goal', PoseStamped, queue_size=10)
        
        #test goal
        self.goal=[[-3.0, -13.0, 3.0],
                [0.0, 0.0, 0],
                [0.0, 0.0, 0.0]]
        rospy.sleep(2)


    def run(self):
    #rate = rospy.Rate(10) # 10hz
        goalMsg=PoseStamped()
        #lazy unpack
        goalMsg.pose.position.x=self.goal[0][0]
        goalMsg.pose.position.y=self.goal[0][1]
        goalMsg.pose.position.z=self.goal[0][2]

        #fill header
        goalMsg.header.stamp = rospy.Time.now()
        goalMsg.header.frame_id = "1"
        #publish
        print goalMsg
        self.goalPub.publish(goalMsg)
        rospy.loginfo("published goal traj")
            #rate.sleep()
            


if __name__ == '__main__':
    captian = Captian()
    captian.run()