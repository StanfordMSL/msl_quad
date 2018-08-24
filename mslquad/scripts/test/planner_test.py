#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np

class Planner:

    def __init__(self):
        rospy.init_node('Planner', anonymous=True)

        self.plan_resolution = 0.25
        self.plan_horizon = 30
        self.trans_listener = tf.TransformListener()

        #path goal and position goal topics
        self.trajPub = rospy.Publisher('command/trajectory', JointTrajectory, queue_size=10)
        
        #test trajectory
        self.wpP=[[0.0, 0.0, 3.0],
                [0.2, 0.4, 3.5],
                [0.0, 0.4, 3.0],
                [0.0, 0.0, 2.5]]

        self.wpT=[0, .1, .2, .3]
        rospy.sleep(10)


    def run(self):
        #rate = rospy.Rate(10) # 10hz
        #while not rospy.is_shutdown():
        trajMsg=JointTrajectory()
        for i in range(4):
            trajPt=JointTrajectoryPoint()
            for pos in self.wpP[i]:
                trajPt.positions.append(pos)
            trajPt.time_from_start=rospy.Duration.from_sec(self.wpT[i])
            trajMsg.points.append(trajPt)
        #fill header
        trajMsg.header.stamp = rospy.Time.now()
        trajMsg.header.frame_id = "1"
        #publish
        self.trajPub.publish(trajMsg)
        rospy.loginfo("published traj")
        rospy.spin()
            #rate.sleep()
            


if __name__ == '__main__':
    planner = Planner()
    planner.run()