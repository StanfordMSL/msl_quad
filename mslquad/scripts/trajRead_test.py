#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory

import numpy as np

class Reader:

    def __init__(self):
        rospy.init_node('trajReader', anonymous=True)
        #path goal and position goal topics
        self.trajPub = rospy.Subscriber('command/trajectory', JointTrajectory, self.readTrajCB)

    def readTrajCB(self, msg):
        rospy.loginfo("Got trajectory")
        for i, pt in enumerate(msg.points):
            rospy.loginfo("trajectory point %d" %i)
            rospy.loginfo(pt.time_from_start)
            rospy.loginfo(pt.positions)
            rospy.loginfo(pt.velocities)
            rospy.loginfo(pt.accelerations)

    def run(self):
        rospy.spin()
            


if __name__ == '__main__':
    reader = Reader()
    reader.run()