#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped

import numpy as np

class Captian:

    def __init__(self):
        rospy.init_node('poseCaptain', anonymous=True)

        #goal topic
        self.goalPub = rospy.Publisher('command/pose', Pose, queue_size=10)
        #wait for connect
        rospy.sleep(2)


    def run(self):
    
        while not rospy.is_shutdown():

            x, y, z = [float(val) for val in raw_input("Enter goal position: ").split()]

            goalMsg=Pose()
            #lazy unpack
            goalMsg.position.x=x
            goalMsg.position.y=y
            goalMsg.position.z=z
            #publish
            self.goalPub.publish(goalMsg)
            rospy.sleep(.5)
                


if __name__ == '__main__':
    captian = Captian()
    captian.run()