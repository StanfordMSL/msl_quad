#!/usr/bin/env python

'''
The MIT License (MIT)
Copyright (c) 2019 Kunal Shah
                kshah.kunal@gmail.com
'''
# std lib imports
import sys
import time
import numpy as np
import numpy.linalg as la

# Standard ROS message
import rospy
from std_msgs.msg import Bool, Time
from geometry_msgs.msg import Pose, PoseStamped, Twist

from quadProxy import QuadProxy


class Tower(object):
    """docstring for Tower
    this class impliments a start and stop/land signals
    for general quad experiemnts"""

    def __init__(self):
        # self.manifest = ["rexquad3"]
        self.manifest = ["rexquad0", "rexquad3", "quad3"]
        print(self.manifest)

        # make proxy objects
        self.proxies = [QuadProxy(quad) for quad in self.manifest]

        # publish
        self.scramblePub = rospy.Publisher(
            'tower/scramble', Time, queue_size=10)

    def scramble(self):
        # BUT I LIKE TO SCRAMBLE THE FARIES
        self.scramblePub.publish(rospy.Time.now())
        rospy.loginfo("something clever when they all move")

    def land(self):
        rospy.logwarn("Landing")
        for proxy in self.proxies:
            proxy.emergencyLand()

    def loop(self):
        usrCmd = raw_input("Enter a commmand: ")
        if usrCmd == 'x':
            self.scramble()
        elif usrCmd == 'd':
            self.land()
        else:
            print('Not a valid commmand')
            print("x to start, d to land")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        print("x to start, d to land")
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    # make tower
    tower = Tower()
    tower.run()
