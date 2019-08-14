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
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from mslquad.srv import EmergencyLand
import tf.transformations as transf


class QuadProxy(object):
    """This class implements a proxy for the comunication beteween
        the user and the quad
        purposes/usage:
            read position/location information from motion capture
            envoke go and land commands"""

    def __init__(self, namespace):
        # Tell ROS this is a node
        rospy.init_node('quadProxy', anonymous=True)
        self.pose = None
        self.lastPose = None
        self.namespace = namespace
        self.poseSub = rospy.Subscriber(
            namespace+'/mavros/local_position/pose', PoseStamped, self.PoseCB)

        # land service
        self.eLandService = rospy.ServiceProxy(
            namespace+'/emergency_land', EmergencyLand)

        # wait for pose
        while self.lastPose is None:
            print("waiting for pose")
            rospy.sleep(.5)
        rospy.loginfo("Monitor active on: {:s}".format(namespace))

        # timers
        self.bounds = [[-.6, 11.], [-1., 3.5], [0., 2.2]]
        self.statusCB = rospy.Timer(rospy.Duration(.5), self.statusCB)

    def inRange(self, x, lower, upper):
        return x < lower or x > upper

    def poseDist(self, p1, p2):
        v1 = np.array([p1.position.x, p1.position.y, p1.position.z])
        v2 = np.array([p2.position.x, p2.position.y, p2.position.z])
        # check dist
        return la.norm(v1-v2) > .2

    def statusCB(self, timer):
        land = False

        # check pose diff
        if self.poseDist(self.pose, self.lastPose):
            rospy.logwarn(self.namespace + " massive pose drift. Landing")
            land = True
        # check bounds
        elif self.inRange(self.pose.position.x,
                          self.bounds[0][0], self.bounds[0][1]):
            rospy.logwarn(self.namespace + " out of x range. Landing")
            land = True
        elif self.inRange(self.pose.position.y,
                          self.bounds[1][0], self.bounds[1][1]):
            rospy.logwarn(self.namespace + " out of y range. Landing")
            land = True
        elif self.inRange(self.pose.position.z,
                          self.bounds[2][0], self.bounds[2][1]):
            rospy.logwarn(self.namespace + " out of z range. Landing")
            land = True

        if land:
            print(self.pose)
            self.emergencyLand()

    def PoseCB(self, msg):
        self.lastPose = self.pose
        self.pose = msg.pose

    def emergencyLand(self):
        eResponse = False
        while not eResponse:
            try:
                ePose = Pose()
                # land at current location
                ePose.position.x = self.pose.position.x
                ePose.position.y = self.pose.position.y
                ePose.position.z = -.1
                eResponse = self.eLandService(ePose)
            except rospy.ServiceException as e:
                rospy.loginfo("Emergency call failed: %s" % e)
                rospy.sleep(.2)
        return True


if __name__ == '__main__':
    name = raw_input("Enter quad name: ")

    quadProxy = QuadProxy(name)
    quadProxy.emergencyLand()
