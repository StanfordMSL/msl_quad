#!/usr/bin/env python

'''
The MIT License (MIT)
Copyright (c) 2019 Kunal Shah
                kshah.kunal@gmail.com
'''
#std lib imports 

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
    """This class implements a proxy for the comunication beteween the user and the quad 
        purposes/usage: 
            read position/location information from motion capture
            envoke go and land commands"""
    def __init__(self, namespace):
        # Tell ROS this is a node
        rospy.init_node('quadProxy', anonymous=True)
        self.pose = None
        self.poseSub = rospy.Subscriber(namespace+'/mavros/local_position/pose', PoseStamped, self.PoseCB) 

        # land service
        self.eLandService = rospy.ServiceProxy(namespace+'/emergency_land', EmergencyLand)

        #wait for pose
        while self.pose is None:
            print("waiting for pose")
            rospy.sleep(.5)
        rospy.loginfo("Monitor active on: {:s}".format(namespace))

    def PoseCB(self, msg):
        self.pose = msg.pose

    def emergencyLand(self):
        eResponse = False
        while not eResponse:
            try:
                ePose = Pose()
                # land at current location
                ePose.position.x = self.pose.position.x
                ePose.position.y = self.pose.position.y
                ePose.position.z = 0 
                eResponse = self.eLandService(ePose)
            except rospy.ServiceException as e:
                rospy.loginfo("Emergency call failed: %s" %e)
        return True

if __name__ == '__main__':
    name = raw_input("Enter quad name: ")

    quadProxy = QuadProxy(name)
    quadProxy.emergencyLand()