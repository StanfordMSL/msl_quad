#!/usr/bin/env python

#ros node that clears quads for flight in simulation, mimics arming and offboard mode
# also sets the home position with launch arg

import rospy
import tf
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import HomePosition, State

import numpy as np

class Clearance:

    def __init__(self):
        rospy.init_node('clearance', anonymous=True)
        #set home position 
        try:
            delay=rospy.get_param('~delay')
        except:
            delay = 1
        self.state=None

        state_sub=rospy.Subscriber('mavros/state',State ,self.stateCB)

        while self.state==None:
            #wait for state message
            rospy.sleep(.5)

        #wait for mavros to boot and not be grumpy
        rospy.sleep(delay)

    def stateCB(self, msg):
        self.state=msg

    def run(self):
        # Arm
        rospy.wait_for_service('mavros/cmd/arming')
        while not self.state.armed:
            try:
                armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
                armResponse = armService(True)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                rospy.loginfo("Clearance: Arming failed: %s" %e)
            rospy.sleep(.5)

        # Set Mode
        rospy.wait_for_service('mavros/set_mode')
        while not self.state.mode=="OFFBOARD":
            try:
                modeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
                modeResponse = modeService(0, 'OFFBOARD')
                rospy.loginfo(modeResponse)
            except rospy.ServiceException as e:
                rospy.loginfo("Clearance: mode switch failed: %s" %e)
            rospy.sleep(.5)

        rospy.loginfo("Clearance: Vechicle cleared for takeoff")

if __name__ == '__main__':
    clear = Clearance()
    clear.run()