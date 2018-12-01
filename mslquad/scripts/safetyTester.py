#!/usr/bin/env python

import numpy as np

#ros
import rospy
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from emergency.srv import Emergency


class Tester:
    def __init__(self):
        rospy.init_node('Tester', anonymous=True)

        self.poseSub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.poseCB)

        rospy.loginfo("Tester: Safety tester initalization complete")

    def poseCB(self, msg):
        #msg is PoseStamped
        self.pose=msg.pose


    def run(self):
        rospy.sleep(5)
        rospy.wait_for_service('emergency')

        try:
            eService = rospy.ServiceProxy('emergency', Emergency)
            ePose= self.pose
            ePose.position.z = 0 
            eResponse = eService(True, ePose, None)
            rospy.loginfo(eResponse)
        except rospy.ServiceException as e:
            rospy.loginfo("Tester: emergency call failed: %s" %e)
            rospy.sleep(.5)
        rospy.spin()
            


if __name__ == '__main__':
    tester = Tester()
    tester.run()