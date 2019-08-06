#!/usr/bin/env python

import rospy
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np

class Conductor:

	def __init__(self):
		rospy.init_node('poseConductor', anonymous=True)
		self.sub = rospy.Subscriber('mavros/local_position/pose',PoseStamped,self.poseCallBack)
		self.pub = rospy.Publisher('command/pose', Pose, queue_size = 10)

		self.file_path = rospy.get_param('~file_path')
		self.distTol = rospy.get_param('~tolerance')
		self.waypoints = np.loadtxt(open(self.file_path),delimiter=',',skiprows=1)
		
		rospy.sleep(2)

	def poseCallBack(self,message):
		self.pose = message.pose

	def get_distance(self):
		current = np.array((self.pose.position.x,
				   		    self.pose.position.y,
				            self.pose.position.z))
		destination = np.array((self.goalPose.position.x,
					            self.goalPose.position.y,
					            self.goalPose.position.z))
		return np.linalg.norm(destination - current)

	def runPath(self):
		while not rospy.is_shutdown():
			for point in self.waypoints:
				self.goalPose = Pose()
				self.goalPose.position.x = point[0]
				self.goalPose.position.y = point[1]
				self.goalPose.position.z = point[2]
				self.next_target(self.goalPose)

	def next_target(self,pose):
		self.pub.publish(pose)

		distance  = self.get_distance()
		while distance > self.distTol:
			distance = self.get_distance()
			print("Pose: %.3f, %.3f, %.3f | Distance: %.3f" % (self.pose.position.x,self.pose.position.y,self.pose.position.z,distance))
			rospy.sleep(0.1)

		print("\t\n*** Here at %.3f, %.3f, %.3f ***\n" % (pose.position.x,pose.position.y,pose.position.z))
		rospy.sleep(1)


if __name__ == '__main__':

	user = " "
	while user != "x":
		user = raw_input("Enter 'x' to begin: ")

	conductor = Conductor()
	conductor.runPath()





