#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

#trajectory solve code
from path.objdyn import LoadSim
from path.visual import VizTool, TrajPlotter
from path.robot import Thruster
from path.trajectory import PolynomialTraj, PolynomialTrajOpt

import numpy as np

class Planner:

    def __init__(self):
        rospy.init_node('Planner', anonymous=True)

        self.trajectory
        self.timeResolution = 0.25
        self.trajBranchIdx = 3
        self.trans_listener = tf.TransformListener()

        #path goal and position goal topics
        self.trajPub = rospy.Publisher('command/trajectory', JointTrajectory, queue_size=10)
        self.goalSub = rospy.Subscriber('command/goal', JointTrajectoryPoint, self.getGoalCB)


    def getGoalCB(self, msg):




    def buildTrajectoryMsg(self, trajectory, timeResolution=self.timeResolution):
        #build the trajectory_msgs/JointTrajectory message 
        trajMsg=JointTrajectory()
        for t in np.arange(0.0, trajectory,ts, timeResolution):
            #get empty traj pt
            trajPt=JointTrajectoryPoint()
            #get output
            trajPtFlat=traj_opt_time.get_flatout(t)
            #fill point
            trajPt.positions.append(trajPtFlat[:,0])
            trajPt.velocties.append(trajPtFlat[:,1])
            trajPt.accelerations.append(trajPtFlat[:,2])
            trajPt.time_from_start=rospy.Duration.from_sec(t)
            #append to trajectory message
            trajMsg.points.append(trajPt)
        #fill header
        trajMsg.header.stamp = rospy.Time.now()
        trajMsg.header.frame_id = "1"


        return trajMsg

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