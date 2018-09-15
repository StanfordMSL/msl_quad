#!/usr/bin/env python

import rospy
import tf
import std_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped

#trajectory solve code
from path.objdyn import LoadSim
from path.trajectory import PolynomialTraj, PolynomialTrajOpt
from path.waypoint import Keyframes, KeyframesPool
from path.visual import TrajPlotter

import numpy as np

class Planner:
    def __init__(self):
        rospy.init_node('Planner', anonymous=True)

        self.timeResolution = .2
        self.trajBranchIdx = 3
        self.trans_listener = tf.TransformListener()
        self.speed=1.0 # straight line speed goal, used for calculating time for trajectory
        self.pose=Pose()

        #path goal and position goal topics
        self.trajPub = rospy.Publisher('command/trajectory', JointTrajectory, queue_size=10)
        self.goalSub = rospy.Subscriber('command/goal', PoseStamped, self.getGoalCB)
        self.poseSub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.updatePoseCB)

        rospy.loginfo("Navigation: Planner initalization complete")

    def posePosDist(self, pose):
        #calulates the l2 dist between self.pose and pose
        d = np.array(
            [pose.position.x-self.pose.position.x, 
            pose.position.y-self.pose.position.y,
            pose.position.z-self.pose.position.z])
        return np.linalg.norm(d)


    def updatePoseCB(self, msg):
        #msg is PoseStamped
        self.pose=msg.pose

    def getGoalCB(self, msg):
        #msg is Pose Stamped
        rospy.loginfo("got goal")
        goalPose=msg.pose
        currentPose=self.pose
        
        #calcuate trajectory time
        goalDist=self.posePosDist(goalPose)
        trajTime= goalDist/self.speed
        #build keyframes

        kf=Keyframes(currentPose.position.x, currentPose.position.y, currentPose.position.z,0)
        
        #preloaded paths
        # kfpool = KeyframesPool()  
        # kf = kfpool.get_keyframes(name = '003')

        #add points to make problem feasiable
        nPts=4
        for i in range(nPts-1):
            frac= (i+1.)/(nPts-1)
            kf.add_waypoint_pos_only(trajTime/(nPts-1), 
                    (1.0-frac)*currentPose.position.x + frac*goalPose.position.x,
                    (1.0-frac)*currentPose.position.y + frac*goalPose.position.y,
                    (1.0-frac)*currentPose.position.z + frac*goalPose.position.z)
        

        #calculate trajectory
        for wp in kf.wps:
            print wp
        load=LoadSim()

        trajectory = PolynomialTrajOpt(load, kf.wps, kf.ts) 

        
        # plot Traj
        # tp = TrajPlotter()
        # tp.plot_traj(trajectory, 'r')
        # tp.show()


        trajMsg=self.buildTrajectoryMsg(trajectory)
        self.trajPub.publish(trajMsg)
        rospy.loginfo("Navigation: Trajectory Sent")



    def buildTrajectoryMsg(self, trajectory):
        #build the trajectory_msgs/JointTrajectory message 
        trajMsg=JointTrajectory()
        print trajectory.ts
        print sum(trajectory.ts)
        for t in np.arange(0.0, sum(trajectory.ts), self.timeResolution):
            #get empty traj pt
            trajPt=JointTrajectoryPoint()
            #get output
            trajPtFlat=trajectory.get_flatout(t)
            print trajPtFlat[:,0]
            #fill point
            trajPt.positions=trajPtFlat[:,0]
            trajPt.velocities=trajPtFlat[:,1]
            trajPt.accelerations=trajPtFlat[:,2]
            trajPt.time_from_start=rospy.Duration.from_sec(t)
            #append to trajectory message
            trajMsg.points.append(trajPt)
        #fill header
        trajMsg.header.stamp = rospy.Time.now()
        trajMsg.header.frame_id = str(1)


        return trajMsg

    def run(self):
        rospy.spin()
            


if __name__ == '__main__':
    planner = Planner()
    planner.run()