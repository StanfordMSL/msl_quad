/**************************************************************************
  File Name    : px4_base_controller.cpp
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Oct 27, 2018.
  Description  : Base class for interfacing with px4
**************************************************************************/

#ifndef __PX4_BASE_CONTROLLER_H__
#define __PX4_BASE_CONTROLLER_H__

#include "ros/ros.h"
#include <string>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>

class PX4BaseController {
public:
    PX4BaseController();
    virtual ~PX4BaseController();

    // return yaw angle in radius, [-pi, pi]
    double getYawRad(void) const;

protected:
    std::string quadNS_; // ROS name space
    double fixedHeight_; // the fixed height that the quad will be flying at
    double maxVel_;

    ros::NodeHandle nh_;
    geometry_msgs::PoseStamped curPose_; // current pose of quad from px4
    trajectory_msgs::MultiDOFJointTrajectory desTraj_; // desired trajectory from the planner

    ros::Publisher px4SetVelPub_; // px4 setpoint_velocity command
    ros::Publisher odomPub_; // publish pose of px4 agent to the planner

    // calculate desired velocity vector given desired position
    // vmax is the maximum velocity. return the norm of the position error
    double calcVelCmd(Eigen::Vector3d& desVel, const Eigen::Vector3d& desPos, 
        const double vmax, const double kp);

    // in this version, vmax only constrains XY velocity. Z vel is independent
    double calcVelCmd2D(Eigen::Vector3d& desVel, const Eigen::Vector3d& desPos, 
            const double vmax, const double kp);

    virtual void takeoff(const double desx, const double desy, const double desz);
    virtual void controlLoop(void); // main controller loop, overload in derived class

private:
    ros::Subscriber cmdTrajSub_; // game planner command traj sub
    ros::Subscriber px4PoseSub_; // px4 pose sub

    ros::Timer controlTimer_;

    void pathCB(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& traj);
    void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg); // pose callback on PX4 local position
    void controlTimerCB(const ros::TimerEvent& event);
};

#endif