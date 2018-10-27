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

protected:
    std::string quadNS_; // ROS name space
    float fixedHeight_; // the fixed height that the quad will be flying at
    float maxVel_;

    geometry_msgs::PoseStamped curPose_; // current pose of quad from px4
    trajectory_msgs::MultiDOFJointTrajectory desTraj_; // desired trajectory from the planner

    // calculate desired velocity vector given desired position
    // vmax is the maximum velocity. return the norm of the position error
    float calcVelCmd(Eigen::Vector3d& desVel, const Eigen::Vector3d& desPos, 
        const float vmax, const float kp);

    // in this version, vmax only constrains XY velocity. Z vel is independent
    float calcVelCmd2D(Eigen::Vector3d& desVel, const Eigen::Vector3d& desPos, 
            const float vmax, const float kp);

    virtual void takeoff(const float desx, const float desy, const float desz);
    virtual void controlTimerCB(const ros::TimerEvent& event); // main controller loop

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmdTrajSub_; // game planner command traj sub
    ros::Subscriber px4PoseSub_; // px4 pose sub
    ros::Publisher px4SetVelPub_; // px4 setpoint_velocity command
    ros::Publisher odomPub_; // publish pose of px4 agent to the planner

    ros::Timer controlTimer_;

    void pathCB(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& traj);
    void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg); // pose callback on PX4 local position    
};

#endif