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
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "mavros_msgs/ActuatorControl.h"
#include <Eigen/Dense>
#include <string>
#include "mslquad/EmergencyLand.h"

class PX4BaseController {
public:
    PX4BaseController();
    virtual ~PX4BaseController();

    enum class State {
        AUTO,
        EMERGENCY_LAND
    };
    
    double getYawRad(void) const; // return yaw angle in radius, [-pi, pi]
    inline Eigen::Vector3d getLinVel(void) const {
        return Eigen::Vector3d(curVel_.twist.linear.x,
            curVel_.twist.linear.y, curVel_.twist.linear.z);
    };
    inline Eigen::Vector3d getAngVel(void) const {
        return Eigen::Vector3d(curVel_.twist.angular.x,
            curVel_.twist.angular.y, curVel_.twist.angular.z);
    }
    inline Eigen::Vector3d getPosition(void) const {
        return Eigen::Vector3d(curPose_.pose.position.x,
            curPose_.pose.position.y, curPose_.pose.position.z);
    };
    Eigen::Matrix3d getRotMat(void) const; // get rotation matrix
    inline double getControlLoopFreq(void) const {return controlLoopFreq_;};

    static double getDist(const geometry_msgs::PoseStamped &ps1,
                        const geometry_msgs::PoseStamped &ps2);

protected:
    std::string quadNS_; // ROS name space
    double takeoffHeight_; // in 2D mode, this is the constant height the quad will be flying at
    double maxVel_;
    State state_;
    bool flagOnly2D_; // if enabled, quadrotor only operates at a fixed height (takeoffHeight_)

    ros::NodeHandle nh_;
    geometry_msgs::PoseStamped curPose_; // current pose of quad from px4
    geometry_msgs::TwistStamped curVel_; // current vellocity from px4
    geometry_msgs::PoseStamped curVisionPose_; // current mocap pose, used for sanity check
    geometry_msgs::PoseStamped takeoffPose_; // record the position before takeoff
    geometry_msgs::Pose emergencyLandPose_;
    trajectory_msgs::MultiDOFJointTrajectory desTraj_; // desired trajectory from the planner

    ros::Publisher px4SetVelPub_; // px4 setpoint_velocity command
    ros::Publisher px4SetPosPub_; // 
    ros::Publisher odomPub_; // publish pose of px4 agent to the planner
    ros::Publisher actuatorPub_; // px4 actuator_control topic

    // calculate desired velocity vector given desired position
    // vmax is the maximum velocity. return the norm of the position error
    double calcVelCmd(Eigen::Vector3d& desVel, const Eigen::Vector3d& desPos, 
        const double vmax, const double kp) const;

    // in this version, vmax only constrains XY velocity. Z vel is independent
    double calcVelCmd2D(Eigen::Vector3d& desVel, const Eigen::Vector3d& desPos, 
            const double vmax, const double kp) const;

    virtual void takeoff(const double desx, const double desy, const double desz);
    virtual void controlLoop(void); // main controller loop (fast, up to >200 Hz), overload in derived class
    virtual void slowLoop(void); // slow loop (<= 10Hz), good for publishing visualization data, etc 

private:
    ros::Subscriber cmdTrajSub_; // game planner command traj sub
    ros::Subscriber px4PoseSub_; // px4 pose sub
    ros::Subscriber px4VelSub_; // px4 velocity sub
    ros::Subscriber visionPoseSub_; // subscribe to mocap
    ros::ServiceServer emergencyLandSrv_; // service for emergency landing 

    ros::Timer controlTimer_; // for fast loop
    ros::Timer slowTimer_;  // for slow loop

    double controlLoopFreq_; // frequency for fast control loop
    double slowLoopFreq_; // frequency for slower loop

    void pathCB(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& traj);
    void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg); // pose callback on PX4 local position
    void velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void visionPoseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void controlTimerCB(const ros::TimerEvent& event);
    void slowTimerCB(const ros::TimerEvent& event);
    bool emergencyLandHandle(
        mslquad::EmergencyLand::Request &req,
        mslquad::EmergencyLand::Response &res);
};

#endif