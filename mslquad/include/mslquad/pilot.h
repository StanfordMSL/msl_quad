/* copyright[2021] <msl>
**************************************************************************
  File Name    : pilot.h
  Author       : Kunal Shah, Jun En Low, Alexander Koufos
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Feb 9, 2021.
  Description  : px4 pilot class
**************************************************************************/

#ifndef __PILOT_H__
#define __PILOT_H__

// std
#include <string>
#include <Eigen/Dense>

// ros
#include "ros/ros.h"

// msg
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
// #include "geometry_msgs/TwistStamped.h"
// #include "geometry_msgs/Transform.h"
// #include "trajectory_msgs/MultiDOFJointTrajectory.h"
// #include "nav_msgs/Odometry.h"
// #include "mavros_msgs/ActuatorControl.h"
// service
#include "mslquad/Land.h"

// type aliases
using Pub = ros::Publisher;
using Sub = ros::Subscriber; 
using PoseSP = geometry_msgs::PoseStamped;
using Pose = geometry_msgs::Pose;


class Pilot {
 public:
  Pilot();
  virtual ~Pilot();
  std::string namespace = "/";



  // template<class msg_t>
  
 protected:
  // state machine
      enum class State {
        INIT,
        TAKEOFF,
        HOVER,
        FLIGHT,
        LAND,
    };
  // internals 
  std::string quadNS_ = "/";

  // poses
  PoseSP localPose;  // current flight pose
  PoseSP vrpnPose; // pose from opti-track
  Pose initPose;  // pose at initialization 

  Twist twist; // current flight twist 

  // void passback(const geometry_msgs::PoseStamped::ConstPtr& msg,
  //               geometry_msgs::PoseStamped& receipt);

  ros::NodeHandle nh_;
  geometry_msgs::PoseStamped local_pose;
  // void CB(const geometry_msgs::PoseStamped::ConstPtr& msg);
  // auto CB = boost::bind(
  //                   passback,
  //                   std::placeholders::_1,
  //                   local_pose);
  ros::Subscriber PoseSub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
                  quadNS_+"mavros/local_position/pose",
                  1,
                  std::bind(
                    Pilot::passback,
                    this,
                    std::placeholders::_1,
                    std::ref(local_pose))
                  );
 private:
    // publishers
    // px4 publishers
    Pub pub_px4SetPose;  // px4 setpoint_position command
    Pub pub_px4SetVel;  // px4 setpoint_velocity command
    // Pub odomPub_;  // publish pose of px4 agent to the planner
    // Pub actuatorPub_;  // px4 actuator_control topic

    // subscribers
    // flightroom
    Sub sub_vrpn;  // subscribe to vrpn pose

    // user commands
    Sub sub_cmdPose;
    Sub sub_cmdVel;
    Sub sub_cmdAccel;


    // callback methods

    // actions methods 
    void takeoff(void);
    // main controller loop (fast, up to >200 Hz)
    void controlLoop(void);

    void sub_vrpnCB(const poseSP::ConstPtr& msg);
    void sub_cmdPoseCB(const poseSP::ConstPtr& msg);
    // void velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void controlTimerCB(const ros::TimerEvent& event);


};  // Pilot



//   ros::Subscriber px4PoseSub_;  // px4 pose sub
//   ros::Subscriber px4VelSub_;  // px4 velocity sub
//   ros::Duration poseTimeDiff_;  // difference btwn current time and last time
//   ros::Subscriber vrpnSub_;  // subscribe to vrpn
//   geometry_msgs::PoseStamped curVrpnPose_;  // current mocap pose,
//   ros::ServiceServer emergencyLandSrv_;  // service for emergency landing
//   // FOR EMERGENCY ONLY SET BY THE EMERGENCY SERVICE HANDLER
//   geometry_msgs::Pose emergencyPose_;

//   // control loops
//   double controlLoopFreq_;  // frequency for fast control loop
//   ros::Timer controlTimer_;  // for fast loop
//   double slowLoopFreq_;  // frequency for slower loop
//   ros::Timer slowTimer_;  // for slow loop


//   void pathCB(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& traj);
//   // pose callback on PX4 local position
//   void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
//   void velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg);
//   void vrpnSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
//   void controlTimerCB(const ros::TimerEvent& event);
//   void slowTimerCB(const ros::TimerEvent& event);

//     // overrides the main control loop in the case of emergency
//     // timer callback
//     virtual void statusLoop(void);
//     // failsafe function called if state:: is changed to emergency land
//     void emergencyFailsafe(void);
//     bool emergencyLandHandle(
//         mslquad::EmergencyLand::Request &req,
//         mslquad::EmergencyLand::Response &res);


//     enum class State {
//         AUTO,
//         STANDBY,
//         POSE,
//         OVERRIDE,
//         LAND
//     };

//  protected:
//     static const std::string quadNS_;  // ROS name space
//     static const double takeoffHeight_;  // in 2D mode, this is the constant height
//     double maxVel_;
//     State state_;
//     bool flagOnly2D_;  // if enabled, quadrotor only operates at a fixed height

//     ros::NodeHandle nh_;
//     geometry_msgs::PoseStamped curPose_;  // current pose of quad from px4
//     geometry_msgs::TwistStamped curVel_;  // current velocity from px4
//     geometry_msgs::PoseStamped takeoffPose_;  // record the position at takeoff
//     geometry_msgs::PoseStamped hoverPose_;  // pose to hover
//     trajectory_msgs::MultiDOFJointTrajectory desTraj_;  // traj from planner

//     ros::Publisher px4SetVelPub_;  // px4 setpoint_velocity command
//     ros::Publisher px4SetPosPub_;  // px4 setpoint_position command
//     ros::Publisher odomPub_;  // publish pose of px4 agent to the planner
//     ros::Publisher actuatorPub_;  // px4 actuator_control topic
//     ros::Subscriber cmdTrajSub_;  // command traj sub

//     // calculate desired velocity vector given desired position
//     // vmax is the maximum velocity. return the norm of the position error
//     double calcVelCmd(Eigen::Vector3d& desVel,
//                       const Eigen::Vector3d& desPos,
//                       const double vmax,
//                       const double kp) const;

//     // in this version, vmax only constrains XY velocity. Z vel is independent
//     double calcVelCmd2D(Eigen::Vector3d& desVel,
//                         const Eigen::Vector3d& desPos,
//                         const double vmax,
//                         const double kp) const;

//     virtual void takeoff(void);
//     // main controller loop (fast, up to >200 Hz), overload in derived class
//     virtual void controlLoop(void);
//     // slow loop (<= 10Hz), good for publishing visualization data, etc
//     virtual void slowLoop(void);

//  private:
//     ros::Subscriber px4PoseSub_;  // px4 pose sub
//     ros::Subscriber px4VelSub_;  // px4 velocity sub
//     ros::Duration poseTimeDiff_;  // difference btwn current time and last time
//     ros::Subscriber vrpnSub_;  // subscribe to vrpn
//     geometry_msgs::PoseStamped curVrpnPose_;  // current mocap pose,
//     ros::ServiceServer emergencyLandSrv_;  // service for emergency landing
//     // FOR EMERGENCY ONLY SET BY THE EMERGENCY SERVICE HANDLER
//     geometry_msgs::Pose emergencyPose_;

//     // control loops
//     double controlLoopFreq_;  // frequency for fast control loop
//     ros::Timer controlTimer_;  // for fast loop
//     double slowLoopFreq_;  // frequency for slower loop
//     ros::Timer slowTimer_;  // for slow loop


//     void pathCB(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& traj);
//     // pose callback on PX4 local position
//     void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
//     void velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg);
//     void vrpnSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
//     void controlTimerCB(const ros::TimerEvent& event);
//     void slowTimerCB(const ros::TimerEvent& event);

//     // overrides the main control loop in the case of emergency
//     // timer callback
//     virtual void statusLoop(void);
//     // failsafe function called if state:: is changed to emergency land
//     void emergencyFailsafe(void);
//     bool emergencyLandHandle(
//         mslquad::EmergencyLand::Request &req,
//         mslquad::EmergencyLand::Response &res);
// };





#endif  // __PILOT_H__
