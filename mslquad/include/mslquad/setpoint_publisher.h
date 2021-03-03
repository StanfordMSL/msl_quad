/* copyright[2021] <msl>
**************************************************************************
  File Name    : setpoint_publisher.h
  Author       : Kunal Shah, Jun En Low, Alexander Koufos
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Feb 9, 2021.
  Description  : px4 setpoint publisher class. Isolates and ensures mavros
                 requirements are always satisfied.
**************************************************************************/

#ifndef __SETPOINT_PUBLISHER_H__
#define __SETPOINT_PUBLISHER_H__

// std
#include <string>
// #include <Eigen/Dense>

// ros
#include "ros/ros.h"

// msg
#include "mavros_msgs/State.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
// #include "geometry_msgs/Transform.h"
// #include "trajectory_msgs/MultiDOFJointTrajectory.h"
// #include "nav_msgs/Odometry.h"

// services
#include <mavros_msgs/CommandTOL.h>

class SetpointPublisher {
 public:
  SetpointPublisher();
  virtual ~SetpointPublisher();
  
  std::string m_namespace = "/";
  enum class sp_states {
    INIT,
    TAKEOFF,
    STANDBY,
    HOVER,
    FLIGHT,
    LAND,
    FAILSAFE,
  } sp_state;
  
 protected:
  // ros internals 
  ros::NodeHandle m_nh;
  mavros_msgs::State m_mavrosState;
  
  // state machine
  sp_states m_state;
  
  // parameters
  float m_takeoffAlt;   // takeoff altitude
  float m_controlHRate; // high rate
  float m_controlLRate; // low rate

  // Current States
  geometry_msgs::PoseStamped m_localPose;  // EKF fused flight pose
  geometry_msgs::PoseStamped m_visionPose; // raw 

  // Target States
  geometry_msgs::PoseStamped m_initPose;   // for setpoints at initialization (position control)
  geometry_msgs::PoseStamped m_landPose;   // for setpoints at landing (position control)

  geometry_msgs::Pose  m_targetPose;       // target pose (position control)
  geometry_msgs::Twist m_targetVel;        // target velocity (velocity control)
  geometry_msgs::Twist m_targetVel;  // target velocity


  // add initTime

  geometry_msgs::Twist m_localVel; // current flight velocity
  geometry_msgs::Twist m_localAccel; //current flight acceleration 
  
 private:
  // PUBLISHERS
  // px4 publishers
  ros::Publisher pub_px4SetPose;  // px4 setpoint_position command
  ros::Publisher pub_px4SetVel;  // px4 setpoint_velocity command
  // Pub odomPub_;  // publish pose of px4 agent to the planner
  // Pub actuatorPub_;  // px4 actuator_control topic

  // SUBSCRIBERS
  // px4 subscribers
  ros::Subscriber sub_px4GetPose;  // px4 get local position
  ros::Subscriber sub_px4GetVel;  // px4 get local velocity

  // flight room
  ros::Subscriber sub_vrpn;  // subscribe to vrpn pose

  // user commands
  ros::Subscriber sub_cmdPose;
  ros::Subscriber sub_cmdVel;
  ros::Subscriber sub_cmdAccel;

  // control timers
  ros::Timer m_fastLoop;  // for fast loop
  ros::Timer m_slowLoop;  // for slow loop
  // add one more for high rate
  ros::Timer m_statusLoop;  // for status loop

  // callback methods
  // loops
  void controlLoopCB(const ros::TimerEvent& event);
  void statusLoopCB(const ros::TimerEvent& event);
  // px4 getters
  void sub_px4GetPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void sub_px4GetVelCB(const geometry_msgs::TwistStamped;::ConstPtr& msg);
  // vrpn
  void sub_vrpnCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
  // user commands
  void sub_cmdPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
  // void velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg);

  // status methods
  void preFlight(void);
  void poseDelay(void);
  void poseDrift(void);

  // main controller loop (fast, up to >200 Hz)
  // main controller loop (fast, up to >200 Hz)
  void slowLoop(void);
  void fastLoop(void);

  // misc methods
  float poseDistance(geometry_msgs::Pose p1,
                     geometry_msgs::Pose p2); // checks distance btw two poses

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
 
// };





#endif  // __SETPOINT_PUBLISHER_H__
