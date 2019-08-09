/* copyright[2019] <msl>
**************************************************************************
  File Name    : traj_track_controller.h
  Author       : Kunal Shah
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Aug 5, 2019.
  Description  : Open Loop Trajectory tracking
**************************************************************************/

#ifndef __TRAJ_TRACK_CONTROLELR_H__
#define __TRAJ_TRACK_CONTROLELR_H__

#include <string>
#include<mslquad/px4_base_controller.h>
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Time.h"


class TrajTrackController : public PX4BaseController {
 public:
    TrajTrackController();
    ~TrajTrackController();

 protected:
    int trajIdx = 0;
    int trajNPoints = 0;
    float trajTimeStep = 0;
    float trajKp = 0;
    ros::Time trajStartTime;
    trajectory_msgs::MultiDOFJointTrajectoryPoint traj;
    void controlLoop(void) override;
    void slowLoop(void) override;
    void takeoff(void) override;
    // traj topic
    ros::Subscriber trajTargetSub_;  // px4 pose sub
    void trajTargetCB(
        const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg);
    // traj file
    bool loadTrajFile;
    void parseTrajFile(std::string* trajFilePtr);
    // reverse the trajectory for speed up
    // internals
    Eigen::Vector3d desPos;  // desired position
    Eigen::Vector3d desVel;  // desired velocity
    float posError = 100;  // distance to target pos
    bool updateTarget(void);
    // go and stop signals
    bool scramble = false;
    ros::Subscriber scambleSub_;
    void scrambleSubCB(const std_msgs::Time::ConstPtr& msg);
};
#endif
