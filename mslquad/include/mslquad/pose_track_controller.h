/**************************************************************************
  File Name    : pose_track_controller.h
  Author       : Kunal Shah
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Dec 3, 2018.
  Description  : Basic control + pose tracking
**************************************************************************/

#ifndef __POSE_TRACK_CONTROLELR_H__
#define __POSE_TRACK_CONTROLELR_H__

#include<mslquad/px4_base_controller.h>

class PoseTrackController : public PX4BaseController {
public:
    PoseTrackController();
    ~PoseTrackController();

protected:
    geometry_msgs::PoseStamped targetPoseSp_;

    void controlLoop(void) override;

private:
    ros::Subscriber poseTargetSub_; // px4 pose sub

    void poseTargetCB(const geometry_msgs::Pose::ConstPtr& msg);
};

#endif