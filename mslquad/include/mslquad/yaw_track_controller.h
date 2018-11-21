/**************************************************************************
  File Name    : yaw_track_controller.h
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Oct 27, 2018.
  Description  : Basic control + yaw tracking
**************************************************************************/

#ifndef __YAW_TRACK_CONTROLELR_H__
#define __YAW_TRACK_CONTROLELR_H__

#include<mslquad/px4_base_controller.h>

class YawTrackController : public PX4BaseController {
public:
    YawTrackController();
    ~YawTrackController();

protected:
    geometry_msgs::PoseStamped yawTargetPose_;

    void controlLoop(void) override;

private:
    ros::Subscriber yawTargetPoseSub_; // px4 pose sub

    double diffYawPrev_; // variable for derivative control
    double KP_YAW_; // P and D gains for yaw control
    double KD_YAW_;

    void yawTargetPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

#endif