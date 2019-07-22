/* copyright[2019] <msl>
**************************************************************************
  File Name    : pose_track_controller.cpp
  Author       : Kunal Shah
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Dec 3, 2018.
  Description  : Basic control + pose tracking
**************************************************************************/

#include<mslquad/pose_track_controller.h>
#include<iostream>

PoseTrackController::PoseTrackController() {
    // retrieve ROS parameter
    std::string poseTargetTopic;
    ros::param::param<std::string>("pose_target_topic",
                           poseTargetTopic,
                           "command/pose");
    // ROS subs and pub
    poseTargetSub_ = nh_.subscribe<geometry_msgs::Pose>(
        poseTargetTopic, 1, &PoseTrackController::poseTargetCB, this);
    ROS_INFO_STREAM("Subscribed: "<< poseTargetTopic);

    // inital pose is pose after takeoff
    targetPoseSp_.pose.position.x = takeoffPose_.pose.position.x;
    targetPoseSp_.pose.position.y = takeoffPose_.pose.position.y;
    targetPoseSp_.pose.position.z = takeoffHeight_;
}

PoseTrackController::~PoseTrackController() {
}

void PoseTrackController::poseTargetCB(
        const geometry_msgs::Pose::ConstPtr& msg) {
    targetPoseSp_.pose = *msg;
}

void PoseTrackController::controlLoop(void) {
    // this is the world's easiest controller
    targetPoseSp_.header.stamp = ros::Time::now();
    px4SetPosPub_.publish(targetPoseSp_);
}
