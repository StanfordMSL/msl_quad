/**************************************************************************
  File Name    : yaw_track_controller.cpp
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Oct 27, 2018.
  Description  : Basic control + yaw tracking
**************************************************************************/

#include<mslquad/yaw_track_controller.h>
#include<iostream>
#include<cmath>

#define PI_M 3.1415926

YawTrackController::YawTrackController() : diffYawPrev_(0), KP_YAW_(0.8), KD_YAW_(0) {
    // retrieve ROS parameter
    std::string yawTargetTopic;
    ros::param::get("~yaw_target_topic", yawTargetTopic);
    ros::param::get("~kp_yaw", KP_YAW_);
    ros::param::get("~kd_yaw", KD_YAW_);
    // ROS subs and pub
    yawTargetPoseSub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        yawTargetTopic, 
        1, &YawTrackController::yawTargetPoseCB, this);
}

YawTrackController::~YawTrackController() {

}

void YawTrackController::yawTargetPoseCB(
        const geometry_msgs::PoseStamped::ConstPtr& msg) {
    yawTargetPose_ = *msg;
}

void YawTrackController::controlLoop(void) {
    double relX = yawTargetPose_.pose.position.x - curPose_.pose.position.x;
    double relY = yawTargetPose_.pose.position.y - curPose_.pose.position.y;
    double desYaw = atan2(relY, relX);
    double curYaw = getYawRad();

    double diffYaw = desYaw-curYaw;
    if(diffYaw>PI_M) diffYaw -= 2*PI_M;
    if(diffYaw<-PI_M) diffYaw += 2*PI_M; 
    double diffYawDot = diffYawPrev_ - diffYaw;
    diffYawPrev_ = diffYaw;
    
    geometry_msgs::Twist twist;
    Eigen::Vector3d desVel;
    Eigen::Vector3d desPos;
    if(0 == desTraj_.points.size()) { // not receiving desired trajectory
        desPos(0) = takeoffPose_.pose.position.x;
        desPos(1) = takeoffPose_.pose.position.y;
        desPos(2) = takeoffHeight_;
    } else {
        desPos(0) = desTraj_.points[1].transforms[0].translation.x;
        desPos(1) = desTraj_.points[1].transforms[0].translation.y;
        if(flagOnly2D_) {
            desPos(2) = takeoffHeight_;
            calcVelCmd2D(desVel, desPos, maxVel_, 4.0);
        } else {
            desPos(2) = desTraj_.points[1].transforms[0].translation.z;
            calcVelCmd(desVel, desPos, maxVel_, 4.0);
        }
    }
    calcVelCmd(desVel, desPos, maxVel_, 4.0);
    twist.linear.x = desVel(0);
    twist.linear.y = desVel(1);
    twist.linear.z = desVel(2);
    twist.angular.z = KP_YAW_ * diffYaw + KD_YAW_ * diffYawDot; // yaw tracking, diffYaw > 0 turn left, otherwise turn right
    px4SetVelPub_.publish(twist);
}