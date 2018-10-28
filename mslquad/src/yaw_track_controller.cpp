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

YawTrackController::YawTrackController() {
    yawTargetPoseSub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/quad2/mavros/vision_pose/pose", 
        1, &YawTrackController::yawTargetPoseCB, this);
    // TODO: check whether the topic is indeed being published
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
    const double K_YAW = 0.5; // TODO: make parameter
    
    geometry_msgs::Twist twist;
    if(0 == desTraj_.points.size()) {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
    } else {
        // std::cout << "Traj #: " << desTraj.header.seq << std::endl; // print the seq # of traj
        Eigen::Vector3d desVel;
        Eigen::Vector3d desPos;
        desPos(0) = desTraj_.points[1].transforms[0].translation.x;
        desPos(1) = desTraj_.points[1].transforms[0].translation.y;
        desPos(2) = fixedHeight_;
        calcVelCmd2D(desVel, desPos, maxVel_, 4.0);
        geometry_msgs::Twist twist;
        twist.linear.x = desVel(0);
        twist.linear.y = desVel(1);
        twist.linear.z = desVel(2);
    }
    twist.angular.z = K_YAW * diffYaw; // yaw tracking, diffYaw > 0 turn left, otherwise turn right
    px4SetVelPub_.publish(twist);
}