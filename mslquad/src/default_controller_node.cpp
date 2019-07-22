/**************************************************************************
  File Name    : default_controller_node.cpp
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Oct 27, 2018.
  Description  : main ROS node file to instantiate different types of 
                 controllers, according to launch file.
**************************************************************************/

#include<ros/ros.h>
#include<iostream>
#include<string>
#include"mslquad/px4_base_controller.h"
#include"mslquad/yaw_track_controller.h"
#include"mslquad/pose_track_controller.h"
#include"mslquad/se3_controller.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "default_controller_node");
    PX4BaseController *px4agent;
    std::string ctrlType;
    ros::param::get("~controller_type", ctrlType);
    if (ctrlType == "default") {
        px4agent = new PX4BaseController();
    } else if (ctrlType == "se3") {
        px4agent = new SE3Controller();
    } else if (ctrlType == "yaw_track") {
        px4agent = new YawTrackController();
    } else if (ctrlType == "pose_track") {
        px4agent = new PoseTrackController();
    } else {
        ctrlType = "default";
        px4agent = new PX4BaseController();
    }
    std::cout << "PX4 controller node initiated. Controller type: "
              << ctrlType << std::endl;
    ros::spin();
    delete px4agent;
    return 0;
}