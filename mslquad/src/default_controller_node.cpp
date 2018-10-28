/**************************************************************************
  File Name    : default_controller_node.cpp
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Oct 27, 2018.
**************************************************************************/

#include<ros/ros.h>
#include<iostream>
#include<mslquad/px4_base_controller.h>
#include<mslquad/yaw_track_controller.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "default_controller_node");
    //PX4BaseController *px4agent = new PX4BaseController();
    PX4BaseController *px4agent = new YawTrackController();
    std::cout << "PX4 agent initiated." << std::endl;
    ros::spin();
    delete px4agent;
    return 0;
}