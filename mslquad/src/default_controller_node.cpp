#include<ros/ros.h>
#include<iostream>
#include<mslquad/px4_base_controller.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "default_controller_node");
    PX4BaseController px4agent;
    std::cout << "PX4 agent initiated." << std::endl;
    ros::spin();
    return 0;
}