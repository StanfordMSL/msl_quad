// Class implementation for interfacing with optitrack
// Author: Zijian Wang, zjwang@stanford.edu
// Multi-robot Systems Lab, Stanford University
// Date: Jan 31, 2017

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include "quad_lib/optitrack_listener.h"


Optitrack::Optitrack() {
    _sub_pose = _nh.subscribe<geometry_msgs::PoseStamped>("/Robot_1/pose", 10, &Optitrack::pose_callback, this);

    std::cout << "[optitrack_listener.cpp]: object initiated." << std::endl;
}

Optitrack::~Optitrack() {
    std::cout << "[optitrack_listener.cpp]: object destroyed." << std::endl;
}

void Optitrack::pose_callback(const geometry_msgs::PoseStampedConstPtr &ps) {
    pose_stamped = *ps;
}

void Optitrack::update_pos(void) {
    pos[0] = pose_stamped.pose.position.x;
    pos[1] = pose_stamped.pose.position.y;
    pos[2] = pose_stamped.pose.position.z;
}

void Optitrack::update_rpy(void) {
    double r, p, y;
    tf::quaternionMsgToTF(pose_stamped.pose.orientation, Q);
    tf::Matrix3x3(Q).getRPY(r, p, y);
    rpy[0] = r;
    rpy[1] = p;
    rpy[2] = y;
}
