/* copyright[2019] <msl>
**************************************************************************
  File Name    : pilot.cpp
  Author       : Kunal Shah, Jun En Low, Alexander Koufos
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Feb 9, 2021.
  Description  : px4 pilot class
**************************************************************************/

#include<mslquad/pilot.h>
#include<cmath>

Pilot::Pilot() {
  ROS_INFO("Initalizing Controller");
}

void Pilot::passback(const geometry_msgs::PoseStamped::ConstPtr& msg,
                geometry_msgs::PoseStamped& receipt) {
  receipt = *msg;
}

Pilot::~Pilot() {
  ROS_INFO("Terminating Controller");
}

int main(int argc, char const *argv[]) {
    Pilot pilot;
    return 0;
  }
