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

  ros::param::get("~namespace", namespace);
  ros::param::get("~takeoff_height", takeoffHeight);
  ros::param::get("~max_vel", maxVel);
  ros::param::get("~control_freq", controlRate);

  ROS_INFO_STREAM(namespace << "set to INIT");


  controlTimer = nh_.createTimer(
        ros::Duration(1.0/controlRate),
        &Pilot::controlTimerCB, this);
}


void Pilot::controlTimerCB(const ros::TimerEvent& event) {
    if (state_ == State::EMERGENCY_LAND) {
        this->emergencyFailsafe();
    } else {
        this->controlLoop();
    }
}


// void Pilot::passback(const geometry_msgs::PoseStamped::ConstPtr& msg,
//                 geometry_msgs::PoseStamped& receipt) {
//   receipt = *msg;
// }

Pilot::~Pilot() {
  ROS_INFO("Terminating Controller");
}

int main(int argc, char const *argv[]) {
    Pilot pilot;
    ROS_INFO(pilot.namespace+" initlaized")
    ros::spin();
    return 0;
  }
