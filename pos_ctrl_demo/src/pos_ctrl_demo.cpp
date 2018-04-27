#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

double curX, curY, curZ;

double dist(double x1, double y1, double z1,
            double x2, double y2, double z2) {
  return std::sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2));
}

void poseCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  curX = msg->pose.position.x;
  curY = msg->pose.position.y;
  curZ = msg->pose.position.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pos_ctrl_demo");
  ros::NodeHandle nh;

  ros::Publisher pubPosCmd;
  ros::Subscriber subPose;

  pubPosCmd = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
  subPose = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, poseCb);

  ros::Duration(0.1).sleep(); // sleep for a while, and wait optitrack stream to sync

  geometry_msgs::PoseStamped posCmd;
  double cmdX = 0.0;
  double cmdY = 0.0;
  double cmdZ = 1.2;
  posCmd.pose.position.x = cmdX;
  posCmd.pose.position.y = cmdY;
  posCmd.pose.position.z = cmdZ;
  posCmd.pose.orientation.x = 0.0;
  posCmd.pose.orientation.y = 0.0;
  posCmd.pose.orientation.z = 0.707;
  posCmd.pose.orientation.w = 0.707;

  ros::Rate rate(50.0); // very high rate may block the messages

  std::cout << "Start position control demo..." << std::endl;

  double d = 10000;
  bool isReached = false;
  while(ros::ok()) { 
    if (!isReached) {
      double d = dist(cmdX, cmdY, cmdZ, curX, curY, curZ);
      if(d > 0.1) {
        std::cout << "Current dist: " << d << std::endl;
      } else {
        isReached = true;
        posCmd.pose.position.z = 0.0;
        std::cout << "Reached goal, descending..." << std::endl;
      }
    }

    posCmd.header.stamp = ros::Time::now();
    pubPosCmd.publish(posCmd); 
				
    ros::spinOnce();
	  rate.sleep();
  }

  return 0;
}
