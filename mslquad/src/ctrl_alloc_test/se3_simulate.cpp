#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "se3_simulate");

  ros::NodeHandle n;
  ros::Publisher se3_pub = n.advertise<std_msgs::Float32MultiArray>("se3_command", 1000);

  ros::Rate loop_rate(10);

  float force = 1.05*9.81;
  float tau_x = 0;
  float tau_y = 0;
  float tau_z = 0;

  while (ros::ok())
  {
    std_msgs::Float32MultiArray command;

    command.data= {force, tau_x, tau_y, tau_z};

    se3_pub.publish(command);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}