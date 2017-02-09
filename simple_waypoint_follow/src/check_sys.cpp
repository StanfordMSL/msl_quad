// Utility node to print debug information
// Author: Zijian Wang, zjwang@stanford.edu
// Date: Feb 08, 2017

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include "quad_lib/optitrack_listener.h"

// utility program to check system status, such as optitrack output

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "check_sys");
  ros::NodeHandle nh;
  Optitrack opti;

  ros::Duration(0.5).sleep(); // sleep for a while, and wait optitrack stream to sync
  std::cout << "check_sys node was initialized successfully! Now running..." << std::endl;


  ros::Rate rate(10.0);
  while(ros::ok()) { 
    opti.update_pos();
    opti.update_rpy();
    cout << "(x, y, z) = " << setprecision(5) << opti.pos[0] << ", " << opti.pos[1] << ", " << opti.pos[2] << endl;
    cout << "(r, p, y) = " << setprecision(5) << opti.rpy[0]*180.0/3.14159 << ", " << opti.rpy[1]*180.0/3.14159 << ", " << opti.rpy[2]*180.0/3.14159 << endl;
				
    ros::spinOnce();
	rate.sleep();
  }

  return 0;
}
