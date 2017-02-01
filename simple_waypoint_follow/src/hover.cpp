#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include "quad_lib/optitrack_listener.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hover");
  Optitrack opti;
  std::cout << "hover node was initialized successfully!" << std::endl;

  ros::Rate rate(50.0); // very high rate may block the messages
  while(ros::ok()) { 
    opti.update_pos();
    opti.update_rpy();
    cout << "(x, y, z) = " << opti.pos[0] << ", " << opti.pos[1] << ", " << opti.pos[2] << endl;
    cout << "(r, p, y) = " << opti.rpy[0] << ", " << opti.rpy[1] << ", " << opti.rpy[2] << endl;
				
    ros::spinOnce();
	rate.sleep();
  }

  return 0;
}
