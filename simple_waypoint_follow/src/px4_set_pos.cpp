// Position holding using setpoint_position provided by PX4 and MAVROS
// Author: Zijian Wang, zjwang@stanford.edu
// Date: Feb 08, 2017

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include "quad_lib/optitrack_listener.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "px4_set_pos");
  ros::NodeHandle nh;
  Optitrack opti;

  ros::Publisher pub_mocap; // publish optitrack info directly to Pixhawk
  ros::Publisher pub_set_pos;

  geometry_msgs::PoseStamped enu_pose; // used to convert optitrack pose into ENU frame
  tf::Quaternion enu_q; // used to convert optitrack pose into ENU frame
  
  pub_mocap = nh.advertise<geometry_msgs::PoseStamped>("mavros/mocap/pose", 50);
  pub_set_pos = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);

  ros::Duration(0.5).sleep(); // sleep for a while, and wait optitrack stream to sync
  std::cout << "px4_set_pos node was initialized successfully! Now running..." << std::endl;

  geometry_msgs::PoseStamped pose_cmd;
  tf::Quaternion Q_cmd;

  ros::Rate rate(50.0); // very high rate may block the messages
  while(ros::ok()) { 

    //update optitrack readings of position and orientation
    opti.update_pos();
    opti.update_rpy();

/*
    // send NED coordinates, this does not work
    pub_mocap.publish(opti.pose_stamped);
    
    Q_cmd.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(Q_cmd, pose_cmd.pose.orientation);
    pose_cmd.pose.position.x = 0.0;
    pose_cmd.pose.position.y = 0.0;
    pose_cmd.pose.position.z = -0.3;   
    pub_set_pos.publish(pose_cmd); 
*/


    // test, this succeeded
    // Although Pixhawk uses NED, MAVROS uses a "weird" ENU coordinate system
    // To send MAVROS coordinates: X-forward, Y-West, Z-Up
    enu_q.setRPY(opti.rpy[0], -opti.rpy[1], -opti.rpy[2]);
    tf::quaternionTFToMsg(enu_q, enu_pose.pose.orientation);
    enu_pose.pose.position.x = opti.pose_stamped.pose.position.x;
    enu_pose.pose.position.y = -opti.pose_stamped.pose.position.y;
    enu_pose.pose.position.z = -opti.pose_stamped.pose.position.z;
    pub_mocap.publish(enu_pose);

    Q_cmd.setRPY(0.0, 0.0, 0.0);
    tf::quaternionTFToMsg(Q_cmd, pose_cmd.pose.orientation);
    pose_cmd.pose.position.x = 0.0;
    pose_cmd.pose.position.y = 0.0;
    pose_cmd.pose.position.z = 0.3; // hover at z=0.3 
    pub_set_pos.publish(pose_cmd); 

				
    ros::spinOnce();
	rate.sleep();
  }

  return 0;
}
