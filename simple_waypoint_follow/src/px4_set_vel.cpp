// Position holding using setpoint_position provided by PX4 and MAVROS
// Author: Zijian Wang, zjwang@stanford.edu
// Date: Feb 10, 2017

// Note:
// This program utilizes the velocity controller on Pixhawk
// Based on flight test, it performs better than the position controller
// However, PID gains must be tuned carefully

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_broadcaster.h>
#include "quad_lib/optitrack_listener.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "px4_set_pos");
  ros::NodeHandle nh;
  Optitrack opti;

  ros::Publisher pub_mocap; // publish optitrack info directly to Pixhawk
  ros::Publisher pub_set_pos;
  ros::Publisher pub_vel_cmd;

  geometry_msgs::PoseStamped enu_pose; // used to convert optitrack pose into ENU frame
  tf::Quaternion enu_q; // used to convert optitrack pose into ENU frame

  geometry_msgs::TwistStamped twist_cmd; 
  
  pub_mocap = nh.advertise<geometry_msgs::PoseStamped>("mavros/mocap/pose", 50);
  pub_set_pos = nh.advertise<geometry_msgs::PoseStamped>("setpoint_attitude/attitude", 1);
  pub_vel_cmd = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);

  ros::Duration(0.5).sleep(); // sleep for a while, and wait optitrack stream to sync
  std::cout << "px4_set_pos node was initialized successfully! Now running..." << std::endl;

  geometry_msgs::PoseStamped pose_cmd;
  tf::Quaternion Q_cmd;

  double des_height = 1.0;
  std::cout << "The desired height was set to " << des_height << "m." << std::endl;

  ros::Rate rate(50.0); // very high rate may block the messages
  while(ros::ok()) { 

    //update optitrack readings of position and orientation
    opti.update_pos();
    opti.update_rpy();

    // Publish pose from mocap to Pixhawk
    // Although Pixhawk uses NED, MAVROS uses a "weird" ENU coordinate system
    // To send MAVROS coordinates: X-forward, Y-West, Z-Up
    enu_q.setRPY(opti.rpy[0], -opti.rpy[1], -opti.rpy[2]);
    tf::quaternionTFToMsg(enu_q, enu_pose.pose.orientation);
    enu_pose.pose.position.x = opti.pose_stamped.pose.position.x;
    enu_pose.pose.position.y = -opti.pose_stamped.pose.position.y;
    enu_pose.pose.position.z = -opti.pose_stamped.pose.position.z;
    pub_mocap.publish(enu_pose);

    
    double pos_mavros[3];
    double kp = 2.0;
    const double MAX_SPD = 0.8;

    pos_mavros[0] = opti.pos[0];
    pos_mavros[1] = - opti.pos[1];
    pos_mavros[2] = - opti.pos[2];

    twist_cmd.header.stamp = ros::Time::now();
    twist_cmd.twist.linear.x = 1.5*kp * (0.0 - pos_mavros[0]); // simple proportional speed controller
    twist_cmd.twist.linear.y = 1.5*kp * (0.0 - pos_mavros[1]);
    twist_cmd.twist.linear.z = kp * (des_height - pos_mavros[2]);

    // TODO: Yaw control?

    // speed limit
    if (twist_cmd.twist.linear.x > MAX_SPD )
      { twist_cmd.twist.linear.x = MAX_SPD; }
    if (twist_cmd.twist.linear.x < -MAX_SPD)
      { twist_cmd.twist.linear.x = -MAX_SPD; }

    if (twist_cmd.twist.linear.y > MAX_SPD)
      { twist_cmd.twist.linear.y = MAX_SPD; }
    if (twist_cmd.twist.linear.y < -MAX_SPD)
      { twist_cmd.twist.linear.y = -MAX_SPD; }

    if (twist_cmd.twist.linear.z > MAX_SPD)
      { twist_cmd.twist.linear.z = MAX_SPD; }
    if (twist_cmd.twist.linear.z < -MAX_SPD)
      { twist_cmd.twist.linear.z = -MAX_SPD; }


    pub_vel_cmd.publish(twist_cmd);
    
				
    ros::spinOnce();
	rate.sleep();
  }

  return 0;
}
