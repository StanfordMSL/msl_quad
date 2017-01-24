// Tele-operation code for quadrotor with Pixhawk
// Author: Zijian Wang, zjwang@stanford.edu
// Multi-robot Systems Lab, Stanford University
// Date: Jan 23, 2017

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>


#define MAX_RP_ANG    20  // degree
#define MAX_Y_ANG     45
#define THROTTLE_MAX  0.8 // a number between 0 and 1

// class declaration
class Telop{
private:
  ros::NodeHandle node_handle_; // the underscore is a tradition for notating private variables in a class
  ros::Subscriber joy_subscriber_;
  ros::Publisher att_publisher_; // attitude publisher
  ros::Publisher thr_publisher_; // throttle publisher

  geometry_msgs::PoseStamped att_;
  std_msgs::Float64 throttle_;

  double roll_cmd_, pitch_cmd_, yaw_cmd_, throttle_cmd_; // recent commands retrieved from Joystick
  tf::Quaternion q_cmd_; // quaternion converted from roll pitch yaw

  void joystick_callback(const sensor_msgs::JoyConstPtr &joy);

public:
  Telop(); // constructor
  void pub_att_thr(void); // publish attitude and throttle
};

// class function implementations
Telop::Telop()
{
  // create subscriber and bind callback function
  joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("/joy", 50, &Telop::joystick_callback, this); // must pass in: 1. the callback function; 2. the object instance by "this" pointer

  // create publishers
  att_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 50);
  thr_publisher_ = node_handle_.advertise<std_msgs::Float64>("mavros/setpoint_attitude/att_throttle", 50);

  // important! otherwise, the quadrotor will have half throttle initially
  throttle_cmd_ = -1.0; // raw joystick is from [-1,1]
}

void Telop::joystick_callback(const sensor_msgs::JoyConstPtr &joy)
{
  roll_cmd_     = -joy->axes[0];
  pitch_cmd_    =  joy->axes[1];
  yaw_cmd_      =  joy->axes[2];
  throttle_cmd_ =  joy->axes[3]; // raw data is [-1,1]. need to map to [0,1]
}

void Telop::pub_att_thr(void) // publish attitude and throttle
{
  q_cmd_.setRPY(roll_cmd_,pitch_cmd_,yaw_cmd_);

  att_.header.stamp = ros::Time::now();
  //att_.header.seq      = countNumber;
  //att_.header.frame_id = 1;
  att_.pose.position.x = 0.0;
  att_.pose.position.y = 0.0;
  att_.pose.position.z = 0.0;
  tf::quaternionTFToMsg(q_cmd_, att_.pose.orientation); // set orientation to message

  throttle_.data = 0.5*THROTTLE_MAX*(throttle_cmd_ + 1); // raw data is [-1,1]. need to map to [0,1]
  att_publisher_.publish(att_);
  thr_publisher_.publish(throttle_);
}


// main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "quad_telop");
  Telop telop; // create an instance of the class
  std::cout << "quad_telop node was initialized successfully!" << std::endl;

  ros::Rate rate(50.0); // very high rate may block the messages
  while(ros::ok()) { 
    telop.pub_att_thr();
				
    ros::spinOnce();
	rate.sleep();
  }

  return 0;
}
