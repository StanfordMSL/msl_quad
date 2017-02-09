// Simple program that tries to use PID and mocap position reading to hold position
// Does not work very well since the controller is too rudimentary

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include "quad_lib/optitrack_listener.h"

using namespace std;

double z_des = -0.3;
double Kz_p = 0.2;
double Kz_i = 0.0005;
double Kang = 0.1; // gain for euler angle

#define ANG_LIMIT 0.1

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hover");
  ros::NodeHandle nh;
  Optitrack opti;

  ros::Publisher pub_att; // attitude publisher
  ros::Publisher pub_thr; // throttle publisher

  pub_att = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude", 50);
  pub_thr = nh.advertise<std_msgs::Float64>("mavros/setpoint_attitude/att_throttle", 50);

  ros::Duration(0.5).sleep(); // sleep for a while, and wait optitrack stream to sync
  std::cout << "hover node was initialized successfully! Now running..." << std::endl;

  // variables for control
  double thr_cmd;
  double z_err, z_err_sum;
  double roll_cmd, pitch_cmd;
  double x_err, y_err;
  std_msgs::Float64 thr_msg;
  geometry_msgs::PoseStamped pose_msg;
  tf::Quaternion Q_cmd;

  pose_msg.pose.position.x = 0.0;
  pose_msg.pose.position.y = 0.0;
  pose_msg.pose.position.z = 0.0;

  ros::Rate rate(50.0); // very high rate may block the messages
  while(ros::ok()) { 

    //update optitrack readings of position and orientation
    opti.update_pos();
    opti.update_rpy();

    // throttle feedback
    z_err = z_des - opti.pos[2];
    z_err_sum += z_err;
    if(z_err_sum > 200)  z_err_sum = 200;
    if(z_err_sum < -200) z_err_sum = -200;

    thr_cmd = 0.5 + Kz_p * (-z_err) + Kz_i * (-z_err_sum); // 0.5 is feedforward, 0.53 is roughly the throttle that balances the weight
    if(thr_cmd < 0.4)
      thr_cmd = 0.4;
    if(thr_cmd > 0.67)
      thr_cmd = 0.67;

    thr_msg.data = thr_cmd;

    // roll pitch feedback
    x_err = 0.0 - opti.pos[0];
    y_err = 0.0 - opti.pos[1]; 
    roll_cmd = Kang * x_err;
    pitch_cmd = -Kang * y_err;
    if(roll_cmd > ANG_LIMIT)  roll_cmd = ANG_LIMIT;
    if(roll_cmd < -ANG_LIMIT)  roll_cmd = -ANG_LIMIT;
    if(pitch_cmd > ANG_LIMIT)  pitch_cmd = ANG_LIMIT;
    if(pitch_cmd < -ANG_LIMIT)  pitch_cmd = -ANG_LIMIT;
    Q_cmd.setRPY(roll_cmd, pitch_cmd, 0.0);
    tf::quaternionTFToMsg(Q_cmd, pose_msg.pose.orientation);
    
    // publish command
    pub_thr.publish(thr_msg);
    pub_att.publish(pose_msg);
    		
    ros::spinOnce();
	rate.sleep();
  }

  return 0;
}
