#include <iostream>
#include "ros/ros.h"
#include "mavros_msgs/ActuatorControl.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <Eigen/Dense>

class SE3Controller {
private:
  ros::NodeHandle nh;
  ros::Subscriber poseSub;
  ros::Subscriber velSub;
  ros::Subscriber joySub;
  ros::Publisher actuatorPub;

  Eigen::Matrix3d mea_R;
  Eigen::Vector3d mea_wb;
  Eigen::Vector3d mea_pos;
  Eigen::Vector3d mea_vel;

  Eigen::Matrix4d W;

  double fzCmd; // result of SE3 controller computation
  Eigen::Vector3d tauCmd; // result of SE3 controller computation
  double motorCmd[4];
  double joyCmd[4]; // desired thrust + euler angles from joystick

  double KP;
  double KV;
  double KR;
  double KW;
  double M;
  double g;
  double TCOEFF;
  
  void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void joyCB(const sensor_msgs::JoyConstPtr &joy);

public:
  SE3Controller(void);
  void calcSE3(
          const Eigen::Vector3d &r_euler, 
          const Eigen::Vector3d &r_wb, 
          const Eigen::Vector3d &r_pos,
          const Eigen::Vector3d &r_vel, 
          const Eigen::Vector3d &r_acc);
  void joySE3(void); // give reference input to SE3 controller from joystick. This enables manual joystick control
};

SE3Controller::SE3Controller(void) {
  KP = 4.0; //
  KV = 4.0; //
  KR = 2.0; // 2.0
  KW = 0.5; // 0.5
  M = 1.5; // Iris in Gazebo
  g = 9.8;
  TCOEFF = 6.57; // thrust coefficient

  /*
  // Iris in Gazebo
  W << 1, 1, 1, 1,
      -0.22, 0.2, 0.22, -0.2,
      -0.13, 0.13, -0.13, 0.13,
      -0.06, -0.06, 0.06, 0.06;
  */
  
  // MSL quad
  W << 1, 1, 1, 1,
      -0.12, 0.12, 0.12, -0.12,
      -0.12, 0.12, -0.12, 0.12,
      -0.06, -0.06, 0.06, 0.06;
  
  // handle ros parameters
  ros::param::get("~KP", KP);
  ros::param::get("~KV", KV);
  ros::param::get("~KR", KR);
  ros::param::get("~KW", KW);
  ros::param::get("~M", M);
  ros::param::get("~TCOEFF", TCOEFF);

  std::cout << "Using the following parameters: " << std::endl;
  std::cout << "KP = " << KP << std::endl;
  std::cout << "KV = " << KV << std::endl;
  std::cout << "KR = " << KR << std::endl;
  std::cout << "KW = " << KW << std::endl;
  std::cout << "M = " << M << std::endl;
  std::cout << "TCOEFF = " << TCOEFF << std::endl;

  poseSub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, &SE3Controller::poseSubCB, this);
  velSub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 1, &SE3Controller::velSubCB, this);
  actuatorPub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);

  joySub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, &SE3Controller::joyCB, this);
}

void SE3Controller::poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // By default, MAVROS gives local_position/pose in ENU
  mea_pos(0) = msg->pose.position.x;
  mea_pos(1) = msg->pose.position.y;
  mea_pos(2) = msg->pose.position.z;

  double q1 = msg->pose.orientation.w;
  double q2 = msg->pose.orientation.x;
  double q3 = msg->pose.orientation.y;
  double q4 = msg->pose.orientation.z;
  double q1s = q1*q1;
  double q2s = q2*q2;
  double q3s = q3*q3;
  double q4s = q4*q4;
  // MATLAB version
  //mea_R <<
  //  q1s+q2s-q3s-q4s, 2.0*(q2*q3+q1*q4) ,2.0*(q2*q4-q1*q3),
  //  2.0*(q2*q3-q1*q4), q1s-q2s+q3s-q4s, 2.0*(q3*q4+q1*q2),
  //  2.0*(q2*q4-q1*q3), 2.0*(q3*q4-q1*q2), q1s-q2s-q3s+q4s;

  // Wikipedia version
  mea_R <<
    q1s+q2s-q3s-q4s, 2.0*(q2*q3-q1*q4) ,2.0*(q2*q4+q1*q3),
    2.0*(q2*q3+q1*q4), q1s-q2s+q3s-q4s, 2.0*(q3*q4-q1*q2),
    2.0*(q2*q4-q1*q3), 2.0*(q3*q4+q1*q2), q1s-q2s-q3s+q4s;
}

void SE3Controller::velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  // By default, MAVROS gives local_position/velocity in ENU
  mea_vel(0) = msg->twist.linear.x;
  mea_vel(1) = msg->twist.linear.y;
  mea_vel(2) = msg->twist.linear.z;
  mea_wb(0) = msg->twist.angular.x;
  mea_wb(1) = msg->twist.angular.y;
  mea_wb(2) = msg->twist.angular.z;
}

void SE3Controller::joyCB(const sensor_msgs::Joy::ConstPtr& joy) {
  joyCmd[0] = 0.2*joy->axes[3]; // [-2,2], can go either up or down
  joyCmd[1] = -0.2*joy->axes[0];
  joyCmd[2] = 0.2*joy->axes[1];
  joyCmd[3] = 1.0*joy->axes[2];
}

void SE3Controller::joySE3(void) {
  KP = 0.0; // need to disable position control
  Eigen::Vector3d r_euler(joyCmd[1], joyCmd[2], joyCmd[3]);
  Eigen::Vector3d r_wb(0, 0, 0);
  Eigen::Vector3d r_pos(0, 0, 0); 
  Eigen::Vector3d r_vel(joyCmd[2], -joyCmd[1], joyCmd[0]); 
  Eigen::Vector3d r_acc(0, 0, 0);
  calcSE3(r_euler, r_wb, r_pos, r_vel, r_acc);
}

void SE3Controller::calcSE3(const Eigen::Vector3d &r_euler, const Eigen::Vector3d &r_wb, 
          const Eigen::Vector3d &r_pos, const Eigen::Vector3d &r_vel, 
          const Eigen::Vector3d &r_acc) {
  // Note: this code is modified to directly compute in ENU, since MAVROS gives ENU by default
  Eigen::Vector3d zw(0.0,0.0,1.0);
  Eigen::Vector3d Fdes = -KP*(mea_pos-r_pos) - KV*(mea_vel-r_vel) + M*g*zw + M*r_acc;
  fzCmd = Fdes.dot(mea_R.col(2));

  Eigen::Vector3d zb_des = Fdes / Fdes.norm();
  Eigen::Vector3d xc_des(std::cos(r_euler(2)), std::sin(r_euler(2)), 0.0); // only need yaw from reference euler
  Eigen::Vector3d zb_des_cross_xc_des = zb_des.cross(xc_des);
  Eigen::Vector3d yb_des = zb_des_cross_xc_des / zb_des_cross_xc_des.norm();
  Eigen::Vector3d xb_des = yb_des.cross(zb_des);
  Eigen::MatrixXd R_des(3,3);
  R_des.col(0) = xb_des;
  R_des.col(1) = yb_des;
  R_des.col(2) = zb_des;

  Eigen::MatrixXd temp(3,3);
  temp = R_des.transpose()*mea_R - mea_R.transpose()*R_des;
  Eigen::Vector3d eR(temp(2,1), temp(0,2), temp(1,0));
  eR = 0.5*eR;
  Eigen::Vector3d ew = mea_wb-r_wb;
  eR(2) /= 3.0; // Note: reduce gain on yaw
  ew(2) /= 3.0;
  tauCmd = -KR*eR - KW*ew;

  //std::cout << "fzCmd = " << fzCmd << std::endl;
  //std::cout << "tauCmd = " << tauCmd << std::endl;

  Eigen::Vector4d wrench(fzCmd, tauCmd(0), tauCmd(1), tauCmd(2));
  Eigen::Vector4d ffff = W.colPivHouseholderQr().solve(wrench);

  //std::cout << ffff << std::endl;
  for(int i=0; i<4; i++) {
    // motorCmd[i] = ffff(i)/4.55; // msl quad
    // motorCmd[i] = ffff(i)/6.57; // iris in gazebo
    motorCmd[i] = ffff(i)/TCOEFF;
  }

  // publish commands
  mavros_msgs::ActuatorControl cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.group_mix = 0;
  for(int i=0; i<4; i++) {
    cmd.controls[i] = motorCmd[i];
  }
  actuatorPub.publish(cmd);
}
  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "se3_controller");
  ros::NodeHandle nh;

  SE3Controller se3ctrl;

  ros::Rate rate(200.0);

  Eigen::Vector3d r_euler(0, 0, 0);
  Eigen::Vector3d r_wb(0, 0, 0);
  Eigen::Vector3d r_pos(0, 0, 1.0); 
  Eigen::Vector3d r_vel(0, 0, 0); 
  Eigen::Vector3d r_acc(0, 0, 0);

  while(ros::ok()) {
    //se3ctrl.calcSE3(r_euler, r_wb, r_pos, r_vel, r_acc);
    se3ctrl.joySE3();
    ros::spinOnce();
	  rate.sleep();    
  }

  return 0;
}
