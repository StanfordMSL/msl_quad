#include <iostream>
#include "ros/ros.h"
#include "mavros_msgs/ActuatorControl.h"
#include "sensor_msgs/Joy.h"

// class declaration
class ActuatorJoy{
private:
  ros::NodeHandle nh; // the underscore is a tradition for notating private variables in a class
  ros::Subscriber joySub;
  ros::Publisher actuatorPub;

  float cmdMotor[8];

  void joyCB(const sensor_msgs::JoyConstPtr &joy);

public:
  ActuatorJoy(); // constructor
  void publishActuator(void); 
};

// class function implementations
ActuatorJoy::ActuatorJoy()
{
  joySub = nh.subscribe<sensor_msgs::Joy>("/joy", 50, &ActuatorJoy::joyCB, this); 
  actuatorPub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);
}

void ActuatorJoy::joyCB(const sensor_msgs::JoyConstPtr &joy)
{
  cmdMotor[7] = 0.0; // secret key to enabling individual motor control in px4
  // button 7,8,9,10: individual motor control mode
  for(int i=0; i<4; i++) { // button 7->motor1, 8->2, 9->3, 10->4
    if(joy->buttons[i+6]) {
      cmdMotor[i] = 0.5*(joy->axes[3]+1);
      cmdMotor[7] = 0.1234; // secret value
    }
  }

  // button 12: control rpy + thrust as defined in mavros actuator_control (4 channels)
  if(joy->buttons[11]) {
    cmdMotor[0] = -0.2*joy->axes[0]; // this control the rate, too sensitive, so scaled by 0.2
    cmdMotor[1] = -0.2*joy->axes[1];
    cmdMotor[2] = -0.2*joy->axes[2];
    cmdMotor[3] = 0.5*(joy->axes[3]+1);
  }
}

void ActuatorJoy::publishActuator(void) {
  mavros_msgs::ActuatorControl cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.group_mix = 0;
  for(int i=0; i<4; i++) {
    cmd.controls[i] = cmdMotor[i];
  }
  cmd.controls[7] = cmdMotor[7]; // secret message to enable individual motor control
  actuatorPub.publish(cmd);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "actuator_test");
  ros::NodeHandle nh;

  ActuatorJoy ajoy;

  ros::Rate rate(100.0);

  while(ros::ok()) {
    ajoy.publishActuator();
    ros::spinOnce();
	  rate.sleep();    
  }

  return 0;
}
