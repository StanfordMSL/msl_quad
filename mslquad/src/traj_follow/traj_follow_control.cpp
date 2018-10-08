/**************************************************************************
  File Name    : traj_follow.cpp
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Aug 5, 2018.
  Description  : 
    - Trajectory following. Tested with up to 4 quadrotors.
    - Uses a look-ahead strategy (i.e., track the "next" waypoint
        on the trajectory).
    - Assume the commanded trajectory is updated at least 10Hz
    - Currently, quadrotor only operates at a fixed height (commanded
        trajectory is 2D). 3D is TODO.
**************************************************************************/

#include "ros/ros.h"
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <string>
#include <cmath>

class PX4Agent{
private:
    geometry_msgs::PoseStamped pose; // current pose of quad from px4
    trajectory_msgs::MultiDOFJointTrajectory desTraj; // desired trajectory from the planner
    std::string quad_ns;
    float fixed_height; // the fixed height that the quad will be flying at
    float max_vel;

    ros::NodeHandle nh;
    ros::Subscriber cmdTrajSub; // game planner command traj sub
    ros::Subscriber px4PoseSub; // px4 pose sub
    ros::Publisher px4SetVelPub; // px4 setpoint_velocity command
    ros::Publisher odomPub; // publish pose of px4 agent to the planner

    ros::Timer controlTimer;

    void pathCB(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& traj);
    void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg); // pose callback on PX4 local position
    void controlTimerCB(const ros::TimerEvent& event);

    // calculate desired velocity vector given desired position
    // vmax is the maximum velocity. return the norm of the position error
    float calcVelCmd(Eigen::Vector3d& desVel, const Eigen::Vector3d& desPos, 
            const float vmax, const float kp);
    // in this version, vmax only constrains XY velocity. Z vel is independent
    float calcVelCmd2D(Eigen::Vector3d& desVel, const Eigen::Vector3d& desPos, 
            const float vmax, const float kp);
    
    void takeoff(const float desx, const float desy, const float desz);

public:
    PX4Agent();
    ~PX4Agent();
};

PX4Agent::PX4Agent() {
    // retrieve parameters
    quad_ns = "/";
    std::string strtmp;
    ros::param::get("~quad_ns", strtmp);
    quad_ns += strtmp;
    quad_ns += "/";

    ros::param::get("~fixed_height", fixed_height);

    ros::param::get("~max_vel", max_vel);

    // pub and subs
    cmdTrajSub = nh.subscribe("command/trajectory", 1, &PX4Agent::pathCB, this);
    px4PoseSub = nh.subscribe<geometry_msgs::PoseStamped>(quad_ns+"mavros/local_position/pose", 1, &PX4Agent::poseSubCB, this);
    px4SetVelPub = nh.advertise<geometry_msgs::Twist>(quad_ns+"mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    odomPub = nh.advertise<nav_msgs::Odometry>("ground_truth/odometry", 1);
    
    // wait for initial position of the quad
    while (ros::ok() && pose.header.seq < 1000) {
        std::cout << "[PX4Agent.cpp]: Waiting for initial position." << std::endl;
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    // take off first at the current location
    takeoff(pose.pose.position.x, pose.pose.position.y, fixed_height);

    // start timer, operate under timer callbacks
    controlTimer = nh.createTimer(ros::Duration(0.1), &PX4Agent::controlTimerCB, this); // TODO: make the control freq changeable
}

PX4Agent::~PX4Agent() {
}

void PX4Agent::pathCB(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& traj) {
    desTraj = *traj;
}

void PX4Agent::poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // store the currect pose
    pose = *msg;
    // publish odom to planner
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link"; // TODO: change this
    odom.pose.pose.position.x = msg->pose.position.x;
    odom.pose.pose.position.y = msg->pose.position.y;
    odom.pose.pose.orientation.w = msg->pose.orientation.w;
    odom.pose.pose.orientation.x = msg->pose.orientation.x;
    odom.pose.pose.orientation.y = msg->pose.orientation.y;
    odom.pose.pose.orientation.z = msg->pose.orientation.z;
    
    odomPub.publish(odom);
}

float PX4Agent::calcVelCmd(Eigen::Vector3d& desVel, const Eigen::Vector3d& desPos, 
            const float vmax, const float kp) {
    // 3d velocity < vmax
    Eigen::Vector3d curPos(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    Eigen::Vector3d errPos = desPos-curPos;
    desVel = kp*errPos;
    if(desVel.norm() > vmax) {
        desVel = vmax * desVel / desVel.norm();
    }
    return errPos.norm();    
}

float PX4Agent::calcVelCmd2D(Eigen::Vector3d& desVel, const Eigen::Vector3d& desPos, 
            const float vmax, const float kp) {
    // 2d velocity < vmax
    // XY
    Eigen::Vector2d curPos(pose.pose.position.x, pose.pose.position.y);
    Eigen::Vector2d desPos2D(desPos(0), desPos(1));
    Eigen::Vector2d errPos = desPos2D-curPos;
    Eigen::Vector2d desVelXY = kp*errPos;
    if(desVelXY.norm() > vmax) {
        desVelXY = vmax * desVelXY / desVelXY.norm();
    }
    // Z
    float errZ = desPos(2)-pose.pose.position.z;
    float desVelZ = 1.5*errZ; // TODO: make it a parameter
    if(std::fabs(desVelZ) > 1.0) { // TODO: make it a parameter
        desVelZ = 1.0 * desVelZ / std::fabs(desVelZ);
    }
    desVel(0) = desVelXY(0);
    desVel(1) = desVelXY(1);
    desVel(2) = desVelZ;

    return errPos.norm();
}

void PX4Agent::takeoff(const float desx, const float desy, const float desz) {
    Eigen::Vector3d desPos(desx, desy, desz);
    Eigen::Vector3d curPos;
    Eigen::Vector3d desVel;
    float posErr = 1000;
    ros::Rate rate(10);
    geometry_msgs::Twist twist;
    std::cout << "Taking off..." << std::endl;
    while(ros::ok() && posErr>0.1 ) {
        posErr = calcVelCmd(desVel, desPos, 1.0, 4.0);
        twist.linear.x = desVel(0);
        twist.linear.y = desVel(1);
        twist.linear.z = desVel(2);
        px4SetVelPub.publish(twist);
        rate.sleep();
        ros::spinOnce();
    }
    std::cout << "Finished taking off. Hovering..." << std::endl;
    // reset vel to zero after the takeoff procedure
    for(int i=0; i<10; i++) {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        px4SetVelPub.publish(twist);
        rate.sleep();
        ros::spinOnce();
    }
}

void PX4Agent::controlTimerCB(const ros::TimerEvent& event) {
    geometry_msgs::Twist twist;
    if(0 == desTraj.points.size()) {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        px4SetVelPub.publish(twist);
    } else {
        // std::cout << "Traj #: " << desTraj.header.seq << std::endl; // print the seq # of traj
        Eigen::Vector3d desVel;
        Eigen::Vector3d desPos;
        desPos(0) = desTraj.points[1].transforms[0].translation.x;
        desPos(1) = desTraj.points[1].transforms[0].translation.y;
        desPos(2) = fixed_height;
        calcVelCmd2D(desVel, desPos, max_vel, 4.0);
        geometry_msgs::Twist twist;
        twist.linear.x = desVel(0);
        twist.linear.y = desVel(1);
        twist.linear.z = desVel(2);
        px4SetVelPub.publish(twist);
    }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "PX4_Agent");
  PX4Agent px4agent;
  std::cout << "PX4 agent initiated." << std::endl;
  ros::spin();
  return 0;
}