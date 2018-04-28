/**************************************************************************
  File Name    : pos_ctrl_test.cpp
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Apr. 27th, 2018.
  Descrption   : Test sending position setpoint to Pixhawk
                 Tested with PX4 firmware version v1.7.3
**************************************************************************/
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iomanip> // setw
#include <iostream>

using namespace std;

class PX4Agent{
private:
    geometry_msgs::PoseStamped pose; // current pose of quad from px4
    geometry_msgs::PoseStamped initPose; // initial pose of the quad. Used as ref point for pos control test
    geometry_msgs::PoseStamped poseCmd; // position setpoint to px4

    ros::NodeHandle nh;
    ros::Subscriber px4PoseSub; // px4 pose sub
    ros::Publisher px4SetPosPub;

    ros::Timer controlTimer;

    vector<vector<double>* > waypoints;
    bool autoland; // whether to land automatically after finishing all waypoints
    double takeoffHeight;
    double reachRadius; // radius to determine if a waypoint is reached

    void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg); // pose callback on PX4 local position
    void controlTimerCB(const ros::TimerEvent& event);

    double dist(const vector<double>* p); // calculate dist to a point from current position of the quad

public:
    PX4Agent();
    ~PX4Agent();
};

PX4Agent::PX4Agent() : autoland(true), takeoffHeight(1.2), reachRadius(0.05) {
    px4PoseSub = nh.subscribe<geometry_msgs::PoseStamped>(
            "mavros/local_position/pose", 1, &PX4Agent::poseSubCB, this);
    px4SetPosPub = nh.advertise<geometry_msgs::PoseStamped>(
            "mavros/setpoint_position/local", 1);

    // wait for the initial position of the quad
    while(ros::ok() && pose.header.seq < 1000) {
        cout << "Waiting for local_position/pose..." << endl;
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    initPose = pose;
    cout << "Quad init pos: " << pose.pose.position.x << ", "
                << pose.pose.position.y << ", "
                << pose.pose.position.z << endl;

    // retrieve parameters
    if(ros::param::has("~autoland")) {
        ros::param::get("~autoland", autoland);
    }
    if(ros::param::has("~takeoff_height")) {
        ros::param::get("~takeoff_height", takeoffHeight);
    }
    if(ros::param::has("~reach_radius")) {
        ros::param::get("~reach_radius", reachRadius);
    }

    // Define waypoints. Make sure waypoints are relative to initial position
    if(ros::param::has("pos_ctrl_test/waypoints")) {
        cout << "Found waypoints specified in ros parameter server!" << endl;
        std::vector<double> wptVec;
        ros::param::get("pos_ctrl_test/waypoints", wptVec); // format: x1,y1,z1, x2,y2,z2, ...
        if(wptVec.size() % 3 != 0) {
            ROS_ERROR("Invalid waypoints!");
            ros::shutdown();
            return;
        }
        // first waypoint is above the original position
        vector<double>* p = new vector<double>;
        p->push_back(initPose.pose.position.x);
        p->push_back(initPose.pose.position.y);
        p->push_back(initPose.pose.position.z + takeoffHeight);
        waypoints.push_back(p);        
        // other follow-on waypoints
        for(int i=0; i<wptVec.size()/3; i++) {
            vector<double>* p = new vector<double>;
            p->push_back(initPose.pose.position.x + wptVec[3*i]);
            p->push_back(initPose.pose.position.y + wptVec[3*i+1]);
            p->push_back(initPose.pose.position.z + wptVec[3*i+2]);
            waypoints.push_back(p);
        }
    } else {
        cout << "No waypoints specified. Will only hover!" << endl;
        vector<double>* p = new vector<double>;
        p->push_back(initPose.pose.position.x);
        p->push_back(initPose.pose.position.y);
        p->push_back(initPose.pose.position.z + takeoffHeight);
        waypoints.push_back(p);
    }
    // print waypoints
    cout << "Waypoints: " << endl;
    for(auto it = waypoints.begin(); it != waypoints.end(); it++) {
        for(auto j=(*it)->begin(); j!=(*it)->end(); j++) {
            cout << setprecision(3) << *j << ", ";
        }
        cout << endl;
    }

    // start timer, operate under timer callbacks
    controlTimer = nh.createTimer(ros::Duration(0.1), &PX4Agent::controlTimerCB, this); // TODO: make the control freq changeable
}

PX4Agent::~PX4Agent() {
    cout << "PX4 agent destroyed" << endl;
}

void PX4Agent::poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // store the currect pose
    pose = *msg;
}

double PX4Agent::dist(const vector<double>* p) {
    return sqrt(pow((*p)[0]-pose.pose.position.x, 2) + 
                pow((*p)[1]-pose.pose.position.y, 2) +
                pow((*p)[2]-pose.pose.position.z, 2));
}

void PX4Agent::controlTimerCB(const ros::TimerEvent& event) {
    if(!waypoints.empty()) {
        // retrieve waypoint
        vector<double>* p = waypoints.front();
        // assemble command
        poseCmd.pose.position.x = (*p)[0];
        poseCmd.pose.position.y = (*p)[1];
        poseCmd.pose.position.z = (*p)[2];
        poseCmd.pose.orientation.x = initPose.pose.orientation.x;
        poseCmd.pose.orientation.y = initPose.pose.orientation.y;
        poseCmd.pose.orientation.z = initPose.pose.orientation.z;
        poseCmd.pose.orientation.w = initPose.pose.orientation.w;      
        // check if reached the waypoint
        if(dist(p)<reachRadius) {
            cout << "Reached waypoint: " << setprecision(3) <<
                        (*p)[0] << ", " << (*p)[1] << ", " << (*p)[2] << endl;
            waypoints.erase(waypoints.begin()); // if yes, remove the waypoint
        }
    } else { // reached all the waypoints
        if(autoland) {
            poseCmd.pose.position.x = initPose.pose.position.x;
            poseCmd.pose.position.y = initPose.pose.position.y;
            poseCmd.pose.position.z = initPose.pose.position.z;
            poseCmd.pose.orientation.x = initPose.pose.orientation.x;
            poseCmd.pose.orientation.y = initPose.pose.orientation.y;
            poseCmd.pose.orientation.z = initPose.pose.orientation.z;
            poseCmd.pose.orientation.w = initPose.pose.orientation.w;
        }
        // if not autoland, just keep publishing the previous commands             
    }

    poseCmd.header.stamp = ros::Time::now();
    px4SetPosPub.publish(poseCmd); // must keep publishing to maintain offboard state
}

int main(int argc, char **argv){
  ros::init(argc, argv, "PX4_Agent");
  PX4Agent px4agent;
  cout << "PX4 agent initiated." << endl;
  ros::spin();
  return 0;
}