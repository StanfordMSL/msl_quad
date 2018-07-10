/**************************************************************************
  File Name    : follower.cpp
  Author       : Kunal Shah
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Jul;. 09th, 2018.
  Descrption   : Waypoint follower for PX4
                 Tested with NOTHING 
                 **************************************************************************/
// ros 
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
// mavros 
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
//math
#include <Eigen/Dense>
#include <vector>
#include <cmath>
//io
#include <iomanip> // setw
#include <iostream>

using namespace std;

class PX4Agent{
private:
    mavros_msgs::State curState; //current state of the quad from px4
    geometry_msgs::PoseStamped curPose; // current pose of quad from px4
    geometry_msgs::PoseStamped initPose; // initial pose of the quad. Used as ref point for pos control test
    geometry_msgs::PoseStamped cmdPose; // position setpoint to px4

    ros::NodeHandle nh;
    ros::Subscriber px4PoseSub; // px4 pose sub
    ros::Subscriber px4StateSub; // px3 state sub 
    ros::Publisher px4SetPosPub;

    ros::Timer controlTimer;

    vector<vector<double>* > waypoints;
    bool autoland; // whether to land automatically after finishing all waypoints
    double takeoffHeight;
    double reachRadius; // radius to determine if a waypoint is reached

    void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg); // pose callback on PX4 local position
    void stateSubCB(const mavros_msgs::State::ConstPtr& msg); // state callbacj on PX4 state
    void controlTimerCB(const ros::TimerEvent& event);

    double dist(const vector<double>* p); // calculate dist to a point from current position of the quad

public:
    PX4Agent();
    ~PX4Agent();
};

PX4Agent::PX4Agent() : autoland(false), takeoffHeight(1.2), reachRadius(0.05) {
    //SUBCRIBERS
    //get pose 
    px4PoseSub = nh.subscribe<geometry_msgs::PoseStamped>(
            "mavros/local_position/pose", 1, &PX4Agent::poseSubCB, this);
    //get state 
    px4StateSub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &PX4Agent::stateSubCB, this);
    //SERVICE CALLS
    //arm 
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    //mode
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    //PUBLISHERS
    //send cmd pose
    px4SetPosPub = nh.advertise<geometry_msgs::PoseStamped>(
            "mavros/setpoint_position/local", 5);


    // wait for FCU connection
    while(ros::ok() && !curState.connected){
        ros::spinOnce();
        ros::Duration(.05).sleep();
    }

    // wait for the initial position of the quad
    while(ros::ok() && curPose.header.seq < 1000) {
        cout << "Waiting for local_position/pose..." << endl;
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    initPose = curPose;
    cout << "Navigation: Inital Position: " 
                << initPose.pose.position.x << ", "
                << initPose.pose.position.y << ", "
                << initPose.pose.position.z << endl;

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

    // inital hover pose
    cmdPose.pose.position.x = initPose.pose.position.x;
    cmdPose.pose.position.y = initPose.pose.position.y;
    cmdPose.pose.position.z = initPose.pose.position.z+takeoffHeight;

    //send a few setpoints before starting to make px4 behave
    // it needs poses queued else it won't go to off board mode
    for(int i = 100; ros::ok() && i > 0; --i){
        px4SetPosPub.publish(cmdPose);
        ros::spinOnce();
        ros::Duration(.05).sleep();
    }
    //make inital waypoint list
    cout << "Navigation: Building hover waypoint" << endl;
    vector<double>* p = new vector<double>;
    p->push_back(cmdPose.pose.position.x);
    p->push_back(cmdPose.pose.position.y);
    p->push_back(cmdPose.pose.position.z);
    waypoints.push_back(p);
    
    //run offboard mode and arm controller
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){ //switch to off board and arm 
        if( curState.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(1.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Flight: Offboard Enabled");
            }
            last_request = ros::Time::now();
        }   
        else {
            if( !curState.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Flight: Controller Armed");
                    break;
                }
                last_request = ros::Time::now();
            }
        }
    }
    ROS_INFO("Flight: Vehicle cleared for takeoff");
    // start timer, operate under timer callbacks
    controlTimer = nh.createTimer(ros::Duration(0.1), &PX4Agent::controlTimerCB, this); // TODO: make the control freq changeable
}//end initalization



PX4Agent::~PX4Agent() {
    cout << "PX4 agent destroyed" << endl;
}

void PX4Agent::poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // store the currect pose
    curPose = *msg;
}
void PX4Agent::stateSubCB(const mavros_msgs::State::ConstPtr& msg){
    curState = *msg;
}

double PX4Agent::dist(const vector<double>* p) {
    return sqrt(pow((*p)[0]-curPose.pose.position.x, 2) + 
                pow((*p)[1]-curPose.pose.position.y, 2) +
                pow((*p)[2]-curPose.pose.position.z, 2));
}

// void PX4Agent::UpdateWaypointsCB(const ma) { //TODO callback that turns the msg to the waypoints


// }

void PX4Agent::controlTimerCB(const ros::TimerEvent& event) {
    if(!waypoints.empty()) {
        // retrieve waypoint
        vector<double>* p = waypoints.front();
        // assemble command
        cmdPose.pose.position.x = (*p)[0];
        cmdPose.pose.position.y = (*p)[1];
        cmdPose.pose.position.z = (*p)[2];
        cmdPose.pose.orientation.x = initPose.pose.orientation.x;
        cmdPose.pose.orientation.y = initPose.pose.orientation.y;
        cmdPose.pose.orientation.z = initPose.pose.orientation.z;
        cmdPose.pose.orientation.w = initPose.pose.orientation.w;      
        // check if reached the waypoint
        if(dist(p)<reachRadius) {
            cout << "Reached waypoint: " << setprecision(3) <<
                        (*p)[0] << ", " << (*p)[1] << ", " << (*p)[2] << endl;
            waypoints.erase(waypoints.begin()); // if yes, remove the waypoint
        }
    } else { // reached all the waypoints \\\todo add safe land 
        if(autoland) {
            ROS_INFO("Navigation: Trajectory complete. Landing");
            if(curPose.pose.position.z > initPose.pose.position.z+.2){  //TODO ADD LANDING HEIGHT
                cmdPose.pose.position.z = cmdPose.pose.position.z - .1;
            }
            cmdPose.pose.orientation.x = initPose.pose.orientation.x;
            cmdPose.pose.orientation.y = initPose.pose.orientation.y;
            cmdPose.pose.orientation.z = initPose.pose.orientation.z;
            cmdPose.pose.orientation.w = initPose.pose.orientation.w; 
        }
        else{
            ROS_INFO("Navigation: Trajectory complete. Hovering");
        }
        // if not autoland, just keep publishing the previous commands             
    }

    cmdPose.header.stamp = ros::Time::now();
    px4SetPosPub.publish(cmdPose); // must keep publishing to maintain offboard state
}

int main(int argc, char **argv){
  ros::init(argc, argv, "PX4_Agent");
  PX4Agent px4agent;
  cout << "PX4 agent initiated." << endl;
  ros::spin();
  return 0;
}