/**************************************************************************
  File Name    : vel_ctrl_test.cpp
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Apr. 27th, 2018.
  Descrption   : Test sending velocity setpoint to Pixhawk
**************************************************************************/
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iomanip>
#include <iostream>

#define RAD_2_DEGREE (180.0/3.1415926)

using namespace std;

class PX4Agent{
private:
    geometry_msgs::PoseStamped pose; // current pose of quad from px4
    geometry_msgs::PoseStamped initPose; // initial pose of the quad. 
                                         // Can be used as ref point for waypoint test
    geometry_msgs::Twist twistCmd; // velocity command sent to px4, NOT stamped

    ros::NodeHandle nh;
    ros::Subscriber px4PoseSub; // px4 pose sub
    ros::Publisher px4SetVelPub;

    ros::Timer controlTimer;

    vector<vector<float>* > waypoints;
    float initYaw; // initial yaw angle before takeoff

    // parameters from launch file
    bool autoland; // whether to land automatically after finishing all waypoints
    float takeoffHeight;
    float reachRadius; // radius to determine if a waypoint is reached
    float ctrlFreq; // control frequency
    float gainKpLinVel; // proportional gain on linear velocity
    float gainKpAngVel; // proportional gain on angular velocity
    float yawMaintain; // desired yaw angle during the test, unit: rad
    float maxVel; // max linear velocity, unit: m/s
    bool isWptRelative; // is waypoint relative to takeoff place or absolute coordinate?

    // ROS callback functions
    void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg); // pose callback on PX4 local position
    void controlTimerCB(const ros::TimerEvent& event);

    // calculate desired linear/angular velocity given desired position and yaw angle
    // subject to max velocity limit
    float calcVelCmd(Eigen::Vector3d& desLinVel, float& desAngVelZ, 
                    const Eigen::Vector3d& desPos, const float desYaw,
                    const float vmax, const float kpLinVel, const float kpAngVel);
    // calculate dist to a point from current position of the quad
    float dist(const vector<float>* p); 
    // compute yaw angle from quaternion
    float quat2yaw(float qw, float qx, float qy, float qz);

public:
    PX4Agent();
    ~PX4Agent();
};

PX4Agent::PX4Agent() : 
        autoland(true), 
        takeoffHeight(1.2), 
        reachRadius(0.1), 
        ctrlFreq(50),
        gainKpLinVel(1.5), 
        yawMaintain(0.0), 
        maxVel(0.5), 
        isWptRelative(false) {
    // set up subscriber/publisher
    px4PoseSub = nh.subscribe<geometry_msgs::PoseStamped>(
            "mavros/local_position/pose", 1, &PX4Agent::poseSubCB, this);
    px4SetVelPub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 1);

    // wait for the initial position of the quad
    while(ros::ok() && pose.header.seq < 1000) {
        cout << "Waiting for local_position/pose..." << endl;
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    initPose = pose;
    cout << endl;
    cout << "Quad init pos: " << pose.pose.position.x << ", "
                << pose.pose.position.y << ", "
                << pose.pose.position.z << endl;
    initYaw = quat2yaw(
        initPose.pose.orientation.w,
        initPose.pose.orientation.x,
        initPose.pose.orientation.y,
        initPose.pose.orientation.z);
    cout << "Quad init yaw angle: " << initYaw * RAD_2_DEGREE << " degree" << endl;

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
    if(ros::param::has("~ctrl_freq")) {
        ros::param::get("~ctrl_freq", ctrlFreq);
    }
    if(ros::param::has("~reach_rad")) {
        ros::param::get("~reach_rad", reachRadius);
    }
    if(ros::param::has("~gain_kp_lin_vel")) {
        ros::param::get("~gain_kp_lin_vel", gainKpLinVel);
    }
    if(ros::param::has("~gain_kp_ang_vel")) {
        ros::param::get("~gain_kp_ang_vel", gainKpAngVel);
    }
    if(ros::param::has("~yaw_maintain")) {
        ros::param::get("~yaw_maintain", yawMaintain);
    }
    if(ros::param::has("~max_vel")) {
        ros::param::get("~max_vel", maxVel);
    }
    if(ros::param::has("~waypoint_relative")) {
        ros::param::get("~waypoint_relative", isWptRelative);
    }

    // print some parameters
    cout << endl;
    cout << "Control frequency: " << ctrlFreq << "Hz" << endl;
    cout << "Reach radius: " << reachRadius << "m" << endl;
    cout << "Relative waypoint: " << isWptRelative << endl;

    // Define waypoints. Make sure waypoints are relative to initial position
    cout << endl;
    if(ros::param::has("vel_ctrl_test/waypoints")) {
        cout << "Found waypoints specified in ros parameter server!" << endl;
        std::vector<float> wptVec;
        ros::param::get("vel_ctrl_test/waypoints", wptVec); // format: x1,y1,z1, x2,y2,z2, ...
        if(wptVec.size() % 3 != 0) {
            ROS_ERROR("Invalid waypoints!");
            ros::shutdown();
            return;
        }
        // first waypoint is above the original position
        vector<float>* p = new vector<float>;
        p->push_back(initPose.pose.position.x);
        p->push_back(initPose.pose.position.y);
        p->push_back(initPose.pose.position.z + takeoffHeight);
        waypoints.push_back(p);        
        // other follow-on waypoints
        for(int i=0; i<wptVec.size()/3; i++) {
            vector<float>* p = new vector<float>;
            if(isWptRelative) {
                p->push_back(initPose.pose.position.x + wptVec[3*i]);
                p->push_back(initPose.pose.position.y + wptVec[3*i+1]);
                p->push_back(initPose.pose.position.z + wptVec[3*i+2]);
            } else {
                p->push_back(wptVec[3*i]);
                p->push_back(wptVec[3*i+1]);
                p->push_back(wptVec[3*i+2]);                
            }
            waypoints.push_back(p);
        }
        // last waypoint is aright above the original position
        p = new vector<float>;
        p->push_back(initPose.pose.position.x);
        p->push_back(initPose.pose.position.y);
        p->push_back(initPose.pose.position.z + takeoffHeight);
        waypoints.push_back(p);          
    } else {
        cout << "No waypoints specified. Will only hover!" << endl;
        vector<float>* p = new vector<float>;
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
    controlTimer = nh.createTimer(ros::Duration(0.1), &PX4Agent::controlTimerCB, this);
}

PX4Agent::~PX4Agent() {
    cout << "PX4 agent destroyed" << endl;
}

void PX4Agent::poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // store the currect pose
    pose = *msg;
}

float PX4Agent::dist(const vector<float>* p) {
    return sqrt(pow((*p)[0]-pose.pose.position.x, 2) + 
                pow((*p)[1]-pose.pose.position.y, 2) +
                pow((*p)[2]-pose.pose.position.z, 2));
}

float PX4Agent::quat2yaw(float qw, float qx, float qy, float qz) {
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion
    return atan2( 2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz) );
}

float PX4Agent::calcVelCmd(Eigen::Vector3d& desLinVel, float& desAngVelZ,
                        const Eigen::Vector3d& desPos, const float desYaw,
                        const float vmax, const float kpLinVel, const float kpAngVel) {
    // Input:
    //   desYaw: in rad

    // compute linear velocity
    Eigen::Vector3d curPos(
                pose.pose.position.x, 
                pose.pose.position.y, 
                pose.pose.position.z);
    Eigen::Vector3d errPos = desPos-curPos;
    desLinVel = kpLinVel * errPos;
    if(desLinVel.norm() > vmax) {
        desLinVel = vmax * desLinVel / desLinVel.norm();
    }
    // compute angular velocity
    // extract yaw angle from quaternion. px4 only receives yaw angular vel
    // roll pitch are not independently controllable
    float yaw = quat2yaw(
        pose.pose.orientation.w,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z);
    float errYaw = desYaw - yaw;
    desAngVelZ = kpAngVel * errYaw;
    // return distance to desired position
    return errPos.norm();
}

// main timer callback function running in real time
void PX4Agent::controlTimerCB(const ros::TimerEvent& event) {
    if(!waypoints.empty()) {
        // retrieve waypoint
        vector<float>* p = waypoints.front();

        // compute velocity commands
        Eigen::Vector3d desLinVel;
        Eigen::Vector3d desPos;
        float desAngVelZ;
        desPos(0) = (*p)[0];
        desPos(1) = (*p)[1];
        desPos(2) = (*p)[2];
        calcVelCmd(desLinVel, desAngVelZ,
                desPos, yawMaintain, maxVel, 
                gainKpLinVel, gainKpAngVel);
        twistCmd.linear.x = desLinVel(0);
        twistCmd.linear.y = desLinVel(1);
        twistCmd.linear.z = desLinVel(2);
        twistCmd.angular.z = desAngVelZ;
        
        // check if reached the waypoint
        if(dist(p)<reachRadius) {
            cout << "Reached waypoint: " << setprecision(3) <<
                        (*p)[0] << ", " << (*p)[1] << ", " << (*p)[2] << endl;
            waypoints.erase(waypoints.begin()); // if yes, remove the waypoint
        }
    } else { // reached all the waypoints
        if(autoland) {
            Eigen::Vector3d desLinVel;
            Eigen::Vector3d desPos;
            float desAngVelZ;
            desPos(0) = initPose.pose.position.x;
            desPos(1) = initPose.pose.position.y;
            desPos(2) = initPose.pose.position.z + 0.1; // reserve some clearance from ground
            calcVelCmd(desLinVel, desAngVelZ, 
                    desPos, initYaw, maxVel, 
                    gainKpLinVel, gainKpAngVel);
            twistCmd.linear.x = desLinVel(0);
            twistCmd.linear.y = desLinVel(1);
            twistCmd.linear.z = desLinVel(2);
            twistCmd.angular.z = desAngVelZ;
        }
        // if not autoland, just keep publishing the previous commands             
    }

    px4SetVelPub.publish(twistCmd);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "PX4_Agent");
  PX4Agent px4agent;
  cout << "PX4 agent initiated." << endl;
  ros::spin();
  return 0;
}