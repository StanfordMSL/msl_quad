/* copyright[2019] <msl>
**************************************************************************
  File Name    : pose_track_controller.cpp
  Author       : Kunal Shah
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Dec 3, 2018.
  Description  : Basic control + pose tracking
**************************************************************************/

#include<mslquad/traj_track_controller.h>
#include<iostream>
#include <fstream>
#include <algorithm>

TrajTrackController::TrajTrackController() {
    // retrieve ROS parameter
    ros::param::get("~traj_timestep", trajTimeStep);
    ros::param::get("~traj_Kp", trajKp);
    // load traj from topic
    cmdTrajSub_.shutdown();  // close other trajectory topics
    std::string trajTargetTopic;
    ros::param::get("~traj_target_topic", trajTargetTopic);
    ROS_INFO_STREAM("Subscribing to: "<< trajTargetTopic);
    trajTargetSub_ =
                nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
                     trajTargetTopic, 1,
                     &TrajTrackController::trajTargetCB, this);
    // check for traj file
    ros::param::get("~load_traj_file", loadTrajFile);
    if (loadTrajFile) {
        ROS_INFO_STREAM("Loading Trajectory File");
        std::string trajFile;
        ros::param::get("~traj_file", trajFile);
        parseTrajFile(&trajFile);
    }
    // set desired position to 1st waypoint
    while (!updateTarget()) {
        ROS_INFO_STREAM("Awaiting Trajectory");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    // ROS subs and pub
    scambleSub_ = nh_.subscribe<std_msgs::Bool>(
                  "/tower/scramble", 1,
                   &TrajTrackController::scrambleSubCB, this);
}

TrajTrackController::~TrajTrackController() {
}

void TrajTrackController::trajTargetCB(
        const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg) {
    traj = *msg;
    ROS_INFO_STREAM("Trajectory Received");
    trajIdx = 0;  // reset traj index counter
}
void TrajTrackController::scrambleSubCB(
        const std_msgs::Bool::ConstPtr& msg) {
    std::cout << "got data" << msg->data << std::endl;
    scramble = true;
    ROS_INFO_STREAM("Executing Command");
    trajStartTime = ros::Time::now();
}
void TrajTrackController::parseTrajFile(std::string* trajFilePtr) {
        std::ifstream f;
    std::cout << "Opening: " << *trajFilePtr << std::endl;
    f.open(*trajFilePtr);
    if (!f) {
        std::cerr <<
        "ERROR: Unable to open traj file: "<< *trajFilePtr <<std::endl;
    }
    if (f.is_open()) {
        // init vars to parse with
        float px, py, pz;
        float vx, vy, vz;
        // parse the ines
        while (f >> px >> py >> pz >> vx >> vy >> vz) {
            printf("%2.4f, %2.4f, %2.4f: %2.4f, %2.4f, %2.4f\n",
                    px, py, pz, vx, vy, vz);
            // fill traj
            geometry_msgs::Transform tf;
            geometry_msgs::Twist tw;
            // pos
            tf.translation.x = px + takeoffPose_.pose.position.x;
            tf.translation.y = py + takeoffPose_.pose.position.y;
            tf.translation.z = pz + takeoffPose_.pose.position.z;
            // vel
            tw.linear.x = vx;
            tw.linear.y = vy;
            tw.linear.z = vz;
            // push back
            traj.transforms.push_back(tf);
            traj.velocities.push_back(tw);
        }
    }
    // save number of points
    trajNPoints = traj.transforms.size();
    // I don't think we can leave this blank
    traj.time_from_start = ros::Duration(1);
}
bool TrajTrackController::updateTarget(void) {
    // calculate aboslute target pos from relative value in trajectory
    if (traj.transforms.size() > 0 &&
        trajIdx < traj.transforms.size()) {
        desPos(0) = traj.transforms[trajIdx].translation.x;
        desPos(1) = traj.transforms[trajIdx].translation.y;
        desPos(2) = traj.transforms[trajIdx].translation.z;
        ROS_INFO_STREAM("Moving to next waypoint");
        std::cout << desPos << std::endl;
        return true;
    }
    return false;  // return false if there's no trajectory
}
void TrajTrackController::takeoff(void) {
    Eigen::Vector3d desPos(takeoffPose_.pose.position.x,
                           takeoffPose_.pose.position.y,
                           takeoffHeight_);
    Eigen::Vector3d curPos;
    Eigen::Vector3d desVel;
    double posErr = 1000;
    ros::Rate rate(10);  // change to meet the control loop?
    geometry_msgs::Twist twist;
    std::cout << "Launch" << std::endl;
    while (ros::ok() && posErr >0.1) {
        // take off to starting position
        posErr = calcVelCmd(desVel, desPos, maxVel_, trajKp);
        twist.linear.x = desVel(0);
        twist.linear.y = desVel(1);
        twist.linear.z = desVel(2);
        px4SetVelPub_.publish(twist);
        rate.sleep();
        ros::spinOnce();
    }
    // set hover pose
    geometry_msgs::PoseStamped hoverPose = takeoffPose_;
        hoverPose.pose.position.z = takeoffHeight_;
    ROS_INFO_STREAM("Standby");
}
void TrajTrackController::slowLoop(void) {
    // do nothing
}
void TrajTrackController::controlLoop(void) {
    // follow the trajectory
    if (!scramble) {
        px4SetPosPub_.publish(hoverPose_);
        ros::spinOnce();
        return;
    }
    float timeIdx;
    timeIdx = ((ros::Time::now() - trajStartTime).toSec())/trajTimeStep;
    if (static_cast<int>(timeIdx) > trajIdx) {  // proceed to next
        trajIdx++;
        updateTarget();
    }
    // check if we are at the end
    if (trajIdx == traj.transforms.size()) {
        ROS_INFO_STREAM("Trajectory Complete");
        // reset to a hover
        hoverPose_.pose.position.x = desPos(0);
        hoverPose_.pose.position.y = desPos(1);
        hoverPose_.pose.position.z = desPos(2);
        scramble = false;
        ROS_INFO_STREAM("Standby");
    } else {
        // p control on velocity to target
        calcVelCmd(desVel, desPos, maxVel_, trajKp);
        // velocity to publish
        geometry_msgs::Twist twist;
        twist.linear.x = desVel(0);
        twist.linear.y = desVel(1);
        twist.linear.z = desVel(2);
        px4SetVelPub_.publish(twist);
    }
}
