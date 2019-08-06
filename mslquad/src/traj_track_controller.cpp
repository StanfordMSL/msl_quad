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
TrajTrackController::TrajTrackController() {
    // retrieve ROS parameter
    ros::V_string params;
    std::string namesp;
    namesp = nh_.getNamespace();
    std::cout << namesp << std::endl;
    nh_.getParamNames(params);
    for (auto & element : params) {
        std::cout << element << std::endl;
    }
    // load traj from topic or file
    ros::param::get("~load_traj_file", loadTrajFile);
    std::cout << loadTrajFile << std::endl;

    if (loadTrajFile) {
        ROS_INFO_STREAM("Loading Trajectory File");
        std::string trajFile;
        ros::param::get("~traj_file", trajFile);
        ROS_INFO("Got param: %s", trajFile.c_str());
        parseTrajFile(&trajFile);
    } else {
        cmdTrajSub_.shutdown();  // close other trajectory topics
        std::string trajTargetTopic;
        ros::param::get("~traj_target_topic", trajTargetTopic);
        ROS_INFO_STREAM("Subscribing to: "<< trajTargetTopic);
        trajTargetSub_ =
            nh_.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            trajTargetTopic, 1, &TrajTrackController::trajTargetCB, this);
    }
    // ROS subs and pub
    scambleSub_ = nh_.subscribe<std_msgs::Bool>(
        "/tower/scramble", 1, &TrajTrackController::scrambleSubCB, this);
}

TrajTrackController::~TrajTrackController() {
}

void TrajTrackController::trajTargetCB(
        const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg) {
    traj = *msg;
    ROS_INFO_STREAM("Trajectory Received");
}
void TrajTrackController::scrambleSubCB(
        const std_msgs::Bool::ConstPtr& msg) {
    scramble = msg-> data;
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
        f >> px >> py >> pz >> vx >> vy >> vz;
        printf("%2.4f, %2.4f, %2.4f: %2.4f, %2.4f, %2.4f", px, py, pz,
                                                           vx, vy, vz);
        // fill traj
        geometry_msgs::Transform tf;
        geometry_msgs::Twist tw;
        // pos
        tf.translation.x = px;
        tf.translation.y = py;
        tf.translation.z = pz;
        // vel
        tw.linear.x = vx;
        tw.linear.y = vy;
        tw.linear.z = vz;
        // push back
        traj.transforms.push_back(tf);
        traj.velocities.push_back(tw);
    }
    // I don't think we can leave this blank
    traj.time_from_start = ros::Duration(1);
}
void TrajTrackController::takeoff(const double desx,
                                  const double desy,
                                  const double desz) {
    Eigen::Vector3d desPos(desx, desy, desz);
    Eigen::Vector3d curPos;
    Eigen::Vector3d desVel;
    double posErr = 1000;
    ros::Rate rate(10);  // change to meet the control loop?
    geometry_msgs::Twist twist;
    std::cout << "Takeoff" << std::endl;
    while (ros::ok() && posErr >0.1) {
        // take off to starting position
        posErr = calcVelCmd(desVel, desPos, maxVel_, 4.0);
        twist.linear.x = desVel(0);
        twist.linear.y = desVel(1);
        twist.linear.z = desVel(2);
        px4SetVelPub_.publish(twist);
        rate.sleep();
        ros::spinOnce();
    }
    ROS_INFO_STREAM("Takeoff Complete");
}
void TrajTrackController::controlLoop(void) {
    // follow the trajectory
    ROS_INFO_STREAM("Standby: Hovering");
    while (!scramble) {
        geometry_msgs::PoseStamped hoverPose = takeoffPose_;
        hoverPose.pose.position.z = takeoffHeight_;
        px4SetPosPub_.publish(hoverPose);
    }
    ROS_INFO_STREAM("Executing Command");
    ROS_ERROR("HARD LAND BOYES");
    emergencyLandPose_ = curPose_.pose;
    emergencyLandPose_.position.z = takeoffPose_.pose.position.z;
    state_ = State::EMERGENCY_LAND;
    //     // print the seq # of traj
    //     // std::cout << "Traj #: " << desTraj.header.seq << std::endl;
    //     Eigen::Vector3d desVel;
    //     Eigen::Vector3d desPos;
    //     desPos(0) = desTraj_.points[1].transforms[0].translation.x;
    //     desPos(1) = desTraj_.points[1].transforms[0].translation.y;
    //     if (flagOnly2D_) {
    //         desPos(2) = takeoffHeight_;
    //         calcVelCmd2D(desVel, desPos, maxVel_, 4.0);
    //     } else {
    //         desPos(2) = desTraj_.points[1].transforms[0].translation.z;
    //         calcVelCmd(desVel, desPos, maxVel_, 4.0);
    //     }
    //     geometry_msgs::Twist twist;
    //     twist.linear.x = desVel(0);
    //     twist.linear.y = desVel(1);
    //     twist.linear.z = desVel(2);
    //     px4SetVelPub_.publish(twist);
}
