/**************************************************************************
  File Name    : px4_base_controller.cpp
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Oct 27, 2018.
  Description  : Base class for interfacing with px4
**************************************************************************/

#include<mslquad/px4_base_controller.h>
#include<cmath>

PX4BaseController::PX4BaseController() : 
        quadNS_("/"), 
        fixedHeight_(0), 
        maxVel_(0) {
    // retrieve parameters
    std::string strtmp;
    ros::param::get("~quad_ns", strtmp);
    quadNS_ += strtmp;
    quadNS_ += "/";

    ros::param::get("~fixed_height", fixedHeight_);
    ros::param::get("~max_vel", maxVel_);

    // pub and subs
    cmdTrajSub_ = nh_.subscribe(
        quadNS_+"command/trajectory", 
        1, &PX4BaseController::pathCB, this);
    px4PoseSub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        quadNS_+"mavros/local_position/pose", 
        1, &PX4BaseController::poseSubCB, this);
    px4SetVelPub_ = nh_.advertise<geometry_msgs::Twist>(
        quadNS_+"mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    odomPub_ = nh_.advertise<nav_msgs::Odometry>("ground_truth/odometry", 1);

    // wait for initial position of the quad
    while (ros::ok() && curPose_.header.seq < 1000) {
        std::cout << "[PX4BaseController.cpp]: Waiting for initial position." << std::endl;
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    // take off first at the current location
    takeoff(curPose_.pose.position.x, curPose_.pose.position.y, fixedHeight_);

    // start timer, operate under timer callbacks
    controlTimer_ = nh_.createTimer(
        ros::Duration(0.1), 
        &PX4BaseController::controlTimerCB, this); // TODO: make the control freq changeable
}


PX4BaseController::~PX4BaseController() {

}


void PX4BaseController::takeoff(const double desx, const double desy, const double desz) {
    Eigen::Vector3d desPos(desx, desy, desz);
    Eigen::Vector3d curPos;
    Eigen::Vector3d desVel;
    double posErr = 1000;
    ros::Rate rate(10);
    geometry_msgs::Twist twist;
    std::cout << "Taking off..." << std::endl;
    while(ros::ok() && posErr>0.1 ) {
        posErr = calcVelCmd(desVel, desPos, 1.0, 4.0);
        twist.linear.x = desVel(0);
        twist.linear.y = desVel(1);
        twist.linear.z = desVel(2);
        px4SetVelPub_.publish(twist);
        rate.sleep();
        ros::spinOnce();
    }
    std::cout << "Finished taking off. Hovering..." << std::endl;
    // reset vel to zero after the takeoff procedure
    for(int i=0; i<10; i++) {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        px4SetVelPub_.publish(twist);
        rate.sleep();
        ros::spinOnce();
    }
}

double PX4BaseController::calcVelCmd(
        Eigen::Vector3d& desVel, 
        const Eigen::Vector3d& desPos, 
        const double vmax, const double kp) {
    // 3d velocity < vmax
    Eigen::Vector3d curPos(curPose_.pose.position.x, curPose_.pose.position.y, curPose_.pose.position.z);
    Eigen::Vector3d errPos = desPos-curPos;
    desVel = kp*errPos;
    if(desVel.norm() > vmax) {
        desVel = vmax * desVel / desVel.norm();
    }
    return errPos.norm();    
}

double PX4BaseController::calcVelCmd2D(
        Eigen::Vector3d& desVel, 
        const Eigen::Vector3d& desPos, 
        const double vmax, const double kp) {
    // 2d velocity < vmax
    // XY
    Eigen::Vector2d curPos(curPose_.pose.position.x, curPose_.pose.position.y);
    Eigen::Vector2d desPos2D(desPos(0), desPos(1));
    Eigen::Vector2d errPos = desPos2D-curPos;
    Eigen::Vector2d desVelXY = kp*errPos;
    if(desVelXY.norm() > vmax) {
        desVelXY = vmax * desVelXY / desVelXY.norm();
    }
    // Z
    double errZ = desPos(2)-curPose_.pose.position.z;
    double desVelZ = 1.5*errZ; // TODO: make it a parameter
    if(std::fabs(desVelZ) > 1.0) { // TODO: make it a parameter
        desVelZ = 1.0 * desVelZ / std::fabs(desVelZ);
    }
    desVel(0) = desVelXY(0);
    desVel(1) = desVelXY(1);
    desVel(2) = desVelZ;

    return errPos.norm();
}

void PX4BaseController::pathCB(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& traj) {
    desTraj_ = *traj;
}

void PX4BaseController::poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // store the currect pose
    curPose_ = *msg;
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
    
    odomPub_.publish(odom);
}

void PX4BaseController::controlTimerCB(const ros::TimerEvent& event) {
    this->controlLoop();
}

void PX4BaseController::controlLoop(void) {
    // default control loop
    geometry_msgs::Twist twist;
    if(0 == desTraj_.points.size()) {
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        px4SetVelPub_.publish(twist);
    } else {
        // std::cout << "Traj #: " << desTraj.header.seq << std::endl; // print the seq # of traj
        Eigen::Vector3d desVel;
        Eigen::Vector3d desPos;
        desPos(0) = desTraj_.points[1].transforms[0].translation.x;
        desPos(1) = desTraj_.points[1].transforms[0].translation.y;
        desPos(2) = fixedHeight_;
        calcVelCmd2D(desVel, desPos, maxVel_, 4.0);
        geometry_msgs::Twist twist;
        twist.linear.x = desVel(0);
        twist.linear.y = desVel(1);
        twist.linear.z = desVel(2);
        px4SetVelPub_.publish(twist);
    }
}

double PX4BaseController::getYawRad(void) const {
    double q0 = curPose_.pose.orientation.w;
    double q1 = curPose_.pose.orientation.x;
    double q2 = curPose_.pose.orientation.y;
    double q3 = curPose_.pose.orientation.z;
    return atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}