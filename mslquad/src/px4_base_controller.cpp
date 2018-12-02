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
        maxVel_(0),
        controlLoopFreq_(20),
        slowLoopFreq_(10),
        state_(State::AUTO),
        eMode_(EmergencyMode::DISABLE) {
    // retrieve parameters
    std::string strtmp;
    ros::param::get("~quad_ns", strtmp);
    if(strtmp.size()!=0) {
        quadNS_ += strtmp;
        quadNS_ += "/";
    }
    ros::param::get("~fixed_height", fixedHeight_);
    ros::param::get("~max_vel", maxVel_);
    ros::param::get("~control_freq", controlLoopFreq_);
    ros::param::get("~slow_freq", slowLoopFreq_);

    //initial time keeping
    poseTime_= ros::Time::now();

    // pub and subs
    // mavros related, namespace under quadNS_
    px4PoseSub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        quadNS_+"mavros/local_position/pose", 
        1, &PX4BaseController::poseSubCB, this);
    px4VelSub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
        quadNS_+"mavros/local_position/velocity",
        1, &PX4BaseController::velSubCB, this);
    visionPoseSub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        quadNS_+"mavros/vision_pose/pose",
        1, &PX4BaseController::visionPoseSubCB, this);
    px4SetVelPub_ = nh_.advertise<geometry_msgs::Twist>(
        quadNS_+"mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    px4SetPosPub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        quadNS_+"mavros/setpoint_position/local", 1);
    actuatorPub_ = nh_.advertise<mavros_msgs::ActuatorControl>(
        quadNS_+"mavros/actuator_control", 1);
    emergencySrv_ = nh_.advertiseService(
        quadNS_+"emergency", 
        &PX4BaseController::emergencyHandle, this);
    // non-mavros related, namespace depending on the group ns in the launch file
    odomPub_ = nh_.advertise<nav_msgs::Odometry>("ground_truth/odometry", 1);
    cmdTrajSub_ = nh_.subscribe(
        "command/trajectory", 
        1, &PX4BaseController::pathCB, this);    

    // wait for mocap pose
    while (ros::ok() && curVisionPose_.header.seq < 200) {
        std::cout << quadNS_ << ": waiting for mocap pose." << std::endl;
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    // wait for initial position of the quad
    while (ros::ok() && curPose_.header.seq < 1000) {
        std::cout << quadNS_ << ": waiting for initial pose." << std::endl;
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    // check if vision_pose and local_position are consistent, for safety
    bool mocapCheck = false;
    while(ros::ok() && !mocapCheck) {
        mocapCheck = true;
        for(int i=0; i<10; ++i) { // must be consistent for 10 checks
            if(getDist(curPose_, curVisionPose_) > 0.08) {
                mocapCheck = false;
                std::cout << quadNS_ << ": mocap inconsistent." << std::endl;
            }
            ros::Duration(0.2).sleep();
        }
    }
    //visionPoseSub_.shutdown(); // shutdown subscriber after sanity check

    // start slow timer
    slowTimer_ = nh_.createTimer(
        ros::Duration(1.0/slowLoopFreq_),
        &PX4BaseController::slowTimerCB, this);

    takeoffPose_ = curPose_; // record takeoff postion before takeoff
    // take off first at the current location
    bool isAutoTakeOff = false;
    ros::param::get("~auto_takeoff", isAutoTakeOff);
    if(isAutoTakeOff)
        takeoff(curPose_.pose.position.x, curPose_.pose.position.y, fixedHeight_);

    // start faster timer for main control loop
    controlTimer_ = nh_.createTimer(
        ros::Duration(1.0/controlLoopFreq_), 
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

double PX4BaseController::calcVelCmd(Eigen::Vector3d& desVel, 
        const Eigen::Vector3d& desPos, 
        const double vmax, const double kp) const {
    // 3d velocity < vmax
    Eigen::Vector3d curPos(
        curPose_.pose.position.x, curPose_.pose.position.y, curPose_.pose.position.z);
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
        const double vmax, const double kp) const {
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


//topic callbacks
void PX4BaseController::pathCB(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& traj) {
    desTraj_ = *traj;
}

void PX4BaseController::poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // store the currect pose
    poseTimeDiff_ = (*msg).header.stamp - poseTime_;
    poseTime_ = (*msg).header.stamp;
    curPose_ = *msg;
}

void PX4BaseController::velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    curVel_ = *msg;
}

void PX4BaseController::visionPoseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    curVisionPose_ = *msg;
}


//timer callbacks
void PX4BaseController::controlTimerCB(const ros::TimerEvent& event) {
    if(state_ == State::EMERGENCY) {
        this->emergencyOverride();
    } else {         
        this->controlLoop();
    }
}

void PX4BaseController::slowTimerCB(const ros::TimerEvent& event) {
    this->emergencyLoop();
    this->slowLoop();
}


//control loops
void PX4BaseController::controlLoop(void) {
    // default control loop
    if(0 == desTraj_.points.size()) {
        geometry_msgs::PoseStamped hoverPose = takeoffPose_;
        hoverPose.pose.position.z = fixedHeight_;
        px4SetPosPub_.publish(hoverPose);
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

void PX4BaseController::slowLoop(void) {
    // publish odom to planner
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link"; // TODO: change this
    odom.pose.pose.position.x = curPose_.pose.position.x;
    odom.pose.pose.position.y = curPose_.pose.position.y;
    odom.pose.pose.position.z = curPose_.pose.position.z;
    odom.pose.pose.orientation.w = curPose_.pose.orientation.w;
    odom.pose.pose.orientation.x = curPose_.pose.orientation.x;
    odom.pose.pose.orientation.y = curPose_.pose.orientation.y;
    odom.pose.pose.orientation.z = curPose_.pose.orientation.z;
    odomPub_.publish(odom);
}

void PX4BaseController::emergencyLoop(void) {

    //pose time delay check
    if(poseTimeDiff_.toSec() > 0.5) {
        ROS_ERROR("pose delay critical");
        std::cout <<"BEEP BEEP BEEP" <<std::endl; //beeps for Zijian
    }else if(poseTimeDiff_.toSec() > 0.2) {
        ROS_WARN("pose delay");
    }

    //pose consistency check
    if(getDist(curPose_, curVisionPose_) > 0.2) {
        ROS_WARN("pose inconsistent");
    }
}

void PX4BaseController::emergencyOverride(void){
    ROS_WARN("Emergency Override Enabled");
    try{
        if (eMode_ == EmergencyMode::POSE){
            geometry_msgs::PoseStamped ps;
            ps.header.stamp = ros::Time::now();
            ps.pose = emergencyPose_;
            px4SetPosPub_.publish(ps);
        }else if(eMode_ == EmergencyMode::VEL){
            geometry_msgs::Twist tw;
            if((ros::Time::now()- eTime_).toSec() < 0.5){ //within half a second{
                tw = emergencyVel_;
            }else {
                tw.linear.x=0;
                tw.linear.y=0;
                tw.linear.z=0;
            }
            px4SetVelPub_.publish(tw);
        }
    }catch (...){
        ROS_INFO("Exception occured during emergency service call: Holding Position ");
        geometry_msgs::Twist tw;
        //hard stop
        tw.linear.x=0;
        tw.linear.y=0;
        tw.linear.z=0;
        px4SetVelPub_.publish(tw);
    }
}


//helper functions
double PX4BaseController::getYawRad(void) const {
    double q0 = curPose_.pose.orientation.w;
    double q1 = curPose_.pose.orientation.x;
    double q2 = curPose_.pose.orientation.y;
    double q3 = curPose_.pose.orientation.z;
    return atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}

Eigen::Matrix3d PX4BaseController::getRotMat(void) const {
    double q1 = curPose_.pose.orientation.w;
    double q2 = curPose_.pose.orientation.x;
    double q3 = curPose_.pose.orientation.y;
    double q4 = curPose_.pose.orientation.z;
    double q1s = q1*q1;
    double q2s = q2*q2;
    double q3s = q3*q3;
    double q4s = q4*q4;
    Eigen::Matrix3d R;
    R <<
        q1s+q2s-q3s-q4s, 2.0*(q2*q3-q1*q4) ,2.0*(q2*q4+q1*q3),
        2.0*(q2*q3+q1*q4), q1s-q2s+q3s-q4s, 2.0*(q3*q4-q1*q2),
        2.0*(q2*q4-q1*q3), 2.0*(q3*q4+q1*q2), q1s-q2s-q3s+q4s;
    return R;
}

double PX4BaseController::getDist(
        const geometry_msgs::PoseStamped &ps1,
        const geometry_msgs::PoseStamped &ps2) {
    return sqrt(
        pow(ps1.pose.position.x-ps2.pose.position.x, 2) +
        pow(ps1.pose.position.y-ps2.pose.position.y, 2) +
        pow(ps1.pose.position.z-ps2.pose.position.z, 2));
}


//service handlers
bool PX4BaseController::emergencyHandle(
        mslquad::Emergency::Request &req,
        mslquad::Emergency::Response &res) {
    //handles the emergency service call
    state_ = State::EMERGENCY; //state of emergency
    eTime_ = ros::Time::now();
    if(req.usePose){
        eMode_= EmergencyMode::POSE;
        emergencyPose_ = req.pose;
    } else{
        eMode_ = EmergencyMode::VEL;
        emergencyVel_ = req.vel;
    }
    res.success = true;
    return true;
}