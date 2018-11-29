/**************************************************************************
  File Name    : safety.cpp
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Nov 20, 2018.
  Description  : Safety guard to prevent collision while operating multiple
                 quadrotors. When the clearance falls below the minimal
                 value, will request emergency landing by all tracked quads,
                 and terminate the normal autonomous behavior. Use this as 
                 the last safety check, not an active collision avoidance/
**************************************************************************/
#include "mslquad/safety.h"
#include<ros/ros.h>

// BEGIN: PoseTracker
PoseTracker::PoseTracker(const std::string &poseTopic, const std::string &velTopic) : 
        isInit_(false) {
    std::cout << "Safety guard subscribing to " << poseTopic << std::endl;
    std::cout << "Safety guard subscribing to " << velTopic << std::endl;
    velSub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
        velTopic,
        1, &PoseTracker::velSubCB, this);
    poseSub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        poseTopic, 
        1, &PoseTracker::poseSubCB, this);
}

PoseTracker::~PoseTracker() {

}

void PoseTracker::poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pose_ = *msg;
    isInit_ = true;
}

void PoseTracker::velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    vel_ = *msg;
}
// END: PoseTracker

// BEGIN: Safety
Safety::Safety() : 
checkMinDist_(0.6),
sepDist_(0.5),
dtPred_(0.1),
minX_(-7),
maxX_(7),
minY_(-2.7),
maxY_(2.7) {

    std::string quadNamePre;
    nh_.getParam("/safety/quad_name", quadNamePre);
    
    std::vector<int> quadIdx;
    ros::param::get("/safety/quad_ids", quadIdx);

    if(quadNamePre.size()==0 || quadIdx.size()==0) {
        ROS_ERROR("No quads specified for safety guard. Shutting down...");
        ros::shutdown();
        return;
    }
    
    for(const int id : quadIdx) {
        quadNames_.push_back(quadNamePre+std::to_string(id));
    }

    ros::param::get("~check_min_dist", checkMinDist_);
    ros::param::get("~sep_dist", sepDist_);
    ros::param::get("~dt_pred", dtPred_);
    ros::param::get("~min_x", minX_);
    ros::param::get("~max_x", maxX_);
    ros::param::get("~min_y", minY_);
    ros::param::get("~max_y", maxY_);

    // print parameters
    std::cout << "Safety guard parameters:" << std::endl;
    std::cout << "quad names: ";
    for(const auto &name : quadNames_) {
        std::cout << name << " ";
    }
    std::cout << std::endl;
    /*
    std::cout << std::endl;
    std::cout << "check_min_dist: " << checkMinDist_ << std::endl;
    std::cout << "sep_dist: " << sepDist_ << std::endl;
    std::cout << "dt_pred: " << dtPred_ << std::endl;
    std::cout << "min_x: " << minX_ << std::endl;
    std::cout << "max_x: " << maxX_ << std::endl;
    std::cout << "min_y: " << minY_ << std::endl;
    std::cout << "max_y: " << maxY_ << std::endl;
    std::cout << std::endl;
    */

    for(const auto &name : quadNames_) {
        // sub to pose
        std::string topic = "/";
        topic += name;
        std::string posTopic = topic + "/mavros/local_position/pose";
        std::string velTOpic = topic + "/mavros/local_position/velocity";
        poseTrackers_.push_back(std::make_unique<PoseTracker>(posTopic, velTOpic));
        // create landing client
        std::string srvName = "/";
        srvName += name;
        srvName += "/emergency_land";
        ros::service::waitForService(srvName, -1);
        landClis_.push_back(
            nh_.serviceClient<mslquad::Emergency>(srvName)
        );
    }

    bool waitInit = true;
    while(ros::ok() && waitInit) {
        waitInit = false;
        for(const auto &pt : poseTrackers_) {
            if(!pt->isInit_) {
                waitInit = true;
            }
        }
        ROS_INFO("Safety guard waiting to receive initial pose from quads.");
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    mainTimer_ = nh_.createTimer(
        ros::Duration(0.05), // running at 20Hz by default
        &Safety::mainTimerCB, this);
    ROS_INFO("Safety guard running.");
}

Safety::~Safety() {

}

void Safety::mainTimerCB(const ros::TimerEvent& event) {
    // flight tested with two quads
    // TODO: flight test with more than two quads
    bool requestLand = false;
    std::vector<geometry_msgs::Pose> landPosAll;
    for(const auto &t : poseTrackers_) {
        landPosAll.push_back(t->pose_.pose);
    }
    for(int i=0; i<poseTrackers_.size()-1; ++i) {
        // get distance based on current pose
        double dist = getDist(poseTrackers_[i]->pose_.pose, poseTrackers_[i+1]->pose_.pose);
        // get predicted distance after dtPred time
        geometry_msgs::Pose posePred1 = predPoseGivenVel(
            poseTrackers_[i]->pose_.pose,
            poseTrackers_[i]->vel_, dtPred_);
        geometry_msgs::Pose posePred2 = predPoseGivenVel(
            poseTrackers_[i+1]->pose_.pose,
            poseTrackers_[i+1]->vel_, dtPred_);
        double distPred = getDist(posePred1, posePred2);

        if(dist < checkMinDist_ || distPred < checkMinDist_) {
            ROS_INFO("Triggered safety guard");
            ROS_INFO("dist = %f", dist);
            ROS_INFO("distPred = %f", distPred);
            requestLand = true;
            auto unitVec = getUnitVecFromTwoPoseXY(
                poseTrackers_[i]->pose_.pose, 
                poseTrackers_[i+1]->pose_.pose);
            landPosAll[i].position.x += sepDist_*unitVec.first;
            landPosAll[i].position.y += sepDist_*unitVec.second;
            landPosAll[i+1].position.x -= sepDist_*unitVec.first;
            landPosAll[i+1].position.y -= sepDist_*unitVec.second;
            landPosAll[i].position.x = clamp(landPosAll[i].position.x, minX_, maxX_);
            landPosAll[i].position.y = clamp(landPosAll[i].position.y, minY_, maxY_);
            landPosAll[i+1].position.x = clamp(landPosAll[i+1].position.x, minX_, maxX_);
            landPosAll[i+1].position.y = clamp(landPosAll[i+1].position.y, minY_, maxY_);
        }
    }
    if(requestLand) {
        for(int i=0; i<poseTrackers_.size(); ++i) {
            mslquad::Emergency srv;
            srv.request.pose = landPosAll[i];
            srv.request.usePose = true;
            if(landClis_[i].call(srv)) {
                ROS_INFO("Emergency service called");
            } else {
                ROS_ERROR("Failed to call emergency landing");
            }
        }
        mainTimer_.stop();
        ROS_INFO("Safety guard ended. Please restart.");
        ros::shutdown();
    }
}

double Safety::getDist(
        const geometry_msgs::Pose &p1,
        const geometry_msgs::Pose &p2) {
    return sqrt(
        pow(p1.position.x-p2.position.x, 2) +
        pow(p1.position.y-p2.position.y, 2) +
        pow(p1.position.z-p2.position.z, 2));
}

geometry_msgs::Pose Safety::predPoseGivenVel(
        const geometry_msgs::Pose &pose,
        const geometry_msgs::TwistStamped &vel,
        const double dt) {
    geometry_msgs::Pose newPose;
    newPose.position.x = pose.position.x + vel.twist.linear.x * dt;
    newPose.position.y = pose.position.y + vel.twist.linear.y * dt;
    newPose.position.z = pose.position.z + vel.twist.linear.z * dt;
    return newPose;
}

std::pair<double, double> Safety::getUnitVecFromTwoPoseXY(
        const geometry_msgs::Pose &p1,
        const geometry_msgs::Pose &p2) {
    double dx = p1.position.x-p2.position.x;
    double dy = p1.position.y-p2.position.y;
    double norm = sqrt(dx*dx + dy*dy);
    return {dx/norm, dy/norm};
}
// END: Safety

int main(int argc, char **argv){
    ros::init(argc, argv, "safety");
    Safety safetyGuard;
    ros::spin();
    return 0;
}