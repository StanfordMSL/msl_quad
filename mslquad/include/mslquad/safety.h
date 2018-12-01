/**************************************************************************
  File Name    : safety.h
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
#ifndef __SAFETY_H__
#define __SAFETY_H__

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mslquad/Emergency.h"
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <iostream>

class PoseTracker{
public:
    PoseTracker(const std::string &poseTopic, const std::string &velTopic);
    ~PoseTracker();

    geometry_msgs::PoseStamped pose_;
    geometry_msgs::TwistStamped vel_;
    bool isInit_;
private:
    ros::NodeHandle nh_;
    ros::Subscriber poseSub_;
    ros::Subscriber velSub_;

    void poseSubCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void velSubCB(const geometry_msgs::TwistStamped::ConstPtr& msg);
};

class Safety{
public:
    Safety();
    ~Safety();

    double getDist(const geometry_msgs::Pose &p1,
                        const geometry_msgs::Pose &p2);
    geometry_msgs::Pose predPoseGivenVel(
        const geometry_msgs::Pose &pose,
        const geometry_msgs::TwistStamped &vel,
        const double dt);
    std::pair<double, double> getUnitVecFromTwoPoseXY(
        const geometry_msgs::Pose &p1,
        const geometry_msgs::Pose &p2);
    inline double clamp(double x, double minX, double maxX) {
        if(x>maxX) return maxX;
        if(x<minX) return minX;
        return x;
    }

private:
    ros::NodeHandle nh_;
    ros::Timer mainTimer_;
    std::vector<std::string> quadNames_;
    std::vector<std::unique_ptr<PoseTracker>> poseTrackers_;
    std::vector<ros::ServiceClient> landClis_;

    // constant parameter to be retrieved from launch file
    double checkMinDist_; // this is the desired minimal clearance between quads
    double sepDist_; // how much we move the quad backwards to prevent collision
    double dtPred_; // used to predict future position given velocity
    double minX_; // boundary of the room
    double maxX_;
    double minY_;
    double maxY_;

    void mainTimerCB(const ros::TimerEvent& event);
};
#endif