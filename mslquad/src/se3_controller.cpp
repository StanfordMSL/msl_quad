/**************************************************************************
  File Name    : se3_controller.cpp
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Nov 6, 2018.
  Description  : SE(3) controller
**************************************************************************/
#include "mslquad/se3_controller.h"
#include <cmath>

SE3Controller::SE3Controller() : quadFrame_("mslquad"),
                                 KP_(6.0), KV_(4.0), KR_(0.8), KW_(0.1),
                                 M_(1.04), g_(9.8)
{
    // handle ros parameters
    ros::param::get("~KP", KP_);
    ros::param::get("~KV", KV_);
    ros::param::get("~KR", KR_);
    ros::param::get("~KW", KW_);
    ros::param::get("~M", M_);
    ros::param::get("~quadFrame", quadFrame_);

    if (quadFrame_ == "mslquad")
    {
        W_ << 1, 1, 1, 1,
            -0.12, 0.12, 0.12, -0.12,
            -0.12, 0.12, -0.12, 0.12,
            -0.06, -0.06, 0.06, 0.06;
    }
    else if (quadFrame_ == "iris")
    {
        // Iris in Gazebo
        W_ << 1, 1, 1, 1,
            -0.22, 0.2, 0.22, -0.2,
            -0.13, 0.13, -0.13, 0.13,
            -0.06, -0.06, 0.06, 0.06;
    }
    else
    {
        ROS_ERROR("Unknown model name!");
    }

    if (getControlLoopFreq() < 200)
    {
        ROS_ERROR("Need at least 200Hz in se3 control.");
    }

    std::cout << "SE3 controller is using the following parameters: "
              << std::endl;
    std::cout << "KP = " << KP_ << std::endl;
    std::cout << "KV = " << KV_ << std::endl;
    std::cout << "KR = " << KR_ << std::endl;
    std::cout << "KW = " << KW_ << std::endl;
    std::cout << "M = " << M_ << std::endl;
    std::cout << "QuadFrame: " << quadFrame_ << std::endl;
}

SE3Controller::~SE3Controller()
{
}

void SE3Controller::controlLoop(void)
{
    double altitude = (1 + joyCmds_.axes[3]) / 2;

    // TODO: do something more interesting than hovering (e.g., traj following)
    Eigen::Vector3d r_euler(0, 0, 0);
    Eigen::Vector3d r_wb(0, 0, 0);
    Eigen::Vector3d r_pos(0, 0, altitude);
    Eigen::Vector3d r_vel(0, 0, 0);
    Eigen::Vector3d r_acc(0, 0, 0);
    se3control(r_euler, r_wb, r_pos, r_vel, r_acc);
}

void SE3Controller::se3control(const Eigen::Vector3d &r_euler,
                               const Eigen::Vector3d &r_wb, const Eigen::Vector3d &r_pos,
                               const Eigen::Vector3d &r_vel, const Eigen::Vector3d &r_acc)
{
    // Note: everything is in ENU frame
    // get state measurement
    Eigen::Vector3d mea_pos = getPosition();
    Eigen::Vector3d mea_vel = getLinVel();
    Eigen::Vector3d mea_wb = getAngVel();
    Eigen::Matrix3d mea_R = getRotMat();

    // declare desired total thrust and moment
    double fzCmd = 0;
    Eigen::Vector3d tauCmd;

    // declare PWM signal for each propeller
    double motorCmd[4];

    // se3 controller calculation
    Eigen::Vector3d zw(0.0, 0.0, 1.0);
    Eigen::Vector3d Fdes = -KP_ * (mea_pos - r_pos) - KV_ * (mea_vel - r_vel) + M_ * g_ * zw + M_ * r_acc;
    fzCmd = Fdes.dot(mea_R.col(2));

    Eigen::Vector3d zb_des = Fdes / Fdes.norm();
    Eigen::Vector3d xc_des(std::cos(r_euler(2)), std::sin(r_euler(2)), 0.0); // only need yaw from reference euler
    Eigen::Vector3d zb_des_cross_xc_des = zb_des.cross(xc_des);
    Eigen::Vector3d yb_des = zb_des_cross_xc_des / zb_des_cross_xc_des.norm();
    Eigen::Vector3d xb_des = yb_des.cross(zb_des);
    Eigen::MatrixXd R_des(3, 3);
    R_des.col(0) = xb_des;
    R_des.col(1) = yb_des;
    R_des.col(2) = zb_des;

    Eigen::MatrixXd temp(3, 3);
    temp = R_des.transpose() * mea_R - mea_R.transpose() * R_des;
    Eigen::Vector3d eR(temp(2, 1), temp(0, 2), temp(1, 0));
    eR = 0.5 * eR;
    Eigen::Vector3d ew = mea_wb - r_wb;
    eR(2) /= 3.0; // Note: reduce gain on yaw. TODO: make gains a 3d vector
    ew(2) /= 3.0;
    tauCmd = -KR_ * eR - KW_ * ew;

    //std::cout << "fzCmd = " << fzCmd << std::endl;
    //std::cout << "tauCmd = " << tauCmd << std::endl;

    // solve for individual motor thrust
    Eigen::Vector4d wrench(fzCmd, tauCmd(0), tauCmd(1), tauCmd(2));
    Eigen::Vector4d ffff = W_.colPivHouseholderQr().solve(wrench);

    //std::cout << ffff << std::endl;

    // solve for PWM command using motor calibration data
    // TODO: these numbers are only good for mslquad
    const double c_a = 4.67636;
    const double c_b = 1.68915;
    const double c_c = -0.05628;

    for (int i = 0; i < 4; i++)
    {
        motorCmd[i] = (-c_b + sqrt((c_b * c_b) - (4 * c_a * (c_c - ffff(i))))) / (2 * c_a);

        if (motorCmd[i] < 0)
        {
            motorCmd[i] = 0;
        }
        else if (motorCmd[i] > 0.8)
        {
            motorCmd[i] = 0.8;
        }
        else
        {
            // Carry On
        }
    }

    // publish commands
    mavros_msgs::ActuatorControl cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.group_mix = 0;
    for (int i = 0; i < 4; i++)
    {
        cmd.controls[i] = motorCmd[i];
    }
    cmd.controls[7] = 0.1234; // secret key to enabling direct motor control in px4
    actuatorPub_.publish(cmd);
}