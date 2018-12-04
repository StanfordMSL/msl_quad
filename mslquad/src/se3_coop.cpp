/**************************************************************************
  File Name    : se3_controller.cpp
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Nov 6, 2018.
  Description  : SE(3) controller
**************************************************************************/
#include "mslquad/se3_coop.h"
#include <cmath>

SE3Coop::SE3Coop() : quadFrame_("mslquad"),
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
    ros::param::get("~quad_addr", quad_addr);

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
    std::cout << "quad_addr: " << quad_addr << std::endl;
    
    ros::Duration(3).sleep();

    // =================================================================================
    // Setup the coop configuration (quad and motor positions).
    quad_count = 2;
    arm_length = 0.35;

    float base_angle = 2 * M_PI / quad_count;

    for (int i = 0; i < quad_count; i++)
    {
        quad.push_back(profile_struct());
        quad[i].x_offset = arm_length * cos(base_angle * i + (M_PI / 2));
        quad[i].y_offset = arm_length * sin(base_angle * i + (M_PI / 2));

        quad[i].rot_offset = 0;
        quad[i].arm_length = 0.2;

        quad[i].motor_prof[0] = 4.67636;
        quad[i].motor_prof[1] = 1.68915;
        quad[i].motor_prof[2] = -0.05628;
        quad[i].motor_prof[3] = 1.0 / 60.0;

        for (int j = 0; j < 2; j++)
        {
            float angle;

            angle = (base_angle * i + (M_PI / 2)) - (M_PI / 4) + (j * M_PI / 2);

            quad[i].motor_pos[2 * j][0] = quad[i].x_offset + (quad[i].arm_length * cos(angle));
            quad[i].motor_pos[2 * j][1] = quad[i].y_offset + (quad[i].arm_length * sin(angle));

            angle += M_PI;
            quad[i].motor_pos[(2 * j) + 1][0] = quad[i].x_offset + (quad[i].arm_length * cos(angle));
            quad[i].motor_pos[(2 * j) + 1][1] = quad[i].y_offset + (quad[i].arm_length * sin(angle));
        }
    }

    // Setup this quad's specific address.
    //quad_addr = 0;
    //quad_addr = stoi(this->quadNS_);

    // Setup the variables for the math we'll be doing.
    A.resize(4, quad_count * 4);
    x_ls.resize(quad_count, 1);

    Eigen::Vector4f feeder;
    for (int i = 0; i < quad_count; i++)
    {
        for (int j = 0; j < 4; j++)
        {

            feeder(0) = 1;
            feeder(1) = quad[i].motor_pos[j][1];
            feeder(2) = -quad[i].motor_pos[j][0];
            if (j <= 1)
            {
                feeder(3) = -quad[i].motor_prof[3];
            }
            else
            {
                feeder(3) = quad[i].motor_prof[3];
            }
            A.col(4 * i + j) = feeder;
        }
    }
  //  std::string sep = "\n----------------------------------------\n";
  //  std::cout << A << sep;

    // =================================================================================
}

SE3Coop::~SE3Coop()
{
}

void SE3Coop::controlLoop(void)
{
    // TODO: do something more interesting than hovering (e.g., traj following)
    Eigen::Vector3d r_euler(0, 0, 0);
    Eigen::Vector3d r_wb(0, 0, 0);
    Eigen::Vector3d r_pos(0, 0, 0.5);
    Eigen::Vector3d r_vel(0, 0, 0);
    Eigen::Vector3d r_acc(0, 0, 0);
    se3control(r_euler, r_wb, r_pos, r_vel, r_acc);
}

void SE3Coop::se3control(const Eigen::Vector3d &r_euler,
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

    // =================================================================================
    // Solve for individual motor thrust
    y(0) = fzCmd;
    y(1) = tauCmd(0);
    y(2) = tauCmd(1);
    y(3) = tauCmd(2);
/*
    int motor_count = quad_count * 4;
    int row_count = (motor_count) + 4;
    int column_count = (motor_count) + 4;

    Eigen::MatrixXf A_tilde(row_count, column_count);
    Eigen::VectorXf y_tilde(row_count);
    Eigen::VectorXf x_tilde(row_count);

    A_tilde.topLeftCorner(motor_count, motor_count) = Eigen::MatrixXf::Identity(motor_count, motor_count);
    A_tilde.topRightCorner(motor_count, 4) = A.transpose();
    A_tilde.bottomLeftCorner(4, motor_count) = A;
    A_tilde.bottomRightCorner(4, 4) = Eigen::MatrixXf::Zero(4, 4);

    y_tilde.head(motor_count) = Eigen::VectorXf::Zero(motor_count);
    y_tilde.tail(4) = y;

    x_tilde = A_tilde.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y_tilde);
    x_ls = x_tilde.head(motor_count);
*/
    // Basic Least Square
    x_ls = A.transpose() * ((A * A.transpose()).inverse()) * y;

    // solve for PWM command using motor calibration data
    // TODO: these numbers are only good for mslquad
    const double c_a = 4.67636;
    const double c_b = 1.68915;
    const double c_c = -0.05628;

    int loc_add = 4 * quad_addr;

    for (int i = 0; i < 4; i++)
    {
        motorCmd[i] = (-c_b + sqrt((c_b * c_b) - (4 * c_a * (c_c - x_ls(loc_add + i))))) / (2 * c_a);

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
    // =================================================================================
/*
    Eigen::Vector4f y_fwd = A * x_ls;

    std::string sep = "\n----------------------------------------\n";
    for (int i = 0; i < 4; i++)
    {
        std::cout << y[i] << "  " << y_fwd[i] << std::endl;
    }

    std::cout << sep;
*/
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