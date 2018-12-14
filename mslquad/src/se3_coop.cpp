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
                     M_(1.04), g_(9.8), kr_roll(2.5), kr_pitch(2), kr_yaw(0.7),
                     kw_roll(0.125), kw_pitch(0.1), kw_yaw(0.03)
{
    // handle ros parameters
    ros::param::get("~KP", KP_);
    ros::param::get("~KV", KV_);
    ros::param::get("~KR", KR_);
    ros::param::get("~KW", KW_);
    ros::param::get("~M", M_);
    ros::param::get("~quadFrame", quadFrame_);
    ros::param::get("~quad_addr", quad_addr);
    ros::param::get("~quad_count", quad_count);
    ros::param::get("~bar_radius", bar_radius);
    ros::param::get("~arm_radius", arm_radius);
    ros::param::get("~c_deg2", c_deg2);
    ros::param::get("~c_deg1", c_deg1);
    ros::param::get("~c_deg0", c_deg0);
    ros::param::get("~c_ft_lin", c_ft_lin);
    ros::param::get("~solver_type", solver_type);
    ros::param::get("~kr_roll", kr_roll);
    ros::param::get("~kr_pitch", kr_pitch);
    ros::param::get("~kr_yaw", kr_yaw);
    ros::param::get("~kw_roll", kw_roll);
    ros::param::get("~kw_pitch", kw_pitch);
    ros::param::get("~kw_yaw", kw_yaw);

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
    std::cout << "Quad Address / Quad Total: " << quad_addr << "/" << quad_count << std::endl;
    std::cout << "bar_radius: " << bar_radius << std::endl;
    std::cout << "arm_radius: " << arm_radius << std::endl;
    std::cout << "Motor Profile: " << c_deg2 << "x^2 +" << c_deg1 << "x +" << c_deg0 << std::endl;
    std::cout << "Motor Force -> Torque Linear Approx: " << c_ft_lin << std::endl;

    // Setup the coop configuration (quad and motor positions).
    double base_angle = 2 * M_PI / quad_count; // angle between each quad (radians)

    for (int i = 0; i < quad_count; i++)
    {
        quad.push_back(profile_struct());
        quad[i].x_offset = bar_radius * cos(base_angle * i + (M_PI / 2));
        quad[i].y_offset = bar_radius * sin(base_angle * i + (M_PI / 2));

        quad[i].rot_offset = 0;
        quad[i].arm_radius = arm_radius;

        for (int j = 0; j < 2; j++)
        {
            double angle = (base_angle * i + (M_PI / 2)) - (M_PI / 4) + (j * M_PI / 2);

            quad[i].motor_pos[2 * j][0] = quad[i].x_offset + (quad[i].arm_radius * cos(angle));
            quad[i].motor_pos[2 * j][1] = quad[i].y_offset + (quad[i].arm_radius * sin(angle));

            angle += M_PI;
            quad[i].motor_pos[(2 * j) + 1][0] = quad[i].x_offset + (quad[i].arm_radius * cos(angle));
            quad[i].motor_pos[(2 * j) + 1][1] = quad[i].y_offset + (quad[i].arm_radius * sin(angle));
        }
        // TEMPORARY HACK FOR GETTING IT SYNCED WITH PX4
        if (i == 1)
        {
            double sw_0x = quad[i].motor_pos[0][0];
            double sw_1x = quad[i].motor_pos[1][0];
            double sw_2x = quad[i].motor_pos[2][0];
            double sw_3x = quad[i].motor_pos[3][0];

            double sw_0y = quad[i].motor_pos[0][1];
            double sw_1y = quad[i].motor_pos[1][1];
            double sw_2y = quad[i].motor_pos[2][1];
            double sw_3y = quad[i].motor_pos[3][1];

            quad[i].motor_pos[0][0] = sw_1x;
            quad[i].motor_pos[1][0] = sw_0x;
            quad[i].motor_pos[2][0] = sw_3x;
            quad[i].motor_pos[3][0] = sw_2x;

            quad[i].motor_pos[0][1] = sw_1y;
            quad[i].motor_pos[1][1] = sw_0y;
            quad[i].motor_pos[2][1] = sw_3y;
            quad[i].motor_pos[3][1] = sw_2y;
        }
    }

    // Setup the variables for the math we'll be doing.
    A.resize(4, quad_count * 4);
    x_ls.resize(quad_count * 4, 1);

    Eigen::Vector4d feeder;
    for (int i = 0; i < quad_count; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            feeder(0) = 1;
            feeder(1) = -quad[i].motor_pos[j][0];
            feeder(2) = -quad[i].motor_pos[j][1];
            if (j <= 1)
            {
                feeder(3) = -c_ft_lin;
            }
            else
            {
                feeder(3) = c_ft_lin;
            }
            A.col(4 * i + j) = feeder;
        }
    }

    visualise();

    ros::Duration(3).sleep();
}

SE3Coop::~SE3Coop()
{
}

void SE3Coop::controlLoop(void)
{
    double altitude;
    // double altitude = joyCmds_.axes[3];

    if (joyCmds_.axes.size() == 0) // This is because the msg is empty if joystick is untouched.
    {
        altitude = -1;
    }
    else
    {
        altitude = joyCmds_.axes[3];
    }
    //std::cout << altitude << std::endl;
    // TODO: do something more interesting than hovering (e.g., traj following)
    Eigen::Vector3d r_euler(0, 0, 0);
    Eigen::Vector3d r_wb(0, 0, 0);
    Eigen::Vector3d r_pos(0, 0, altitude);

    //std::cout << r_pos << std::endl;

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
    eR(0) *= kr_roll;
    eR(1) *= kr_pitch;
    eR(2) *= kr_yaw;
    ew(0) *= kw_roll;
    ew(1) *= kw_pitch;
    ew(2) *= kw_yaw;
    tauCmd = -eR - ew;

    //std::cout << "fzCmd = " << fzCmd << std::endl;
    //std::cout << "tauCmd = " << tauCmd << std::endl;

    // =================================================================================
    // Solve for individual motor thrust
    /*
    // Static Test
    y(0) = 23.0;
    y(1) = 0.0;
    y(2) = 1.0;
    y(3) = 0.0;
    */

    y(0) = fzCmd;
    y(1) = tauCmd(0);
    y(2) = tauCmd(1);
    y(3) = tauCmd(2);

    //std::cout << y << std::endl;

    //std::string sep = "\n----------------------------------------\n";
    //std::cout << y << sep;

    linear_solver(solver_type);

    // solve for PWM command using motor calibration data
    for (int i = 0; i < 4; i++)
    {
        double force = x_ls((quad_addr * 4) + i);

        double inner = (c_deg1 * c_deg1) - (4 * c_deg2 * (c_deg0 - force));

        motorCmd[i] = (-c_deg1 + sqrt(inner)) / (2 * c_deg2);

        // Check for values under range of motor model capabilities
        if (motorCmd[i] < 0)
        {
            motorCmd[i] = 0;
        }
        else if (motorCmd[i] > 0.8)
        {
            motorCmd[i] = 0.8;
        }

        if (inner < 0)
        {
            motorCmd[i] = 0;
        }
    }

    // =================================================================================

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

void SE3Coop::visualise(void)
{
    std::string sep = "\n----------------------------------------\n";
    std::cout << sep << A << sep;
    if (solver_type == 0)
    {
        std::cout << "Solver Type: Basic Least Square Solver" << sep;
    }
    else if (solver_type == 1)
    {
        std::cout << "Solver Type: QR Factorization with Pivot" << sep;
    }
    else if (solver_type == 2)
    {
        std::cout << "Solver Type: Jacobi with ObjFunc Options" << sep;
    }
    else if (solver_type == 3)
    {
        std::cout << "Solver Type: QR Factorization with ObjFunc Options" << sep;
    }
    else
    {
        std::cout << "Warning, solver definition overwritten. Do not fly." << sep;
    }

    std::cout << "Quad Address  |  \tMotor 1\t\t\t|\tMotor 2\t\t\t|\tMotor 3\t\t\t|\tMotor 4" << std::endl;

    for (int i = 0; i < quad_count; i++)
    {
        std::cout << "\t" << i << "\t";
        for (int j = 0; j < 4; j++)
        {
            std::cout << "[x: " << quad[i].motor_pos[j][0] << "]\t"
                      << "[y: " << quad[i].motor_pos[j][1] << "]\t";
        }
        std::cout << sep;
    }
}

void SE3Coop::linear_solver(int type)
{
    double relative_error;

    int motor_count = quad_count * 4;
    int row_count = (motor_count) + 4;
    int column_count = (motor_count) + 4;

    Eigen::MatrixXd A_tilde(row_count, column_count);
    Eigen::VectorXd y_tilde(row_count);
    Eigen::VectorXd x_tilde(row_count);

    if (type == 0)
    { // Default. Basic Least Squares.
        x_ls = A.transpose() * ((A * A.transpose()).inverse()) * y;
    }
    else if (type == 1) // QR with column pivoting (fast,accurate).
    {
        x_ls = A.colPivHouseholderQr().solve(y);
    }

    else if (type == 2) // Jacobi (slow, accurate).
    {
        A_tilde.topLeftCorner(motor_count, motor_count) = Eigen::MatrixXd::Identity(motor_count, motor_count);
        A_tilde.topRightCorner(motor_count, 4) = A.transpose();
        A_tilde.bottomLeftCorner(4, motor_count) = A;
        A_tilde.bottomRightCorner(4, 4) = Eigen::MatrixXd::Zero(4, 4);

        y_tilde.head(motor_count) = Eigen::VectorXd::Zero(motor_count);
        y_tilde.tail(4) = y;

        x_tilde = A_tilde.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y_tilde);
        x_ls = x_tilde.head(motor_count);
    }
    else if (type == 3) // QR with column pivoting (fast,accurate). TODO: Finish ability to switch objective functions.
    {
        A_tilde.topLeftCorner(motor_count, motor_count) = Eigen::MatrixXd::Identity(motor_count, motor_count);
        A_tilde.topRightCorner(motor_count, 4) = A.transpose();
        A_tilde.bottomLeftCorner(4, motor_count) = A;
        A_tilde.bottomRightCorner(4, 4) = Eigen::MatrixXd::Zero(4, 4);

        y_tilde.head(motor_count) = Eigen::VectorXd::Zero(motor_count);
        y_tilde.tail(4) = y;

        x_tilde = A_tilde.colPivHouseholderQr().solve(y_tilde);
        x_ls = x_tilde.head(motor_count);
    }

    else
    {
        // Shouldn't ever be here!
        std::cout << "You messed up somewhere. The Default Least Square Solver Overwritten!" << std::endl;
    }

    relative_error = (A * x_ls - y).norm() / y.norm();
    //std::cout << relative_error << std::endl;
}
