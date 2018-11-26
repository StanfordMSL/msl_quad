/**************************************************************************
  File Name    : se3_controller.h
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Nov 6, 2018.
  Description  : SE(3) controller
**************************************************************************/

#ifndef SE3_COOP_H
#define SE3_COOP_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <Eigen/Dense>

using std::vector;

#include<mslquad/px4_base_controller.h>

class SE3Coop : public PX4BaseController {
public:
    SE3Coop();
    ~SE3Coop();

protected:
    void controlLoop(void) override;
    void se3control(
        const Eigen::Vector3d &r_euler, 
        const Eigen::Vector3d &r_wb, 
        const Eigen::Vector3d &r_pos,
        const Eigen::Vector3d &r_vel, 
        const Eigen::Vector3d &r_acc);

private:
    double KP_;
    double KV_;
    double KR_;
    double KW_;
    double M_;
    double g_;
    std::string quadFrame_;

    Eigen::Matrix4d W_;

    // =================================================================================
    int quad_addr;
    int quad_count;

    struct profile_struct
    {
        float rot_offset;      // Rotate offset, 0 at y collinear to bar
        float x_offset;        // x-axis arm offset from CoM to CoQ
        float y_offset;        // y-axis arm offset from CoM to CoQ
        float arm_radius;      // x and y offset of motors (assume symmetrical frame)
        float motor_pos[4][2]; // Individual Motor Positions (relative to global), numbered in PX4 convention
    };

    vector<profile_struct> quad;
    Eigen::MatrixXf A;
    Eigen::VectorXf x_ls;
    Eigen::Vector4f y;

    double bar_radius;
    double arm_radius;
    double c_deg2;
    double c_deg1;
    double c_deg0;
    double c_ft_lin;
    int solver_type;

    void visualise(void);
    void linear_solver(int type = 0);
    // =================================================================================

};

#endif