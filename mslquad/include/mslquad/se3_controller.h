/**************************************************************************
  File Name    : se3_controller.h
  Author       : Zijian Wang
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : zjwang@stanford.edu
  Create Time  : Nov 6, 2018.
  Description  : SE(3) controller
**************************************************************************/

#ifndef __SE3_CONTROLELR_H__
#define __SE3_CONTROLELR_H__

#include<mslquad/px4_base_controller.h>

class SE3Controller : public PX4BaseController {
public:
    SE3Controller();
    ~SE3Controller();

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
};

#endif