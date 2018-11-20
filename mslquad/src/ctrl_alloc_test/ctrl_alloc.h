#ifndef CTRL_ALLOC_H
#define CTRL_ALLOC_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <Eigen/Dense>

using std::vector;

#include "ros/ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

class ctrl_alloc
{
  public:
    ctrl_alloc(int quad_count, float arm_length);
    ~ctrl_alloc();

    void se3Callback(const std_msgs::Float32MultiArray::ConstPtr &command);

    struct profile_struct
    {
        int address;           // Q address taken CCW from right.
        float rot_offset;      // Rotate offset, 0 at y collinear to bar
        float x_offset;        // x-axis arm offset from CoM to CoQ
        float y_offset;        // y-axis arm offset from CoM to CoQ
        float arm_length;      // x and y offset of motors (assume symmetrical frame)
        float motor_prof[4];   // Motor profile [(f=[0]x^2+[1]x+[2]), (tau = [3]f)]
        float motor_pos[4][2]; // Individual Motor Positions (relative to global), numbered in PX4 convention
    };

    vector<profile_struct> quad;
    Eigen::MatrixXf A;
    Eigen::VectorXf x_ln;
    Eigen::Vector4f y;
    Eigen::Vector4f y_tilde;

  private:
    //
};

#endif