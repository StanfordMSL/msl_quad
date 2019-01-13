#ifndef __ICRA18_CTRL_H__
#define __ICRA18_CTRL_H__

#include <Eigen/Dense>

// must to be called once before calling icra18_ctrl()
// otherwise cvxgen solver won't work
void icra18_setup(void);

// fres: 4d vector, solution force to 4 motors
// W: wrench matrix in the special control frame
// wdes: desired wrench in special control frame, ordered as (fz, tx, ty, tz)
// f_min: minimal motor thrust (N)
// f_max: maximal motor thrust (N)
// D: offset from COM of quadrotor to COM of object
// isNED: whether it's in NED frame or ENU
void icra18_ctrl(Eigen::Vector4d &fres,
                 Eigen::Matrix4d W,
                 Eigen::Vector4d wdes,
                 const double f_min,
                 const double f_max,
                 const double D,
                 bool isNED = false);
#endif