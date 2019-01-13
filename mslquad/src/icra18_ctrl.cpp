#include "mslquad/icra18_ctrl.h"
#include "mslquad/cvxgen_se3.h"
#include "mslquad/cvxgen_se3_comp.h"
#include <cmath>

void icra18_setup(void) {
    se3::cvxgen_setup();
    se3comp::cvxgen_setup();
}

void icra18_ctrl(Eigen::Vector4d &fres,
                 Eigen::Matrix4d W,
                 Eigen::Vector4d wdes,
                 const double f_min,
                 const double f_max,
                 const double D,
                 bool isNED) {
    // assume cvxgen_setup has been called
    Eigen::Vector4d fopt_temp;
    Eigen::Vector4d v4d;
    double pmin, pmax, pstar;
    const double ty_bias = 100.0;
    Eigen::Vector4d wdes_temp(wdes);
    wdes_temp(2) = ty_bias;
    
    // solve pmin
    se3::cvxgen_solve(fopt_temp, W, wdes_temp, f_min, f_max);
    v4d = W * fopt_temp;
    pmin = v4d(2) + D*wdes(0);

    // solve pmax
    for(int i=0; i<4; ++i) { // in order to solve pmax
        W(2, i) = -W(2, i); 
    }
    se3::cvxgen_solve(fopt_temp, W, wdes_temp, f_min, f_max);
    for(int i=0; i<4; ++i) { // revert back the temp change
        W(2, i) = -W(2, i); 
    }
    v4d = W * fopt_temp;
    pmax = v4d(2) + D*wdes(0);

    // compute pstar
    pstar = fmin( fabs(pmin), fabs(pmax) );

    // compensation optimization
    double t_comp;
    if(isNED) { // NED, i.e., wdes(0) or fz is negative
        if(wdes(2) > 0) 
            t_comp = 2.0*wdes(2) - D*wdes(0) - pstar;
        else 
            t_comp = wdes(2);
    } else { // ENU
        if(wdes(2) > 0)
            t_comp = wdes(2);
        else 
            t_comp = 2.0*wdes(2) - D*wdes(0) + pstar;
    }
    wdes(2) = t_comp;
    se3comp::cvxgen_solve(fres, W, wdes, f_min, f_max, -D*wdes(0)-pstar, -D*wdes(0)+pstar);
}