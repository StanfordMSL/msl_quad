// solve uncompensated problem, i.e., Eq.(14) in ICRA 18 paper
#ifndef __CVXGEN_SE3_H__
#define __CVXGEN_SE3_H__

#include <iostream>
#include <Eigen/Dense>
//extern "C" {
    #include "cvxgen_se3/solver.h"
//}

// ---------------------------------------------------- 
namespace se3 {
// ---------------------------------------------------- 
// These variables must be declared as global variables
// see the comments in solver.h
Vars vars;
Params params;
Workspace work;
Settings settings;

void cvxgen_setup(void) {
    set_defaults();
    setup_indexing();
    settings.verbose = 1;
    return;
}

void cvxgen_solve(
        Eigen::Vector4d& f, 
        const Eigen::Matrix4d& W, 
        const Eigen::Vector4d& wdes, 
        const double fmin, 
        const double fmax) {
    params.W_row2[0] = W(1,0);
    params.W_row2[1] = W(1,1);
    params.W_row2[2] = W(1,2);
    params.W_row2[3] = W(1,3);

    params.W_row3[0] = W(2,0);
    params.W_row3[1] = W(2,1);
    params.W_row3[2] = W(2,2);
    params.W_row3[3] = W(2,3);
    
    params.W_row4[0] = W(3,0);
    params.W_row4[1] = W(3,1);
    params.W_row4[2] = W(3,2);
    params.W_row4[3] = W(3,3);
    
    params.wdes[0] = wdes(0);
    params.wdes[1] = wdes(1);
    params.wdes[2] = wdes(2);
    params.wdes[3] = wdes(3);
    
    params.FMIN[0] = fmin;
    params.FMAX[0] = fmax;

    solve();

    //std::cout << vars.f[0] << ", " << vars.f[1] << ", " << vars.f[2] << ", " << vars.f[3] << std::endl;

    f(0) = vars.f[0];
    f(1) = vars.f[1];
    f(2) = vars.f[2];
    f(3) = vars.f[3];
}
} // namespace se3
#endif