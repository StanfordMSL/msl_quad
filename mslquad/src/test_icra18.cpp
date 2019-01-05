#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <ctime> // calculate delta time when solving optimization

#include "cvxgen_se3.h"
#include "cvxgen_se3_comp.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_icra18");
    double tStart, tEnd;

    cout << "test_icra18" << endl;

    // test cvxgen_se3
    cout << endl;
    cout << "==================================" << endl;
    cout << "Testing cvxgen_se3..." << endl; 

    se3::cvxgen_setup();
    Eigen::Vector4d f;
    Eigen::Matrix4d W;
    Eigen::Vector4d wdes;
    double fmin = -0.9;
    double fmax = 0;
    W << 
        1.        ,  1.        ,  1.        ,  1.        ,
        0.14427404,  0.13850993, -0.14427404, -0.13850993,
        -0.69803295, -0.41524897, -0.42101309, -0.70379706,
        0.02,       -0.02,        0.02,       -0.02;

    wdes << 
        -2.477196503010238,
        -3.33393549e-04,
        100,
        -1.74337901e-07;

    tStart = clock();
    se3::cvxgen_solve(f, W, wdes, fmin, fmax);
    tEnd = clock();
    cout << "time elapsed: " << (double)(clock() - tStart)*1000/CLOCKS_PER_SEC << "ms" << endl;

    cout << "expected result: " << endl
        << -0.889942 << endl << -0.338597 << endl << -0.34866 << endl << -0.899997 << endl;

    cout << "actual result: " << endl << f << endl;

    // test cvxgen_se3_comp
    cout << endl;
    cout << "==================================" << endl;
    cout << "Testing cvxgen_se3_comp..." << endl; 

    se3comp::cvxgen_setup();
    fmin = -0.9;
    fmax = 0;
    double wy_lb = 1.3880837730700393;
    double wy_ub = 1.6949999404120033;

    W << 
        1.         ,  1.        ,  1.        ,  1.         ,
        -0.14431893,  0.13846316,  0.14431893, -0.13846316,
        -0.75787466, -0.76373043, -0.48094835, -0.47509257,
        0.02       , -0.02      ,  0.02      , -0.02       ;

    wdes << 
        -2.488720105489267,
        -6.23088251e-04,
        -2.15139793e-02,
        -8.10975028e-06;

    tStart = clock();
    se3comp::cvxgen_solve(f, W, wdes, fmin, fmax, wy_lb, wy_ub);
    tEnd = clock();
    cout << "time elapsed: " << (double)(clock() - tStart)*1000/CLOCKS_PER_SEC << "ms" << endl;

    cout << "expected result: " << endl
        << -0.355555 << endl << -0.346322 << endl << -0.889007 << endl << -0.897835 << endl;

    cout << "actual result: " << endl << f << endl;   

    return 0;
}