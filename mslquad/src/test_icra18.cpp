#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <ctime> // calculate delta time when solving optimization

#include "cvxgen_se3.h"
//#include "cvxgen_se3_comp.h"

using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_icra18");
    double tStart, tEnd;

    cout << "test_icra18" << endl;

    // test cvxgen_se3
    cout << endl;
    cout << "==================================";
    cout << "Testing cvxgen_se3..." << endl; 

    cvxgen_se3_setup();
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
    cvxgen_se3_solve(f, W, wdes, fmin, fmax);
    tEnd = clock();
    cout << "time elapsed: " << (double)(clock() - tStart)*1000/CLOCKS_PER_SEC << "ms" << endl;

    cout << "expected result: " << endl
        << -0.889942 << endl << -0.338597 << endl << -0.34866 << endl << -0.899997 << endl;

    cout << "actual result: " << endl << f << endl;

    // test cvxgen_se3_com
    //cvxgen_se3_comp_setup();


    return 0;
}