#include <ros/ros.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <ctime> // calculate delta time when solving optimization
#include <cmath>

#include "cvxgen_se3.h"
#include "cvxgen_se3_comp.h"

using namespace std;

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
                 bool isNED = false) {
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
    double f_min = -0.9;
    double f_max = 0;
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
    se3::cvxgen_solve(f, W, wdes, f_min, f_max);
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
    f_min = -0.9;
    f_max = 0;
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
    se3comp::cvxgen_solve(f, W, wdes, f_min, f_max, wy_lb, wy_ub);
    tEnd = clock();
    cout << "time elapsed: " << (double)(clock() - tStart)*1000/CLOCKS_PER_SEC << "ms" << endl;

    cout << "expected result: " << endl
        << -0.355555 << endl << -0.346322 << endl << -0.889007 << endl << -0.897835 << endl;

    cout << "actual result: " << endl << f << endl;   

    // test entire optimization pipeline in ICRA18
    cout << endl;
    cout << "==================================" << endl;
    cout << "Testing entire icra18 controller..." << endl;

    W << 
        1.        ,  1.        ,  1.        ,  1.        ,
        0.01191424, -0.19964481, -0.01191424,  0.19964481,
        -0.31662265, -0.50435321, -0.71591227, -0.5281817 ,
        0.02      , -0.02      ,  0.02      , -0.02      ;

    wdes <<
        -2.4500880932845672,
        1.36481376e-02,
        2.69207734e-02,
        9.35221127e-09;
    f_min = -0.9;
    f_max = 0;
    double D = 0.516267457279;

    tStart = clock();
    icra18_ctrl(f, W, wdes, f_min, f_max, D, true);
    tEnd = clock();
    cout << "time elapsed: " << (double)(clock() - tStart)*1000/CLOCKS_PER_SEC << "ms" << endl;

    cout << "expected result: " << endl
        << -0.761566 << endl << -0.655598 << endl << -0.463478 << endl << -0.569447 << endl;

    cout << "actual result: " << endl << f << endl;

    // test integration with se3 controller
    cout << endl;
    cout << "==================================" << endl;
    cout << "Testing integration with se3 controller..." << endl;
    
    const double barRadius = 0.35;
    wdes << 28.52, 0.0665586, -0.280075, -0.000368148;
    Eigen::Matrix4d Wb;
    Wb <<  1, 1, 1, 1,
          -0.119501, 0.119501, 0.119501, -0.119501,
          -0.119501, 0.119501, -0.119501, 0.119501,
          -0.01666, -0.01666, 0.01666, 0.01666;
    Eigen::Matrix4d m1, m2;
    double alpha = 0;
    m1 << 1, 0, 0, 0,
          barRadius*sin(alpha), 1, 0, 0,
          -barRadius*cos(alpha), 0, 1, 0,
          0, 0, 0, 1;
    alpha = 3.1415926;
    m2 << 1, 0, 0, 0,
          barRadius*sin(alpha), 1, 0, 0,
          -barRadius*cos(alpha), 0, 1, 0,
          0, 0, 0, 1;

    Eigen::Matrix4d Wobj1, Wobj2;
    Wobj1 = m1 * Wb;
    Wobj2 = m2 * Wb;

    Eigen::Vector4d thrust1, thrust2;
    thrust1 << 3.61831, 3.69638, 3.75203, 3.55161; // correct solution
    thrust2 << 3.43914, 3.51721, 3.57286, 3.37245;
    Eigen::Vector4d wverify = Wobj1*thrust1 + Wobj2*thrust2;

    cout << "Verify wrench matrices. Residue check (smaller better): " << (wverify-wdes).norm() << endl;

    Eigen::Matrix4d Rco1, Rco2;
    alpha = 0;
    Rco1 << 1, 0, 0, 0,
            0, cos(alpha), sin(alpha), 0,
            0, -sin(alpha), cos(alpha), 0,
            0, 0, 0, 1;
    icra18_ctrl(thrust1, Rco1*Wobj1, Rco1*(0.5*wdes), 0, 4.3, barRadius); // 4.3N ~ 80% PWM
    alpha = 3.1415926;
    Rco2 << 1, 0, 0, 0,
            0, cos(alpha), sin(alpha), 0,
            0, -sin(alpha), cos(alpha), 0,
            0, 0, 0, 1;
    icra18_ctrl(thrust2, Rco2*Wobj2, Rco2*(0.5*wdes), 0, 4.3, barRadius);

    cout << "-- thrust1: " << endl << thrust1 << endl;
    cout << "-- thrust2: " << endl << thrust2 << endl;
    wverify = Wobj1*thrust1 + Wobj2*thrust2;
    cout << "wverify: " << endl << wverify << endl;


    return 0;
}