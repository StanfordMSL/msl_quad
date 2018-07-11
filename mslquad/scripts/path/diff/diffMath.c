/**************************************************************************
 * File Name      : diffMath.c
 * Author         : Dingjiang Zhou
 *                  Boston University, Boston, 02215
 * Contact        : zhoudingjiang@gmail.com
 * Create Time    : Sun, Aug. 10th, 2014. 10:45:18 PM
 * Last Modified  :
 * Purpose        :
 * Example Code   :
 * Example Output :
 **************************************************************************/
//#include "mexDef.h"  /* if used for pure C code, comment this line out */
#include "dMath.h"

/* derivative of atan2(y,x) w.s.t. t */
double atan2diff1(double y, double x, double yd1, double xd1)
{
    double thetad1 = 1/(x*x+y*y)*(-y*xd1 + x*yd1);
    return thetad1;
}


double atan2diff2(double y, double x, double yd1, double xd1, double yd2, double xd2)
{
    double x2y2 = (x*x+y*y);
    double thetad2 = (2*y*(x*xd1*xd1+2*y*xd1*yd1-x*yd1*yd1))/(x2y2*x2y2)
            -(xd2*y-x*yd2+2*xd1*yd1)/x2y2;
    return thetad2;
}


int    RRd1Rd2fromEulerPre(double ** R, double ** Rd1, double ** Rd2,
        double * euler, double * eulerd1, double * eulerd2)
{
    double phi = euler[0];
    double the = euler[1];
    double psi = euler[2];
    
    double H = eulerd1[0];
    double I = eulerd1[1];
    double J = eulerd1[2];
    
    double K = eulerd2[0];
    double L = eulerd2[1];
    double M = eulerd2[2];
    
    double O = sin(phi);
    double N = cos(phi);
    double RR = sin(the);
    double S = cos(the);
    double T = sin(psi);
    double U = cos(psi);
    /* substitution--------- */
    double a = S*T;
    double b = U*RR;
    double c = O*T;
    double d = N*b;
    double e = N*U;
    double f = S*O;
    double g = N*T;
    double h = U*O;
    double ii = U*S;
    double jj = T*RR;
    double kk = O*RR;
    double l = N*S;
    double m = N*RR;
    double n = I*S;
    double o = I*N;
    double p = J*J;
    double q = I*I;
    double r = I*U;
    double s = J*I;
    double t = H*H;
    double u = H*I;
    double v = H*J;
    double w = L*S;
    double x = M*RR;
    double y = K*RR;
    double z = c*RR;
    double A = g*RR;
    double B = h*RR;
    double C = e*S;
    double D = n*c;
    double E = r*f;
    double F = o*a;
    
    /*
    R = 
      [ ii, B - g, c + e*RR;
        a , e + z, A- h    ;
       -RR,     f,    l    ];
    */
    R[0][0] = ii;
    R[0][1] = B - g;
    R[0][2] = c + e*RR;
    R[1][0] = a;
    R[1][1] = e + z;
    R[1][2] = A - h;
    R[2][0] = -RR;
    R[2][1] = f;
    R[2][2] = l;
    /*
    Rd1 = [-J*a-I*b  , H*(c+d)-J*(e+z)+E, H*(g-B)+J*(h-A)+n*e;
            J*ii-I*jj, D-J*(g-B)-H*(h-A), J*(c+d)-H*(e+z)+F  ;
           -n        , H*l-I*kk         , -H*f-I*m           ];
    */
    Rd1[0][0] = -J*a-I*b;
    Rd1[0][1] = H*(c+d)-J*(e+z)+E;
    Rd1[0][2] = H*(g-B)+J*(h-A)+n*e;
    Rd1[1][0] = J*ii-I*jj;
    Rd1[1][1] = D-J*(g-B)-H*(h-A);
    Rd1[1][2] = J*(c+d)-H*(e+z)+F;
    Rd1[2][0] = -n;
    Rd1[2][1] = H*l-I*kk;
    Rd1[2][2] = -H*f-I*m;
    /*
    Rd2 = ...
            [-ii*(p+q)+2*jj*s-M*a-L*b,...
            K*(c+d)-M*e+(t+p)*g+L*U*f-x*c-(t+p+q)*B+2*(v*(h-A)+u*C-J*D),...
            (K-x)*g+M*h-(t+p)*c+w*e-y*h-(t+p+q)*d+2*(v*(e+z)-H*E-J*F);
    -a*(p+q)-2*b*s+M*ii-L*jj,...
            2*v*c-(M-y)*g-(t+p)*e-(K-x)*h+w*c-(t+p+q)*z+2*(v*d+H*F+J*E),...
            (M-y)*c-K*e+(t+p)*h+M*d+L*N*a-(t+p+q)*A+2*(v*(g-B)+s*C-H*D);
    RR*q-w,-f*(t+q)-2*m*u+K*l-L*kk,-l*(t+q)+2*kk*u-K*f-L*m];
     */
    Rd2[0][0] = -ii*(p+q)+2*jj*s-M*a-L*b;
    Rd2[0][1] = K*(c+d)-M*e+(t+p)*g+L*U*f-x*c-(t+p+q)*B+2*(v*(h-A)+u*C-J*D);
    Rd2[0][2] = (K-x)*g+M*h-(t+p)*c+w*e-y*h-(t+p+q)*d+2*(v*(e+z)-H*E-J*F);
    Rd2[1][0] = -a*(p+q)-2*b*s+M*ii-L*jj;
    Rd2[1][1] = 2*v*c-(M-y)*g-(t+p)*e-(K-x)*h+w*c-(t+p+q)*z+2*(v*d+H*F+J*E);
    Rd2[1][2] = (M-y)*c-K*e+(t+p)*h+M*d+L*N*a-(t+p+q)*A+2*(v*(g-B)+s*C-H*D);
    Rd2[2][0] = RR*q-w;
    Rd2[2][1] = -f*(t+q)-2*m*u+K*l-L*kk;
    Rd2[2][2] = -l*(t+q)+2*kk*u-K*f-L*m;
    
    return 0;
}