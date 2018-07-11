/**************************************************************************
   File Name      : DiffMath.h
   Author         : Dingjiang Zhou
                    Boston University, Boston, 02215
   Contact        : zhoudingjiang@gmail.com
   Create Time    : Sun, Aug. 10th, 2014. 10:45:18 PM
   Last Modified  : 
**************************************************************************/
#ifndef __DIFFMATH_H__
#define __DIFFMATH_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef uchar
#define uchar unsigned char
#endif

/* functions declaration */
double atan2diff1(double,double,double,double);
double atan2diff2(double,double,double,double,double,double);
int    RRd1Rd2fromEulerPre(double**,double**,double**,double*,double*,double*);


#endif