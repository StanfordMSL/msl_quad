/*****************************************************************************************
File Name     : diff2SttInC.h
Author        : Dingjiang Zhou
                Boston University, Boston, 02215
Email         : zdj@bu.edu zhoudingjiang@gmail.com
Create Time   : Mon, Aug. 11th, 2014. 01:05:40 AM
Last Modified :  
Purpose       : 
*****************************************************************************************/
#ifndef __DIFF2STTINC_H__
#define __DIFF2STTINC_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "quadDef.h"
#include "dMath.h"

#ifndef uchar
#define uchar unsigned char 
#endif

int diff2SttInp(States*,Inputs*,double**,double**,double*,       /* output */
                    double*,double*,double*,                         /* output */
                    Params*,FlatOut*);                                /* input */ 

#endif


/* ---------------------------------- end of file ------------------------------------- */