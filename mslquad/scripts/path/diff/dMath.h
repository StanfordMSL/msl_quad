/**************************************************************************
   File Name      : dMath.h
   Author         : Dingjiang Zhou
                    Boston University, Boston, 02215
   Contact        : zhoudingjiang@gmail.com
   Create Time    : Mon, Aug. 19th, 2013. 05:16:36 PM
   Last Modified  : Mon, Aug. 19th, 2013. 05:16:36 PM
**************************************************************************/
#ifndef __DMATH_H__
#define __DMATH_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef uchar
#define uchar unsigned char
#endif
/* functions declaration */
void     vecPrint(double*,uchar);
void     mtrxPrint(double**,uchar,uchar);
int      vecMtrxp(double*,double*,double**,uchar,uchar);
double  *vecMtrx(double*,double**,uchar,uchar);
double  *vecMtrx3(double*,double**);
int      vecMtrx3p(double*,double*,double**);
double  *vecBlk(double*,uchar,uchar);
int      vecBlkp(double*,double*,uchar,uchar);
double  *mtrxVec(double**,uchar,uchar,double*);
double  *mtrxVec3(double**,double*);
int      mtrxVec3p(double*,double**,double*);
double  *mtrxVec12(double**,double*);
int      mtrxVec12p(double*,double**,double*);
int      mtrxVec12pMxOdr(double*,double*,double*);
double **mtrxMtrx(double**,uchar,uchar,double**,uchar,uchar);
double **mtrxMtrx3(double**,double**);
int      mtrxMtrx3p(double**,double**,double**);
int      mtrxMtrx6p(double**,double**,double**);
double **mtrxMtrx12(double**,double**);
int      mtrxMtrx12p(double**,double**,double**);
int      mtrxMtrx12pMxOdrFront(double**,double*,double**);
int      mtrxMtrx12pMxOdrAfter(double**,double**,double*);
int      mtrxMtrx12pMxOdrOutAfter(double*,double**,double*);
double **mtrxMtrxMtrx(double**,uchar,uchar,double**,uchar,uchar,double**,uchar,uchar);
double **mtrxMtrxMtrx3(double**,double**,double**);
int      mtrxMtrxMtrx3p(double**,double**,double**,double**);
int      mtrxMtrxMtrx6p(double**,double**,double**,double**);
double **mtrxMtrxMtrx12(double**,double**,double**);
int      mtrxMtrxMtrx12p(double**,double**,double**,double**);
double **mtrxTranp(double**,uchar,uchar);
int      mtrxTranpp(double**,double**,uchar,uchar);
double **mtrxTranp3(double**);
int      mtrxTranp3p(double**,double**);
double **mtrxTranp12(double**);
int      mtrxTranp12p(double**,double**);
int      mtrxTranp12pMxOdrIn(double**,double*);
double   vecMtrxCol(double*,uchar,double**,uchar);
double   vecMtrxCol3(double*,double**,uchar);
double  *normalize(double*,uchar);
double  *normalize3(double*);
int      normalize3p(double*,double*);
int      normalize3self(double*);
double   norm1(double);
double   norm3(double*);
double   distance3(double*,double*);
double  *vecCross(double*,double*);
int      vecCrossp(double*,double*,double*);
double **threeVec2Mtrx(double*,double*,double*,uchar);
double **threeVec2Mtrx3(double*,double*,double*);
int      threeVec2Mtrx3p(double**,double*,double*,double*);
double   vecDot(double*,double*,uchar);
double   vecDot3(double*,double*);
double  *vecXscale(double*,double,uchar);
double  *vecXscale3(double*,double);
int      vecXscale3p(double*,double*,double);
double  *vecAdd(double*,double*,uchar);
double  *vecAdd3(double*,double*);
int      vecAdd3p(double*,double*,double*);
double  *vecAdd12(double*,double*);
int      vecAdd12p(double*,double*,double*);
double  *vecSub(double*,double*,uchar);
double  *vecSub3(double*,double*);
int      vecSub3p(double*,double*,double*);
double  *vecSub12(double*,double*);
int      vecSub12p(double*,double*,double*);
double **mtrxFromMtrxBlk(double**,uchar,uchar,uchar,uchar,uchar,uchar);
int      mtrxFromMtrxBlkp(double**,double**,uchar,uchar,uchar,uchar,uchar,uchar);
double **mtrxAdd(double**,double**,uchar,uchar);
double **mtrxAdd3(double**,double**);
int      mtrxAdd3p(double**,double**,double**);
double **mtrxAdd12(double**,double**);
int      mtrxAdd12p(double**,double**,double**);
double **mtrxSub(double**,double**,uchar,uchar);
double **mtrxSub3(double**,double**);
int      mtrxSub3p(double**,double**,double**);
double **mtrxSub12(double**,double**);
int      mtrxSub12p(double**,double**,double**);
double **mtrxOpp(double**,uchar,uchar);
int      mtrxOppp(double**,double**,uchar,uchar);
double **mtrx2Dinv(double**);
int      mtrx2Dinvp(double**,double**);
double **mtrx3Dinv(double**);
int      mtrx3Dinvp(double**,double**);
double **mtrxBlkInv(double**,uchar);
/*int      mtrxBlkInvp(double **,double**,uchar); useless to have */

double **rotZYX(double*);
int      rotZYXp(double**,double*);
double  *rot2eulerZYX(double **);
int      rot2eulerZYXp(double*,double **);
double **kinEqZYX(double*); /* see : function elrd1 = wbToElrd1(elr, wb) */
int      kinEqZYXp(double**,double*);
int      wbToElrd1p(double*,double*,double*);
double **invKinZYX(double*);
int      invKinZYXp(double**,double*);
double *invTensor(double**);
int     invTensorp(double*,double**);
double **mtrxEye(uchar);

/* added at Sun, Jul. 27th, 2014. 11:42:56 PM */
int mtrx6Dinvp(double**,double**);
int mtrx9Dinvp(double**,double**);
int mtrx12Dinvp(double**,double**);
int mtrx18Dinvp(double**,double**);
int mtrxMtrx9p(double**,double**,double**);
int mtrxMtrx18p(double**,double**,double**);
int mtrxMtrx6333p(double**,double**,double**);
int mtrxMtrx3336p(double**,double**,double**);
int mtrxMtrx3663p(double**,double**,double**);
int mtrxMtrx6336p(double**,double**,double**);
int mtrxMtrx3666p(double**,double**,double**);
int mtrxMtrx6663p(double**,double**,double**);
/* added at Sat, Aug. 02nd, 2014. 09:05:01 PM */
int mtrxXtensorp(double**,double**,double*);
/* int tensorp(double**,double*);*/


int mtrxMtrx2p(double**,double**,double**);
int mtrx2Dinvp(double**,double**);
int mtrx4Dinvp(double**,double**);

double max(double*,uchar);
#endif


/* --------------------------- end of file ----------------------------- */



