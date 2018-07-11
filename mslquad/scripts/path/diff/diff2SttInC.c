/*****************************************************************************************
 * File Name     : diff2SttInC.c
 * Author        : Dingjiang Zhou
 * Boston University, Boston, 02215
 * Email         : zdj@bu.edu zhoudingjiang@gmail.com
 * Create Time   : Sun, Aug. 10th, 2014. 10:45:18 PM
 * Last Modified : Sun, Aug. 10th, 2014. 10:45:18 PM
 * Purpose       : C version for diffThrm2StatesInputs.m
 * ------------------------------------------------------------------------------------------
 * INPUTS
 *
 * OUTPUTS
 *
 *****************************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "quadDef.h"
#include "dMath.h"
#include "diffMath.h" /* remember to include the file, otherwise won't give you a good result */
#ifndef uchar
#define uchar unsigned char
#endif

/* function [state, input, annx] = diffThrm2StatesInputs(param, flatout) */
int diff2SttInp(States * state, Inputs * input, double ** Rd1, double ** Rd2,    /* output */
                double * wbd1, double * euler, double * eulerd1, double * eulerd2, /* output */
                Params * param, FlatOut * flatout)                                 /* input */
{
    uchar i,j;
    uchar sizeD3 = sizeof(double)*3;
    uchar sizeDpr3 = sizeof(double*)*3;
    double g = 9.8;    
    double sig4   = flatout->sig[3];
    double sigd14 = flatout->sigd1[3];
    double sigd21 = flatout->sigd2[0];
    double sigd22 = flatout->sigd2[1];
    double sigd23 = flatout->sigd2[2];
    double sigd24 = flatout->sigd2[3];
    double sigd31 = flatout->sigd3[0];
    double sigd32 = flatout->sigd3[1];
    double sigd33 = flatout->sigd3[2];
    double sigd41 = flatout->sigd4[0];
    double sigd42 = flatout->sigd4[1];
    double sigd43 = flatout->sigd4[2];
    
    /* beta(1:7) and yaw ------------------------------ */
    for(i=0;i<3;i++){
    state->v[i] = flatout->sigd1[i];
    state->h[i] = flatout->sig[i];
    }
    euler[2]   = sig4;
    eulerd1[2] = sigd14;
    eulerd2[2] = sigd24;
    
    /* theta = beta(8) and phi = beta(9) and R ------------------------------ */
    double Spsi = sin(sig4);
    double Cpsi = cos(sig4);
    
    double betaA = -Cpsi*sigd21 - Spsi*sigd22;
    double betaB = -sigd23 + g;
    double betaC = -Spsi*sigd21 + Cpsi*sigd22;
    double PP = betaA*betaA;
    double QQ = betaB*betaB;
    double betaD = sqrt(PP + QQ);
    
    double the = atan2(betaA,betaB);
    double phi = atan2(betaC,betaD);
    
    euler[0] = phi;
    euler[1] = the;
    /* input, fz = gama(1) ----------------------------------------------- */
    input->fz = -param->m*sqrt(sigd21*sigd21 + sigd22*sigd22 + (sigd23-g)*(sigd23-g));
    /* derivatives for betaA, betaB, betaC and betaD ------------------------------ */
    double AA = sigd22*Cpsi;
    double BB = sigd21*Spsi;
    double CC = sigd31*Cpsi;
    double DD = sigd32*Spsi;
    double EE = sigd32*Cpsi;
    double FF = sigd21*Cpsi;
    double GG = sigd22*Spsi;
    double HH = sigd31*Spsi;
    double KK = sigd14*sigd14;
    double betaAd1 = - sigd14*(AA - BB) - CC - DD;
    double betaBd1 = - sigd33;
    double betaCd1 = EE - sigd14*(FF + GG) - HH;
    double LL = betaA*betaAd1;
    double MM = betaB*betaBd1;
    double NN = sigd22*sigd24;
    double OO = sigd21*sigd24;
    double betaDd1 = 1/betaD*(LL + MM);
    double betaAd2 = 2*sigd14*(HH-EE) - (sigd42-OO)*Spsi - (NN+sigd41)*Cpsi + KK*(FF+GG);
    double betaBd2 = - sigd43;
    double betaCd2 = (sigd42-OO)*Cpsi - (sigd41+NN)*Spsi - 2*sigd14*(CC+DD) - KK*(AA-BB);
    double XX = betaAd2*betaA + betaBd2*betaB;
    double betaDd2 = (PP*(XX + betaBd1*betaBd1) - 2*LL*MM +  (XX + betaAd1*betaAd1)*QQ )/(betaD*betaD*betaD);
    
    /* derivatives for theta and phi -------------------------------------------------- */
    double thed1 = atan2diff1(betaA,betaB,betaAd1,betaBd1);
    double thed2 = atan2diff2(betaA,betaB,betaAd1,betaBd1,betaAd2,betaBd2);
    double phid1 = atan2diff1(betaC,betaD,betaCd1,betaDd1);
    double phid2 = atan2diff2(betaC,betaD,betaCd1,betaDd1,betaCd2,betaDd2);
    eulerd1[0] = phid1;
    eulerd1[1] = thed1;
    eulerd2[0] = phid2;
    eulerd2[1] = thed2;
    /* --------------------------------------------------------------------------------- */
    /* R,Rd1,Rd2 from euler */
    RRd1Rd2fromEulerPre(state->R,Rd1,Rd2,euler,eulerd1,eulerd2);
    double **RT = (double**)malloc(sizeDpr3);
    double **Omega = (double**)malloc(sizeDpr3);
    double **Omegad1 = (double**)malloc(sizeDpr3);
    for(i=0;i<3;i++){
        RT[i] = (double*)malloc(sizeD3);
        Omega[i] = (double*)malloc(sizeD3);
        Omegad1[i] = (double*)malloc(sizeD3);
        for(j=0;j<3;j++){
            RT[i][j] = state->R[j][i];
        }
    }
    /* \dot R  = R * \Omega ==> */
    /* Omega = state.R'*Rd1; */
    mtrxMtrx3p(Omega,RT,Rd1);
    state->wb[0] = Omega[2][1];
    state->wb[1] = Omega[0][2];
    state->wb[2] = Omega[1][0];
    /* Omegad1 = Rd1'*Rd1 + state.R'*Rd2; */
    /* use Omega as a temporary variable */
    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            Omega[i][j] = Rd1[j][i];
        }
    }
    mtrxMtrx3p(Omegad1,Omega,Rd1);
    wbd1[0] = Omegad1[2][1];
    wbd1[1] = Omegad1[0][2];
    wbd1[2] = Omegad1[1][0];
    mtrxMtrx3p(Omegad1,RT,Rd2);
    wbd1[0] += Omegad1[2][1];
    wbd1[1] += Omegad1[0][2];
    wbd1[2] += Omegad1[1][0];
    
	/* input, tau = gama(2) ----------------------------------------------- */
    /* input.tau = param->J*wbd1 + Omega*param->J*state.wb; */
    /* Omega was changed, so change it back */
    mtrxMtrx3p(Omega,RT,Rd1);
    input->tau[0] = param->J[0][0]*wbd1[0] + param->J[0][1]*wbd1[1] + param->J[0][2]*wbd1[2];
    input->tau[1] = param->J[1][0]*wbd1[0] + param->J[1][1]*wbd1[1] + param->J[1][2]*wbd1[2];
    input->tau[2] = param->J[2][0]*wbd1[0] + param->J[2][1]*wbd1[1] + param->J[2][2]*wbd1[2];
    mtrxMtrx3p(Omegad1,Omega,param->J);
    input->tau[0] += Omegad1[0][0]*state->wb[0] + Omegad1[0][1]*state->wb[1] + Omegad1[0][2]*state->wb[2];
    input->tau[1] += Omegad1[1][0]*state->wb[0] + Omegad1[1][1]*state->wb[1] + Omegad1[1][2]*state->wb[2];
    input->tau[2] += Omegad1[2][0]*state->wb[0] + Omegad1[2][1]*state->wb[1] + Omegad1[2][2]*state->wb[2];
    /* free some space */
    for(i=0;i<3;i++){
        free(Omega[i]);
        free(Omegad1[i]);
        free(RT[i]);
    }
    free(Omega);
    free(Omegad1);
    free(RT);
    
    return 0;
}