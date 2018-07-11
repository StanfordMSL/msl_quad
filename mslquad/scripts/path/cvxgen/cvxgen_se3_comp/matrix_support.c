/* Produced by CVXGEN, 2018-03-26 22:22:07 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(1)-rhs[1]*(1)-rhs[2]*(1)-rhs[3]*(1);
  lhs[1] = -rhs[0]*(params.W_row2[0])-rhs[1]*(params.W_row2[1])-rhs[2]*(params.W_row2[2])-rhs[3]*(params.W_row2[3]);
  lhs[2] = -rhs[0]*(params.W_row4[0])-rhs[1]*(params.W_row4[1])-rhs[2]*(params.W_row4[2])-rhs[3]*(params.W_row4[3]);
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(1)-rhs[1]*(params.W_row2[0])-rhs[2]*(params.W_row4[0]);
  lhs[1] = -rhs[0]*(1)-rhs[1]*(params.W_row2[1])-rhs[2]*(params.W_row4[1]);
  lhs[2] = -rhs[0]*(1)-rhs[1]*(params.W_row2[2])-rhs[2]*(params.W_row4[2]);
  lhs[3] = -rhs[0]*(1)-rhs[1]*(params.W_row2[3])-rhs[2]*(params.W_row4[3]);
  lhs[4] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.W_row3[0])-rhs[1]*(params.W_row3[1])-rhs[2]*(params.W_row3[2])-rhs[3]*(params.W_row3[3])-rhs[4]*(-1);
  lhs[1] = -rhs[0]*(-params.W_row3[0])-rhs[1]*(-params.W_row3[1])-rhs[2]*(-params.W_row3[2])-rhs[3]*(-params.W_row3[3])-rhs[4]*(-1);
  lhs[2] = -rhs[0]*(-1);
  lhs[3] = -rhs[1]*(-1);
  lhs[4] = -rhs[2]*(-1);
  lhs[5] = -rhs[3]*(-1);
  lhs[6] = -rhs[0]*(1);
  lhs[7] = -rhs[1]*(1);
  lhs[8] = -rhs[2]*(1);
  lhs[9] = -rhs[3]*(1);
  lhs[10] = -rhs[0]*(-params.W_row3[0])-rhs[1]*(-params.W_row3[1])-rhs[2]*(-params.W_row3[2])-rhs[3]*(-params.W_row3[3]);
  lhs[11] = -rhs[0]*(params.W_row3[0])-rhs[1]*(params.W_row3[1])-rhs[2]*(params.W_row3[2])-rhs[3]*(params.W_row3[3]);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(params.W_row3[0])-rhs[1]*(-params.W_row3[0])-rhs[2]*(-1)-rhs[6]*(1)-rhs[10]*(-params.W_row3[0])-rhs[11]*(params.W_row3[0]);
  lhs[1] = -rhs[0]*(params.W_row3[1])-rhs[1]*(-params.W_row3[1])-rhs[3]*(-1)-rhs[7]*(1)-rhs[10]*(-params.W_row3[1])-rhs[11]*(params.W_row3[1]);
  lhs[2] = -rhs[0]*(params.W_row3[2])-rhs[1]*(-params.W_row3[2])-rhs[4]*(-1)-rhs[8]*(1)-rhs[10]*(-params.W_row3[2])-rhs[11]*(params.W_row3[2]);
  lhs[3] = -rhs[0]*(params.W_row3[3])-rhs[1]*(-params.W_row3[3])-rhs[5]*(-1)-rhs[9]*(1)-rhs[10]*(-params.W_row3[3])-rhs[11]*(params.W_row3[3]);
  lhs[4] = -rhs[0]*(-1)-rhs[1]*(-1);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
}
void fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
  work.q[3] = 0;
  work.q[4] = 1;
}
void fillh(void) {
  work.h[0] = params.wdes[2];
  work.h[1] = -params.wdes[2];
  work.h[2] = -params.FMIN[0];
  work.h[3] = -params.FMIN[0];
  work.h[4] = -params.FMIN[0];
  work.h[5] = -params.FMIN[0];
  work.h[6] = params.FMAX[0];
  work.h[7] = params.FMAX[0];
  work.h[8] = params.FMAX[0];
  work.h[9] = params.FMAX[0];
  work.h[10] = -params.Wy_LB[0];
  work.h[11] = params.Wy_UB[0];
}
void fillb(void) {
  work.b[0] = params.wdes[0];
  work.b[1] = params.wdes[1];
  work.b[2] = params.wdes[3];
}
void pre_ops(void) {
}
