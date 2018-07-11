/* Produced by CVXGEN, 2018-05-12 17:48:38 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double Q_1[16];
  double Q_2[16];
  double Q_3[16];
  double Q_4[16];
  double Q_5[16];
  double Q_6[16];
  double Q_7[16];
  double Q_8[16];
  double wpts[9];
  double tf0123_1[4];
  double tf4567_1[4];
  double tf0123_2[4];
  double tf4567_2[4];
  double tf0123_3[4];
  double tf4567_3[4];
  double tf0123_4[4];
  double tf4567_4[4];
  double tf0123_5[4];
  double tf4567_5[4];
  double tf0123_6[4];
  double tf4567_6[4];
  double tf0123_7[4];
  double tf4567_7[4];
  double tf0123_8[4];
  double tf4567_8[4];
  double final_v_c0123_3[4];
  double final_v_c4567_3[4];
  double final_a_c0123_3[4];
  double final_a_c4567_3[4];
  double final_j_c0123_3[4];
  double final_j_c4567_3[4];
  double final_s_c0123_3[4];
  double final_s_c4567_3[4];
  double final_v_c0123_4[4];
  double final_v_c4567_4[4];
  double final_a_c0123_4[4];
  double final_a_c4567_4[4];
  double final_j_c0123_4[4];
  double final_j_c4567_4[4];
  double final_s_c0123_4[4];
  double final_s_c4567_4[4];
  double final_v_c0123_5[4];
  double final_v_c4567_5[4];
  double final_a_c0123_5[4];
  double final_a_c4567_5[4];
  double final_j_c0123_5[4];
  double final_j_c4567_5[4];
  double final_s_c0123_5[4];
  double final_s_c4567_5[4];
  double final_v_c0123_6[4];
  double final_v_c4567_6[4];
  double final_a_c0123_6[4];
  double final_a_c4567_6[4];
  double final_j_c0123_6[4];
  double final_j_c4567_6[4];
  double final_s_c0123_6[4];
  double final_s_c4567_6[4];
  double final_v_c0123_7[4];
  double final_v_c4567_7[4];
  double final_a_c0123_7[4];
  double final_a_c4567_7[4];
  double final_j_c0123_7[4];
  double final_j_c4567_7[4];
  double final_s_c0123_7[4];
  double final_s_c4567_7[4];
  double final_v_c0123_8[4];
  double final_v_c4567_8[4];
  double final_a_c0123_8[4];
  double final_a_c4567_8[4];
  double final_j_c0123_8[4];
  double final_j_c4567_8[4];
  double final_s_c0123_8[4];
  double final_s_c4567_8[4];
  double v_cont_c0123_1[4];
  double v_cont_c4567_1[4];
  double a_cont_c0123_1[4];
  double a_cont_c4567_1[4];
  double j_cont_c0123_1[4];
  double j_cont_c4567_1[4];
  double s_cont_c0123_1[4];
  double s_cont_c4567_1[4];
  double ds_cont_c0123_1[4];
  double ds_cont_c4567_1[4];
  double v_cont_c0123_2[4];
  double v_cont_c4567_2[4];
  double a_cont_c0123_2[4];
  double a_cont_c4567_2[4];
  double j_cont_c0123_2[4];
  double j_cont_c4567_2[4];
  double s_cont_c0123_2[4];
  double s_cont_c4567_2[4];
  double ds_cont_c0123_2[4];
  double ds_cont_c4567_2[4];
  double v_cont_c0123_3[4];
  double v_cont_c4567_3[4];
  double a_cont_c0123_3[4];
  double a_cont_c4567_3[4];
  double j_cont_c0123_3[4];
  double j_cont_c4567_3[4];
  double s_cont_c0123_3[4];
  double s_cont_c4567_3[4];
  double ds_cont_c0123_3[4];
  double ds_cont_c4567_3[4];
  double v_cont_c0123_4[4];
  double v_cont_c4567_4[4];
  double a_cont_c0123_4[4];
  double a_cont_c4567_4[4];
  double j_cont_c0123_4[4];
  double j_cont_c4567_4[4];
  double s_cont_c0123_4[4];
  double s_cont_c4567_4[4];
  double ds_cont_c0123_4[4];
  double ds_cont_c4567_4[4];
  double v_cont_c0123_5[4];
  double v_cont_c4567_5[4];
  double a_cont_c0123_5[4];
  double a_cont_c4567_5[4];
  double j_cont_c0123_5[4];
  double j_cont_c4567_5[4];
  double s_cont_c0123_5[4];
  double s_cont_c4567_5[4];
  double ds_cont_c0123_5[4];
  double ds_cont_c4567_5[4];
  double v_cont_c0123_6[4];
  double v_cont_c4567_6[4];
  double a_cont_c0123_6[4];
  double a_cont_c4567_6[4];
  double j_cont_c0123_6[4];
  double j_cont_c4567_6[4];
  double s_cont_c0123_6[4];
  double s_cont_c4567_6[4];
  double ds_cont_c0123_6[4];
  double ds_cont_c4567_6[4];
  double v_cont_c0123_7[4];
  double v_cont_c4567_7[4];
  double a_cont_c0123_7[4];
  double a_cont_c4567_7[4];
  double j_cont_c0123_7[4];
  double j_cont_c4567_7[4];
  double s_cont_c0123_7[4];
  double s_cont_c4567_7[4];
  double ds_cont_c0123_7[4];
  double ds_cont_c4567_7[4];
  double *Q[9];
  double *tf0123[9];
  double *tf4567[9];
  double *final_v_c0123[9];
  double *final_v_c4567[9];
  double *final_a_c0123[9];
  double *final_a_c4567[9];
  double *final_j_c0123[9];
  double *final_j_c4567[9];
  double *final_s_c0123[9];
  double *final_s_c4567[9];
  double *v_cont_c0123[8];
  double *v_cont_c4567[8];
  double *a_cont_c0123[8];
  double *a_cont_c4567[8];
  double *j_cont_c0123[8];
  double *j_cont_c4567[8];
  double *s_cont_c0123[8];
  double *s_cont_c4567[8];
  double *ds_cont_c0123[8];
  double *ds_cont_c4567[8];
} Params;
typedef struct Vars_t {
  double *c4567_1; /* 4 rows. */
  double *c4567_2; /* 4 rows. */
  double *c4567_3; /* 4 rows. */
  double *c4567_4; /* 4 rows. */
  double *c4567_5; /* 4 rows. */
  double *c4567_6; /* 4 rows. */
  double *c4567_7; /* 4 rows. */
  double *c4567_8; /* 4 rows. */
  double *c0123_1; /* 4 rows. */
  double *c0123_2; /* 4 rows. */
  double *c0123_3; /* 4 rows. */
  double *c0123_4; /* 4 rows. */
  double *c0123_5; /* 4 rows. */
  double *c0123_6; /* 4 rows. */
  double *c0123_7; /* 4 rows. */
  double *c0123_8; /* 4 rows. */
  double *c4567[9];
  double *c0123[9];
} Vars;
typedef struct Workspace_t {
  double *h;
  double *s_inv;
  double *s_inv_z;
  double b[79];
  double q[64];
  double rhs[143];
  double x[143];
  double *s;
  double *z;
  double *y;
  double lhs_aff[143];
  double lhs_cc[143];
  double buffer[143];
  double buffer2[143];
  double KKT[663];
  double L[1034];
  double d[143];
  double v[143];
  double d_inv[143];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif
