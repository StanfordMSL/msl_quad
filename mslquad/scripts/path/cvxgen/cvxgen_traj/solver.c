/* Produced by CVXGEN, 2018-05-12 17:48:37 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.c. */
/* Description: Main solver file. */
#include "solver.h"
double eval_gap(void) {
  int i;
  double gap;
  gap = 0;
  for (i = 0; i < 0; i++)
    gap += work.z[i]*work.s[i];
  return gap;
}
void set_defaults(void) {
  settings.resid_tol = 1e-6;
  settings.eps = 1e-4;
  settings.max_iters = 25;
  settings.refine_steps = 1;
  settings.s_init = 1;
  settings.z_init = 1;
  settings.debug = 0;
  settings.verbose = 1;
  settings.verbose_refinement = 0;
  settings.better_start = 1;
  settings.kkt_reg = 1e-7;
}
void setup_pointers(void) {
  work.y = work.x + 64;
  work.s = work.x + 143;
  work.z = work.x + 143;
  vars.c0123_1 = work.x + 0;
  vars.c0123_2 = work.x + 4;
  vars.c0123_3 = work.x + 8;
  vars.c0123_4 = work.x + 12;
  vars.c0123_5 = work.x + 16;
  vars.c0123_6 = work.x + 20;
  vars.c0123_7 = work.x + 24;
  vars.c0123_8 = work.x + 28;
  vars.c4567_1 = work.x + 32;
  vars.c4567_2 = work.x + 36;
  vars.c4567_3 = work.x + 40;
  vars.c4567_4 = work.x + 44;
  vars.c4567_5 = work.x + 48;
  vars.c4567_6 = work.x + 52;
  vars.c4567_7 = work.x + 56;
  vars.c4567_8 = work.x + 60;
}
void setup_indexed_params(void) {
  /* In CVXGEN, you can say */
  /*   parameters */
  /*     A[i] (5,3), i=1..4 */
  /*   end */
  /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
  /* vector of doubles. */
  /* If you access parameters that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  params.Q[1] = params.Q_1;
  params.Q[2] = params.Q_2;
  params.Q[3] = params.Q_3;
  params.Q[4] = params.Q_4;
  params.Q[5] = params.Q_5;
  params.Q[6] = params.Q_6;
  params.Q[7] = params.Q_7;
  params.Q[8] = params.Q_8;
  params.tf0123[1] = params.tf0123_1;
  params.tf4567[1] = params.tf4567_1;
  params.tf0123[2] = params.tf0123_2;
  params.tf4567[2] = params.tf4567_2;
  params.tf0123[3] = params.tf0123_3;
  params.tf4567[3] = params.tf4567_3;
  params.tf0123[4] = params.tf0123_4;
  params.tf4567[4] = params.tf4567_4;
  params.tf0123[5] = params.tf0123_5;
  params.tf4567[5] = params.tf4567_5;
  params.tf0123[6] = params.tf0123_6;
  params.tf4567[6] = params.tf4567_6;
  params.tf0123[7] = params.tf0123_7;
  params.tf4567[7] = params.tf4567_7;
  params.tf0123[8] = params.tf0123_8;
  params.tf4567[8] = params.tf4567_8;
  params.final_v_c0123[3] = params.final_v_c0123_3;
  params.final_v_c4567[3] = params.final_v_c4567_3;
  params.final_a_c0123[3] = params.final_a_c0123_3;
  params.final_a_c4567[3] = params.final_a_c4567_3;
  params.final_j_c0123[3] = params.final_j_c0123_3;
  params.final_j_c4567[3] = params.final_j_c4567_3;
  params.final_s_c0123[3] = params.final_s_c0123_3;
  params.final_s_c4567[3] = params.final_s_c4567_3;
  params.final_v_c0123[4] = params.final_v_c0123_4;
  params.final_v_c4567[4] = params.final_v_c4567_4;
  params.final_a_c0123[4] = params.final_a_c0123_4;
  params.final_a_c4567[4] = params.final_a_c4567_4;
  params.final_j_c0123[4] = params.final_j_c0123_4;
  params.final_j_c4567[4] = params.final_j_c4567_4;
  params.final_s_c0123[4] = params.final_s_c0123_4;
  params.final_s_c4567[4] = params.final_s_c4567_4;
  params.final_v_c0123[5] = params.final_v_c0123_5;
  params.final_v_c4567[5] = params.final_v_c4567_5;
  params.final_a_c0123[5] = params.final_a_c0123_5;
  params.final_a_c4567[5] = params.final_a_c4567_5;
  params.final_j_c0123[5] = params.final_j_c0123_5;
  params.final_j_c4567[5] = params.final_j_c4567_5;
  params.final_s_c0123[5] = params.final_s_c0123_5;
  params.final_s_c4567[5] = params.final_s_c4567_5;
  params.final_v_c0123[6] = params.final_v_c0123_6;
  params.final_v_c4567[6] = params.final_v_c4567_6;
  params.final_a_c0123[6] = params.final_a_c0123_6;
  params.final_a_c4567[6] = params.final_a_c4567_6;
  params.final_j_c0123[6] = params.final_j_c0123_6;
  params.final_j_c4567[6] = params.final_j_c4567_6;
  params.final_s_c0123[6] = params.final_s_c0123_6;
  params.final_s_c4567[6] = params.final_s_c4567_6;
  params.final_v_c0123[7] = params.final_v_c0123_7;
  params.final_v_c4567[7] = params.final_v_c4567_7;
  params.final_a_c0123[7] = params.final_a_c0123_7;
  params.final_a_c4567[7] = params.final_a_c4567_7;
  params.final_j_c0123[7] = params.final_j_c0123_7;
  params.final_j_c4567[7] = params.final_j_c4567_7;
  params.final_s_c0123[7] = params.final_s_c0123_7;
  params.final_s_c4567[7] = params.final_s_c4567_7;
  params.final_v_c0123[8] = params.final_v_c0123_8;
  params.final_v_c4567[8] = params.final_v_c4567_8;
  params.final_a_c0123[8] = params.final_a_c0123_8;
  params.final_a_c4567[8] = params.final_a_c4567_8;
  params.final_j_c0123[8] = params.final_j_c0123_8;
  params.final_j_c4567[8] = params.final_j_c4567_8;
  params.final_s_c0123[8] = params.final_s_c0123_8;
  params.final_s_c4567[8] = params.final_s_c4567_8;
  params.v_cont_c0123[1] = params.v_cont_c0123_1;
  params.v_cont_c4567[1] = params.v_cont_c4567_1;
  params.a_cont_c0123[1] = params.a_cont_c0123_1;
  params.a_cont_c4567[1] = params.a_cont_c4567_1;
  params.j_cont_c0123[1] = params.j_cont_c0123_1;
  params.j_cont_c4567[1] = params.j_cont_c4567_1;
  params.s_cont_c0123[1] = params.s_cont_c0123_1;
  params.s_cont_c4567[1] = params.s_cont_c4567_1;
  params.ds_cont_c0123[1] = params.ds_cont_c0123_1;
  params.ds_cont_c4567[1] = params.ds_cont_c4567_1;
  params.v_cont_c0123[2] = params.v_cont_c0123_2;
  params.v_cont_c4567[2] = params.v_cont_c4567_2;
  params.a_cont_c0123[2] = params.a_cont_c0123_2;
  params.a_cont_c4567[2] = params.a_cont_c4567_2;
  params.j_cont_c0123[2] = params.j_cont_c0123_2;
  params.j_cont_c4567[2] = params.j_cont_c4567_2;
  params.s_cont_c0123[2] = params.s_cont_c0123_2;
  params.s_cont_c4567[2] = params.s_cont_c4567_2;
  params.ds_cont_c0123[2] = params.ds_cont_c0123_2;
  params.ds_cont_c4567[2] = params.ds_cont_c4567_2;
  params.v_cont_c0123[3] = params.v_cont_c0123_3;
  params.v_cont_c4567[3] = params.v_cont_c4567_3;
  params.a_cont_c0123[3] = params.a_cont_c0123_3;
  params.a_cont_c4567[3] = params.a_cont_c4567_3;
  params.j_cont_c0123[3] = params.j_cont_c0123_3;
  params.j_cont_c4567[3] = params.j_cont_c4567_3;
  params.s_cont_c0123[3] = params.s_cont_c0123_3;
  params.s_cont_c4567[3] = params.s_cont_c4567_3;
  params.ds_cont_c0123[3] = params.ds_cont_c0123_3;
  params.ds_cont_c4567[3] = params.ds_cont_c4567_3;
  params.v_cont_c0123[4] = params.v_cont_c0123_4;
  params.v_cont_c4567[4] = params.v_cont_c4567_4;
  params.a_cont_c0123[4] = params.a_cont_c0123_4;
  params.a_cont_c4567[4] = params.a_cont_c4567_4;
  params.j_cont_c0123[4] = params.j_cont_c0123_4;
  params.j_cont_c4567[4] = params.j_cont_c4567_4;
  params.s_cont_c0123[4] = params.s_cont_c0123_4;
  params.s_cont_c4567[4] = params.s_cont_c4567_4;
  params.ds_cont_c0123[4] = params.ds_cont_c0123_4;
  params.ds_cont_c4567[4] = params.ds_cont_c4567_4;
  params.v_cont_c0123[5] = params.v_cont_c0123_5;
  params.v_cont_c4567[5] = params.v_cont_c4567_5;
  params.a_cont_c0123[5] = params.a_cont_c0123_5;
  params.a_cont_c4567[5] = params.a_cont_c4567_5;
  params.j_cont_c0123[5] = params.j_cont_c0123_5;
  params.j_cont_c4567[5] = params.j_cont_c4567_5;
  params.s_cont_c0123[5] = params.s_cont_c0123_5;
  params.s_cont_c4567[5] = params.s_cont_c4567_5;
  params.ds_cont_c0123[5] = params.ds_cont_c0123_5;
  params.ds_cont_c4567[5] = params.ds_cont_c4567_5;
  params.v_cont_c0123[6] = params.v_cont_c0123_6;
  params.v_cont_c4567[6] = params.v_cont_c4567_6;
  params.a_cont_c0123[6] = params.a_cont_c0123_6;
  params.a_cont_c4567[6] = params.a_cont_c4567_6;
  params.j_cont_c0123[6] = params.j_cont_c0123_6;
  params.j_cont_c4567[6] = params.j_cont_c4567_6;
  params.s_cont_c0123[6] = params.s_cont_c0123_6;
  params.s_cont_c4567[6] = params.s_cont_c4567_6;
  params.ds_cont_c0123[6] = params.ds_cont_c0123_6;
  params.ds_cont_c4567[6] = params.ds_cont_c4567_6;
  params.v_cont_c0123[7] = params.v_cont_c0123_7;
  params.v_cont_c4567[7] = params.v_cont_c4567_7;
  params.a_cont_c0123[7] = params.a_cont_c0123_7;
  params.a_cont_c4567[7] = params.a_cont_c4567_7;
  params.j_cont_c0123[7] = params.j_cont_c0123_7;
  params.j_cont_c4567[7] = params.j_cont_c4567_7;
  params.s_cont_c0123[7] = params.s_cont_c0123_7;
  params.s_cont_c4567[7] = params.s_cont_c4567_7;
  params.ds_cont_c0123[7] = params.ds_cont_c0123_7;
  params.ds_cont_c4567[7] = params.ds_cont_c4567_7;
}
void setup_indexed_optvars(void) {
  /* In CVXGEN, you can say */
  /*   variables */
  /*     x[i] (5), i=2..4 */
  /*   end */
  /* This function sets up x[3] to be a pointer to x_3, which is a length-5 */
  /* vector of doubles. */
  /* If you access variables that you haven't defined in CVXGEN, the result */
  /* is undefined. */
  vars.c4567[1] = vars.c4567_1;
  vars.c4567[2] = vars.c4567_2;
  vars.c4567[3] = vars.c4567_3;
  vars.c4567[4] = vars.c4567_4;
  vars.c4567[5] = vars.c4567_5;
  vars.c4567[6] = vars.c4567_6;
  vars.c4567[7] = vars.c4567_7;
  vars.c4567[8] = vars.c4567_8;
  vars.c0123[1] = vars.c0123_1;
  vars.c0123[2] = vars.c0123_2;
  vars.c0123[3] = vars.c0123_3;
  vars.c0123[4] = vars.c0123_4;
  vars.c0123[5] = vars.c0123_5;
  vars.c0123[6] = vars.c0123_6;
  vars.c0123[7] = vars.c0123_7;
  vars.c0123[8] = vars.c0123_8;
}
void setup_indexing(void) {
  setup_pointers();
  setup_indexed_params();
  setup_indexed_optvars();
}
void set_start(void) {
  int i;
  for (i = 0; i < 64; i++)
    work.x[i] = 0;
  for (i = 0; i < 79; i++)
    work.y[i] = 0;
  for (i = 0; i < 0; i++)
    work.s[i] = (work.h[i] > 0) ? work.h[i] : settings.s_init;
  for (i = 0; i < 0; i++)
    work.z[i] = settings.z_init;
}
double eval_objv(void) {
  int i;
  double objv;
  /* Borrow space in work.rhs. */
  multbyP(work.rhs, work.x);
  objv = 0;
  for (i = 0; i < 64; i++)
    objv += work.x[i]*work.rhs[i];
  objv *= 0.5;
  for (i = 0; i < 64; i++)
    objv += work.q[i]*work.x[i];
  objv += 0;
  return objv;
}
void fillrhs_aff(void) {
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = work.rhs;
  r2 = work.rhs + 64;
  r3 = work.rhs + 64;
  r4 = work.rhs + 64;
  /* r1 = -A^Ty - G^Tz - Px - q. */
  multbymAT(r1, work.y);
  multbymGT(work.buffer, work.z);
  for (i = 0; i < 64; i++)
    r1[i] += work.buffer[i];
  multbyP(work.buffer, work.x);
  for (i = 0; i < 64; i++)
    r1[i] -= work.buffer[i] + work.q[i];
  /* r2 = -z. */
  for (i = 0; i < 0; i++)
    r2[i] = -work.z[i];
  /* r3 = -Gx - s + h. */
  multbymG(r3, work.x);
  for (i = 0; i < 0; i++)
    r3[i] += -work.s[i] + work.h[i];
  /* r4 = -Ax + b. */
  multbymA(r4, work.x);
  for (i = 0; i < 79; i++)
    r4[i] += work.b[i];
}
void fillrhs_cc(void) {
  int i;
  double *r2;
  double *ds_aff, *dz_aff;
  double mu;
  double alpha;
  double sigma;
  double smu;
  double minval;
  r2 = work.rhs + 64;
  ds_aff = work.lhs_aff + 64;
  dz_aff = work.lhs_aff + 64;
  mu = 0;
  for (i = 0; i < 0; i++)
    mu += work.s[i]*work.z[i];
  /* Don't finish calculating mu quite yet. */
  /* Find min(min(ds./s), min(dz./z)). */
  minval = 0;
  for (i = 0; i < 0; i++)
    if (ds_aff[i] < minval*work.s[i])
      minval = ds_aff[i]/work.s[i];
  for (i = 0; i < 0; i++)
    if (dz_aff[i] < minval*work.z[i])
      minval = dz_aff[i]/work.z[i];
  /* Find alpha. */
  if (-1 < minval)
      alpha = 1;
  else
      alpha = -1/minval;
  sigma = 0;
  for (i = 0; i < 0; i++)
    sigma += (work.s[i] + alpha*ds_aff[i])*
      (work.z[i] + alpha*dz_aff[i]);
  sigma /= mu;
  sigma = sigma*sigma*sigma;
  /* Finish calculating mu now. */
  smu = sigma*mu;
  /* Fill-in the rhs. */
  for (i = 0; i < 64; i++)
    work.rhs[i] = 0;
  for (i = 64; i < 143; i++)
    work.rhs[i] = 0;
  for (i = 0; i < 0; i++)
    r2[i] = work.s_inv[i]*(smu - ds_aff[i]*dz_aff[i]);
}
void refine(double *target, double *var) {
  int i, j;
  double *residual = work.buffer;
  double norm2;
  double *new_var = work.buffer2;
  for (j = 0; j < settings.refine_steps; j++) {
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 143; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
#ifndef ZERO_LIBRARY_MODE
    if (settings.verbose_refinement) {
      if (j == 0)
        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
      else
        printf("After refinement we get squared norm %.6g.\n", norm2);
    }
#endif
    /* Solve to find new_var = KKT \ (target - A*var). */
    ldl_solve(residual, new_var);
    /* Update var += new_var, or var += KKT \ (target - A*var). */
    for (i = 0; i < 143; i++) {
      var[i] -= new_var[i];
    }
  }
#ifndef ZERO_LIBRARY_MODE
  if (settings.verbose_refinement) {
    /* Check the residual once more, but only if we're reporting it, since */
    /* it's expensive. */
    norm2 = 0;
    matrix_multiply(residual, var);
    for (i = 0; i < 143; i++) {
      residual[i] = residual[i] - target[i];
      norm2 += residual[i]*residual[i];
    }
    if (j == 0)
      printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
    else
      printf("After refinement we get squared norm %.6g.\n", norm2);
  }
#endif
}
double calc_ineq_resid_squared(void) {
  /* Calculates the norm ||-Gx - s + h||. */
  double norm2_squared;
  int i;
  /* Find -Gx. */
  multbymG(work.buffer, work.x);
  /* Add -s + h. */
  for (i = 0; i < 0; i++)
    work.buffer[i] += -work.s[i] + work.h[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 0; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];
  return norm2_squared;
}
double calc_eq_resid_squared(void) {
  /* Calculates the norm ||-Ax + b||. */
  double norm2_squared;
  int i;
  /* Find -Ax. */
  multbymA(work.buffer, work.x);
  /* Add +b. */
  for (i = 0; i < 79; i++)
    work.buffer[i] += work.b[i];
  /* Now find the squared norm. */
  norm2_squared = 0;
  for (i = 0; i < 79; i++)
    norm2_squared += work.buffer[i]*work.buffer[i];
  return norm2_squared;
}
void better_start(void) {
  /* Calculates a better starting point, using a similar approach to CVXOPT. */
  /* Not yet speed optimized. */
  int i;
  double *x, *s, *z, *y;
  double alpha;
  work.block_33[0] = -1;
  /* Make sure sinvz is 1 to make hijacked KKT system ok. */
  for (i = 0; i < 0; i++)
    work.s_inv_z[i] = 1;
  fill_KKT();
  ldl_factor();
  fillrhs_start();
  /* Borrow work.lhs_aff for the solution. */
  ldl_solve(work.rhs, work.lhs_aff);
  /* Don't do any refinement for now. Precision doesn't matter too much. */
  x = work.lhs_aff;
  s = work.lhs_aff + 64;
  z = work.lhs_aff + 64;
  y = work.lhs_aff + 64;
  /* Just set x and y as is. */
  for (i = 0; i < 64; i++)
    work.x[i] = x[i];
  for (i = 0; i < 79; i++)
    work.y[i] = y[i];
  /* Now complete the initialization. Start with s. */
  /* Must have alpha > max(z). */
  alpha = -1e99;
  for (i = 0; i < 0; i++)
    if (alpha < z[i])
      alpha = z[i];
  if (alpha < 0) {
    for (i = 0; i < 0; i++)
      work.s[i] = -z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 0; i++)
      work.s[i] = -z[i] + alpha;
  }
  /* Now initialize z. */
  /* Now must have alpha > max(-z). */
  alpha = -1e99;
  for (i = 0; i < 0; i++)
    if (alpha < -z[i])
      alpha = -z[i];
  if (alpha < 0) {
    for (i = 0; i < 0; i++)
      work.z[i] = z[i];
  } else {
    alpha += 1;
    for (i = 0; i < 0; i++)
      work.z[i] = z[i] + alpha;
  }
}
void fillrhs_start(void) {
  /* Fill rhs with (-q, 0, h, b). */
  int i;
  double *r1, *r2, *r3, *r4;
  r1 = work.rhs;
  r2 = work.rhs + 64;
  r3 = work.rhs + 64;
  r4 = work.rhs + 64;
  for (i = 0; i < 64; i++)
    r1[i] = -work.q[i];
  for (i = 0; i < 0; i++)
    r2[i] = 0;
  for (i = 0; i < 0; i++)
    r3[i] = work.h[i];
  for (i = 0; i < 79; i++)
    r4[i] = work.b[i];
}
long solve(void) {
  int i;
  int iter;
  double *dx, *ds, *dy, *dz;
  double minval;
  double alpha;
  work.converged = 0;
  setup_pointers();
  pre_ops();
#ifndef ZERO_LIBRARY_MODE
  if (settings.verbose)
    printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
  fillq();
  fillh();
  fillb();
  if (settings.better_start)
    better_start();
  else
    set_start();
  for (iter = 0; iter < settings.max_iters; iter++) {
    for (i = 0; i < 0; i++) {
      work.s_inv[i] = 1.0 / work.s[i];
      work.s_inv_z[i] = work.s_inv[i]*work.z[i];
    }
    work.block_33[0] = 0;
    fill_KKT();
    ldl_factor();
    /* Affine scaling directions. */
    fillrhs_aff();
    ldl_solve(work.rhs, work.lhs_aff);
    refine(work.rhs, work.lhs_aff);
    /* Centering plus corrector directions. */
    fillrhs_cc();
    ldl_solve(work.rhs, work.lhs_cc);
    refine(work.rhs, work.lhs_cc);
    /* Add the two together and store in aff. */
    for (i = 0; i < 143; i++)
      work.lhs_aff[i] += work.lhs_cc[i];
    /* Rename aff to reflect its new meaning. */
    dx = work.lhs_aff;
    ds = work.lhs_aff + 64;
    dz = work.lhs_aff + 64;
    dy = work.lhs_aff + 64;
    /* Find min(min(ds./s), min(dz./z)). */
    minval = 0;
    for (i = 0; i < 0; i++)
      if (ds[i] < minval*work.s[i])
        minval = ds[i]/work.s[i];
    for (i = 0; i < 0; i++)
      if (dz[i] < minval*work.z[i])
        minval = dz[i]/work.z[i];
    /* Find alpha. */
    if (-0.99 < minval)
      alpha = 1;
    else
      alpha = -0.99/minval;
    /* Update the primal and dual variables. */
    for (i = 0; i < 64; i++)
      work.x[i] += alpha*dx[i];
    for (i = 0; i < 0; i++)
      work.s[i] += alpha*ds[i];
    for (i = 0; i < 0; i++)
      work.z[i] += alpha*dz[i];
    for (i = 0; i < 79; i++)
      work.y[i] += alpha*dy[i];
    work.gap = eval_gap();
    work.eq_resid_squared = calc_eq_resid_squared();
    work.ineq_resid_squared = calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
    if (settings.verbose) {
      work.optval = eval_objv();
      printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
          iter+1, work.optval, work.gap, sqrt(work.eq_resid_squared),
          sqrt(work.ineq_resid_squared), alpha);
    }
#endif
    /* Test termination conditions. Requires optimality, and satisfied */
    /* constraints. */
    if (   (work.gap < settings.eps)
        && (work.eq_resid_squared <= settings.resid_tol*settings.resid_tol)
        && (work.ineq_resid_squared <= settings.resid_tol*settings.resid_tol)
       ) {
      work.converged = 1;
      work.optval = eval_objv();
      return iter+1;
    }
  }
  return iter;
}
