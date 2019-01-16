/* Produced by CVXGEN, 2018-03-26 22:22:07 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: csolve.c. */
/* Description: mex-able file for running cvxgen solver. */
#include "mex.h"
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  int i, j;
  mxArray *xm, *cell, *xm_cell;
  double *src;
  double *dest;
  double *dest_cell;
  int valid_vars;
  int steps;
  int this_var_errors;
  int warned_diags;
  int prepare_for_c = 0;
  int extra_solves;
  const char *status_names[] = {"optval", "gap", "steps", "converged"};
  mwSize dims1x1of1[1] = {1};
  mwSize dims[1];
  const char *var_names[] = {"f"};
  const int num_var_names = 1;
  /* Avoid compiler warnings of unused variables by using a dummy assignment. */
  warned_diags = j = 0;
  extra_solves = 0;
  set_defaults();
  /* Check we got the right number of arguments. */
  if (nrhs == 0)
    mexErrMsgTxt("Not enough arguments: You need to specify at least the parameters.\n");
  if (nrhs > 1) {
    /* Assume that the second argument is the settings. */
    if (mxGetField(prhs[1], 0, "eps") != NULL)
      settings.eps = *mxGetPr(mxGetField(prhs[1], 0, "eps"));
    if (mxGetField(prhs[1], 0, "max_iters") != NULL)
      settings.max_iters = *mxGetPr(mxGetField(prhs[1], 0, "max_iters"));
    if (mxGetField(prhs[1], 0, "refine_steps") != NULL)
      settings.refine_steps = *mxGetPr(mxGetField(prhs[1], 0, "refine_steps"));
    if (mxGetField(prhs[1], 0, "verbose") != NULL)
      settings.verbose = *mxGetPr(mxGetField(prhs[1], 0, "verbose"));
    if (mxGetField(prhs[1], 0, "better_start") != NULL)
      settings.better_start = *mxGetPr(mxGetField(prhs[1], 0, "better_start"));
    if (mxGetField(prhs[1], 0, "verbose_refinement") != NULL)
      settings.verbose_refinement = *mxGetPr(mxGetField(prhs[1], 0,
            "verbose_refinement"));
    if (mxGetField(prhs[1], 0, "debug") != NULL)
      settings.debug = *mxGetPr(mxGetField(prhs[1], 0, "debug"));
    if (mxGetField(prhs[1], 0, "kkt_reg") != NULL)
      settings.kkt_reg = *mxGetPr(mxGetField(prhs[1], 0, "kkt_reg"));
    if (mxGetField(prhs[1], 0, "s_init") != NULL)
      settings.s_init = *mxGetPr(mxGetField(prhs[1], 0, "s_init"));
    if (mxGetField(prhs[1], 0, "z_init") != NULL)
      settings.z_init = *mxGetPr(mxGetField(prhs[1], 0, "z_init"));
    if (mxGetField(prhs[1], 0, "resid_tol") != NULL)
      settings.resid_tol = *mxGetPr(mxGetField(prhs[1], 0, "resid_tol"));
    if (mxGetField(prhs[1], 0, "extra_solves") != NULL)
      extra_solves = *mxGetPr(mxGetField(prhs[1], 0, "extra_solves"));
    else
      extra_solves = 0;
    if (mxGetField(prhs[1], 0, "prepare_for_c") != NULL)
      prepare_for_c = *mxGetPr(mxGetField(prhs[1], 0, "prepare_for_c"));
  }
  valid_vars = 0;
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "FMAX");
  if (xm == NULL) {
    printf("could not find params.FMAX.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("FMAX must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter FMAX must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter FMAX must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter FMAX must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.FMAX;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "FMIN");
  if (xm == NULL) {
    printf("could not find params.FMIN.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("FMIN must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter FMIN must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter FMIN must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter FMIN must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.FMIN;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "W_row2");
  if (xm == NULL) {
    printf("could not find params.W_row2.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("W_row2 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter W_row2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter W_row2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter W_row2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.W_row2;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "W_row3");
  if (xm == NULL) {
    printf("could not find params.W_row3.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("W_row3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter W_row3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter W_row3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter W_row3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.W_row3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "W_row4");
  if (xm == NULL) {
    printf("could not find params.W_row4.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("W_row4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter W_row4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter W_row4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter W_row4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.W_row4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Wy_LB");
  if (xm == NULL) {
    printf("could not find params.Wy_LB.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("Wy_LB must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Wy_LB must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Wy_LB must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Wy_LB must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Wy_LB;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Wy_UB");
  if (xm == NULL) {
    printf("could not find params.Wy_UB.\n");
  } else {
    if (!((mxGetM(xm) == 1) && (mxGetN(xm) == 1))) {
      printf("Wy_UB must be size (1,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Wy_UB must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Wy_UB must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Wy_UB must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Wy_UB;
      src = mxGetPr(xm);
      for (i = 0; i < 1; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "wdes");
  if (xm == NULL) {
    printf("could not find params.wdes.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("wdes must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter wdes must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter wdes must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter wdes must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.wdes;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  if (valid_vars != 8) {
    printf("Error: %d parameters are invalid.\n", 8 - valid_vars);
    mexErrMsgTxt("invalid parameters found.");
  }
  if (prepare_for_c) {
    printf("settings.prepare_for_c == 1. thus, outputting for C.\n");
    for (i = 0; i < 4; i++)
      printf("  params.W_row3[%d] = %.6g;\n", i, params.W_row3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.wdes[%d] = %.6g;\n", i, params.wdes[i]);
    for (i = 0; i < 4; i++)
      printf("  params.W_row2[%d] = %.6g;\n", i, params.W_row2[i]);
    for (i = 0; i < 4; i++)
      printf("  params.W_row4[%d] = %.6g;\n", i, params.W_row4[i]);
    for (i = 0; i < 1; i++)
      printf("  params.FMIN[%d] = %.6g;\n", i, params.FMIN[i]);
    for (i = 0; i < 1; i++)
      printf("  params.FMAX[%d] = %.6g;\n", i, params.FMAX[i]);
    for (i = 0; i < 1; i++)
      printf("  params.Wy_LB[%d] = %.6g;\n", i, params.Wy_LB[i]);
    for (i = 0; i < 1; i++)
      printf("  params.Wy_UB[%d] = %.6g;\n", i, params.Wy_UB[i]);
  }
  /* Perform the actual solve in here. */
  steps = solve();
  /* For profiling purposes, allow extra silent solves if desired. */
  settings.verbose = 0;
  for (i = 0; i < extra_solves; i++)
    solve();
  /* Update the status variables. */
  plhs[1] = mxCreateStructArray(1, dims1x1of1, 4, status_names);
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "optval", xm);
  *mxGetPr(xm) = work.optval;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "gap", xm);
  *mxGetPr(xm) = work.gap;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "steps", xm);
  *mxGetPr(xm) = steps;
  xm = mxCreateDoubleMatrix(1, 1, mxREAL);
  mxSetField(plhs[1], 0, "converged", xm);
  *mxGetPr(xm) = work.converged;
  /* Extract variable values. */
  plhs[0] = mxCreateStructArray(1, dims1x1of1, num_var_names, var_names);
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "f", xm);
  dest = mxGetPr(xm);
  src = vars.f;
  for (i = 0; i < 4; i++) {
    *dest++ = *src++;
  }
}
