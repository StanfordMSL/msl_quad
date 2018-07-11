/* Produced by CVXGEN, 2018-05-12 17:48:34 -0400.  */
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
  const char *var_names[] = {"c0123_1", "c0123_2", "c0123_3", "c0123_4", "c0123_5", "c0123_6", "c0123_7", "c0123_8", "c4567_1", "c4567_2", "c4567_3", "c4567_4", "c4567_5", "c4567_6", "c4567_7", "c4567_8", "c0123", "c4567"};
  const int num_var_names = 18;
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
  xm = mxGetField(prhs[0], 0, "Q_1");
  if (xm == NULL) {
    /* Attempt to pull Q_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Q");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.Q_1 or params.Q{1}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 4))) {
      printf("Q_1 must be size (4,4), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Q_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Q_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Q_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Q_1;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Q_2");
  if (xm == NULL) {
    /* Attempt to pull Q_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Q");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.Q_2 or params.Q{2}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 4))) {
      printf("Q_2 must be size (4,4), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Q_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Q_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Q_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Q_2;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Q_3");
  if (xm == NULL) {
    /* Attempt to pull Q_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Q");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.Q_3 or params.Q{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 4))) {
      printf("Q_3 must be size (4,4), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Q_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Q_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Q_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Q_3;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Q_4");
  if (xm == NULL) {
    /* Attempt to pull Q_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Q");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.Q_4 or params.Q{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 4))) {
      printf("Q_4 must be size (4,4), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Q_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Q_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Q_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Q_4;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Q_5");
  if (xm == NULL) {
    /* Attempt to pull Q_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Q");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.Q_5 or params.Q{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 4))) {
      printf("Q_5 must be size (4,4), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Q_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Q_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Q_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Q_5;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Q_6");
  if (xm == NULL) {
    /* Attempt to pull Q_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Q");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.Q_6 or params.Q{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 4))) {
      printf("Q_6 must be size (4,4), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Q_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Q_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Q_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Q_6;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Q_7");
  if (xm == NULL) {
    /* Attempt to pull Q_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Q");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.Q_7 or params.Q{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 4))) {
      printf("Q_7 must be size (4,4), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Q_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Q_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Q_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Q_7;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "Q_8");
  if (xm == NULL) {
    /* Attempt to pull Q_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "Q");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.Q_8 or params.Q{8}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 4))) {
      printf("Q_8 must be size (4,4), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter Q_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter Q_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter Q_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.Q_8;
      src = mxGetPr(xm);
      for (i = 0; i < 16; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c0123_1");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c0123_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c0123_1 or params.a_cont_c0123{1}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c0123_1 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c0123_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c0123_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c0123_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c0123_1;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c0123_2");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c0123_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c0123_2 or params.a_cont_c0123{2}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c0123_2 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c0123_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c0123_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c0123_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c0123_2;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c0123_3");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c0123_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c0123_3 or params.a_cont_c0123{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c0123_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c0123_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c0123_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c0123_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c0123_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c0123_4");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c0123_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c0123_4 or params.a_cont_c0123{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c0123_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c0123_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c0123_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c0123_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c0123_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c0123_5");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c0123_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c0123_5 or params.a_cont_c0123{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c0123_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c0123_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c0123_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c0123_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c0123_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c0123_6");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c0123_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c0123_6 or params.a_cont_c0123{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c0123_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c0123_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c0123_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c0123_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c0123_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c0123_7");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c0123_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c0123_7 or params.a_cont_c0123{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c0123_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c0123_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c0123_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c0123_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c0123_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c4567_1");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c4567_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c4567_1 or params.a_cont_c4567{1}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c4567_1 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c4567_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c4567_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c4567_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c4567_1;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c4567_2");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c4567_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c4567_2 or params.a_cont_c4567{2}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c4567_2 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c4567_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c4567_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c4567_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c4567_2;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c4567_3");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c4567_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c4567_3 or params.a_cont_c4567{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c4567_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c4567_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c4567_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c4567_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c4567_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c4567_4");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c4567_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c4567_4 or params.a_cont_c4567{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c4567_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c4567_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c4567_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c4567_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c4567_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c4567_5");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c4567_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c4567_5 or params.a_cont_c4567{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c4567_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c4567_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c4567_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c4567_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c4567_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c4567_6");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c4567_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c4567_6 or params.a_cont_c4567{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c4567_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c4567_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c4567_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c4567_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c4567_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "a_cont_c4567_7");
  if (xm == NULL) {
    /* Attempt to pull a_cont_c4567_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "a_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.a_cont_c4567_7 or params.a_cont_c4567{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("a_cont_c4567_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter a_cont_c4567_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter a_cont_c4567_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter a_cont_c4567_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.a_cont_c4567_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c0123_1");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c0123_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c0123_1 or params.ds_cont_c0123{1}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c0123_1 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c0123_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c0123_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c0123_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c0123_1;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c0123_2");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c0123_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c0123_2 or params.ds_cont_c0123{2}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c0123_2 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c0123_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c0123_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c0123_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c0123_2;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c0123_3");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c0123_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c0123_3 or params.ds_cont_c0123{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c0123_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c0123_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c0123_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c0123_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c0123_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c0123_4");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c0123_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c0123_4 or params.ds_cont_c0123{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c0123_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c0123_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c0123_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c0123_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c0123_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c0123_5");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c0123_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c0123_5 or params.ds_cont_c0123{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c0123_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c0123_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c0123_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c0123_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c0123_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c0123_6");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c0123_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c0123_6 or params.ds_cont_c0123{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c0123_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c0123_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c0123_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c0123_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c0123_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c0123_7");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c0123_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c0123_7 or params.ds_cont_c0123{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c0123_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c0123_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c0123_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c0123_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c0123_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c4567_1");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c4567_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c4567_1 or params.ds_cont_c4567{1}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c4567_1 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c4567_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c4567_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c4567_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c4567_1;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c4567_2");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c4567_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c4567_2 or params.ds_cont_c4567{2}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c4567_2 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c4567_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c4567_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c4567_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c4567_2;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c4567_3");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c4567_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c4567_3 or params.ds_cont_c4567{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c4567_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c4567_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c4567_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c4567_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c4567_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c4567_4");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c4567_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c4567_4 or params.ds_cont_c4567{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c4567_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c4567_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c4567_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c4567_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c4567_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c4567_5");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c4567_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c4567_5 or params.ds_cont_c4567{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c4567_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c4567_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c4567_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c4567_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c4567_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c4567_6");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c4567_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c4567_6 or params.ds_cont_c4567{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c4567_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c4567_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c4567_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c4567_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c4567_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "ds_cont_c4567_7");
  if (xm == NULL) {
    /* Attempt to pull ds_cont_c4567_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "ds_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.ds_cont_c4567_7 or params.ds_cont_c4567{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("ds_cont_c4567_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter ds_cont_c4567_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter ds_cont_c4567_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter ds_cont_c4567_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.ds_cont_c4567_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_a_c0123_3");
  if (xm == NULL) {
    /* Attempt to pull final_a_c0123_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_a_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.final_a_c0123_3 or params.final_a_c0123{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_a_c0123_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_a_c0123_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_a_c0123_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_a_c0123_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_a_c0123_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_a_c0123_4");
  if (xm == NULL) {
    /* Attempt to pull final_a_c0123_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_a_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.final_a_c0123_4 or params.final_a_c0123{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_a_c0123_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_a_c0123_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_a_c0123_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_a_c0123_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_a_c0123_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_a_c0123_5");
  if (xm == NULL) {
    /* Attempt to pull final_a_c0123_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_a_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.final_a_c0123_5 or params.final_a_c0123{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_a_c0123_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_a_c0123_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_a_c0123_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_a_c0123_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_a_c0123_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_a_c0123_6");
  if (xm == NULL) {
    /* Attempt to pull final_a_c0123_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_a_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.final_a_c0123_6 or params.final_a_c0123{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_a_c0123_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_a_c0123_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_a_c0123_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_a_c0123_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_a_c0123_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_a_c0123_7");
  if (xm == NULL) {
    /* Attempt to pull final_a_c0123_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_a_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.final_a_c0123_7 or params.final_a_c0123{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_a_c0123_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_a_c0123_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_a_c0123_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_a_c0123_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_a_c0123_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_a_c0123_8");
  if (xm == NULL) {
    /* Attempt to pull final_a_c0123_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_a_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.final_a_c0123_8 or params.final_a_c0123{8}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_a_c0123_8 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_a_c0123_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_a_c0123_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_a_c0123_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_a_c0123_8;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_a_c4567_3");
  if (xm == NULL) {
    /* Attempt to pull final_a_c4567_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_a_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.final_a_c4567_3 or params.final_a_c4567{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_a_c4567_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_a_c4567_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_a_c4567_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_a_c4567_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_a_c4567_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_a_c4567_4");
  if (xm == NULL) {
    /* Attempt to pull final_a_c4567_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_a_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.final_a_c4567_4 or params.final_a_c4567{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_a_c4567_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_a_c4567_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_a_c4567_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_a_c4567_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_a_c4567_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_a_c4567_5");
  if (xm == NULL) {
    /* Attempt to pull final_a_c4567_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_a_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.final_a_c4567_5 or params.final_a_c4567{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_a_c4567_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_a_c4567_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_a_c4567_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_a_c4567_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_a_c4567_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_a_c4567_6");
  if (xm == NULL) {
    /* Attempt to pull final_a_c4567_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_a_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.final_a_c4567_6 or params.final_a_c4567{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_a_c4567_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_a_c4567_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_a_c4567_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_a_c4567_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_a_c4567_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_a_c4567_7");
  if (xm == NULL) {
    /* Attempt to pull final_a_c4567_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_a_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.final_a_c4567_7 or params.final_a_c4567{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_a_c4567_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_a_c4567_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_a_c4567_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_a_c4567_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_a_c4567_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_a_c4567_8");
  if (xm == NULL) {
    /* Attempt to pull final_a_c4567_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_a_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.final_a_c4567_8 or params.final_a_c4567{8}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_a_c4567_8 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_a_c4567_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_a_c4567_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_a_c4567_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_a_c4567_8;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_j_c0123_3");
  if (xm == NULL) {
    /* Attempt to pull final_j_c0123_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_j_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.final_j_c0123_3 or params.final_j_c0123{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_j_c0123_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_j_c0123_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_j_c0123_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_j_c0123_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_j_c0123_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_j_c0123_4");
  if (xm == NULL) {
    /* Attempt to pull final_j_c0123_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_j_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.final_j_c0123_4 or params.final_j_c0123{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_j_c0123_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_j_c0123_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_j_c0123_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_j_c0123_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_j_c0123_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_j_c0123_5");
  if (xm == NULL) {
    /* Attempt to pull final_j_c0123_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_j_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.final_j_c0123_5 or params.final_j_c0123{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_j_c0123_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_j_c0123_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_j_c0123_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_j_c0123_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_j_c0123_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_j_c0123_6");
  if (xm == NULL) {
    /* Attempt to pull final_j_c0123_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_j_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.final_j_c0123_6 or params.final_j_c0123{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_j_c0123_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_j_c0123_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_j_c0123_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_j_c0123_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_j_c0123_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_j_c0123_7");
  if (xm == NULL) {
    /* Attempt to pull final_j_c0123_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_j_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.final_j_c0123_7 or params.final_j_c0123{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_j_c0123_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_j_c0123_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_j_c0123_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_j_c0123_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_j_c0123_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_j_c0123_8");
  if (xm == NULL) {
    /* Attempt to pull final_j_c0123_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_j_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.final_j_c0123_8 or params.final_j_c0123{8}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_j_c0123_8 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_j_c0123_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_j_c0123_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_j_c0123_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_j_c0123_8;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_j_c4567_3");
  if (xm == NULL) {
    /* Attempt to pull final_j_c4567_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_j_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.final_j_c4567_3 or params.final_j_c4567{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_j_c4567_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_j_c4567_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_j_c4567_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_j_c4567_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_j_c4567_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_j_c4567_4");
  if (xm == NULL) {
    /* Attempt to pull final_j_c4567_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_j_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.final_j_c4567_4 or params.final_j_c4567{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_j_c4567_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_j_c4567_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_j_c4567_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_j_c4567_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_j_c4567_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_j_c4567_5");
  if (xm == NULL) {
    /* Attempt to pull final_j_c4567_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_j_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.final_j_c4567_5 or params.final_j_c4567{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_j_c4567_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_j_c4567_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_j_c4567_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_j_c4567_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_j_c4567_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_j_c4567_6");
  if (xm == NULL) {
    /* Attempt to pull final_j_c4567_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_j_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.final_j_c4567_6 or params.final_j_c4567{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_j_c4567_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_j_c4567_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_j_c4567_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_j_c4567_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_j_c4567_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_j_c4567_7");
  if (xm == NULL) {
    /* Attempt to pull final_j_c4567_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_j_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.final_j_c4567_7 or params.final_j_c4567{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_j_c4567_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_j_c4567_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_j_c4567_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_j_c4567_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_j_c4567_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_j_c4567_8");
  if (xm == NULL) {
    /* Attempt to pull final_j_c4567_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_j_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.final_j_c4567_8 or params.final_j_c4567{8}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_j_c4567_8 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_j_c4567_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_j_c4567_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_j_c4567_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_j_c4567_8;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_s_c0123_3");
  if (xm == NULL) {
    /* Attempt to pull final_s_c0123_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_s_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.final_s_c0123_3 or params.final_s_c0123{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_s_c0123_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_s_c0123_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_s_c0123_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_s_c0123_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_s_c0123_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_s_c0123_4");
  if (xm == NULL) {
    /* Attempt to pull final_s_c0123_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_s_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.final_s_c0123_4 or params.final_s_c0123{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_s_c0123_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_s_c0123_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_s_c0123_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_s_c0123_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_s_c0123_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_s_c0123_5");
  if (xm == NULL) {
    /* Attempt to pull final_s_c0123_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_s_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.final_s_c0123_5 or params.final_s_c0123{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_s_c0123_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_s_c0123_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_s_c0123_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_s_c0123_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_s_c0123_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_s_c0123_6");
  if (xm == NULL) {
    /* Attempt to pull final_s_c0123_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_s_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.final_s_c0123_6 or params.final_s_c0123{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_s_c0123_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_s_c0123_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_s_c0123_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_s_c0123_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_s_c0123_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_s_c0123_7");
  if (xm == NULL) {
    /* Attempt to pull final_s_c0123_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_s_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.final_s_c0123_7 or params.final_s_c0123{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_s_c0123_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_s_c0123_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_s_c0123_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_s_c0123_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_s_c0123_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_s_c0123_8");
  if (xm == NULL) {
    /* Attempt to pull final_s_c0123_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_s_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.final_s_c0123_8 or params.final_s_c0123{8}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_s_c0123_8 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_s_c0123_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_s_c0123_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_s_c0123_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_s_c0123_8;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_s_c4567_3");
  if (xm == NULL) {
    /* Attempt to pull final_s_c4567_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_s_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.final_s_c4567_3 or params.final_s_c4567{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_s_c4567_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_s_c4567_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_s_c4567_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_s_c4567_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_s_c4567_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_s_c4567_4");
  if (xm == NULL) {
    /* Attempt to pull final_s_c4567_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_s_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.final_s_c4567_4 or params.final_s_c4567{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_s_c4567_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_s_c4567_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_s_c4567_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_s_c4567_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_s_c4567_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_s_c4567_5");
  if (xm == NULL) {
    /* Attempt to pull final_s_c4567_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_s_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.final_s_c4567_5 or params.final_s_c4567{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_s_c4567_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_s_c4567_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_s_c4567_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_s_c4567_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_s_c4567_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_s_c4567_6");
  if (xm == NULL) {
    /* Attempt to pull final_s_c4567_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_s_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.final_s_c4567_6 or params.final_s_c4567{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_s_c4567_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_s_c4567_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_s_c4567_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_s_c4567_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_s_c4567_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_s_c4567_7");
  if (xm == NULL) {
    /* Attempt to pull final_s_c4567_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_s_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.final_s_c4567_7 or params.final_s_c4567{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_s_c4567_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_s_c4567_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_s_c4567_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_s_c4567_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_s_c4567_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_s_c4567_8");
  if (xm == NULL) {
    /* Attempt to pull final_s_c4567_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_s_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.final_s_c4567_8 or params.final_s_c4567{8}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_s_c4567_8 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_s_c4567_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_s_c4567_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_s_c4567_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_s_c4567_8;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_v_c0123_3");
  if (xm == NULL) {
    /* Attempt to pull final_v_c0123_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_v_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.final_v_c0123_3 or params.final_v_c0123{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_v_c0123_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_v_c0123_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_v_c0123_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_v_c0123_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_v_c0123_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_v_c0123_4");
  if (xm == NULL) {
    /* Attempt to pull final_v_c0123_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_v_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.final_v_c0123_4 or params.final_v_c0123{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_v_c0123_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_v_c0123_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_v_c0123_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_v_c0123_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_v_c0123_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_v_c0123_5");
  if (xm == NULL) {
    /* Attempt to pull final_v_c0123_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_v_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.final_v_c0123_5 or params.final_v_c0123{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_v_c0123_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_v_c0123_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_v_c0123_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_v_c0123_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_v_c0123_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_v_c0123_6");
  if (xm == NULL) {
    /* Attempt to pull final_v_c0123_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_v_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.final_v_c0123_6 or params.final_v_c0123{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_v_c0123_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_v_c0123_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_v_c0123_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_v_c0123_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_v_c0123_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_v_c0123_7");
  if (xm == NULL) {
    /* Attempt to pull final_v_c0123_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_v_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.final_v_c0123_7 or params.final_v_c0123{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_v_c0123_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_v_c0123_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_v_c0123_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_v_c0123_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_v_c0123_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_v_c0123_8");
  if (xm == NULL) {
    /* Attempt to pull final_v_c0123_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_v_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.final_v_c0123_8 or params.final_v_c0123{8}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_v_c0123_8 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_v_c0123_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_v_c0123_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_v_c0123_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_v_c0123_8;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_v_c4567_3");
  if (xm == NULL) {
    /* Attempt to pull final_v_c4567_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_v_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.final_v_c4567_3 or params.final_v_c4567{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_v_c4567_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_v_c4567_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_v_c4567_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_v_c4567_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_v_c4567_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_v_c4567_4");
  if (xm == NULL) {
    /* Attempt to pull final_v_c4567_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_v_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.final_v_c4567_4 or params.final_v_c4567{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_v_c4567_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_v_c4567_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_v_c4567_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_v_c4567_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_v_c4567_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_v_c4567_5");
  if (xm == NULL) {
    /* Attempt to pull final_v_c4567_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_v_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.final_v_c4567_5 or params.final_v_c4567{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_v_c4567_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_v_c4567_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_v_c4567_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_v_c4567_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_v_c4567_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_v_c4567_6");
  if (xm == NULL) {
    /* Attempt to pull final_v_c4567_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_v_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.final_v_c4567_6 or params.final_v_c4567{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_v_c4567_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_v_c4567_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_v_c4567_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_v_c4567_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_v_c4567_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_v_c4567_7");
  if (xm == NULL) {
    /* Attempt to pull final_v_c4567_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_v_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.final_v_c4567_7 or params.final_v_c4567{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_v_c4567_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_v_c4567_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_v_c4567_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_v_c4567_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_v_c4567_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "final_v_c4567_8");
  if (xm == NULL) {
    /* Attempt to pull final_v_c4567_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "final_v_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.final_v_c4567_8 or params.final_v_c4567{8}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("final_v_c4567_8 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter final_v_c4567_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter final_v_c4567_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter final_v_c4567_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.final_v_c4567_8;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c0123_1");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c0123_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c0123_1 or params.j_cont_c0123{1}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c0123_1 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c0123_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c0123_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c0123_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c0123_1;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c0123_2");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c0123_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c0123_2 or params.j_cont_c0123{2}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c0123_2 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c0123_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c0123_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c0123_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c0123_2;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c0123_3");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c0123_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c0123_3 or params.j_cont_c0123{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c0123_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c0123_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c0123_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c0123_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c0123_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c0123_4");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c0123_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c0123_4 or params.j_cont_c0123{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c0123_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c0123_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c0123_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c0123_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c0123_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c0123_5");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c0123_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c0123_5 or params.j_cont_c0123{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c0123_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c0123_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c0123_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c0123_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c0123_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c0123_6");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c0123_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c0123_6 or params.j_cont_c0123{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c0123_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c0123_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c0123_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c0123_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c0123_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c0123_7");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c0123_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c0123_7 or params.j_cont_c0123{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c0123_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c0123_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c0123_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c0123_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c0123_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c4567_1");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c4567_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c4567_1 or params.j_cont_c4567{1}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c4567_1 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c4567_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c4567_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c4567_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c4567_1;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c4567_2");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c4567_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c4567_2 or params.j_cont_c4567{2}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c4567_2 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c4567_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c4567_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c4567_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c4567_2;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c4567_3");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c4567_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c4567_3 or params.j_cont_c4567{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c4567_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c4567_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c4567_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c4567_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c4567_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c4567_4");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c4567_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c4567_4 or params.j_cont_c4567{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c4567_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c4567_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c4567_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c4567_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c4567_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c4567_5");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c4567_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c4567_5 or params.j_cont_c4567{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c4567_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c4567_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c4567_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c4567_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c4567_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c4567_6");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c4567_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c4567_6 or params.j_cont_c4567{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c4567_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c4567_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c4567_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c4567_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c4567_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "j_cont_c4567_7");
  if (xm == NULL) {
    /* Attempt to pull j_cont_c4567_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "j_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.j_cont_c4567_7 or params.j_cont_c4567{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("j_cont_c4567_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter j_cont_c4567_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter j_cont_c4567_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter j_cont_c4567_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.j_cont_c4567_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c0123_1");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c0123_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c0123_1 or params.s_cont_c0123{1}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c0123_1 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c0123_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c0123_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c0123_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c0123_1;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c0123_2");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c0123_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c0123_2 or params.s_cont_c0123{2}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c0123_2 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c0123_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c0123_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c0123_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c0123_2;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c0123_3");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c0123_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c0123_3 or params.s_cont_c0123{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c0123_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c0123_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c0123_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c0123_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c0123_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c0123_4");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c0123_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c0123_4 or params.s_cont_c0123{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c0123_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c0123_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c0123_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c0123_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c0123_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c0123_5");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c0123_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c0123_5 or params.s_cont_c0123{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c0123_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c0123_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c0123_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c0123_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c0123_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c0123_6");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c0123_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c0123_6 or params.s_cont_c0123{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c0123_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c0123_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c0123_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c0123_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c0123_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c0123_7");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c0123_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c0123_7 or params.s_cont_c0123{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c0123_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c0123_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c0123_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c0123_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c0123_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c4567_1");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c4567_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c4567_1 or params.s_cont_c4567{1}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c4567_1 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c4567_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c4567_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c4567_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c4567_1;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c4567_2");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c4567_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c4567_2 or params.s_cont_c4567{2}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c4567_2 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c4567_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c4567_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c4567_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c4567_2;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c4567_3");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c4567_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c4567_3 or params.s_cont_c4567{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c4567_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c4567_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c4567_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c4567_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c4567_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c4567_4");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c4567_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c4567_4 or params.s_cont_c4567{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c4567_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c4567_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c4567_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c4567_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c4567_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c4567_5");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c4567_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c4567_5 or params.s_cont_c4567{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c4567_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c4567_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c4567_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c4567_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c4567_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c4567_6");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c4567_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c4567_6 or params.s_cont_c4567{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c4567_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c4567_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c4567_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c4567_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c4567_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "s_cont_c4567_7");
  if (xm == NULL) {
    /* Attempt to pull s_cont_c4567_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "s_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.s_cont_c4567_7 or params.s_cont_c4567{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("s_cont_c4567_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter s_cont_c4567_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter s_cont_c4567_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter s_cont_c4567_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.s_cont_c4567_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf0123_1");
  if (xm == NULL) {
    /* Attempt to pull tf0123_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.tf0123_1 or params.tf0123{1}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf0123_1 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf0123_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf0123_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf0123_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf0123_1;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf0123_2");
  if (xm == NULL) {
    /* Attempt to pull tf0123_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.tf0123_2 or params.tf0123{2}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf0123_2 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf0123_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf0123_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf0123_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf0123_2;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf0123_3");
  if (xm == NULL) {
    /* Attempt to pull tf0123_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.tf0123_3 or params.tf0123{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf0123_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf0123_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf0123_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf0123_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf0123_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf0123_4");
  if (xm == NULL) {
    /* Attempt to pull tf0123_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.tf0123_4 or params.tf0123{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf0123_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf0123_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf0123_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf0123_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf0123_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf0123_5");
  if (xm == NULL) {
    /* Attempt to pull tf0123_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.tf0123_5 or params.tf0123{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf0123_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf0123_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf0123_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf0123_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf0123_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf0123_6");
  if (xm == NULL) {
    /* Attempt to pull tf0123_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.tf0123_6 or params.tf0123{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf0123_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf0123_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf0123_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf0123_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf0123_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf0123_7");
  if (xm == NULL) {
    /* Attempt to pull tf0123_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.tf0123_7 or params.tf0123{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf0123_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf0123_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf0123_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf0123_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf0123_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf0123_8");
  if (xm == NULL) {
    /* Attempt to pull tf0123_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.tf0123_8 or params.tf0123{8}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf0123_8 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf0123_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf0123_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf0123_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf0123_8;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf4567_1");
  if (xm == NULL) {
    /* Attempt to pull tf4567_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.tf4567_1 or params.tf4567{1}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf4567_1 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf4567_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf4567_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf4567_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf4567_1;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf4567_2");
  if (xm == NULL) {
    /* Attempt to pull tf4567_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.tf4567_2 or params.tf4567{2}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf4567_2 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf4567_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf4567_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf4567_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf4567_2;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf4567_3");
  if (xm == NULL) {
    /* Attempt to pull tf4567_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.tf4567_3 or params.tf4567{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf4567_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf4567_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf4567_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf4567_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf4567_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf4567_4");
  if (xm == NULL) {
    /* Attempt to pull tf4567_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.tf4567_4 or params.tf4567{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf4567_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf4567_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf4567_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf4567_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf4567_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf4567_5");
  if (xm == NULL) {
    /* Attempt to pull tf4567_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.tf4567_5 or params.tf4567{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf4567_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf4567_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf4567_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf4567_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf4567_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf4567_6");
  if (xm == NULL) {
    /* Attempt to pull tf4567_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.tf4567_6 or params.tf4567{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf4567_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf4567_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf4567_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf4567_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf4567_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf4567_7");
  if (xm == NULL) {
    /* Attempt to pull tf4567_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.tf4567_7 or params.tf4567{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf4567_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf4567_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf4567_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf4567_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf4567_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "tf4567_8");
  if (xm == NULL) {
    /* Attempt to pull tf4567_8 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "tf4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 7);
  }
  if (xm == NULL) {
    printf("could not find params.tf4567_8 or params.tf4567{8}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("tf4567_8 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter tf4567_8 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter tf4567_8 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter tf4567_8 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.tf4567_8;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c0123_1");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c0123_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c0123_1 or params.v_cont_c0123{1}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c0123_1 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c0123_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c0123_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c0123_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c0123_1;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c0123_2");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c0123_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c0123_2 or params.v_cont_c0123{2}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c0123_2 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c0123_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c0123_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c0123_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c0123_2;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c0123_3");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c0123_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c0123_3 or params.v_cont_c0123{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c0123_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c0123_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c0123_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c0123_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c0123_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c0123_4");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c0123_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c0123_4 or params.v_cont_c0123{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c0123_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c0123_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c0123_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c0123_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c0123_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c0123_5");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c0123_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c0123_5 or params.v_cont_c0123{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c0123_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c0123_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c0123_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c0123_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c0123_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c0123_6");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c0123_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c0123_6 or params.v_cont_c0123{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c0123_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c0123_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c0123_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c0123_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c0123_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c0123_7");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c0123_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c0123");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c0123_7 or params.v_cont_c0123{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c0123_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c0123_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c0123_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c0123_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c0123_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c4567_1");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c4567_1 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 0);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c4567_1 or params.v_cont_c4567{1}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c4567_1 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c4567_1 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c4567_1 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c4567_1 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c4567_1;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c4567_2");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c4567_2 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 1);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c4567_2 or params.v_cont_c4567{2}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c4567_2 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c4567_2 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c4567_2 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c4567_2 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c4567_2;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c4567_3");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c4567_3 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 2);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c4567_3 or params.v_cont_c4567{3}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c4567_3 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c4567_3 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c4567_3 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c4567_3 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c4567_3;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c4567_4");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c4567_4 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 3);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c4567_4 or params.v_cont_c4567{4}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c4567_4 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c4567_4 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c4567_4 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c4567_4 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c4567_4;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c4567_5");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c4567_5 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 4);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c4567_5 or params.v_cont_c4567{5}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c4567_5 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c4567_5 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c4567_5 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c4567_5 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c4567_5;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c4567_6");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c4567_6 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 5);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c4567_6 or params.v_cont_c4567{6}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c4567_6 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c4567_6 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c4567_6 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c4567_6 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c4567_6;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "v_cont_c4567_7");
  if (xm == NULL) {
    /* Attempt to pull v_cont_c4567_7 from a cell array, as an additional option. */
    cell = mxGetField(prhs[0], 0, "v_cont_c4567");
    if (cell != NULL)
      xm = mxGetCell(cell, 6);
  }
  if (xm == NULL) {
    printf("could not find params.v_cont_c4567_7 or params.v_cont_c4567{7}.\n");
  } else {
    if (!((mxGetM(xm) == 4) && (mxGetN(xm) == 1))) {
      printf("v_cont_c4567_7 must be size (4,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter v_cont_c4567_7 must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter v_cont_c4567_7 must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter v_cont_c4567_7 must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.v_cont_c4567_7;
      src = mxGetPr(xm);
      for (i = 0; i < 4; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  this_var_errors = 0;
  xm = mxGetField(prhs[0], 0, "wpts");
  if (xm == NULL) {
    printf("could not find params.wpts.\n");
  } else {
    if (!((mxGetM(xm) == 9) && (mxGetN(xm) == 1))) {
      printf("wpts must be size (9,1), not (%d,%d).\n", mxGetM(xm), mxGetN(xm));
      this_var_errors++;
    }
    if (mxIsComplex(xm)) {
      printf("parameter wpts must be real.\n");
      this_var_errors++;
    }
    if (!mxIsClass(xm, "double")) {
      printf("parameter wpts must be a full matrix of doubles.\n");
      this_var_errors++;
    }
    if (mxIsSparse(xm)) {
      printf("parameter wpts must be a full matrix.\n");
      this_var_errors++;
    }
    if (this_var_errors == 0) {
      dest = params.wpts;
      src = mxGetPr(xm);
      for (i = 0; i < 9; i++)
        *dest++ = *src++;
      valid_vars++;
    }
  }
  if (valid_vars != 143) {
    printf("Error: %d parameters are invalid.\n", 143 - valid_vars);
    mexErrMsgTxt("invalid parameters found.");
  }
  if (prepare_for_c) {
    printf("settings.prepare_for_c == 1. thus, outputting for C.\n");
    for (i = 0; i < 16; i++)
      printf("  params.Q_1[%d] = %.6g;\n", i, params.Q_1[i]);
    for (i = 0; i < 16; i++)
      printf("  params.Q_2[%d] = %.6g;\n", i, params.Q_2[i]);
    for (i = 0; i < 16; i++)
      printf("  params.Q_3[%d] = %.6g;\n", i, params.Q_3[i]);
    for (i = 0; i < 16; i++)
      printf("  params.Q_4[%d] = %.6g;\n", i, params.Q_4[i]);
    for (i = 0; i < 16; i++)
      printf("  params.Q_5[%d] = %.6g;\n", i, params.Q_5[i]);
    for (i = 0; i < 16; i++)
      printf("  params.Q_6[%d] = %.6g;\n", i, params.Q_6[i]);
    for (i = 0; i < 16; i++)
      printf("  params.Q_7[%d] = %.6g;\n", i, params.Q_7[i]);
    for (i = 0; i < 16; i++)
      printf("  params.Q_8[%d] = %.6g;\n", i, params.Q_8[i]);
    for (i = 0; i < 9; i++)
      printf("  params.wpts[%d] = %.6g;\n", i, params.wpts[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf0123_1[%d] = %.6g;\n", i, params.tf0123_1[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf4567_1[%d] = %.6g;\n", i, params.tf4567_1[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf0123_2[%d] = %.6g;\n", i, params.tf0123_2[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf4567_2[%d] = %.6g;\n", i, params.tf4567_2[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf0123_3[%d] = %.6g;\n", i, params.tf0123_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf4567_3[%d] = %.6g;\n", i, params.tf4567_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf0123_4[%d] = %.6g;\n", i, params.tf0123_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf4567_4[%d] = %.6g;\n", i, params.tf4567_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf0123_5[%d] = %.6g;\n", i, params.tf0123_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf4567_5[%d] = %.6g;\n", i, params.tf4567_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf0123_6[%d] = %.6g;\n", i, params.tf0123_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf4567_6[%d] = %.6g;\n", i, params.tf4567_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf0123_7[%d] = %.6g;\n", i, params.tf0123_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf4567_7[%d] = %.6g;\n", i, params.tf4567_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf0123_8[%d] = %.6g;\n", i, params.tf0123_8[i]);
    for (i = 0; i < 4; i++)
      printf("  params.tf4567_8[%d] = %.6g;\n", i, params.tf4567_8[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_v_c0123_3[%d] = %.6g;\n", i, params.final_v_c0123_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_v_c4567_3[%d] = %.6g;\n", i, params.final_v_c4567_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_a_c0123_3[%d] = %.6g;\n", i, params.final_a_c0123_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_a_c4567_3[%d] = %.6g;\n", i, params.final_a_c4567_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_j_c0123_3[%d] = %.6g;\n", i, params.final_j_c0123_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_j_c4567_3[%d] = %.6g;\n", i, params.final_j_c4567_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_s_c0123_3[%d] = %.6g;\n", i, params.final_s_c0123_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_s_c4567_3[%d] = %.6g;\n", i, params.final_s_c4567_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_v_c0123_4[%d] = %.6g;\n", i, params.final_v_c0123_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_v_c4567_4[%d] = %.6g;\n", i, params.final_v_c4567_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_a_c0123_4[%d] = %.6g;\n", i, params.final_a_c0123_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_a_c4567_4[%d] = %.6g;\n", i, params.final_a_c4567_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_j_c0123_4[%d] = %.6g;\n", i, params.final_j_c0123_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_j_c4567_4[%d] = %.6g;\n", i, params.final_j_c4567_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_s_c0123_4[%d] = %.6g;\n", i, params.final_s_c0123_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_s_c4567_4[%d] = %.6g;\n", i, params.final_s_c4567_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_v_c0123_5[%d] = %.6g;\n", i, params.final_v_c0123_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_v_c4567_5[%d] = %.6g;\n", i, params.final_v_c4567_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_a_c0123_5[%d] = %.6g;\n", i, params.final_a_c0123_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_a_c4567_5[%d] = %.6g;\n", i, params.final_a_c4567_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_j_c0123_5[%d] = %.6g;\n", i, params.final_j_c0123_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_j_c4567_5[%d] = %.6g;\n", i, params.final_j_c4567_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_s_c0123_5[%d] = %.6g;\n", i, params.final_s_c0123_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_s_c4567_5[%d] = %.6g;\n", i, params.final_s_c4567_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_v_c0123_6[%d] = %.6g;\n", i, params.final_v_c0123_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_v_c4567_6[%d] = %.6g;\n", i, params.final_v_c4567_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_a_c0123_6[%d] = %.6g;\n", i, params.final_a_c0123_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_a_c4567_6[%d] = %.6g;\n", i, params.final_a_c4567_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_j_c0123_6[%d] = %.6g;\n", i, params.final_j_c0123_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_j_c4567_6[%d] = %.6g;\n", i, params.final_j_c4567_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_s_c0123_6[%d] = %.6g;\n", i, params.final_s_c0123_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_s_c4567_6[%d] = %.6g;\n", i, params.final_s_c4567_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_v_c0123_7[%d] = %.6g;\n", i, params.final_v_c0123_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_v_c4567_7[%d] = %.6g;\n", i, params.final_v_c4567_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_a_c0123_7[%d] = %.6g;\n", i, params.final_a_c0123_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_a_c4567_7[%d] = %.6g;\n", i, params.final_a_c4567_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_j_c0123_7[%d] = %.6g;\n", i, params.final_j_c0123_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_j_c4567_7[%d] = %.6g;\n", i, params.final_j_c4567_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_s_c0123_7[%d] = %.6g;\n", i, params.final_s_c0123_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_s_c4567_7[%d] = %.6g;\n", i, params.final_s_c4567_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_v_c0123_8[%d] = %.6g;\n", i, params.final_v_c0123_8[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_v_c4567_8[%d] = %.6g;\n", i, params.final_v_c4567_8[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_a_c0123_8[%d] = %.6g;\n", i, params.final_a_c0123_8[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_a_c4567_8[%d] = %.6g;\n", i, params.final_a_c4567_8[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_j_c0123_8[%d] = %.6g;\n", i, params.final_j_c0123_8[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_j_c4567_8[%d] = %.6g;\n", i, params.final_j_c4567_8[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_s_c0123_8[%d] = %.6g;\n", i, params.final_s_c0123_8[i]);
    for (i = 0; i < 4; i++)
      printf("  params.final_s_c4567_8[%d] = %.6g;\n", i, params.final_s_c4567_8[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c0123_1[%d] = %.6g;\n", i, params.v_cont_c0123_1[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c4567_1[%d] = %.6g;\n", i, params.v_cont_c4567_1[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c0123_1[%d] = %.6g;\n", i, params.a_cont_c0123_1[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c4567_1[%d] = %.6g;\n", i, params.a_cont_c4567_1[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c0123_1[%d] = %.6g;\n", i, params.j_cont_c0123_1[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c4567_1[%d] = %.6g;\n", i, params.j_cont_c4567_1[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c0123_1[%d] = %.6g;\n", i, params.s_cont_c0123_1[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c4567_1[%d] = %.6g;\n", i, params.s_cont_c4567_1[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c0123_1[%d] = %.6g;\n", i, params.ds_cont_c0123_1[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c4567_1[%d] = %.6g;\n", i, params.ds_cont_c4567_1[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c0123_2[%d] = %.6g;\n", i, params.v_cont_c0123_2[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c4567_2[%d] = %.6g;\n", i, params.v_cont_c4567_2[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c0123_2[%d] = %.6g;\n", i, params.a_cont_c0123_2[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c4567_2[%d] = %.6g;\n", i, params.a_cont_c4567_2[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c0123_2[%d] = %.6g;\n", i, params.j_cont_c0123_2[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c4567_2[%d] = %.6g;\n", i, params.j_cont_c4567_2[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c0123_2[%d] = %.6g;\n", i, params.s_cont_c0123_2[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c4567_2[%d] = %.6g;\n", i, params.s_cont_c4567_2[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c0123_2[%d] = %.6g;\n", i, params.ds_cont_c0123_2[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c4567_2[%d] = %.6g;\n", i, params.ds_cont_c4567_2[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c0123_3[%d] = %.6g;\n", i, params.v_cont_c0123_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c4567_3[%d] = %.6g;\n", i, params.v_cont_c4567_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c0123_3[%d] = %.6g;\n", i, params.a_cont_c0123_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c4567_3[%d] = %.6g;\n", i, params.a_cont_c4567_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c0123_3[%d] = %.6g;\n", i, params.j_cont_c0123_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c4567_3[%d] = %.6g;\n", i, params.j_cont_c4567_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c0123_3[%d] = %.6g;\n", i, params.s_cont_c0123_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c4567_3[%d] = %.6g;\n", i, params.s_cont_c4567_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c0123_3[%d] = %.6g;\n", i, params.ds_cont_c0123_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c4567_3[%d] = %.6g;\n", i, params.ds_cont_c4567_3[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c0123_4[%d] = %.6g;\n", i, params.v_cont_c0123_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c4567_4[%d] = %.6g;\n", i, params.v_cont_c4567_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c0123_4[%d] = %.6g;\n", i, params.a_cont_c0123_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c4567_4[%d] = %.6g;\n", i, params.a_cont_c4567_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c0123_4[%d] = %.6g;\n", i, params.j_cont_c0123_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c4567_4[%d] = %.6g;\n", i, params.j_cont_c4567_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c0123_4[%d] = %.6g;\n", i, params.s_cont_c0123_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c4567_4[%d] = %.6g;\n", i, params.s_cont_c4567_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c0123_4[%d] = %.6g;\n", i, params.ds_cont_c0123_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c4567_4[%d] = %.6g;\n", i, params.ds_cont_c4567_4[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c0123_5[%d] = %.6g;\n", i, params.v_cont_c0123_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c4567_5[%d] = %.6g;\n", i, params.v_cont_c4567_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c0123_5[%d] = %.6g;\n", i, params.a_cont_c0123_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c4567_5[%d] = %.6g;\n", i, params.a_cont_c4567_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c0123_5[%d] = %.6g;\n", i, params.j_cont_c0123_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c4567_5[%d] = %.6g;\n", i, params.j_cont_c4567_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c0123_5[%d] = %.6g;\n", i, params.s_cont_c0123_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c4567_5[%d] = %.6g;\n", i, params.s_cont_c4567_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c0123_5[%d] = %.6g;\n", i, params.ds_cont_c0123_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c4567_5[%d] = %.6g;\n", i, params.ds_cont_c4567_5[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c0123_6[%d] = %.6g;\n", i, params.v_cont_c0123_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c4567_6[%d] = %.6g;\n", i, params.v_cont_c4567_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c0123_6[%d] = %.6g;\n", i, params.a_cont_c0123_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c4567_6[%d] = %.6g;\n", i, params.a_cont_c4567_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c0123_6[%d] = %.6g;\n", i, params.j_cont_c0123_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c4567_6[%d] = %.6g;\n", i, params.j_cont_c4567_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c0123_6[%d] = %.6g;\n", i, params.s_cont_c0123_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c4567_6[%d] = %.6g;\n", i, params.s_cont_c4567_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c0123_6[%d] = %.6g;\n", i, params.ds_cont_c0123_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c4567_6[%d] = %.6g;\n", i, params.ds_cont_c4567_6[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c0123_7[%d] = %.6g;\n", i, params.v_cont_c0123_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.v_cont_c4567_7[%d] = %.6g;\n", i, params.v_cont_c4567_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c0123_7[%d] = %.6g;\n", i, params.a_cont_c0123_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.a_cont_c4567_7[%d] = %.6g;\n", i, params.a_cont_c4567_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c0123_7[%d] = %.6g;\n", i, params.j_cont_c0123_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.j_cont_c4567_7[%d] = %.6g;\n", i, params.j_cont_c4567_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c0123_7[%d] = %.6g;\n", i, params.s_cont_c0123_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.s_cont_c4567_7[%d] = %.6g;\n", i, params.s_cont_c4567_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c0123_7[%d] = %.6g;\n", i, params.ds_cont_c0123_7[i]);
    for (i = 0; i < 4; i++)
      printf("  params.ds_cont_c4567_7[%d] = %.6g;\n", i, params.ds_cont_c4567_7[i]);
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
  /* Create cell arrays for indexed variables. */
  dims[0] = 8;
  cell = mxCreateCellArray(1, dims);
  mxSetField(plhs[0], 0, "c0123", cell);
  dims[0] = 8;
  cell = mxCreateCellArray(1, dims);
  mxSetField(plhs[0], 0, "c4567", cell);
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c0123_1", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c0123");
  mxSetCell(cell, 0, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c0123_1;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c0123_2", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c0123");
  mxSetCell(cell, 1, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c0123_2;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c0123_3", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c0123");
  mxSetCell(cell, 2, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c0123_3;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c0123_4", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c0123");
  mxSetCell(cell, 3, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c0123_4;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c0123_5", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c0123");
  mxSetCell(cell, 4, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c0123_5;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c0123_6", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c0123");
  mxSetCell(cell, 5, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c0123_6;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c0123_7", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c0123");
  mxSetCell(cell, 6, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c0123_7;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c0123_8", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c0123");
  mxSetCell(cell, 7, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c0123_8;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c4567_1", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c4567");
  mxSetCell(cell, 0, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c4567_1;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c4567_2", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c4567");
  mxSetCell(cell, 1, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c4567_2;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c4567_3", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c4567");
  mxSetCell(cell, 2, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c4567_3;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c4567_4", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c4567");
  mxSetCell(cell, 3, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c4567_4;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c4567_5", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c4567");
  mxSetCell(cell, 4, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c4567_5;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c4567_6", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c4567");
  mxSetCell(cell, 5, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c4567_6;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c4567_7", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c4567");
  mxSetCell(cell, 6, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c4567_7;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
  xm = mxCreateDoubleMatrix(4, 1, mxREAL);
  mxSetField(plhs[0], 0, "c4567_8", xm);
  xm_cell = mxCreateDoubleMatrix(4, 1, mxREAL);
  cell = mxGetField(plhs[0], 0, "c4567");
  mxSetCell(cell, 7, xm_cell);
  dest = mxGetPr(xm);
  dest_cell = mxGetPr(xm_cell);
  src = vars.c4567_8;
  for (i = 0; i < 4; i++) {
    *dest++ = *src;
    *dest_cell++ = *src++;
  }
}
