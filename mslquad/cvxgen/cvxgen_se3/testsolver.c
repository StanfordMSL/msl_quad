/* Produced by CVXGEN, 2017-09-04 14:14:14 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
  double time;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  settings.max_iters = 10;
  settings.eps = 1e-3;
  settings.resid_tol = 1e-3;
  tic();
  num_iters = solve();
  time = tocq();
  printf("Timed %d solves over %.6f seconds.\n", NUMTESTS, time);
  printf("f = %f, %f, %f, %f\n", vars.f[0], vars.f[1], vars.f[2], vars.f[3]);
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.wdes[0] = 2.45;
  params.wdes[1] = 0.0;
  params.wdes[2] = 0.1*9.81*5*0.51;
  params.wdes[3] = 0.0;
	params.W_row2[0] = -0.12;
	params.W_row2[1] = 0.12;
	params.W_row2[2] = 0.12;
	params.W_row2[3] = -0.12;
	params.W_row3[0] = 0.63;
	params.W_row3[1] = 0.39;
	params.W_row3[2] = 0.63;
	params.W_row3[3] = 0.39;
	params.W_row4[0] = 0.05;
	params.W_row4[1] = 0.05;
	params.W_row4[2] = -0.05;
	params.W_row4[3] = -0.05;
	params.FMIN[0] = 0.0;
	params.FMAX[0] = 2.5 * 9.81 / 4.0;
}

