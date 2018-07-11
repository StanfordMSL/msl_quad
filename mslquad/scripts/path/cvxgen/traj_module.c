#include <Python.h>
#include "solver.h"

// ---------------------------------------------------- 
// These variables must be declared as global variables
// see the comments in solver.h
Vars vars;
Params params;
Workspace work;
Settings settings;
// ---------------------------------------------------- 

static PyObject *cvxtrajError;

static PyObject *
cvxgen_solve_traj(PyObject *self, PyObject *args)
{
    // Important: only support up to 8 sections.
    // input args are:
    // Q matrices: 8 x (4,4)
    // wpts: (9)
    // tf0123[i] (4), tf4567[i] (4), i=1..8: 8x8
    // final_v_c0123[i](4), final_v_c4567[i](4), and for acc, jerk, snap, i=3..8: 8x4x6
    // v_cont_c0123_1[i](4), v_cont_c4567[i](4), and for acc, jerk, snap, dsnap, i=1..7: 8x5x7
    if (!PyArg_ParseTuple(args, 
            "(dddddddddddddddd)(dddddddddddddddd)(dddddddddddddddd)(dddddddddddddddd)(dddddddddddddddd)(dddddddddddddddd)(dddddddddddddddd)(dddddddddddddddd)(ddddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)(dddddddd)", 
            &params.Q_1[0], &params.Q_1[1], &params.Q_1[2], &params.Q_1[3],
            &params.Q_1[4], &params.Q_1[5], &params.Q_1[6], &params.Q_1[7],
            &params.Q_1[8], &params.Q_1[9], &params.Q_1[10], &params.Q_1[11],
            &params.Q_1[12], &params.Q_1[13], &params.Q_1[14], &params.Q_1[15],
            &params.Q_2[0], &params.Q_2[1], &params.Q_2[2], &params.Q_2[3],
            &params.Q_2[4], &params.Q_2[5], &params.Q_2[6], &params.Q_2[7],
            &params.Q_2[8], &params.Q_2[9], &params.Q_2[10], &params.Q_2[11],
            &params.Q_2[12], &params.Q_2[13], &params.Q_2[14], &params.Q_2[15],
            &params.Q_3[0], &params.Q_3[1], &params.Q_3[2], &params.Q_3[3],
            &params.Q_3[4], &params.Q_3[5], &params.Q_3[6], &params.Q_3[7],
            &params.Q_3[8], &params.Q_3[9], &params.Q_3[10], &params.Q_3[11],
            &params.Q_3[12], &params.Q_3[13], &params.Q_3[14], &params.Q_3[15],
            &params.Q_4[0], &params.Q_4[1], &params.Q_4[2], &params.Q_4[3],
            &params.Q_4[4], &params.Q_4[5], &params.Q_4[6], &params.Q_4[7],
            &params.Q_4[8], &params.Q_4[9], &params.Q_4[10], &params.Q_4[11],
            &params.Q_4[12], &params.Q_4[13], &params.Q_4[14], &params.Q_4[15],
            &params.Q_5[0], &params.Q_5[1], &params.Q_5[2], &params.Q_5[3],
            &params.Q_5[4], &params.Q_5[5], &params.Q_5[6], &params.Q_5[7],
            &params.Q_5[8], &params.Q_5[9], &params.Q_5[10], &params.Q_5[11],
            &params.Q_5[12], &params.Q_5[13], &params.Q_5[14], &params.Q_5[15],
            &params.Q_6[0], &params.Q_6[1], &params.Q_6[2], &params.Q_6[3],
            &params.Q_6[4], &params.Q_6[5], &params.Q_6[6], &params.Q_6[7],
            &params.Q_6[8], &params.Q_6[9], &params.Q_6[10], &params.Q_6[11],
            &params.Q_6[12], &params.Q_6[13], &params.Q_6[14], &params.Q_6[15],
            &params.Q_7[0], &params.Q_7[1], &params.Q_7[2], &params.Q_7[3],
            &params.Q_7[4], &params.Q_7[5], &params.Q_7[6], &params.Q_7[7],
            &params.Q_7[8], &params.Q_7[9], &params.Q_7[10], &params.Q_7[11],
            &params.Q_7[12], &params.Q_7[13], &params.Q_7[14], &params.Q_7[15],
            &params.Q_8[0], &params.Q_8[1], &params.Q_8[2], &params.Q_8[3],
            &params.Q_8[4], &params.Q_8[5], &params.Q_8[6], &params.Q_8[7],
            &params.Q_8[8], &params.Q_8[9], &params.Q_8[10], &params.Q_8[11],
            &params.Q_8[12], &params.Q_8[13], &params.Q_8[14], &params.Q_8[15],

            &params.wpts[0], &params.wpts[1], &params.wpts[2], &params.wpts[3], &params.wpts[4],
            &params.wpts[5], &params.wpts[6], &params.wpts[7], &params.wpts[8],

            &params.tf0123_1[0], &params.tf0123_1[1], &params.tf0123_1[2], &params.tf0123_1[3],
            &params.tf4567_1[0], &params.tf4567_1[1], &params.tf4567_1[2], &params.tf4567_1[3], 
            &params.tf0123_2[0], &params.tf0123_2[1], &params.tf0123_2[2], &params.tf0123_2[3],
            &params.tf4567_2[0], &params.tf4567_2[1], &params.tf4567_2[2], &params.tf4567_2[3],
            &params.tf0123_3[0], &params.tf0123_3[1], &params.tf0123_3[2], &params.tf0123_3[3],
            &params.tf4567_3[0], &params.tf4567_3[1], &params.tf4567_3[2], &params.tf4567_3[3], 
            &params.tf0123_4[0], &params.tf0123_4[1], &params.tf0123_4[2], &params.tf0123_4[3],
            &params.tf4567_4[0], &params.tf4567_4[1], &params.tf4567_4[2], &params.tf4567_4[3], 
            &params.tf0123_5[0], &params.tf0123_5[1], &params.tf0123_5[2], &params.tf0123_5[3],
            &params.tf4567_5[0], &params.tf4567_5[1], &params.tf4567_5[2], &params.tf4567_5[3], 
            &params.tf0123_6[0], &params.tf0123_6[1], &params.tf0123_6[2], &params.tf0123_6[3],
            &params.tf4567_6[0], &params.tf4567_6[1], &params.tf4567_6[2], &params.tf4567_6[3], 
            &params.tf0123_7[0], &params.tf0123_7[1], &params.tf0123_7[2], &params.tf0123_7[3],
            &params.tf4567_7[0], &params.tf4567_7[1], &params.tf4567_7[2], &params.tf4567_7[3], 
            &params.tf0123_8[0], &params.tf0123_8[1], &params.tf0123_8[2], &params.tf0123_8[3],
            &params.tf4567_8[0], &params.tf4567_8[1], &params.tf4567_8[2], &params.tf4567_8[3],

            &params.final_v_c0123_3[0], &params.final_v_c0123_3[1], &params.final_v_c0123_3[2], &params.final_v_c0123_3[3],
            &params.final_v_c4567_3[0], &params.final_v_c4567_3[1], &params.final_v_c4567_3[2], &params.final_v_c4567_3[3], 
            &params.final_a_c0123_3[0], &params.final_a_c0123_3[1], &params.final_a_c0123_3[2], &params.final_a_c0123_3[3],
            &params.final_a_c4567_3[0], &params.final_a_c4567_3[1], &params.final_a_c4567_3[2], &params.final_a_c4567_3[3],
            &params.final_j_c0123_3[0], &params.final_j_c0123_3[1], &params.final_j_c0123_3[2], &params.final_j_c0123_3[3],
            &params.final_j_c4567_3[0], &params.final_j_c4567_3[1], &params.final_j_c4567_3[2], &params.final_j_c4567_3[3],
            &params.final_s_c0123_3[0], &params.final_s_c0123_3[1], &params.final_s_c0123_3[2], &params.final_s_c0123_3[3],
            &params.final_s_c4567_3[0], &params.final_s_c4567_3[1], &params.final_s_c4567_3[2], &params.final_s_c4567_3[3],
            
            &params.final_v_c0123_4[0], &params.final_v_c0123_4[1], &params.final_v_c0123_4[2], &params.final_v_c0123_4[3],
            &params.final_v_c4567_4[0], &params.final_v_c4567_4[1], &params.final_v_c4567_4[2], &params.final_v_c4567_4[3], 
            &params.final_a_c0123_4[0], &params.final_a_c0123_4[1], &params.final_a_c0123_4[2], &params.final_a_c0123_4[3],
            &params.final_a_c4567_4[0], &params.final_a_c4567_4[1], &params.final_a_c4567_4[2], &params.final_a_c4567_4[3],
            &params.final_j_c0123_4[0], &params.final_j_c0123_4[1], &params.final_j_c0123_4[2], &params.final_j_c0123_4[3],
            &params.final_j_c4567_4[0], &params.final_j_c4567_4[1], &params.final_j_c4567_4[2], &params.final_j_c4567_4[3],
            &params.final_s_c0123_4[0], &params.final_s_c0123_4[1], &params.final_s_c0123_4[2], &params.final_s_c0123_4[3],
            &params.final_s_c4567_4[0], &params.final_s_c4567_4[1], &params.final_s_c4567_4[2], &params.final_s_c4567_4[3],
    
            &params.final_v_c0123_5[0], &params.final_v_c0123_5[1], &params.final_v_c0123_5[2], &params.final_v_c0123_5[3],
            &params.final_v_c4567_5[0], &params.final_v_c4567_5[1], &params.final_v_c4567_5[2], &params.final_v_c4567_5[3], 
            &params.final_a_c0123_5[0], &params.final_a_c0123_5[1], &params.final_a_c0123_5[2], &params.final_a_c0123_5[3],
            &params.final_a_c4567_5[0], &params.final_a_c4567_5[1], &params.final_a_c4567_5[2], &params.final_a_c4567_5[3],
            &params.final_j_c0123_5[0], &params.final_j_c0123_5[1], &params.final_j_c0123_5[2], &params.final_j_c0123_5[3],
            &params.final_j_c4567_5[0], &params.final_j_c4567_5[1], &params.final_j_c4567_5[2], &params.final_j_c4567_5[3],
            &params.final_s_c0123_5[0], &params.final_s_c0123_5[1], &params.final_s_c0123_5[2], &params.final_s_c0123_5[3],
            &params.final_s_c4567_5[0], &params.final_s_c4567_5[1], &params.final_s_c4567_5[2], &params.final_s_c4567_5[3],

            &params.final_v_c0123_6[0], &params.final_v_c0123_6[1], &params.final_v_c0123_6[2], &params.final_v_c0123_6[3],
            &params.final_v_c4567_6[0], &params.final_v_c4567_6[1], &params.final_v_c4567_6[2], &params.final_v_c4567_6[3], 
            &params.final_a_c0123_6[0], &params.final_a_c0123_6[1], &params.final_a_c0123_6[2], &params.final_a_c0123_6[3],
            &params.final_a_c4567_6[0], &params.final_a_c4567_6[1], &params.final_a_c4567_6[2], &params.final_a_c4567_6[3],
            &params.final_j_c0123_6[0], &params.final_j_c0123_6[1], &params.final_j_c0123_6[2], &params.final_j_c0123_6[3],
            &params.final_j_c4567_6[0], &params.final_j_c4567_6[1], &params.final_j_c4567_6[2], &params.final_j_c4567_6[3],
            &params.final_s_c0123_6[0], &params.final_s_c0123_6[1], &params.final_s_c0123_6[2], &params.final_s_c0123_6[3],
            &params.final_s_c4567_6[0], &params.final_s_c4567_6[1], &params.final_s_c4567_6[2], &params.final_s_c4567_6[3],

            &params.final_v_c0123_7[0], &params.final_v_c0123_7[1], &params.final_v_c0123_7[2], &params.final_v_c0123_7[3],
            &params.final_v_c4567_7[0], &params.final_v_c4567_7[1], &params.final_v_c4567_7[2], &params.final_v_c4567_7[3], 
            &params.final_a_c0123_7[0], &params.final_a_c0123_7[1], &params.final_a_c0123_7[2], &params.final_a_c0123_7[3],
            &params.final_a_c4567_7[0], &params.final_a_c4567_7[1], &params.final_a_c4567_7[2], &params.final_a_c4567_7[3],
            &params.final_j_c0123_7[0], &params.final_j_c0123_7[1], &params.final_j_c0123_7[2], &params.final_j_c0123_7[3],
            &params.final_j_c4567_7[0], &params.final_j_c4567_7[1], &params.final_j_c4567_7[2], &params.final_j_c4567_7[3],
            &params.final_s_c0123_7[0], &params.final_s_c0123_7[1], &params.final_s_c0123_7[2], &params.final_s_c0123_7[3],
            &params.final_s_c4567_7[0], &params.final_s_c4567_7[1], &params.final_s_c4567_7[2], &params.final_s_c4567_7[3],
    
            &params.final_v_c0123_8[0], &params.final_v_c0123_8[1], &params.final_v_c0123_8[2], &params.final_v_c0123_8[3],
            &params.final_v_c4567_8[0], &params.final_v_c4567_8[1], &params.final_v_c4567_8[2], &params.final_v_c4567_8[3], 
            &params.final_a_c0123_8[0], &params.final_a_c0123_8[1], &params.final_a_c0123_8[2], &params.final_a_c0123_8[3],
            &params.final_a_c4567_8[0], &params.final_a_c4567_8[1], &params.final_a_c4567_8[2], &params.final_a_c4567_8[3],
            &params.final_j_c0123_8[0], &params.final_j_c0123_8[1], &params.final_j_c0123_8[2], &params.final_j_c0123_8[3],
            &params.final_j_c4567_8[0], &params.final_j_c4567_8[1], &params.final_j_c4567_8[2], &params.final_j_c4567_8[3],
            &params.final_s_c0123_8[0], &params.final_s_c0123_8[1], &params.final_s_c0123_8[2], &params.final_s_c0123_8[3],
            &params.final_s_c4567_8[0], &params.final_s_c4567_8[1], &params.final_s_c4567_8[2], &params.final_s_c4567_8[3],

            &params.v_cont_c0123_1[0], &params.v_cont_c0123_1[1], &params.v_cont_c0123_1[2], &params.v_cont_c0123_1[3],
            &params.v_cont_c4567_1[0], &params.v_cont_c4567_1[1], &params.v_cont_c4567_1[2], &params.v_cont_c4567_1[3],
            &params.a_cont_c0123_1[0], &params.a_cont_c0123_1[1], &params.a_cont_c0123_1[2], &params.a_cont_c0123_1[3],
            &params.a_cont_c4567_1[0], &params.a_cont_c4567_1[1], &params.a_cont_c4567_1[2], &params.a_cont_c4567_1[3],
            &params.j_cont_c0123_1[0], &params.j_cont_c0123_1[1], &params.j_cont_c0123_1[2], &params.j_cont_c0123_1[3],
            &params.j_cont_c4567_1[0], &params.j_cont_c4567_1[1], &params.j_cont_c4567_1[2], &params.j_cont_c4567_1[3],
            &params.s_cont_c0123_1[0], &params.s_cont_c0123_1[1], &params.s_cont_c0123_1[2], &params.s_cont_c0123_1[3],
            &params.s_cont_c4567_1[0], &params.s_cont_c4567_1[1], &params.s_cont_c4567_1[2], &params.s_cont_c4567_1[3],
            &params.ds_cont_c0123_1[0], &params.ds_cont_c0123_1[1], &params.ds_cont_c0123_1[2], &params.ds_cont_c0123_1[3],
            &params.ds_cont_c4567_1[0], &params.ds_cont_c4567_1[1], &params.ds_cont_c4567_1[2], &params.ds_cont_c4567_1[3],
            
            &params.v_cont_c0123_2[0], &params.v_cont_c0123_2[1], &params.v_cont_c0123_2[2], &params.v_cont_c0123_2[3],
            &params.v_cont_c4567_2[0], &params.v_cont_c4567_2[1], &params.v_cont_c4567_2[2], &params.v_cont_c4567_2[3],
            &params.a_cont_c0123_2[0], &params.a_cont_c0123_2[1], &params.a_cont_c0123_2[2], &params.a_cont_c0123_2[3],
            &params.a_cont_c4567_2[0], &params.a_cont_c4567_2[1], &params.a_cont_c4567_2[2], &params.a_cont_c4567_2[3],
            &params.j_cont_c0123_2[0], &params.j_cont_c0123_2[1], &params.j_cont_c0123_2[2], &params.j_cont_c0123_2[3],
            &params.j_cont_c4567_2[0], &params.j_cont_c4567_2[1], &params.j_cont_c4567_2[2], &params.j_cont_c4567_2[3],
            &params.s_cont_c0123_2[0], &params.s_cont_c0123_2[1], &params.s_cont_c0123_2[2], &params.s_cont_c0123_2[3],
            &params.s_cont_c4567_2[0], &params.s_cont_c4567_2[1], &params.s_cont_c4567_2[2], &params.s_cont_c4567_2[3],
            &params.ds_cont_c0123_2[0], &params.ds_cont_c0123_2[1], &params.ds_cont_c0123_2[2], &params.ds_cont_c0123_2[3],
            &params.ds_cont_c4567_2[0], &params.ds_cont_c4567_2[1], &params.ds_cont_c4567_2[2], &params.ds_cont_c4567_2[3],

            &params.v_cont_c0123_3[0], &params.v_cont_c0123_3[1], &params.v_cont_c0123_3[2], &params.v_cont_c0123_3[3],
            &params.v_cont_c4567_3[0], &params.v_cont_c4567_3[1], &params.v_cont_c4567_3[2], &params.v_cont_c4567_3[3],
            &params.a_cont_c0123_3[0], &params.a_cont_c0123_3[1], &params.a_cont_c0123_3[2], &params.a_cont_c0123_3[3],
            &params.a_cont_c4567_3[0], &params.a_cont_c4567_3[1], &params.a_cont_c4567_3[2], &params.a_cont_c4567_3[3],
            &params.j_cont_c0123_3[0], &params.j_cont_c0123_3[1], &params.j_cont_c0123_3[2], &params.j_cont_c0123_3[3],
            &params.j_cont_c4567_3[0], &params.j_cont_c4567_3[1], &params.j_cont_c4567_3[2], &params.j_cont_c4567_3[3],
            &params.s_cont_c0123_3[0], &params.s_cont_c0123_3[1], &params.s_cont_c0123_3[2], &params.s_cont_c0123_3[3],
            &params.s_cont_c4567_3[0], &params.s_cont_c4567_3[1], &params.s_cont_c4567_3[2], &params.s_cont_c4567_3[3],
            &params.ds_cont_c0123_3[0], &params.ds_cont_c0123_3[1], &params.ds_cont_c0123_3[2], &params.ds_cont_c0123_3[3],
            &params.ds_cont_c4567_3[0], &params.ds_cont_c4567_3[1], &params.ds_cont_c4567_3[2], &params.ds_cont_c4567_3[3],

            &params.v_cont_c0123_4[0], &params.v_cont_c0123_4[1], &params.v_cont_c0123_4[2], &params.v_cont_c0123_4[3],
            &params.v_cont_c4567_4[0], &params.v_cont_c4567_4[1], &params.v_cont_c4567_4[2], &params.v_cont_c4567_4[3],
            &params.a_cont_c0123_4[0], &params.a_cont_c0123_4[1], &params.a_cont_c0123_4[2], &params.a_cont_c0123_4[3],
            &params.a_cont_c4567_4[0], &params.a_cont_c4567_4[1], &params.a_cont_c4567_4[2], &params.a_cont_c4567_4[3],
            &params.j_cont_c0123_4[0], &params.j_cont_c0123_4[1], &params.j_cont_c0123_4[2], &params.j_cont_c0123_4[3],
            &params.j_cont_c4567_4[0], &params.j_cont_c4567_4[1], &params.j_cont_c4567_4[2], &params.j_cont_c4567_4[3],
            &params.s_cont_c0123_4[0], &params.s_cont_c0123_4[1], &params.s_cont_c0123_4[2], &params.s_cont_c0123_4[3],
            &params.s_cont_c4567_4[0], &params.s_cont_c4567_4[1], &params.s_cont_c4567_4[2], &params.s_cont_c4567_4[3],
            &params.ds_cont_c0123_4[0], &params.ds_cont_c0123_4[1], &params.ds_cont_c0123_4[2], &params.ds_cont_c0123_4[3],
            &params.ds_cont_c4567_4[0], &params.ds_cont_c4567_4[1], &params.ds_cont_c4567_4[2], &params.ds_cont_c4567_4[3],
        
            &params.v_cont_c0123_5[0], &params.v_cont_c0123_5[1], &params.v_cont_c0123_5[2], &params.v_cont_c0123_5[3],
            &params.v_cont_c4567_5[0], &params.v_cont_c4567_5[1], &params.v_cont_c4567_5[2], &params.v_cont_c4567_5[3],
            &params.a_cont_c0123_5[0], &params.a_cont_c0123_5[1], &params.a_cont_c0123_5[2], &params.a_cont_c0123_5[3],
            &params.a_cont_c4567_5[0], &params.a_cont_c4567_5[1], &params.a_cont_c4567_5[2], &params.a_cont_c4567_5[3],
            &params.j_cont_c0123_5[0], &params.j_cont_c0123_5[1], &params.j_cont_c0123_5[2], &params.j_cont_c0123_5[3],
            &params.j_cont_c4567_5[0], &params.j_cont_c4567_5[1], &params.j_cont_c4567_5[2], &params.j_cont_c4567_5[3],
            &params.s_cont_c0123_5[0], &params.s_cont_c0123_5[1], &params.s_cont_c0123_5[2], &params.s_cont_c0123_5[3],
            &params.s_cont_c4567_5[0], &params.s_cont_c4567_5[1], &params.s_cont_c4567_5[2], &params.s_cont_c4567_5[3],
            &params.ds_cont_c0123_5[0], &params.ds_cont_c0123_5[1], &params.ds_cont_c0123_5[2], &params.ds_cont_c0123_5[3],
            &params.ds_cont_c4567_5[0], &params.ds_cont_c4567_5[1], &params.ds_cont_c4567_5[2], &params.ds_cont_c4567_5[3],

            &params.v_cont_c0123_6[0], &params.v_cont_c0123_6[1], &params.v_cont_c0123_6[2], &params.v_cont_c0123_6[3],
            &params.v_cont_c4567_6[0], &params.v_cont_c4567_6[1], &params.v_cont_c4567_6[2], &params.v_cont_c4567_6[3],
            &params.a_cont_c0123_6[0], &params.a_cont_c0123_6[1], &params.a_cont_c0123_6[2], &params.a_cont_c0123_6[3],
            &params.a_cont_c4567_6[0], &params.a_cont_c4567_6[1], &params.a_cont_c4567_6[2], &params.a_cont_c4567_6[3],
            &params.j_cont_c0123_6[0], &params.j_cont_c0123_6[1], &params.j_cont_c0123_6[2], &params.j_cont_c0123_6[3],
            &params.j_cont_c4567_6[0], &params.j_cont_c4567_6[1], &params.j_cont_c4567_6[2], &params.j_cont_c4567_6[3],
            &params.s_cont_c0123_6[0], &params.s_cont_c0123_6[1], &params.s_cont_c0123_6[2], &params.s_cont_c0123_6[3],
            &params.s_cont_c4567_6[0], &params.s_cont_c4567_6[1], &params.s_cont_c4567_6[2], &params.s_cont_c4567_6[3],
            &params.ds_cont_c0123_6[0], &params.ds_cont_c0123_6[1], &params.ds_cont_c0123_6[2], &params.ds_cont_c0123_6[3],
            &params.ds_cont_c4567_6[0], &params.ds_cont_c4567_6[1], &params.ds_cont_c4567_6[2], &params.ds_cont_c4567_6[3],

            &params.v_cont_c0123_7[0], &params.v_cont_c0123_7[1], &params.v_cont_c0123_7[2], &params.v_cont_c0123_7[3],
            &params.v_cont_c4567_7[0], &params.v_cont_c4567_7[1], &params.v_cont_c4567_7[2], &params.v_cont_c4567_7[3],
            &params.a_cont_c0123_7[0], &params.a_cont_c0123_7[1], &params.a_cont_c0123_7[2], &params.a_cont_c0123_7[3],
            &params.a_cont_c4567_7[0], &params.a_cont_c4567_7[1], &params.a_cont_c4567_7[2], &params.a_cont_c4567_7[3],
            &params.j_cont_c0123_7[0], &params.j_cont_c0123_7[1], &params.j_cont_c0123_7[2], &params.j_cont_c0123_7[3],
            &params.j_cont_c4567_7[0], &params.j_cont_c4567_7[1], &params.j_cont_c4567_7[2], &params.j_cont_c4567_7[3],
            &params.s_cont_c0123_7[0], &params.s_cont_c0123_7[1], &params.s_cont_c0123_7[2], &params.s_cont_c0123_7[3],
            &params.s_cont_c4567_7[0], &params.s_cont_c4567_7[1], &params.s_cont_c4567_7[2], &params.s_cont_c4567_7[3],
            &params.ds_cont_c0123_7[0], &params.ds_cont_c0123_7[1], &params.ds_cont_c0123_7[2], &params.ds_cont_c0123_7[3],
            &params.ds_cont_c4567_7[0], &params.ds_cont_c4567_7[1], &params.ds_cont_c4567_7[2], &params.ds_cont_c4567_7[3]
        )) {
        return NULL;
    }    

    // must-calls of cvxgen
    set_defaults();
    setup_indexing();
    

    //settings.max_iters = 30;
    settings.verbose = 0;
    settings.eps = 1e-2;
    solve();
    
    // return
    // 8 coefficients per section
    // converge or not 
    return Py_BuildValue("(dddddddd)\
                          (dddddddd)\
                          (dddddddd)\
                          (dddddddd)\
                          (dddddddd)\
                          (dddddddd)\
                          (dddddddd)\
                          (dddddddd)\
                          i",
        vars.c0123_1[0], vars.c0123_1[1], vars.c0123_1[2], vars.c0123_1[3], 
        vars.c4567_1[0], vars.c4567_1[1], vars.c4567_1[2], vars.c4567_1[3],
        vars.c0123_2[0], vars.c0123_2[1], vars.c0123_2[2], vars.c0123_2[3], 
        vars.c4567_2[0], vars.c4567_2[1], vars.c4567_2[2], vars.c4567_2[3],
        vars.c0123_3[0], vars.c0123_3[1], vars.c0123_3[2], vars.c0123_3[3], 
        vars.c4567_3[0], vars.c4567_3[1], vars.c4567_3[2], vars.c4567_3[3],
        vars.c0123_4[0], vars.c0123_4[1], vars.c0123_4[2], vars.c0123_4[3], 
        vars.c4567_4[0], vars.c4567_4[1], vars.c4567_4[2], vars.c4567_4[3],
        vars.c0123_5[0], vars.c0123_5[1], vars.c0123_5[2], vars.c0123_5[3], 
        vars.c4567_5[0], vars.c4567_5[1], vars.c4567_5[2], vars.c4567_5[3],
        vars.c0123_6[0], vars.c0123_6[1], vars.c0123_6[2], vars.c0123_6[3], 
        vars.c4567_6[0], vars.c4567_6[1], vars.c4567_6[2], vars.c4567_6[3],
        vars.c0123_7[0], vars.c0123_7[1], vars.c0123_7[2], vars.c0123_7[3], 
        vars.c4567_7[0], vars.c4567_7[1], vars.c4567_7[2], vars.c4567_7[3],
        vars.c0123_8[0], vars.c0123_8[1], vars.c0123_8[2], vars.c0123_8[3], 
        vars.c4567_8[0], vars.c4567_8[1], vars.c4567_8[2], vars.c4567_8[3],
        work.converged
    );
}

static PyMethodDef cvxtraj_methods[] = {
    /* The cast of the function is necessary since PyCFunction values
     * only take two PyObject* parameters, and keywdarg_parrot() takes
     * three.
     */
    {"solve_traj", (PyCFunction)cvxgen_solve_traj, METH_VARARGS,
     "docstring TBD"},
    {NULL, NULL, 0, NULL}   /* sentinel */
};

PyMODINIT_FUNC
initcvxtraj(void)
{
    PyObject *m;

    m = Py_InitModule("cvxtraj", cvxtraj_methods);
    if (m == NULL)
        return;

    cvxtrajError = PyErr_NewException("cvxtraj.error", NULL, NULL);
    Py_INCREF(cvxtrajError);
    PyModule_AddObject(m, "error", cvxtrajError);
}