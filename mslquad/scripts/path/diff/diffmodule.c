// interface to python
// Author: Zijian Wang, zjwang@stanford.edu
// Date: Jul 13, 2017
// Multi-Robot Systems Lab, Stanford University

#include <Python.h>
#include "diff2SttInC.h"
#include "quadDef.h"

static PyObject *diffError;

static PyObject *
diff_diff2SttIn(PyObject *self, PyObject *args)
{
    // ------------ parameters required by diff flatness
    // output
    States state;
    Inputs input;
    double ** Rd1;
    double ** Rd2;
    double * wbd1;
    double * euler; 
    double * eulerd1;
    double * eulerd2;
    // input
    Params param;
    FlatOut flatout;

    // ------------ retrieve input from python
    // parse input
    double m;
    double Ixx, Iyy, Izz, Ixy, Iyz, Ixz;
    double Ixx_i, Iyy_i, Izz_i, Ixy_i, Iyz_i, Ixz_i; // inverse terms in J^-1
    double sig[4], sigd1[4], sigd2[4], sigd3[4], sigd4[4];
    // input args:              m    J      Jinv   sig   sigd1 sigd2 sigd3 sigd4
    if (!PyArg_ParseTuple(args, "d(dddddd)(dddddd)(dddd)(dddd)(dddd)(dddd)(dddd)", 
            &m, 
            &Ixx, &Iyy, &Izz, &Ixy, &Iyz, &Ixz, // J
            &Ixx_i, &Iyy_i, &Izz_i, &Ixy_i, &Iyz_i, &Ixz_i,
            &sig[0], &sig[1], &sig[2], &sig[3],
            &sigd1[0], &sigd1[1], &sigd1[2], &sigd1[3],
            &sigd2[0], &sigd2[1], &sigd2[2], &sigd2[3],
            &sigd3[0], &sigd3[1], &sigd3[2], &sigd3[3],
            &sigd4[0], &sigd4[1], &sigd4[2], &sigd4[3]
            )) {
        return NULL;
    }
    /*
    // verify input
    printf("m = %f \n", m);
    printf("Ixx = %f, Iyy = %f, Izz = %f, Ixy = %f, Iyz = %f, Ixz = %f\n", Ixx, Iyy, Izz, Ixy, Iyz, Ixz);
    printf("Ixx_i = %f, Iyy_i = %f, Izz_i = %f, Ixy_i = %f, Iyz_i = %f, Ixz_i = %f\n", Ixx_i, Iyy_i, Izz_i, Ixy_i, Iyz_i, Ixz_i);
    printf("sig = (%f, %f, %f, %f)\n", sig[0], sig[1], sig[2], sig[3]);
    printf("sigd1 = (%f, %f, %f, %f)\n", sigd1[0], sigd1[1], sigd1[2], sigd1[3]);
    printf("sigd2 = (%f, %f, %f, %f)\n", sigd2[0], sigd2[1], sigd2[2], sigd2[3]);
    printf("sigd3 = (%f, %f, %f, %f)\n", sigd3[0], sigd3[1], sigd3[2], sigd3[3]);
    printf("sigd4 = (%f, %f, %f, %f)\n", sigd4[0], sigd4[1], sigd4[2], sigd4[3]);
    */

    // ------------ allocate memories for computation
    state.v = (double *)malloc(3 * sizeof(double));
    state.R = (double **)malloc(3 * sizeof(double*));
    state.wb = (double *)malloc(3 * sizeof(double));
    state.h = (double *)malloc(3 * sizeof(double));

    input.tau = (double *)malloc(3 * sizeof(double));

    Rd1 = (double **)malloc(3 * sizeof(double*));
    Rd2 = (double **)malloc(3 * sizeof(double*));
    for(int i=0; i<3; i++) {
        Rd1[i] = (double*)malloc(3 * sizeof(double));
        Rd2[i] = (double*)malloc(3 * sizeof(double));
        state.R[i] = (double*)malloc(3 * sizeof(double));
    }
    wbd1 = (double *)malloc(3 * sizeof(double));
    euler = (double *)malloc(3 * sizeof(double));
    eulerd1 = (double *)malloc(3 * sizeof(double));
    eulerd2 = (double *)malloc(3 * sizeof(double));   

    param.J = (double**)malloc(3 * sizeof(double*));
    param.invJ = (double**)malloc(3 * sizeof(double*));
    for(int i=0; i<3; i++){
        param.J[i] = (double*)malloc(3 * sizeof(double));
        param.invJ[i] = (double*)malloc(3 * sizeof(double));
    }

    flatout.sig = (double*)malloc(4 * sizeof(double));
    flatout.sigd1 = (double*)malloc(4 * sizeof(double));
    flatout.sigd2 = (double*)malloc(4 * sizeof(double));
    flatout.sigd3 = (double*)malloc(4 * sizeof(double));
    flatout.sigd4 = (double*)malloc(4 * sizeof(double));

    // ------------ assign values for computation
    param.m = m;

    param.J[0][0] = Ixx;
    param.J[1][1] = Iyy;
    param.J[2][2] = Izz;
    param.J[0][1] = Ixy;
    param.J[1][0] = Ixy;
    param.J[1][2] = Iyz;
    param.J[2][1] = Iyz;
    param.J[0][2] = Ixz;
    param.J[2][0] = Ixz;

    param.invJ[0][0] = Ixx_i;
    param.invJ[1][1] = Iyy_i;
    param.invJ[2][2] = Izz_i;
    param.invJ[0][1] = Ixy_i;
    param.invJ[1][0] = Ixy_i;
    param.invJ[1][2] = Iyz_i;
    param.invJ[2][1] = Iyz_i;
    param.invJ[0][2] = Ixz_i;
    param.invJ[2][0] = Ixz_i;

    for(int i=0; i<4; i++) {
        flatout.sig[i] = sig[i];
        flatout.sigd1[i] = sigd1[i];
        flatout.sigd2[i] = sigd2[i];
        flatout.sigd3[i] = sigd3[i];
        flatout.sigd4[i] = sigd4[i];
    }

    // ------------ perform computation
    diff2SttInp(&state, &input, Rd1, Rd2, wbd1,       // output 
                    euler, eulerd1, eulerd2,           // output
                    &param, &flatout);                // input


    // ------------ assemble output
    // output args:       euler  wb  pos  vel fz tau     R
    return Py_BuildValue("(ddd)(ddd)(ddd)(ddd)d(ddd)(ddddddddd)",
               euler[0], euler[1], euler[2], // euler angles
               state.wb[0], state.wb[1], state.wb[2], // wb
               state.h[0], state.h[1], state.h[2], // pos
               state.v[0], state.v[1], state.v[2], // vel
               input.fz, // fz
               input.tau[0], input.tau[1], input.tau[2], // tau
               state.R[0][0], state.R[0][1], state.R[0][2],
               state.R[1][0], state.R[1][1], state.R[1][2],
               state.R[2][0], state.R[2][1], state.R[2][2] // R
           );

}

static PyMethodDef diff_methods[] = {
    /* The cast of the function is necessary since PyCFunction values
     * only take two PyObject* parameters, and keywdarg_parrot() takes
     * three.
     */
    {"diff2SttIn", (PyCFunction)diff_diff2SttIn, METH_VARARGS,
     "docstring TBD"},
    {NULL, NULL, 0, NULL}   /* sentinel */
};

PyMODINIT_FUNC
initdiff(void)
{
    PyObject *m;

    m = Py_InitModule("diff", diff_methods);
    if (m == NULL)
        return;

    diffError = PyErr_NewException("diff.error", NULL, NULL);
    Py_INCREF(diffError);
    PyModule_AddObject(m, "error", diffError);
}