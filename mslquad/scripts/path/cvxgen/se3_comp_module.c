#include <Python.h>
#include "solver.h"

// Distributed se3 controller with compensation

// ---------------------------------------------------- 
// These variables must be declared as global variables
// see the comments in solver.h
Vars vars;
Params params;
Workspace work;
Settings settings;
// ---------------------------------------------------- 

static PyObject *cvxse3compError;

static PyObject *
cvxgen_solve_se3_comp(PyObject *self, PyObject *args)
{
    // input args are:            W_row_2_to_4,  w_des(4), f_min, f_max, Wy_LB, Wy_UB
    if (!PyArg_ParseTuple(args, "(dddddddddddd)(dddd)(d)(d)(d)(d)", 
            &params.W_row2[0], &params.W_row2[1], &params.W_row2[2], &params.W_row2[3],
            &params.W_row3[0], &params.W_row3[1], &params.W_row3[2], &params.W_row3[3],
            &params.W_row4[0], &params.W_row4[1], &params.W_row4[2], &params.W_row4[3],
            &params.wdes[0], &params.wdes[1], &params.wdes[2], &params.wdes[3],
            &params.FMIN[0],
            &params.FMAX[0],
            &params.Wy_LB[0],
            &params.Wy_UB[0])) {
        return NULL;
    }    

    // must-calls of cvxgen
    set_defaults();
    setup_indexing();
    

    //settings.max_iters = 30;
    settings.verbose = 0;
    solve();
    
    return Py_BuildValue("(dddd)i",
               vars.f[0], vars.f[1], vars.f[2], vars.f[3],
               work.converged
           );
}

static PyMethodDef cvxse3comp_methods[] = {
    /* The cast of the function is necessary since PyCFunction values
     * only take two PyObject* parameters, and keywdarg_parrot() takes
     * three.
     */
    {"solve_se3_comp", (PyCFunction)cvxgen_solve_se3_comp, METH_VARARGS,
     "docstring TBD"},
    {NULL, NULL, 0, NULL}   /* sentinel */
};

PyMODINIT_FUNC
initcvxse3comp(void)
{
    PyObject *m;

    m = Py_InitModule("cvxse3comp", cvxse3comp_methods);
    if (m == NULL)
        return;

    cvxse3compError = PyErr_NewException("cvxse3comp.error", NULL, NULL);
    Py_INCREF(cvxse3compError);
    PyModule_AddObject(m, "error", cvxse3compError);
}