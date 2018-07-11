/**************************************************************************
   File Name      : dMath.c
   Author         : Dingjiang Zhou
                    Boston University, Boston, 02215
   Contact        : zhoudingjiang@gmail.com
   Create Time    : Mon, Aug. 19th, 2013. 05:15:49 PM
   Last Modified  : Mon, Aug. 19th, 2013. 05:15:49 PM
   Purpose        : 
   Example Code   : 
   Example Output : 
**************************************************************************/
//#include "mexDef.h"  /* if used for pure C code, comment this line out */
#include "dMath.h"

void vecPrint(double * v, uchar size)
{
    uchar i;
    for(i=0;i<size;i++){
        #ifdef __MEX__
                mexPrintf("%d:\t%4.4g\n",i+1,v[i]);
        #else
                printf("%d:\t%4.4g\n",i+1,v[i]);
        #endif
    }
}


/* if the matrix is defined as double F[5][5], then this file cannot be 
 * used to print that matrix */
void mtrxPrint(double **m, uchar rows, uchar cols)
{
    uchar i,j;
    for(i=0;i<rows;i++){
        #ifdef __MEX__
                mexPrintf("row %d:\t",i+1);
                for(j=0;j<cols;j++)
                    mexPrintf("%-6.6f\t",m[i][j]);
                mexPrintf("\n");
        #else
                printf("row %d:\t",i+1);
                for(j=0;j<cols;j++)
                    printf("%-6.6f\t",m[i][j]);
                printf("\n");
        #endif
                        
    }
}

double  *vecBlk(double * a, uchar col1, uchar col2)
{
    uchar size = col2 -col1 + 1;
    uchar i;
    double *b = (double*)malloc(size*sizeof(double));
    for(i=0;i<size;i++)
        b[i] = a[i+col1-1];
    return b;
}

int vecBlkp(double * b, double * a, uchar col1, uchar col2)
{
    uchar size = col2 -col1 + 1;
    uchar i;
    for(i=0;i<size;i++)
        b[i] = a[i+col1-1];
    return 0;
}

/* all vectors are considered as row vector and also column vector */
/* size of v is rows */
/* momery of vm must pre-allocated */
int vecMtrxp(double vm[], double v[], double ** m, uchar rows, uchar cols)
{
    uchar i,j;
    for(i=0;i<cols;i++){
        vm[i] = 0;
        for(j=0;j<rows;j++)
            vm[i] += v[j]*m[j][i];
    }
    return 0;    
}

double *vecMtrx(double * v, double ** m, uchar rows, uchar cols)
{
    double *vm = (double*)malloc(cols*sizeof(double));
    uchar i,j;
    for(i=0;i<cols;i++){
        vm[i] = 0;
        for(j=0;j<cols;j++)
            vm[i] += v[j]*m[j][i]; /* vm is a row vector */
    }
    return vm;
}

/* only work for 3X3 matrices */
double *vecMtrx3(double * v, double ** m)
{
    double *vm = (double*)malloc(3*sizeof(double));
    
    
    vm[0] = m[0][0]*v[0] + m[1][0]*v[1] + m[2][0]*v[2];
    vm[1] = m[0][1]*v[0] + m[1][1]*v[1] + m[2][1]*v[2];
    vm[2] = m[0][2]*v[0] + m[1][2]*v[1] + m[2][2]*v[2];
    return vm;
}

int vecMtrx3p(double * vm, double * v, double ** m)
{
    vm[0] = m[0][0]*v[0] + m[1][0]*v[1] + m[2][0]*v[2];
    vm[1] = m[0][1]*v[0] + m[1][1]*v[1] + m[2][1]*v[2];
    vm[2] = m[0][2]*v[0] + m[1][2]*v[1] + m[2][2]*v[2];
    return 0;
}

double *mtrxVec(double ** m, uchar rows, uchar cols, double * v)
{
    double * mv = (double*)malloc(rows*sizeof(double));
    uchar i,j;
    for(i=0;i<rows;i++){
        mv[i] = 0;
        for(j=0;j<cols;j++)
            mv[i] += m[i][j]*v[j];
    }
    return mv;
}

/* only work for 3X3 matrices */
double *mtrxVec3(double ** m, double * v)
{
    double * mv = (double*)malloc(3*sizeof(double));
    mv[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2];
    mv[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2];
    mv[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2];
    return mv;
}

int mtrxVec3p(double * mv, double ** m, double * v)
{
    mv[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2];
    mv[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2];
    mv[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2];
    return 0;
}
/* only work for 12X12 matrices */
double *mtrxVec12(double ** m, double * v)
{
    double * mv = (double*)malloc(12*sizeof(double));
    uchar i,j;
    for(i=0;i<12;i++){
        mv[i] = 0;
        for(j=0;j<12;j++)
            mv[i] += m[i][j]*v[j];
    }
    return mv;
}

int mtrxVec12p(double * mv, double ** m, double * v)
{
    uchar i,j;
    for(i=0;i<12;i++){
        mv[i] = 0;
        for(j=0;j<12;j++)
            mv[i] += m[i][j]*v[j];
    }
    return 0;
}

int mtrxVec12pMxOdr(double * mv, double * mMex, double * v)
{
    uchar i,j;
    for(i=0;i<12;i++){
        mv[i] = 0;
        for(j=0;j<12;j++)
            mv[i] += mMex[i+j*12]*v[j];
    }
    return 0;
}

double **mtrxMtrx(double ** m, uchar rows_m, uchar cols_m, double ** n, uchar rows_n, uchar cols_n)
{
    if(cols_m != rows_n){
        #ifdef __MEX__
                mexPrintf("mtrxMtrx: incompatible matrice dimensions\n");
        #else
                printf("mtrxMtrx: incompatible matrice dimensions\n");
        #endif
    }
    uchar i,j,k;
    double tmp;
    double **mn = (double**)malloc(rows_m*sizeof(double*));
    for(i=0;i<rows_m;i++){
        mn[i] = (double*)malloc(cols_n*sizeof(double));
        for(j=0;j<cols_n;j++){
            tmp = 0;
            for(k=0;k<cols_m;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return mn;
}

/* only work for 3X3 matrices */
double **mtrxMtrx3(double ** m, double ** n)
{
    uchar i,j,k;
    double tmp;
    double **mn = (double**)malloc(3*sizeof(double*));
    for(i=0;i<3;i++){
        mn[i] = (double*)malloc(3*sizeof(double));
        for(j=0;j<3;j++){
            tmp = 0;
            for(k=0;k<3;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return mn;
}

int mtrxMtrx2p(double ** mn, double ** m, double ** n)
{
    uchar i,j,k;
    double tmp;
    for(i=0;i<2;i++){
        for(j=0;j<2;j++){
            tmp = 0;
            for(k=0;k<2;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return 0;
}

int mtrxMtrx3p(double ** mn, double ** m, double ** n)
{
    uchar i,j,k;
    double tmp;
    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            tmp = 0;
            for(k=0;k<3;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return 0;
}

int mtrxMtrx6p(double ** mn, double ** m, double ** n)
{
    uchar i,j,k;
    double tmp;
    for(i=0;i<6;i++){
        for(j=0;j<6;j++){
            tmp = 0;
            for(k=0;k<6;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return 0;
}

/* only work for 12X12 matrices */
double **mtrxMtrx12(double ** m, double ** n)
{
    uchar i,j,k;
    double tmp;
    double **mn = (double**)malloc(12*sizeof(double*));
    for(i=0;i<12;i++){
        mn[i] = (double*)malloc(12*sizeof(double));
        for(j=0;j<12;j++){
            tmp = 0;
            for(k=0;k<12;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return mn;
}

int mtrxMtrx12p(double ** mn, double ** m, double ** n)
{
    uchar i,j,k;
    double tmp;
    for(i=0;i<12;i++){
        for(j=0;j<12;j++){
            tmp = 0;
            for(k=0;k<12;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return 0;
}

int mtrxMtrx12pMxOdrFront(double ** mn, double * mMex, double ** n)
{
    uchar i,j,k;
    double tmp;
    for(i=0;i<12;i++){
        for(j=0;j<12;j++){
            tmp = 0;
            for(k=0;k<12;k++){
                tmp += mMex[i+k*12]*n[k][j];
            }
            mn[i][j] = tmp;
        }
    }
    return 0;
}

int mtrxMtrx12pMxOdrAfter(double ** mn, double ** m, double * nMex)
{
    uchar i,j,k;
    double tmp;
    for(i=0;i<12;i++){
        for(j=0;j<12;j++){
            tmp = 0;
            for(k=0;k<12;k++){
                tmp += m[i][k]*nMex[k+j*12];
            }
            mn[i][j] = tmp;
        }
    }
    return 0;
}

int mtrxMtrx12pMxOdrOutAfter(double * mnMex, double ** m, double * nMex)
{
    uchar i,j,k;
    double tmp;
    for(i=0;i<12;i++){
        for(j=0;j<12;j++){
            tmp = 0;
            for(k=0;k<12;k++){
                tmp += m[i][k]*nMex[k+j*12];
            }
            mnMex[i+j*12] = tmp;
        }
    }
    return 0;
}

double **mtrxMtrxmtrx(double ** m1, uchar rows1, uchar cols1,
                            double ** m2, uchar rows2, uchar cols2,
                            double ** m3, uchar rows3, uchar cols3)
{
    uchar i,j,k;
    if((cols1 != rows2) || (cols2 != rows3)){
        #ifdef __MEX__
                mexPrintf("mtrxMtrxmtrx: incompatible matrice dimensions\n");
        #else
                printf("mtrxMtrxmtrx: incompatible matrice dimensions\n");
        #endif
    }
    double **m1m2   = mtrxMtrx(  m1,rows1,cols1,m2,rows2,cols2);
    double **m1m2m3 = mtrxMtrx(m1m2,rows1,cols2,m3,rows3,cols3);
    return m1m2m3;
}

/* only work for 3X3 matrices */
double **mtrxMtrxmtrx3(double ** m1, double ** m2, double ** m3)
{
    uchar i,j,k;
    double **m1m2   = mtrxMtrx3(  m1,m2);
    double **m1m2m3 = mtrxMtrx3(m1m2,m3);
    return m1m2m3;
}

/* not a good function, since it allocate memory inside and don't free them */
/* free added on Jul 27th, 2014, not tested */
int mtrxMtrxmtrx3p(double **m1m2m3, double ** m1, double ** m2, double ** m3)
{
    uchar i;
    double **m1m2 = (double**)malloc(3*sizeof(double*));
    for(i=0;i<3;i++)
        m1m2[i] = (double*)malloc(3*sizeof(double));
    mtrxMtrx3p(m1m2, m1,m2);
    mtrxMtrx3p(m1m2m3,m1m2,m3);
	/* free new added on Jul 27th, 2014, not tested */	
	for(i=0;i<3;i++)
		free(m1m2[i]);
	free(m1m2);
    return 0;
}

/* not a good function, since it allocate memory inside and don't free them */
/* free added on Jul 27th, 2014, not tested */
int mtrxMtrxmtrx6p(double **m1m2m3, double ** m1, double ** m2, double ** m3)
{
    uchar i;
    double **m1m2 = (double**)malloc(6*sizeof(double*));
    for(i=0;i<6;i++)
        m1m2[i] = (double*)malloc(6*sizeof(double));
    mtrxMtrx6p(m1m2, m1,m2);
    mtrxMtrx6p(m1m2m3,m1m2,m3);
	/* free new added on Jul 27th, 2014, not tested */	
	for(i=0;i<6;i++)
		free(m1m2[i]);
	free(m1m2);
    return 0;
}



double **mtrxMtrxmtrx12(double ** m1, double ** m2, double ** m3)
{
    uchar i,j,k;
    double **m1m2   = mtrxMtrx12(  m1,m2);
    double **m1m2m3 = mtrxMtrx12(m1m2,m3);
    return m1m2m3;
}

int mtrxMtrxmtrx12p(double **m1m2m3, double ** m1, double ** m2, double ** m3)
{
    // double **m1m2; // commented by zijian
    // BEGIN: added by Zijian
    double **m1m2 = (double **) malloc(12 * sizeof(double *));
    for(int i=0; i<12; i++) {
        m1m2[i] = (double *) malloc(12 * sizeof(double));
    }
    // END: added by zijian
    mtrxMtrx12p(m1m2, m1,m2);
    mtrxMtrx12p(m1m2m3,m1m2,m3);
    return 0;
}

double **mtrxTranp(double ** m, uchar rows, uchar cols)
{
    uchar i,j;
    double **mT = (double**)malloc(cols*sizeof(double*));
    for(i=0;i<rows;i++){
        mT[i] = (double*)malloc(rows*sizeof(double));
        for(j=0;j<cols;j++)
            mT[i][j] = m[j][i];
    }
    return mT;
}

int mtrxTranpp(double ** mT, double ** m, uchar rows, uchar cols)
{
    uchar i,j;
    for(i=0;i<rows;i++){
        for(j=0;j<cols;j++)
            mT[i][j] = m[j][i];
    }
    return 0;
}

/* only work for 3X3 matrices */
double **mtrxTranp3(double ** m)
{
    uchar i,j;
    double **mT = (double**)malloc(3*sizeof(double*));
    for(i=0;i<3;i++){
        mT[i] = (double*)malloc(3*sizeof(double));
        for(j=0;j<3;j++)
            mT[i][j] = m[j][i];
    }
    return mT;
}


int mtrxTranp3p(double **mT, double ** m)
{
    uchar i,j;
    for(i=0;i<3;i++){
        for(j=0;j<3;j++)
            mT[i][j] = m[j][i];
    }
    return 0;
}


double **mtrxTranp12(double ** m)
{
    uchar i,j;
    double **mT = (double**)malloc(12*sizeof(double*));
    for(i=0;i<12;i++){
        mT[i] = (double*)malloc(12*sizeof(double));
        for(j=0;j<12;j++)
            mT[i][j] = m[j][i];
    }
    return mT;
}

int mtrxTranp12p(double **mT, double ** m)
{
    uchar i,j;
    for(i=0;i<12;i++){
        for(j=0;j<12;j++)
            mT[i][j] = m[j][i];
    }
    return 0;
}

int mtrxTranp12pMxOdrIn(double **mT, double *m)
{
    uchar i,j;
    for(i=0;i<12;i++){
        for(j=0;j<12;j++)
            mT[i][j] = m[j+i*12];
    }
    return 0;
}

/* a vector X a mtrx column, return a scalar */
/* size is the vector's size, and also mtrx's rows */
double vecMtrxCol(double v[], uchar size, double ** m, uchar col)
{
    uchar i;
    double ret = 0;
    for(i=0;i<size;i++)
        ret += v[i]*m[i][col];
    return ret;
}

/* a vector X a mtrx column, return a scalar */
/* size is the vector's size, and also mtrx's rows */
/* only work for 3X3 matrices */
double vecMtrxCol3(double v[], double ** m, uchar col)
{
    return (v[0]*m[0][col] + v[1]*m[1][col] + v[2]*m[2][col]);
}

double * normalize(double * vecIn, uchar size)
{
    double *vecOut = (double*)malloc(size*sizeof(double));
	double normV = 0;
    uchar i;
    for(i=0;i<size;i++)
        normV += vecIn[i]*vecIn[i];
    if(normV<1e-5){
        #ifdef __MEX__
                mexPrintf("zero vector, return nothing.\n");
        #else
                printf("zero vector, return nothing.\n");
        #endif
    }
    normV = sqrt(normV);
    for(i=0;i<size;i++)
        vecOut[i] = vecIn[i]/normV;
    return vecOut;
}

/* only work for 3X1 vectors */
double * normalize3(double * vecIn)
{
    double *vecOut = (double*)malloc(3*sizeof(double));
	double normV = vecIn[0]*vecIn[0] + vecIn[1]*vecIn[1] + vecIn[2]*vecIn[2];
    if(normV<1e-5){
        #ifdef __MEX__
                mexPrintf("zero vector, return nothing.\n");
        #else
                printf("zero vector, return nothing.\n");
        #endif
    }
    normV = sqrt(normV);
    vecOut[0] = vecIn[0]/normV;
    vecOut[1] = vecIn[1]/normV;
    vecOut[2] = vecIn[2]/normV;
    return vecOut;
}

int normalize3p(double * vecOut, double * vecIn)
{
	double normV = vecIn[0]*vecIn[0] + vecIn[1]*vecIn[1] + vecIn[2]*vecIn[2];
    if(normV<1e-5){
        #ifdef __MEX__
                mexPrintf("dMath.c:normalize3p: zero vector, return nothing.\n");
        #else
                printf("dMath.c:normalize3p: zero vector, return nothing.\n");
        #endif
    }
    normV = sqrt(normV);
    vecOut[0] = vecIn[0]/normV;
    vecOut[1] = vecIn[1]/normV;
    vecOut[2] = vecIn[2]/normV;
    return 0;
}

int normalize3self(double * vec)
{
	double normV = vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2];
    if(normV<1e-5){
        #ifdef __MEX__
                mexPrintf("dMath.c:normalize3self: zero vector, return nothing.\n");
        #else
                printf("dMath.c:normalize3self: zero vector, return nothing.\n");
        #endif
    }
    normV = sqrt(normV);
    vec[0] = vec[0]/normV;
    vec[1] = vec[1]/normV;
    vec[2] = vec[2]/normV;
    return 0;
}

double norm1(double a)
{
	double ret = sqrt(a*a);
	return ret;
}

double norm3(double *a)
{
	double ret = sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
	return ret;
}

double distance3(double *a, double *b)
{
	double dist = 0;
	uchar i;
	for(i=0;i<3;i++){
		dist +=(a[i]-b[i])*(a[i]-b[i]);
	}
	dist = sqrt(dist);
	return dist;
}
/* cross product, only for 3D vector */
double * vecCross(double * a, double * b)
{
	double *c = (double*)malloc(3*sizeof(double));
	c[0] = a[1]*b[2] - a[2]*b[1];
	c[1] = a[2]*b[0] - a[0]*b[2];
	c[2] = a[0]*b[1] - a[1]*b[0];
	return c;
}


int vecCrossp(double * c, double * a, double * b)
{
	c[0] = a[1]*b[2] - a[2]*b[1];
	c[1] = a[2]*b[0] - a[0]*b[2];
	c[2] = a[0]*b[1] - a[1]*b[0];
	return 0;
}
/* "size" determines the rows of the mtrx */
double **threeVec2mtrx(double * a, double * b, double * c, uchar size)
{
    uchar i,j;
    double **mtrx = (double**)malloc(size*sizeof(double*));
    for(i=0;i<3;i++)
        mtrx[i] = (double*)malloc(3*sizeof(double));
    mtrx[0][0] = a[0]; mtrx[0][1] = b[0]; mtrx[0][2] = c[0];
    mtrx[1][0] = a[1]; mtrx[1][1] = b[1]; mtrx[1][2] = c[1];
    mtrx[2][0] = a[2]; mtrx[2][1] = b[2]; mtrx[2][2] = c[2];
    return mtrx;
}



/* "size" determines the rows of the mtrx */
/* only work for 3X3 matrices */
double **threeVec2mtrx3(double * a, double * b, double * c)
{
    uchar i,j;
    double **mtrx = (double**)malloc(3*sizeof(double*));
    for(i=0;i<3;i++)
        mtrx[i] = (double*)malloc(3*sizeof(double));
    mtrx[0][0] = a[0]; mtrx[0][1] = b[0]; mtrx[0][2] = c[0];
    mtrx[1][0] = a[1]; mtrx[1][1] = b[1]; mtrx[1][2] = c[1];
    mtrx[2][0] = a[2]; mtrx[2][1] = b[2]; mtrx[2][2] = c[2];
    return mtrx;
}

int threeVec2mtrx3p(double **mtrx, double * a, double * b, double * c)
{
    uchar i,j;
    mtrx[0][0] = a[0]; mtrx[0][1] = b[0]; mtrx[0][2] = c[0];
    mtrx[1][0] = a[1]; mtrx[1][1] = b[1]; mtrx[1][2] = c[1];
    mtrx[2][0] = a[2]; mtrx[2][1] = b[2]; mtrx[2][2] = c[2];
    return 0;
}

double vecDot(double * a, double * b, uchar size)
{
    double dot = 0;
    uchar i;
    for(i=0;i<size;i++)
        dot += a[i]*b[i];
    return dot;
}

/* only work for 3X3 matrices */
double vecDot3(double * a, double * b)
{
    return (a[0]*b[0] + a[1]*b[1] + a[2]*b[2]);
}

double *vecXscale(double * vecIn, double scale, uchar size)
{
    double *vecOut = (double*)malloc(size*sizeof(double));
    uchar i;
    for(i=0;i<size;i++)
        vecOut[i] = vecIn[i]*scale;
    return vecOut;
}

/* only work for 3X1 vectors */
double *vecXscale3(double * vecIn, double scale)
{
    double *vecOut = (double*)malloc(3*sizeof(double));
    vecOut[0] = vecIn[0]*scale;
    vecOut[1] = vecIn[1]*scale;
    vecOut[2] = vecIn[2]*scale;
    return vecOut;
}

int vecXscale3p(double * vecOut,double * vecIn, double scale)
{
    vecOut[0] = vecIn[0]*scale;
    vecOut[1] = vecIn[1]*scale;
    vecOut[2] = vecIn[2]*scale;
    return 0;
}


double *vecAdd(double * a, double *b, uchar size)
{
    double *c = (double*)malloc(size*sizeof(double));
    uchar i;
    for(i=0;i<size;i++)
        c[i] = a[i] + b[i];
    return c;
}

double *vecAdd3(double * a, double *b)
{
    double *c = (double*)malloc(3*sizeof(double));
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
    return c;
}

int vecAdd3p(double * c, double * a, double *b)
{
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
    return 0;
}

double *vecAdd12(double * a, double *b)
{
    double *c = (double*)malloc(12*sizeof(double));
    uchar i;
    for(i=0;i<12;i++)
        c[i] = a[i] + b[i];
    return c;
}

int vecAdd12p(double * c, double * a, double *b)
{
    c[0]  = a[0]  + b[0];
    c[1]  = a[1]  + b[1];
    c[2]  = a[2]  + b[2];
    c[3]  = a[3]  + b[3];
    c[4]  = a[4]  + b[4];
    c[5]  = a[5]  + b[5];
    c[6]  = a[6]  + b[6];
    c[7]  = a[7]  + b[7];
    c[8]  = a[8]  + b[8];
    c[9]  = a[9]  + b[9];
    c[10] = a[10] + b[10];
    c[11] = a[11] + b[11];
    return 0;
}

double *vecSub(double * a, double *b, uchar size)
{
    double *c = (double*)malloc(size*sizeof(double));
    uchar i;
    for(i=0;i<size;i++)
        c[i] = a[i] - b[i];
    return c;
}

double *vecSub3(double * a, double *b)
{
    double *c = (double*)malloc(3*sizeof(double));
    uchar i;
    c[0] = a[0] - b[0];
    c[1] = a[1] - b[1];
    c[2] = a[2] - b[2];
    return c;
}

int vecSub3p(double * c, double * a, double *b)
{
    c[0] = a[0] - b[0];
    c[1] = a[1] - b[1];
    c[2] = a[2] - b[2];
    return 0;
}

double *vecSub12(double * a, double *b)
{
    double *c = (double*)malloc(12*sizeof(double));
    uchar i;
    for(i=0;i<12;i++)
        c[i] = a[i] - b[i];
    return c;
}

int vecSub12p(double * c, double * a, double *b)
{
    c[0]  = a[0]  - b[0];
    c[1]  = a[1]  - b[1];
    c[2]  = a[2]  - b[2];
    c[3]  = a[3]  - b[3];
    c[4]  = a[4]  - b[4];
    c[5]  = a[5]  - b[5];
    c[6]  = a[6]  - b[6];
    c[7]  = a[7]  - b[7];
    c[8]  = a[8]  - b[8];
    c[9]  = a[9]  - b[9];
    c[10] = a[10] - b[10];
    c[11] = a[11] - b[11];
    return 0;
}


/* notice that row1, row2, col1 and col2 are indices start from 1, not 0 */
double **mtrxFrommtrxBlk(double ** A, uchar rows, uchar cols,
                            uchar row1, uchar row2, uchar col1, uchar col2)
{
    uchar i,j;
    double **Ac;
    if(row2 > rows ||col2 > cols || row1 > row2 || col1 > col2){
        #ifdef __MEX__
                mexPrintf("mtrxFrommtrxBlk: incompatible arguments.\n");
        #else
                printf("mtrxFrommtrxBlk: incompatible arguments.\n");
        #endif        
    }
    uchar rows_Ac = row2 - row1 + 1;
    uchar cols_Ac = col2 - col1 + 1;
    Ac = (double**)malloc(rows_Ac*sizeof(double*));
    for(i=0;i<rows_Ac;i++){
        Ac[i] = (double*)malloc(cols_Ac*sizeof(double));
        for(j=0;j<cols_Ac;j++)
            Ac[i][j] = A[i+row1-1][j+col1-1];
    }
    /* notice that row2 and col2 are not used in the above code */
    
    return Ac;
}

int mtrxFrommtrxBlkp(double ** Ac, double ** A, 
    uchar rows, uchar cols, uchar row1, uchar row2, uchar col1, uchar col2)
{
    uchar i,j;
    if(row2 > rows ||col2 > cols || row1 > row2 || col1 > col2){
        #ifdef __MEX__
                mexPrintf("mtrxFrommtrxBlkp: incompatible arguments.\n");
        #else
                printf("mtrxFrommtrxBlkp: incompatible arguments.\n");
        #endif        
    }
    uchar rows_Ac = row2 - row1 + 1;
    uchar cols_Ac = col2 - col1 + 1;
    for(i=0;i<rows_Ac;i++){
        for(j=0;j<cols_Ac;j++)
            Ac[i][j] = A[i+row1-1][j+col1-1];
    }
    /* notice that row2 and col2 are not used in the above code */
    
    return 0;
}

double **mtrxAdd(double ** a, double ** b, uchar rows, uchar cols)
{
    uchar i,j;
    double **c = (double**)malloc(rows*sizeof(double*));
    for(i=0;i<rows;i++){
        c[i] = (double*)malloc(cols*sizeof(double));
        for(j=0;j<cols;j++)
            c[i][j] = a[i][j] + b[i][j];
    }
    return c;
}

double **mtrxAdd3(double ** a, double ** b)
{
    uchar i;
    double **c = (double**)malloc(3*sizeof(double*));
    for(i=0;i<3;i++){
        c[i] = (double*)malloc(3*sizeof(double));
        c[i][0] = a[i][0] + b[i][0];
        c[i][1] = a[i][1] + b[i][1];
        c[i][2] = a[i][2] + b[i][2];
    }
    return c;
}

int mtrxAdd3p(double ** c, double ** a, double ** b)
{
    uchar i;
    for(i=0;i<3;i++){
        c[i][0] = a[i][0] + b[i][0];
        c[i][1] = a[i][1] + b[i][1];
        c[i][2] = a[i][2] + b[i][2];
    }
    return 0;
}

double **mtrxAdd12(double ** a, double ** b)
{
    uchar i,j;
    double **c = (double**)malloc(12*sizeof(double*));
    for(i=0;i<12;i++){
        c[i] = (double*)malloc(12*sizeof(double));
        for(j=0;j<12;j++)
            c[i][j] = a[i][j] + b[i][j];
    }
    return c;
}

int mtrxAdd12p(double ** c, double ** a, double ** b)
{
    uchar i;
    for(i=0;i<12;i++){
        c[i][0]  = a[i][0]  + b[i][0];
        c[i][1]  = a[i][1]  + b[i][1];
        c[i][2]  = a[i][2]  + b[i][2];
        c[i][3]  = a[i][3]  + b[i][3];
        c[i][4]  = a[i][4]  + b[i][4];
        c[i][5]  = a[i][5]  + b[i][5];
        c[i][6]  = a[i][6]  + b[i][6];
        c[i][7]  = a[i][7]  + b[i][7];
        c[i][8]  = a[i][8]  + b[i][8];
        c[i][9]  = a[i][9]  + b[i][9];
        c[i][10] = a[i][10] + b[i][10];
        c[i][11] = a[i][11] + b[i][11];
    }
    return 0;
}


double **mtrxSub(double ** a, double ** b, uchar rows, uchar cols)
{
    uchar i,j;
    double **c = (double**)malloc(rows*sizeof(double*));
    for(i=0;i<rows;i++){
        c[i] = (double*)malloc(cols*sizeof(double));
        for(j=0;j<cols;j++)
            c[i][j] = a[i][j] - b[i][j];
    }
    return c;
}

double **mtrxSub3(double ** a, double ** b)
{
    uchar i;
    double **c = (double**)malloc(3*sizeof(double*));
    for(i=0;i<3;i++){
        c[i] = (double*)malloc(3*sizeof(double));
        c[i][0] = a[i][0] - b[i][0];
        c[i][1] = a[i][1] - b[i][1];
        c[i][2] = a[i][2] - b[i][2];
    }
    return c;
}

int mtrxSub3p(double ** c, double ** a, double ** b)
{
    uchar i;
    for(i=0;i<3;i++){
        c[i][0] = a[i][0] - b[i][0];
        c[i][1] = a[i][1] - b[i][1];
        c[i][2] = a[i][2] - b[i][2];
    }
    return 0;
}

double **mtrxSub12(double ** a, double ** b)
{
    uchar i,j;
    double **c = (double**)malloc(12*sizeof(double*));
    for(i=0;i<12;i++){
        c[i] = (double*)malloc(12*sizeof(double));
        for(j=0;j<12;j++)
            c[i][j] = a[i][j] - b[i][j];
    }
    return c;
}

int mtrxSub12p(double ** c, double ** a, double ** b)
{
    uchar i;
    for(i=0;i<12;i++){
        c[i][0]  = a[i][0]  - b[i][0];
        c[i][1]  = a[i][1]  - b[i][1];
        c[i][2]  = a[i][2]  - b[i][2];
        c[i][3]  = a[i][3]  - b[i][3];
        c[i][4]  = a[i][4]  - b[i][4];
        c[i][5]  = a[i][5]  - b[i][5];
        c[i][6]  = a[i][6]  - b[i][6];
        c[i][7]  = a[i][7]  - b[i][7];
        c[i][8]  = a[i][8]  - b[i][8];
        c[i][9]  = a[i][9]  - b[i][9];
        c[i][10] = a[i][10] - b[i][10];
        c[i][11] = a[i][11] - b[i][11];
    }
    return 0;
}

double **mtrxOpp(double ** a, uchar rows, uchar cols)
{
    uchar i,j;
    double **b = (double**)malloc(rows*sizeof(double*));
    for(i=0;i<rows;i++){
        b[i] = (double*)malloc(cols*sizeof(double));
        for(j=0;j<cols;j++)
            b[i][j] = - a[i][j];
    }
    return b;
}

int mtrxOppp(double ** b, double ** a, uchar rows, uchar cols)
{
    uchar i,j;
    for(i=0;i<rows;i++){
        for(j=0;j<cols;j++)
            b[i][j] = - a[i][j];
    }
    return 0;
}

/* only for 2X2 mtrx */
double **mtrx2Dinv(double ** A)
{
    uchar i;
    double **invA = (double**)malloc(2*sizeof(double*));
    for(i=0;i<2;i++)
        invA[i] = (double*)malloc(2*sizeof(double));
    double a00,a01,a10,a11;
    a00 = A[0][0]; a01 = A[0][1];
    a10 = A[1][0]; a11 = A[1][1];
    double det = a00*a11 - a01*a10;
    if(det == 0){
        #ifdef __MEX__
                mexPrintf("mtrx2Dinv: singular input mtrx, return nothing.\n");
        #else
                printf("mtrx2Dinv: singular input mtrx, return nothing.\n");
        #endif
    }
    
	invA[0][0] =   a11/det;
	invA[0][1] = - a01/det;
    invA[1][0] = - a10/det;
    invA[1][1] =   a00/det;
	return invA;
}


int mtrx2Dinvp(double ** invA, double ** A)
{
    double a00,a01,a10,a11;
    a00 = A[0][0]; a01 = A[0][1];
    a10 = A[1][0]; a11 = A[1][1];
    double det = a00*a11 - a01*a10;
    if(det == 0){
        #ifdef __MEX__
            mexPrintf("mtrx2Dinvp: singular input mtrx, return nothing.\n");
        #else
            printf("mtrx2Dinvp: singular input mtrx, return nothing.\n");
        #endif
    }
    
	invA[0][0] =   a11/det;
	invA[0][1] = - a01/det;
    invA[1][0] = - a10/det;
    invA[1][1] =   a00/det;
	return 0;
}


/* only for 3X3 mtrx, algorithm from wiki */
double **mtrx3Dinv(double **A)
{
    uchar i;
    double **invA = (double**)malloc(3*sizeof(double*));
    for(i=0;i<3;i++)
        invA[i] = (double*)malloc(3*sizeof(double));
	double a00,a01,a02,a10,a11,a12,a20,a21,a22;
	a00 = A[0][0]; a01 = A[0][1]; a02 = A[0][2];
	a10 = A[1][0]; a11 = A[1][1]; a12 = A[1][2];
	a20 = A[2][0]; a21 = A[2][1]; a22 = A[2][2];
	double det = a00*a11*a22 - a00*a12*a21 - a01*a10*a22 + 
                 a01*a12*a20 + a02*a10*a21 - a02*a11*a20;
	if(det == 0){
        #ifdef __MEX__
                mexPrintf("dMath.c: mtrx3Dinv: singular input mtrx, return nothing.\n");
        #else
                printf("dMath.c: mtrx3Dinv: singular input mtrx, return nothing.\n");
        #endif
    }
	invA[0][0] = (a11*a22-a12*a21)/det;
	invA[0][1] = (a02*a21-a01*a22)/det;
	invA[0][2] = (a01*a12-a02*a11)/det;
	invA[1][0] = (a12*a20-a10*a22)/det;
	invA[1][1] = (a00*a22-a02*a20)/det;
	invA[1][2] = (a02*a10-a00*a12)/det;
	invA[2][0] = (a10*a21-a11*a20)/det;
	invA[2][1] = (a01*a20-a00*a21)/det;
	invA[2][2] = (a00*a11-a01*a10)/det;
	return invA;
}

int mtrx3Dinvp(double **invA, double **A)
{
	double a00,a01,a02,a10,a11,a12,a20,a21,a22;
	a00 = A[0][0]; a01 = A[0][1]; a02 = A[0][2];
	a10 = A[1][0]; a11 = A[1][1]; a12 = A[1][2];
	a20 = A[2][0]; a21 = A[2][1]; a22 = A[2][2];
	double det = a00*a11*a22 - a00*a12*a21 - a01*a10*a22 + 
                 a01*a12*a20 + a02*a10*a21 - a02*a11*a20;
	if(det == 0){
        #ifdef __MEX__
                mexPrintf("dMath.c: mtrx3Dinvp: singular input mtrx, return nothing.\n");
        #else
                printf("dMath.c: mtrx3Dinvp: singular input mtrx, return nothing.\n");
        #endif
    }
	invA[0][0] = (a11*a22-a12*a21)/det;
	invA[0][1] = (a02*a21-a01*a22)/det;
	invA[0][2] = (a01*a12-a02*a11)/det;
	invA[1][0] = (a12*a20-a10*a22)/det;
	invA[1][1] = (a00*a22-a02*a20)/det;
	invA[1][2] = (a02*a10-a00*a12)/det;
	invA[2][0] = (a10*a21-a11*a20)/det;
	invA[2][1] = (a01*a20-a00*a21)/det;
	invA[2][2] = (a00*a11-a01*a10)/det;
	return 0;
}


/* it is two times faster than the matlab function! --verified */
double **mtrxBlkInv(double **A, uchar size)
{
    uchar i,j, hs; /* hs = half size */
    uchar sizeD = sizeof(double);
    uchar sizeDpr = sizeof(double*);
	if(size == 2)
	{
		double **invA = mtrx2Dinv(A);
		return invA;
	}
	if(size == 3)
	{
		double **invA = mtrx3Dinv(A);
		return invA;
	}
    hs = size/2;
    /* divide the mtrxde into four parts: A = [a,b; c,d] */
    double **a = mtrxFrommtrxBlk(A, size, size, 1   , hs  , 1   , hs  );
    double **b = mtrxFrommtrxBlk(A, size, size, 1   , hs  , hs+1, size);
    double **c = mtrxFrommtrxBlk(A, size, size, hs+1, size, 1   , hs  );
    double **d = mtrxFrommtrxBlk(A, size, size, hs+1, size, hs+1, size);
	/*mtrx a,b,c,d,inva,cinva,invab,d_cinvabINV;
	mtrx invA11,invA12,invA21,invA22;*/
    /* recursive */
    double **inva   = mtrxBlkInv(a,hs);
	double **invab  = mtrxMtrx(inva,hs,hs,b,hs,hs);
	double **cinva  = mtrxMtrx(c,hs,hs,inva,hs,hs);
	double **invA22 = mtrxBlkInv(mtrxSub(d,mtrxMtrxmtrx(c,hs,hs,inva,hs,hs,b,hs,hs),hs,hs),hs); /* d_cinvabINV*/

	double **invA11 = mtrxAdd(inva,mtrxMtrxmtrx(invab,hs,hs,invA22,hs,hs,cinva,hs,hs),hs,hs);
	double **invA12 = mtrxOpp(mtrxMtrx(invab,hs,hs,invA22,hs,hs),hs,hs);
	double **invA21 = mtrxOpp(mtrxMtrx(invA22,hs,hs,cinva,hs,hs),hs,hs);
	/* double **invA22 = d_cinvabINV; */

    /* combine into a new mtrx */
    double **invA = (double**)malloc(size*sizeof(double*));
    for(i=0;i<size;i++)
        invA[i] = (double*)malloc(size*sizeof(double));
    for(i=0;i<hs;i++)
        for(j=0;j<hs;j++){
            invA[i][j]       = invA11[i][j];
            invA[i][j+hs]    = invA12[i][j];
            invA[i+hs][j]    = invA21[i][j];
            invA[i+hs][j+hs] = invA22[i][j];
        }
            
    return invA; /* forgot th return, caused a lot of problems */
}

/* make sure euler has only 3 double elements */
double **rotZYX(double *euler)
{
    uchar i;
    /* to save some calculation */
    double Cphi = cos(euler[0]);
    double Sphi = sin(euler[0]);
    double Cthe = cos(euler[1]);
    double Sthe = sin(euler[1]);
    double Cpsi = cos(euler[2]);
    double Spsi = sin(euler[2]);
    double CpsiSphi = Cpsi*Sphi;
    double CphiCpsi = Cphi*Cpsi;
    double SphiSpsi = Sphi*Spsi;
    double CphiSpsi = Cphi*Spsi;

    /* output */
    double **R = (double**)malloc(3*sizeof(double*));
    for(i=0;i<3;i++)
        R[i] = (double*)malloc(3*sizeof(double));
    
    /* {{Cpsi*Cthe, CpsiSphi*Sthe - CphiSpsi, SphiSpsi + CphiCpsi*Sthe},
     {Cthe*Spsi, CphiCpsi + SphiSpsi*Sthe, CphiSpsi*Sthe - CpsiSphi},
     {-Sthe,                Cthe*Sphi,     Cphi*Cthe               }};*/
    
    R[0][0] =  Cpsi*Cthe;
    R[0][1] =  CpsiSphi*Sthe - CphiSpsi;
    R[0][2] =  SphiSpsi + CphiCpsi*Sthe;
    R[1][0] =  Cthe*Spsi;
    R[1][1] =  CphiCpsi + SphiSpsi*Sthe;
    R[1][2] =  CphiSpsi*Sthe - CpsiSphi;
    R[2][0] = -Sthe;
    R[2][1] =  Cthe*Sphi;
    R[2][2] =  Cphi*Cthe;
    
    return R;
}

int rotZYXp(double ** R, double *euler)
{
    /* to save some calculation */
    double Cphi = cos(euler[0]);
    double Sphi = sin(euler[0]);
    double Cthe = cos(euler[1]);
    double Sthe = sin(euler[1]);
    double Cpsi = cos(euler[2]);
    double Spsi = sin(euler[2]);
    double CpsiSphi = Cpsi*Sphi;
    double CphiCpsi = Cphi*Cpsi;
    double SphiSpsi = Sphi*Spsi;
    double CphiSpsi = Cphi*Spsi;

    /* output */
    R[0][0] =  Cpsi*Cthe;
    R[0][1] =  CpsiSphi*Sthe - CphiSpsi;
    R[0][2] =  SphiSpsi + CphiCpsi*Sthe;
    R[1][0] =  Cthe*Spsi;
    R[1][1] =  CphiCpsi + SphiSpsi*Sthe;
    R[1][2] =  CphiSpsi*Sthe - CpsiSphi;
    R[2][0] = -Sthe;
    R[2][1] =  Cthe*Sphi;
    R[2][2] =  Cphi*Cthe;
   
    return 0;
}


double *rot2eulerZYX(double ** R)
{
    double *euler = (double*)malloc(3*sizeof(double));
    euler[0] = atan2(R[2][1],R[2][2]);  /* phi   */
    euler[1] = asin(-R[2][0]);          /* theta */
    euler[2] = atan2(R[1][0],R[0][0]);  /* psi   */

    return euler;
}

int rot2eulerZYXp(double *euler, double ** R)
{
    euler[0] = atan2(R[2][1],R[2][2]);  /* phi   */
    euler[1] = asin(-R[2][0]);          /* theta */
    euler[2] = atan2(R[1][0],R[0][0]);  /* psi   */

    return 0;
}


/* make sure euler has only 3 double elements */
double **kinEqZYX(double *euler)
{
    uchar i;

	/* to save some calculation */
    double Sphi = sin(euler[0]);
    double Cphi = cos(euler[0]);
    double Tthe = tan(euler[1]);
    double Cthe = cos(euler[1]);

    /* output */
    double **Mtrx = (double**)malloc(3*sizeof(double*));
    for(i=0;i<3;i++)
        Mtrx[i] = (double*)malloc(3*sizeof(double));
    
    /*Mtrx = [1 Tthe*Sphi  Tthe*Cphi;
              0 Cphi      -Sphi     ;
              0 Sphi/Cthe  Cphi/Cthe]; */
    
    Mtrx[0][0] =  1;
    Mtrx[0][1] =  Tthe*Sphi;
    Mtrx[0][2] =  Tthe*Cphi;
    Mtrx[1][0] =  0;
    Mtrx[1][1] =  Cphi;
    Mtrx[1][2] = -Sphi;
    Mtrx[2][0] =  0;
    Mtrx[2][1] =  Sphi/Cthe;
    Mtrx[2][2] =  Cphi/Cthe;
    
    return Mtrx;
}


int kinEqZYXp(double ** Mtrx, double *euler)
{
    uchar i;

	/* to save some calculation */
    double Sphi = sin(euler[0]);
    double Cphi = cos(euler[0]);
    double Tthe = tan(euler[1]);
    double Cthe = cos(euler[1]);
    
    /*Mtrx = [1 Tthe*Sphi  Tthe*Cphi;
              0 Cphi      -Sphi     ;
              0 Sphi/Cthe  Cphi/Cthe]; */
    
    Mtrx[0][0] =  1;
    Mtrx[0][1] =  Tthe*Sphi;
    Mtrx[0][2] =  Tthe*Cphi;
    Mtrx[1][0] =  0;
    Mtrx[1][1] =  Cphi;
    Mtrx[1][2] = -Sphi;
    Mtrx[2][0] =  0;
    Mtrx[2][1] =  Sphi/Cthe;
    Mtrx[2][2] =  Cphi/Cthe;
    
    return 0;
}

/* commented by Zijian
int wbToElrd1p(double * elrd1, double * elr, double * wb)
{
    double phi = elr[0];
    double the = elr[1];
    double psi = elr[2];     // not used

	// to save some calculation
    double Sphi = sin(phi);
    double Cphi = cos(phi);
    double Tthe = tan(the);
    double Cthe = cos(the);
    
    //output:
    // Mtrx = [1 Tthe*Sphi  Tthe*Cphi;
    // 0 Cphi      -Sphi     ;
    // 0 Sphi/Cthe  Cphi/Cthe];
    // elrd1 = Mtrx*wb;

    elrd1[0] = wb[0] + Tthe*Sphi*wb[1] + Tthe*Cphi*wb[2];
    elrd1[1] = Cphi*wb[1] - Sphi*wb[2];
    elrd1[2] = (Sphi*wb[1] + Cphi*wb[2])/Cthe;
}
*/

double **invKinZYX(double *euler)
{
    uchar i;

    /* to save some calculation */
    double sinphi = sin(euler[0]);
    double cosphi = cos(euler[0]);
    double sinthe = sin(euler[1]);
    double costhe = cos(euler[1]);

	/* output: */
    double **Mtrx = (double**)malloc(3*sizeof(double*));
    for(i=0;i<3;i++)
        Mtrx[i] = (double*)malloc(3*sizeof(double));
	/* [1,  0     , -sinthe       ;
        0,  cosphi,  costhe*sinphi;
        0, -sinphi,  cosphi*costhe]; */
    Mtrx[0][0] = 1;
    Mtrx[0][1] = 0;
    Mtrx[0][2] = -sinthe;
    Mtrx[1][0] = 0;
    Mtrx[1][1] = cosphi;
    Mtrx[1][2] = costhe*sinphi;
    Mtrx[2][0] = 0;
    Mtrx[2][1] = -sinphi;
    Mtrx[2][2] = cosphi*costhe;
    
    return Mtrx;
}

int invKinZYXp(double **Mtrx, double *euler)
{
    /* to save some calculation */
    double sinphi = sin(euler[0]);
    double cosphi = cos(euler[0]);
    double sinthe = sin(euler[1]);
    double costhe = cos(euler[1]);

	/* output: */
	/* [1,  0     , -sinthe       ;
        0,  cosphi,  costhe*sinphi;
        0, -sinphi,  cosphi*costhe]; */
    Mtrx[0][0] =  1;
    Mtrx[0][1] =  0;
    Mtrx[0][2] = -sinthe;
    Mtrx[1][0] =  0;
    Mtrx[1][1] =  cosphi;
    Mtrx[1][2] =  costhe*sinphi;
    Mtrx[2][0] =  0;
    Mtrx[2][1] = -sinphi;
    Mtrx[2][2] =  cosphi*costhe;
    
    return 0;
}


double *invTensor(double **X)
{
    /* check if x is skew-symmetric 
    if abs(x(2,3)+x(3,2))>1e-4 | abs(x(1,3)+x(3,1))>1e-4 | abs(x(1,2)+x(2,1))>1e-4
        disp('The mtrx is not skew-symmetric.');
        return;
    end*/
    /* tildemtrx=[  0    -x(3)   x(2);
                      x(3)  0  	  -x(1);
                     -x(2)  x(1)   0   ]; (matlab)*/
    double *v = (double*)malloc(3*sizeof(double));
    v[0] = X[2][1]; /* X(3,2); */
    v[1] = X[0][2]; /* X(1,3); */
    v[2] = X[1][0]; /* X(2,1); */
    return v;
}

int invTensorp(double *v, double **X)
{
    v[0] = X[2][1]; /* X(3,2); */
    v[1] = X[0][2]; /* X(1,3); */
    v[2] = X[1][0]; /* X(2,1); */
    return 0;
}

double **mtrxEye(uchar size)
{
    uchar i,j;
    double **A = (double**)malloc(size*sizeof(double*));
    for(i=0;i<size;i++){
        A[i] = (double*)malloc(size*sizeof(double));
        for(j=0;j<size;j++)
            A[i][j] = 0;
    }
    for(i=0;i<size;i++)
        A[i][i] = 1;
    return A;
}

/* mtrx inverse for 4X mtrx */
int mtrx4Dinvp(double ** invA, double **A)
{
    uchar i,j;
	uchar sizeD = sizeof(double);
	uchar sizeDpr = sizeof(double*);
    double **a = (double**)malloc(2*sizeDpr);
    double **b = (double**)malloc(2*sizeDpr);
    double **c = (double**)malloc(2*sizeDpr);
    double **inva   = (double**)malloc(2*sizeDpr);
    double **invab  = (double**)malloc(2*sizeDpr);
    double **cinva  = (double**)malloc(2*sizeDpr);
    double **invA22 = (double**)malloc(2*sizeDpr);
    for(i=0;i<2;i++){
        a[i] = (double*)malloc(2*sizeD);
        b[i] = (double*)malloc(2*sizeD);
        c[i] = (double*)malloc(2*sizeD);
        inva[i]  = (double*)malloc(2*sizeD);
        invab[i] = (double*)malloc(2*sizeD);
        cinva[i] = (double*)malloc(2*sizeD);
        invA22[i]= (double*)malloc(2*sizeD);
        for(j=0;j<2;j++){
            a[i][j] = A[i  ][j  ];
            b[i][j] = A[i  ][j+2];
            c[i][j] = A[i+2][j  ];
        }
    }
    mtrx2Dinvp(inva,a);
	mtrxMtrx2p(invab,inva,b);
	mtrxMtrx2p(cinva,c,inva);
    mtrxMtrx2p(a,c,inva);             /* a = c*inva */
    mtrxMtrx2p(c,a,b);                /* c = a*b = c*inva*b */
    for(i=0;i<2;i++){
        c[i][0] = A[i+2][2] - c[i][0];
        c[i][1] = A[i+2][3] - c[i][1];
    }
    mtrx2Dinvp(invA22,c);               /* invA22 = d_cinvabINV*/
    mtrxMtrx2p(a,invab,invA22);       /* a =invab*invA22 */
    mtrxMtrx2p(c,a,cinva);            /* c = a*cinva = invab*invA22*cinva */
    for(i=0;i<2;i++){
        invA[i][0] = inva[i][0] + c[i][0];  /* invA11 */
        invA[i][1] = inva[i][1] + c[i][1];
    }
    mtrxMtrx2p(c,invab,invA22);
    mtrxMtrx2p(b,invA22,cinva);
    for(i=0;i<2;i++){
        invA[i][2]   = - c[i][0];           /* invA12 */
        invA[i][3]   = - c[i][1];
        invA[i+2][0] = - b[i][0];           /* invA21 */
        invA[i+2][1] = - b[i][1];
        invA[i+2][2] = invA22[i][0];        /* simple copy */
        invA[i+2][3] = invA22[i][1];
    }
	/* to free memory */
	for(i=0;i<2;i++){
		free(a[i]);
		free(b[i]);
		free(c[i]);
		free(inva[i]);
		free(invab[i]);
		free(cinva[i]);
		free(invA22[i]);
	}
	free(a);
	free(b);
	free(c);
	free(inva);
	free(invab);
	free(cinva);
	free(invA22);

    return 0;
}




/* mtrx inverse for 6X6 mtrx */
int mtrx6Dinvp(double ** invA, double **A)
{
    uchar i,j;
	uchar sizeD = sizeof(double);
	uchar sizeDpr = sizeof(double*);
    double **a = (double**)malloc(3*sizeDpr);
    double **b = (double**)malloc(3*sizeDpr);
    double **c = (double**)malloc(3*sizeDpr);
    double **inva   = (double**)malloc(3*sizeDpr);
    double **invab  = (double**)malloc(3*sizeDpr);
    double **cinva  = (double**)malloc(3*sizeDpr);
    double **invA22 = (double**)malloc(3*sizeDpr);
    for(i=0;i<3;i++){
        a[i] = (double*)malloc(3*sizeD);
        b[i] = (double*)malloc(3*sizeD);
        c[i] = (double*)malloc(3*sizeD);
        inva[i]  = (double*)malloc(3*sizeD);
        invab[i] = (double*)malloc(3*sizeD);
        cinva[i] = (double*)malloc(3*sizeD);
        invA22[i]= (double*)malloc(3*sizeD);
        for(j=0;j<3;j++){
            a[i][j] = A[i  ][j  ];
            b[i][j] = A[i  ][j+3];
            c[i][j] = A[i+3][j  ];
        }
    }
    mtrx3Dinvp(inva,a);
	mtrxMtrx3p(invab,inva,b);
	mtrxMtrx3p(cinva,c,inva);
    mtrxMtrx3p(a,c,inva);             /* a = c*inva */
    mtrxMtrx3p(c,a,b);                /* c = a*b = c*inva*b */
    for(i=0;i<3;i++){
        c[i][0] = A[i+3][3] - c[i][0];
        c[i][1] = A[i+3][4] - c[i][1];
        c[i][2] = A[i+3][5] - c[i][2];
    }
    mtrx3Dinvp(invA22,c);               /* invA22 = d_cinvabINV*/
    mtrxMtrx3p(a,invab,invA22);       /* a =invab*invA22 */
    mtrxMtrx3p(c,a,cinva);            /* c = a*cinva = invab*invA22*cinva */
    for(i=0;i<3;i++){
        invA[i][0] = inva[i][0] + c[i][0];  /* invA11 */
        invA[i][1] = inva[i][1] + c[i][1];
        invA[i][2] = inva[i][2] + c[i][2];
    }
    mtrxMtrx3p(c,invab,invA22);
    mtrxMtrx3p(b,invA22,cinva);
    for(i=0;i<3;i++){
        invA[i][3]   = - c[i][0];           /* invA12 */
        invA[i][4]   = - c[i][1];
        invA[i][5]   = - c[i][2];
        invA[i+3][0] = - b[i][0];           /* invA21 */
        invA[i+3][1] = - b[i][1];
        invA[i+3][2] = - b[i][2];
        invA[i+3][3] = invA22[i][0];        /* simple copy */
        invA[i+3][4] = invA22[i][1];
        invA[i+3][5] = invA22[i][2];
    }
	/* to free memory */
	for(i=0;i<3;i++){
		free(a[i]);
		free(b[i]);
		free(c[i]);
		free(inva[i]);
		free(invab[i]);
		free(cinva[i]);
		free(invA22[i]);
	}
	free(a);
	free(b);
	free(c);
	free(inva);
	free(invab);
	free(cinva);
	free(invA22);

    return 0;
}

/* mtrx inverse for 9x9 mtrx */
/* tested */
int mtrx9Dinvp(double ** invA, double **A)
{
    uchar i,j;
	uchar sizeD = sizeof(double);
	uchar sizeDpr = sizeof(double*);
    double **a = (double**)malloc(3*sizeDpr);
    double **b = (double**)malloc(3*sizeDpr);
    double **c = (double**)malloc(6*sizeDpr);
    double **d = (double**)malloc(6*sizeDpr);
	for(i=0;i<3;i++){
		a[i] = (double*)malloc(3*sizeD);
		b[i] = (double*)malloc(6*sizeD);
	}
	for(i=0;i<6;i++){
		c[i] = (double*)malloc(3*sizeD);
		d[i] = (double*)malloc(6*sizeD);
	}
	/* assign */
	for(i=0;i<3;i++){
		a[i][0] = A[i][0];
		a[i][1] = A[i][1];
		a[i][2] = A[i][2];

		b[i][0] = A[i][3];
		b[i][1] = A[i][4];
		b[i][2] = A[i][5];
		b[i][3] = A[i][6];
		b[i][4] = A[i][7];
		b[i][5] = A[i][8];
	}
	for(i=0;i<6;i++){
		c[i][0] = A[i+3][0];
		c[i][1] = A[i+3][1];
		c[i][2] = A[i+3][2];

		d[i][0] = A[i+3][3];
		d[i][1] = A[i+3][4];
		d[i][2] = A[i+3][5];
		d[i][3] = A[i+3][6];
		d[i][4] = A[i+3][7];
		d[i][5] = A[i+3][8];
	}
	

	double **inva = (double**)malloc(3*sizeDpr);
	for(i=0;i<3;i++)
		inva[i] = (double*)malloc(3*sizeD);
	double **invd = (double**)malloc(6*sizeDpr);
	for(i=0;i<6;i++)
		invd[i] = (double*)malloc(6*sizeD);

	double **invab = (double**)malloc(3*sizeDpr);
	for(i=0;i<3;i++)
		invab[i] = (double*)malloc(6*sizeD);
	double **invdc = (double**)malloc(6*sizeDpr);
	for(i=0;i<6;i++)
		invdc[i] = (double*)malloc(3*sizeD);
	mtrx3Dinvp(inva,a);
	mtrx6Dinvp(invd,d);
	mtrxMtrx3336p(invab,inva,b);
	mtrxMtrx6663p(invdc,invd,c);

	double **binvdc = (double**)malloc(3*sizeDpr);
	for(i=0;i<3;i++)
		binvdc[i] = (double*)malloc(3*sizeD);
	mtrxMtrx3663p(binvdc,b,invdc);
	double **cinvab = (double**)malloc(6*sizeDpr);
	for(i=0;i<6;i++)
		cinvab[i] = (double*)malloc(6*sizeD);
	mtrxMtrx6336p(cinvab,c,invab);

	/* binvdc = A - B*D^(-1)*C */
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			binvdc[i][j] = -binvdc[i][j] + a[i][j];
		}
	}
	/* cinvab = D - C*A^(-1)*B) */
	for(i=0;i<6;i++){
		for(j=0;j<6;j++){
			cinvab[i][j] = -cinvab[i][j] + d[i][j];
		}
	}
	
	double **invA11 = (double**)malloc(3*sizeDpr);
	for(i=0;i<3;i++)
		invA11[i] = (double*)malloc(3*sizeD);
	double **invA22 = (double**)malloc(6*sizeDpr);
	for(i=0;i<6;i++)
		invA22[i] = (double*)malloc(6*sizeD);
	mtrx3Dinvp(invA11,binvdc); /* done */
	mtrx6Dinvp(invA22,cinvab); /* done */

	double **invA12 = (double**)malloc(3*sizeDpr);
	for(i=0;i<3;i++)
		invA12[i] = (double*)malloc(6*sizeD);
	mtrxMtrx3666p(invA12,invab,invA22); /* need to invA12 = -invA12 */
	double **invA21 = (double**)malloc(6*sizeDpr);
	for(i=0;i<6;i++)
		invA21[i] = (double*)malloc(3*sizeD);
	mtrxMtrx6333p(invA21,invdc,invA11); /* need to invA21 = -invA21 */

	/* assign */
	for(i=0;i<3;i++){
		invA[i][0] = invA11[i][0];
		invA[i][1] = invA11[i][1];
		invA[i][2] = invA11[i][2];
		
		invA[i][3] = -invA12[i][0];
		invA[i][4] = -invA12[i][1];
		invA[i][5] = -invA12[i][2];
		invA[i][6] = -invA12[i][3];
		invA[i][7] = -invA12[i][4];
		invA[i][8] = -invA12[i][5];
	}
	for(i=0;i<6;i++){		
		invA[i+3][0] = -invA21[i][0];
		invA[i+3][1] = -invA21[i][1];
		invA[i+3][2] = -invA21[i][2];
		
		invA[i+3][3] = invA22[i][0];
		invA[i+3][4] = invA22[i][1];
		invA[i+3][5] = invA22[i][2];
		invA[i+3][6] = invA22[i][3];
		invA[i+3][7] = invA22[i][4];
		invA[i+3][8] = invA22[i][5];
	}
	/* free memory  */
	for(i=0;i<3;i++){
		free(a[i]);
		free(b[i]);
		free(inva[i]);
		free(invab[i]);
		free(binvdc[i]);
		free(invA11[i]);
		free(invA12[i]);
	}
	for(i=0;i<6;i++){
		free(c[i]);
		free(d[i]);
		free(invd[i]);
		free(invdc[i]);
		free(cinvab[i]);
		free(invA21[i]);
		free(invA22[i]);
	}
	free(a);
	free(b);
	free(c);
	free(d);
	free(inva);
	free(invd);
	free(invab);
	free(invdc);
	free(binvdc);
	free(cinvab);
	free(invA11);
	free(invA12);
	free(invA21);
	free(invA22);
    return 0;
}


/* mtrx inverse for 12x12 mtrx */
int mtrx12Dinvp(double **invA, double **A)
{
    uchar i,j;	
	uchar sizeD = sizeof(double);
	uchar sizeDpr = sizeof(double*);
    double **a = (double**)malloc(6*sizeDpr);
    double **b = (double**)malloc(6*sizeDpr);
    double **c = (double**)malloc(6*sizeDpr);
    double **inva   = (double**)malloc(6*sizeDpr);
    double **invab  = (double**)malloc(6*sizeDpr);
    double **cinva  = (double**)malloc(6*sizeDpr);
    double **invA22 = (double**)malloc(6*sizeDpr);
    for(i=0;i<6;i++){
        a[i] = (double*)malloc(6*sizeD);
        b[i] = (double*)malloc(6*sizeD);
        c[i] = (double*)malloc(6*sizeD);
        inva[i]  = (double*)malloc(6*sizeD);
        invab[i] = (double*)malloc(6*sizeD);
        cinva[i] = (double*)malloc(6*sizeD);
        invA22[i]= (double*)malloc(6*sizeD);
        for(j=0;j<6;j++){
            a[i][j] = A[i  ][j  ];
            b[i][j] = A[i  ][j+6];
            c[i][j] = A[i+6][j  ];
        }
    }
    mtrx6Dinvp(inva,a);
	mtrxMtrx6p(invab,inva,b);
	mtrxMtrx6p(cinva,c,inva);
    mtrxMtrx6p(a,c,inva);             /* a = c*inva */
    mtrxMtrx6p(c,a,b);                /* c = a*b = c*inva*b */
    for(i=0;i<6;i++){
        c[i][0] = A[i+6][6] - c[i][0];
        c[i][1] = A[i+6][7] - c[i][1];
        c[i][2] = A[i+6][8] - c[i][2];
        c[i][3] = A[i+6][9] - c[i][3];
        c[i][4] = A[i+6][10]- c[i][4];
        c[i][5] = A[i+6][11]- c[i][5];
    }
	mtrx6Dinvp(invA22,c);                /* invA22 = d_cinvabINV*/
    mtrxMtrx6p(a,invab,invA22);       /* a =invab*invA22 */
    mtrxMtrx6p(c,a,cinva);            /* c = a*cinva = invab*invA22*cinva */
    for(i=0;i<6;i++){
        invA[i][0] = inva[i][0] + c[i][0];  /* invA11 */
        invA[i][1] = inva[i][1] + c[i][1];
        invA[i][2] = inva[i][2] + c[i][2];
        invA[i][3] = inva[i][3] + c[i][3];
        invA[i][4] = inva[i][4] + c[i][4];
        invA[i][5] = inva[i][5] + c[i][5];
    }
    mtrxMtrx6p(c,invab,invA22);
    mtrxMtrx6p(b,invA22,cinva);
    for(i=0;i<6;i++){
        invA[i][6]   = - c[i][0];            /* invA12 */
        invA[i][7]   = - c[i][1];
        invA[i][8]   = - c[i][2];
        invA[i][9]   = - c[i][3];
        invA[i][10]  = - c[i][4];
        invA[i][11]  = - c[i][5];
        invA[i+6][0] = - b[i][0];           /* invA21 */
        invA[i+6][1] = - b[i][1];
        invA[i+6][2] = - b[i][2];
        invA[i+6][3] = - b[i][3];
        invA[i+6][4] = - b[i][4];
        invA[i+6][5] = - b[i][5];
        invA[i+6][6] = invA22[i][0];        /* simple copy */
        invA[i+6][7] = invA22[i][1];
        invA[i+6][8] = invA22[i][2];
        invA[i+6][9] = invA22[i][3];
        invA[i+6][10]= invA22[i][4];
        invA[i+6][11]= invA22[i][5];
    }
	/* to free memory */
	for(i=0;i<6;i++){
		free(a[i]);
		free(b[i]);
		free(c[i]);
		free(inva[i]);
		free(invab[i]);
		free(cinva[i]);
		free(invA22[i]);
	}
	free(a);
	free(b);
	free(c);
	free(inva);
	free(invab);
	free(cinva);
	free(invA22);

    return 0;
}

/* mtrx inverse for 18x18 mtrx */
/* tested */
int mtrx18Dinvp(double ** invA, double ** A)
{
    uchar i,j;
	uchar sizeD9 = 9*sizeof(double);
	uchar sizeDpr9 = 9*sizeof(double*);
    double **a = (double**)malloc(sizeDpr9);
    double **b = (double**)malloc(sizeDpr9);
    double **c = (double**)malloc(sizeDpr9);
    double **d = (double**)malloc(sizeDpr9);
	for(i=0;i<9;i++){
		a[i] = (double*)malloc(sizeD9);
		b[i] = (double*)malloc(sizeD9);
		c[i] = (double*)malloc(sizeD9);
		d[i] = (double*)malloc(sizeD9);
	}
	/* assign */
	for(i=0;i<9;i++){
		a[i][0] = A[i][0];
		a[i][1] = A[i][1];
		a[i][2] = A[i][2];
		a[i][3] = A[i][3];
		a[i][4] = A[i][4];
		a[i][5] = A[i][5];
		a[i][6] = A[i][6];
		a[i][7] = A[i][7];
		a[i][8] = A[i][8];

		b[i][0] = A[i][9];
		b[i][1] = A[i][10];
		b[i][2] = A[i][11];
		b[i][3] = A[i][12];
		b[i][4] = A[i][13];
		b[i][5] = A[i][14];
		b[i][6] = A[i][15];
		b[i][7] = A[i][16];
		b[i][8] = A[i][17];
		
		c[i][0] = A[i+9][0];
		c[i][1] = A[i+9][1];
		c[i][2] = A[i+9][2];
		c[i][3] = A[i+9][3];
		c[i][4] = A[i+9][4];
		c[i][5] = A[i+9][5];
		c[i][6] = A[i+9][6];
		c[i][7] = A[i+9][7];
		c[i][8] = A[i+9][8];

		d[i][0] = A[i+9][9];
		d[i][1] = A[i+9][10];
		d[i][2] = A[i+9][11];
		d[i][3] = A[i+9][12];
		d[i][4] = A[i+9][13];
		d[i][5] = A[i+9][14];
		d[i][6] = A[i+9][15];
		d[i][7] = A[i+9][16];
		d[i][8] = A[i+9][17];
	}

	double **inva = (double**)malloc(sizeDpr9);
	double **invd = (double**)malloc(sizeDpr9);
	double **invab = (double**)malloc(sizeDpr9);
	double **invdc = (double**)malloc(sizeDpr9);
	for(i=0;i<9;i++){
		inva[i] = (double*)malloc(sizeD9);
		invd[i] = (double*)malloc(sizeD9);
		invab[i] = (double*)malloc(sizeD9);
		invdc[i] = (double*)malloc(sizeD9);
	}
	mtrx9Dinvp(inva,a);
	mtrx9Dinvp(invd,d);
	mtrxMtrx9p(invab,inva,b);
	mtrxMtrx9p(invdc,invd,c);

	double **binvdc = (double**)malloc(sizeDpr9);
	double **cinvab = (double**)malloc(sizeDpr9);
	for(i=0;i<9;i++){
		binvdc[i] = (double*)malloc(sizeD9);
		cinvab[i] = (double*)malloc(sizeD9);
	}
	mtrxMtrx9p(binvdc,b,invdc);
	mtrxMtrx9p(cinvab,c,invab);

	/* binvdc = A - B*D^(-1)*C */
	/* cinvab = D - C*A^(-1)*B) */
	for(i=0;i<9;i++){
		for(j=0;j<9;j++){
			binvdc[i][j] = -binvdc[i][j] + a[i][j];
			cinvab[i][j] = -cinvab[i][j] + d[i][j];
		}
	}
	
	double **invA11 = (double**)malloc(sizeDpr9);
	double **invA22 = (double**)malloc(sizeDpr9);
	for(i=0;i<9;i++){
		invA11[i] = (double*)malloc(sizeD9);
		invA22[i] = (double*)malloc(sizeD9);
	}
	mtrx9Dinvp(invA11,binvdc); /* done */
	mtrx9Dinvp(invA22,cinvab); /* done */

	double **invA12 = (double**)malloc(sizeDpr9);
	double **invA21 = (double**)malloc(sizeDpr9);
	for(i=0;i<9;i++){
		invA12[i] = (double*)malloc(sizeD9);
		invA21[i] = (double*)malloc(sizeD9);
	}
	mtrxMtrx9p(invA12,invab,invA22); /* need to invA12 = -invA12 */
	mtrxMtrx9p(invA21,invdc,invA11); /* need to invA21 = -invA21 */

	/* assign */
	for(i=0;i<9;i++){
		invA[i][0]  = invA11[i][0];
		invA[i][1]  = invA11[i][1];
		invA[i][2]  = invA11[i][2];
		invA[i][3]  = invA11[i][3];
		invA[i][4]  = invA11[i][4];
		invA[i][5]  = invA11[i][5];
		invA[i][6]  = invA11[i][6];
		invA[i][7]  = invA11[i][7];
		invA[i][8]  = invA11[i][8];

		invA[i][9]  = -invA12[i][0];
		invA[i][10] = -invA12[i][1];
		invA[i][11] = -invA12[i][2];
		invA[i][12] = -invA12[i][3];
		invA[i][13] = -invA12[i][4];
		invA[i][14] = -invA12[i][5];
		invA[i][15] = -invA12[i][6];
		invA[i][16] = -invA12[i][7];
		invA[i][17] = -invA12[i][8];

		invA[i+9][0]  = -invA21[i][0];
		invA[i+9][1]  = -invA21[i][1];
		invA[i+9][2]  = -invA21[i][2];
		invA[i+9][3]  = -invA21[i][3];
		invA[i+9][4]  = -invA21[i][4];
		invA[i+9][5]  = -invA21[i][5];
		invA[i+9][6]  = -invA21[i][6];
		invA[i+9][7]  = -invA21[i][7];
		invA[i+9][8]  = -invA21[i][8];

		invA[i+9][9]  = invA22[i][0];
		invA[i+9][10] = invA22[i][1];
		invA[i+9][11] = invA22[i][2];
		invA[i+9][12] = invA22[i][3];
		invA[i+9][13] = invA22[i][4];
		invA[i+9][14] = invA22[i][5];
		invA[i+9][15] = invA22[i][6];
		invA[i+9][16] = invA22[i][7];
		invA[i+9][17] = invA22[i][8];
	}
	/* free memory  */
	for(i=0;i<9;i++){
		free(a[i]);
		free(b[i]);
		free(c[i]);
		free(d[i]);
		free(inva[i]);
		free(invd[i]);
		free(invab[i]);
		free(invdc[i]);
		free(binvdc[i]);
		free(cinvab[i]);
		free(invA11[i]);
		free(invA12[i]);
		free(invA21[i]);
		free(invA22[i]);
	}
	free(a);
	free(b);
	free(c);
	free(d);
	free(inva);
	free(invd);
	free(invab);
	free(invdc);
	free(binvdc);
	free(cinvab);
	free(invA11);
	free(invA12);
	free(invA21);
	free(invA22);

    return 0;
}

/* mn = m*n, all are 9x9 matrices */
int mtrxMtrx9p(double ** mn, double ** m, double ** n)
{
	uchar i,j,k;
    double tmp;
    for(i=0;i<9;i++){
        for(j=0;j<9;j++){
            tmp = 0;
            for(k=0;k<9;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return 0;
}

/* mn = m*n, all are 18x18 matrices */
int mtrxMtrx18p(double ** mn, double ** m, double ** n)
{
	uchar i,j,k;
    double tmp;
    for(i=0;i<18;i++){
        for(j=0;j<18;j++){
            tmp = 0;
            for(k=0;k<18;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return 0;
}

/* mn = m*n, m:3x3, n:3x6 --> mn:3x6*/
int mtrxMtrx3336p(double ** mn, double ** m, double ** n)
{
	uchar i,j,k;
    double tmp;
    for(i=0;i<3;i++){
        for(j=0;j<6;j++){
            tmp = 0;
            for(k=0;k<3;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return 0;
}

/* mn = m*n, m:6x3, n:3x3 --> mn:3x6*/
int mtrxMtrx6333p(double ** mn, double ** m, double ** n)
{
	uchar i,j,k;
    double tmp;
    for(i=0;i<6;i++){
        for(j=0;j<3;j++){
            tmp = 0;
            for(k=0;k<3;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return 0;
}

int mtrxMtrx3663p(double ** mn, double ** m, double ** n)
{
	uchar i,j,k;
    double tmp;
    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            tmp = 0;
            for(k=0;k<6;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return 0;
}

int mtrxMtrx6336p(double ** mn, double ** m, double ** n)
{
	uchar i,j,k;
    double tmp;
    for(i=0;i<6;i++){
        for(j=0;j<6;j++){
            tmp = 0;
            for(k=0;k<3;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return 0;
}

int mtrxMtrx3666p(double ** mn, double ** m, double ** n)
{
	uchar i,j,k;
    double tmp;
    for(i=0;i<3;i++){
        for(j=0;j<6;j++){
            tmp = 0;
            for(k=0;k<6;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return 0;
}

int mtrxMtrx6663p(double ** mn, double ** m, double ** n)
{
	uchar i,j,k;
    double tmp;
    for(i=0;i<6;i++){
        for(j=0;j<3;j++){
            tmp = 0;
            for(k=0;k<6;k++)
                tmp += m[i][k]*n[k][j];
            mn[i][j] = tmp;
        }
    }
    return 0;
}

/* only works for a 3x3 matrix with a 3x1 vector */
int mtrxXtensorp(double ** mV, double ** m, double * v)
{
    mV[0][0] = m[0][1]*v[2] - m[0][2]*v[1]; 
    mV[0][1] = m[0][2]*v[0] - m[0][0]*v[2]; 
    mV[0][2] = m[0][0]*v[1] - m[0][1]*v[0];
    mV[1][0] = m[1][1]*v[2] - m[1][2]*v[1]; 
    mV[1][1] = m[1][2]*v[0] - m[1][0]*v[2]; 
    mV[1][2] = m[1][0]*v[1] - m[1][1]*v[0];
    mV[2][0] = m[2][1]*v[2] - m[2][2]*v[1]; 
    mV[2][1] = m[2][2]*v[0] - m[2][0]*v[2]; 
    mV[2][2] = m[2][0]*v[1] - m[2][1]*v[0];
    return 0;
}

/*
int tensorp(double ** Vout, double * v)
{
    Vout[0][0] = 0;
    Vout[1][1] = 0;
    Vout[2][2] = 0;
    Vout[0][1] = -v[2];
    Vout[0][2] =  v[1];
    Vout[1][0] =  v[2];
    Vout[1][2] = -v[0];
    Vout[1][2] = -v[0];
    Vout[2][0] = -v[1];
    Vout[2][1] =  v[0];
    
    return 0;
} */


double max(double * a, uchar N)
{
	double maxNum;
	uchar i;
	maxNum = a[0];
	for(i=1;i<N;i++){
		if(a[i] > maxNum)
			maxNum = a[i];
	}
	return maxNum;
}

/* --------------------------- end of file ----------------------------- */



