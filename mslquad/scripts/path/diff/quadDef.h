/*****************************************************************************************
File Name     : quadDef.h
Author        : Dingjiang Zhou
                Boston University, Boston, 02215
Email         : zdj@bu.edu zhoudingjiang@gmail.com
Create Time   : Tue, Jul. 29th, 2014. 11:26:14 AM
Last Modified : Tue, Jul. 29th, 2014. 11:26:14 AM
Purpose       : define the parameter structure, states structure, inputs structure and 
                maybe others.
*****************************************************************************************/
#ifndef __QUADDEF_H__
#define __QUADDEF_H__


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef uchar
#define uchar unsigned char 
#endif

typedef struct{
	uchar    mdl;   /* model == 1 for K500, model == 2 for KNano and so on*/
    double   L;     /* for plotting */
    double   H;     /* for plotting */
	double **J;		/* inertial matrix */
	double **invJ;  /* the inverse of inertial matirx */
	double   m;
	double  *trck; /* the position of the trackable in the body frame (z-down) */
}Params; /* name: quad parameters */ 

typedef struct{
	double *v;
	double **R;
	double *wb;
	double *h;
}States;

typedef struct{
	double fz;
	double *tau;
}Inputs;

typedef struct{
	double **Qnse;
	double **Rnse;
	double **Pcov;
	double **H;
	double **F;
	double **Pprd;
	double  *Xprd;
}Kalman;

typedef struct{
	double *sig;
	double *sigd1;
	double *sigd2;
	double *sigd3;
	double *sigd4;
}FlatOut; /*Note: Is differnt with the structure in Matlab */

typedef struct{
	double **pkp;
	double **pki;
	double **pkd;
	int 	 piEn; /* enable pki or not */
	double **okp;
	double **oki;
	double **okd;
	int 	 oiEn; /* enable oki or not */
}PIDparam;

typedef struct{
	double *epSm;
	double *eRSm;
}ErrSum;

typedef struct{
	double **pos; /* includes [pos,posd1,...,posd4] */
	double **elr; /* includes [elr,elrd1,elerd3] */
}VRB;

typedef struct{
	double **d0;
	double **d1;
	double **d2;
	double **d3;
	double **d4;
}Rderivs;		 	/* rotation matrix derivatives to fourth order */

typedef struct{
	double *d0;
	double *d1;
	double *d2;
	double *d3;
}Vderivs;

typedef struct{
    int iT;       	/* the length of the following fields */
    double **pos; 	/* each row: [pos',posd1',posd2',posd3',posd4'] */
	double **elr; 	/* each row: [elr',elrd1',elrd2'] */
}vrbTrj;


typedef struct{
    int iT; 		/* the length of the following fields */
    double **fout; 	/* example fout : [80x625 double], where iT = 625 */
}botsTrj;

typedef struct{
	uchar rbtN;
	uchar fmtN;
	double ***rbtP;
	double *tF;
	double *tT;
	double *tA;
	uchar crSc;
	double *crtP;
	double ***mtrx;
}Formation;
#endif


/* ---------------------------------- end of file ------------------------------------- */












