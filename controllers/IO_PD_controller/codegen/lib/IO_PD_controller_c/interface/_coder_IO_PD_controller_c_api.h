/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_IO_PD_controller_c_api.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 28-Aug-2017 11:45:43
 */

#ifndef _CODER_IO_PD_CONTROLLER_C_API_H
#define _CODER_IO_PD_CONTROLLER_C_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_IO_PD_controller_c_api.h"

/* Type Definitions */
#ifndef typedef_struct0_T
#define typedef_struct0_T

typedef struct {
  real_T u_ff;
  real_T u_fb;
  real_T mu;
  real_T phi_des;
  real_T kerr;
  real_T phi_d;
  real_T V_eps;
  real_T d1;
  real_T psi0;
  real_T psi1;
} struct0_T;

#endif                                 /*typedef_struct0_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void IO_PD_controller_c(real_T x[4], real_T x_d[4], real_T *u, struct0_T *
  uparts);
extern void IO_PD_controller_c_api(const mxArray * const prhs[2], const mxArray *
  plhs[2]);
extern void IO_PD_controller_c_atexit(void);
extern void IO_PD_controller_c_initialize(void);
extern void IO_PD_controller_c_terminate(void);
extern void IO_PD_controller_c_xil_terminate(void);

#endif

/*
 * File trailer for _coder_IO_PD_controller_c_api.h
 *
 * [EOF]
 */
