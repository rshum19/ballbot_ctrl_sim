/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_IO_PD_controller_c_api.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 28-Aug-2017 11:45:43
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_IO_PD_controller_c_api.h"
#include "_coder_IO_PD_controller_c_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131450U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "IO_PD_controller_c",                /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[4];
static const mxArray *b_emlrt_marshallOut(const struct0_T *u);
static real_T (*c_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[4];
static real_T (*emlrt_marshallIn(const mxArray *x, const char_T *identifier))[4];
static const mxArray *emlrt_marshallOut(const real_T u);

/* Function Definitions */

/*
 * Arguments    : const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[4]
 */
static real_T (*b_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[4]
{
  real_T (*y)[4];
  y = c_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const struct0_T *u
 * Return Type  : const mxArray *
 */
  static const mxArray *b_emlrt_marshallOut(const struct0_T *u)
{
  const mxArray *y;
  y = NULL;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 0, NULL));
  emlrtAddField(y, emlrt_marshallOut(u->u_ff), "u_ff", 0);
  emlrtAddField(y, emlrt_marshallOut(u->u_fb), "u_fb", 0);
  emlrtAddField(y, emlrt_marshallOut(u->mu), "mu", 0);
  emlrtAddField(y, emlrt_marshallOut(u->phi_des), "phi_des", 0);
  emlrtAddField(y, emlrt_marshallOut(u->kerr), "kerr", 0);
  emlrtAddField(y, emlrt_marshallOut(u->phi_d), "phi_d", 0);
  emlrtAddField(y, emlrt_marshallOut(u->V_eps), "V_eps", 0);
  emlrtAddField(y, emlrt_marshallOut(u->d1), "d1", 0);
  emlrtAddField(y, emlrt_marshallOut(u->psi0), "psi0", 0);
  emlrtAddField(y, emlrt_marshallOut(u->psi1), "psi1", 0);
  return y;
}

/*
 * Arguments    : const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[4]
 */
static real_T (*c_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[4]
{
  real_T (*ret)[4];
  static const int32_T dims[1] = { 4 };

  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
    dims);
  ret = (real_T (*)[4])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const mxArray *x
 *                const char_T *identifier
 * Return Type  : real_T (*)[4]
 */
  static real_T (*emlrt_marshallIn(const mxArray *x, const char_T *identifier))
  [4]
{
  real_T (*y)[4];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(emlrtAlias(x), &thisId);
  emlrtDestroyArray(&x);
  return y;
}

/*
 * Arguments    : const real_T u
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m0;
  y = NULL;
  m0 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[2]
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void IO_PD_controller_c_api(const mxArray * const prhs[2], const mxArray *plhs[2])
{
  real_T (*x)[4];
  real_T (*x_d)[4];
  real_T u;
  struct0_T uparts;

  /* Marshall function inputs */
  x = emlrt_marshallIn(emlrtAlias((const mxArray *)prhs[0]), "x");
  x_d = emlrt_marshallIn(emlrtAlias((const mxArray *)prhs[1]), "x_d");

  /* Invoke the target function */
  IO_PD_controller_c(*x, *x_d, &u, &uparts);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(u);
  plhs[1] = b_emlrt_marshallOut(&uparts);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void IO_PD_controller_c_atexit(void)
{
  mexFunctionCreateRootTLS();
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  IO_PD_controller_c_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void IO_PD_controller_c_initialize(void)
{
  mexFunctionCreateRootTLS();
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, false, 0U, 0);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void IO_PD_controller_c_terminate(void)
{
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_IO_PD_controller_c_api.c
 *
 * [EOF]
 */
