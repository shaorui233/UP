/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_Jump2DBounds_api.c
 *
 * Code generation for function '_coder_Jump2DBounds_api'
 *
 */

/* Include files */
#include "tmwtypes.h"
#include "_coder_Jump2DBounds_api.h"
#include "_coder_Jump2DBounds_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131451U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "Jump2DBounds",                      /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[6];
static const mxArray *b_emlrt_marshallOut(const real_T u[35]);
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *in3,
  const char_T *identifier))[4];
static const mxArray *c_emlrt_marshallOut(const real_T u[26]);
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[4];
static const mxArray *d_emlrt_marshallOut(const real_T u);
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *in5,
  const char_T *identifier))[2];
static const mxArray *e_emlrt_marshallOut(const real_T u[4]);
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *in1, const
  char_T *identifier))[6];
static const mxArray *emlrt_marshallOut(const real_T u[10]);
static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[2];
static real_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt, const
  char_T *identifier);
static real_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6];
static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4];
static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[2];
static real_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);

/* Function Definitions */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[6]
{
  real_T (*y)[6];
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
  static const mxArray *b_emlrt_marshallOut(const real_T u[35])
{
  const mxArray *y;
  const mxArray *m1;
  static const int32_T iv2[1] = { 0 };

  static const int32_T iv3[1] = { 35 };

  y = NULL;
  m1 = emlrtCreateNumericArray(1, iv2, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m1, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m1, *(int32_T (*)[1])&iv3[0], 1);
  emlrtAssign(&y, m1);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *in3,
  const char_T *identifier))[4]
{
  real_T (*y)[4];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(in3), &thisId);
  emlrtDestroyArray(&in3);
  return y;
}
  static const mxArray *c_emlrt_marshallOut(const real_T u[26])
{
  const mxArray *y;
  const mxArray *m2;
  static const int32_T iv4[1] = { 0 };

  static const int32_T iv5[1] = { 26 };

  y = NULL;
  m2 = emlrtCreateNumericArray(1, iv4, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m2, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m2, *(int32_T (*)[1])&iv5[0], 1);
  emlrtAssign(&y, m2);
  return y;
}

static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[4]
{
  real_T (*y)[4];
  y = j_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
  static const mxArray *d_emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m3;
  y = NULL;
  m3 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m3);
  return y;
}

static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *in5,
  const char_T *identifier))[2]
{
  real_T (*y)[2];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(in5), &thisId);
  emlrtDestroyArray(&in5);
  return y;
}
  static const mxArray *e_emlrt_marshallOut(const real_T u[4])
{
  const mxArray *y;
  const mxArray *m4;
  static const int32_T iv6[1] = { 0 };

  static const int32_T iv7[1] = { 4 };

  y = NULL;
  m4 = emlrtCreateNumericArray(1, iv6, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m4, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m4, *(int32_T (*)[1])&iv7[0], 1);
  emlrtAssign(&y, m4);
  return y;
}

static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *in1, const
  char_T *identifier))[6]
{
  real_T (*y)[6];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(in1), &thisId);
  emlrtDestroyArray(&in1);
  return y;
}
  static const mxArray *emlrt_marshallOut(const real_T u[10])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[1] = { 0 };

  static const int32_T iv1[1] = { 10 };

  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m0, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m0, *(int32_T (*)[1])&iv1[0], 1);
  emlrtAssign(&y, m0);
  return y;
}

static real_T (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[2]
{
  real_T (*y)[2];
  y = k_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
  static real_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *dt,
  const char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(sp, emlrtAlias(dt), &thisId);
  emlrtDestroyArray(&dt);
  return y;
}

static real_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = l_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[6]
{
  real_T (*ret)[6];
  static const int32_T dims[1] = { 6 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  ret = (real_T (*)[6])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static real_T (*j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4]
{
  real_T (*ret)[4];
  static const int32_T dims[1] = { 4 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  ret = (real_T (*)[4])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[2]
{
  real_T (*ret)[2];
  static const int32_T dims[1] = { 2 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  ret = (real_T (*)[2])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static real_T l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void Jump2DBounds_api(const mxArray * const prhs[5], const mxArray *plhs[4])
{
  real_T (*decision_vars_lb)[10];
  real_T (*decision_vars_ub)[10];
  real_T (*constraints_ub)[10];
  real_T (*constraints_lb)[10];
  real_T (*in1)[6];
  real_T (*in2)[6];
  real_T (*in3)[4];
  real_T (*in4)[4];
  real_T (*in5)[2];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  decision_vars_lb = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));
  decision_vars_ub = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));
  constraints_ub = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));
  constraints_lb = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]), "in1");
  in2 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[1]), "in2");
  in3 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[2]), "in3");
  in4 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[3]), "in4");
  in5 = e_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[4]), "in5");

  /* Invoke the target function */
  Jump2DBounds(*in1, *in2, *in3, *in4, *in5, *decision_vars_lb,
               *decision_vars_ub, *constraints_ub, *constraints_lb);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*decision_vars_lb);
  plhs[1] = emlrt_marshallOut(*decision_vars_ub);
  plhs[2] = emlrt_marshallOut(*constraints_ub);
  plhs[3] = emlrt_marshallOut(*constraints_lb);
}

void Jump2DBounds_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  Jump2DBounds_xil_terminate();
}

void Jump2DBounds_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

void Jump2DBounds_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

void Jump2DConstraintJacobianFinalSP_api(const mxArray * const prhs[3], const
  mxArray *plhs[2])
{
  real_T (*row_index_final_CJ)[26];
  real_T (*col_index_final_CJ)[26];
  real_T iter;
  real_T NUM_X;
  real_T NUM_C;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  row_index_final_CJ = (real_T (*)[26])mxMalloc(sizeof(real_T [26]));
  col_index_final_CJ = (real_T (*)[26])mxMalloc(sizeof(real_T [26]));

  /* Marshall function inputs */
  iter = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[0]), "iter");
  NUM_X = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "NUM_X");
  NUM_C = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[2]), "NUM_C");

  /* Invoke the target function */
  Jump2DConstraintJacobianFinalSP(iter, NUM_X, NUM_C, *row_index_final_CJ,
    *col_index_final_CJ);

  /* Marshall function outputs */
  plhs[0] = c_emlrt_marshallOut(*row_index_final_CJ);
  plhs[1] = c_emlrt_marshallOut(*col_index_final_CJ);
}

void Jump2DConstraintJacobianFinal_api(const mxArray * const prhs[6], const
  mxArray *plhs[1])
{
  real_T (*constraint_jacobian_final_nz)[26];
  real_T (*in1)[2];
  real_T dt;
  real_T (*in3)[4];
  real_T m;
  real_T Iyy;
  real_T mu_g;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  constraint_jacobian_final_nz = (real_T (*)[26])mxMalloc(sizeof(real_T [26]));

  /* Marshall function inputs */
  in1 = e_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]), "in1");
  dt = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "dt");
  in3 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[2]), "in3");
  m = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[3]), "m");
  Iyy = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[4]), "Iyy");
  mu_g = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[5]), "mu_g");

  /* Invoke the target function */
  Jump2DConstraintJacobianFinal(*in1, dt, *in3, m, Iyy, mu_g,
    *constraint_jacobian_final_nz);

  /* Marshall function outputs */
  plhs[0] = c_emlrt_marshallOut(*constraint_jacobian_final_nz);
}

void Jump2DConstraintJacobianInitialSP_api(const mxArray * const prhs[3], const
  mxArray *plhs[2])
{
  real_T (*row_index_initial_CJ)[26];
  real_T (*col_index_initial_CJ)[26];
  real_T iter;
  real_T NUM_X;
  real_T NUM_C;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  row_index_initial_CJ = (real_T (*)[26])mxMalloc(sizeof(real_T [26]));
  col_index_initial_CJ = (real_T (*)[26])mxMalloc(sizeof(real_T [26]));

  /* Marshall function inputs */
  iter = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[0]), "iter");
  NUM_X = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "NUM_X");
  NUM_C = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[2]), "NUM_C");

  /* Invoke the target function */
  Jump2DConstraintJacobianInitialSP(iter, NUM_X, NUM_C, *row_index_initial_CJ,
    *col_index_initial_CJ);

  /* Marshall function outputs */
  plhs[0] = c_emlrt_marshallOut(*row_index_initial_CJ);
  plhs[1] = c_emlrt_marshallOut(*col_index_initial_CJ);
}

void Jump2DConstraintJacobianInitial_api(const mxArray * const prhs[6], const
  mxArray *plhs[1])
{
  real_T (*constraint_jacobian_initial_nz)[26];
  real_T (*in1)[2];
  real_T dt;
  real_T (*in3)[4];
  real_T m;
  real_T Iyy;
  real_T mu_g;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  constraint_jacobian_initial_nz = (real_T (*)[26])mxMalloc(sizeof(real_T [26]));

  /* Marshall function inputs */
  in1 = e_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]), "in1");
  dt = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "dt");
  in3 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[2]), "in3");
  m = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[3]), "m");
  Iyy = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[4]), "Iyy");
  mu_g = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[5]), "mu_g");

  /* Invoke the target function */
  Jump2DConstraintJacobianInitial(*in1, dt, *in3, m, Iyy, mu_g,
    *constraint_jacobian_initial_nz);

  /* Marshall function outputs */
  plhs[0] = c_emlrt_marshallOut(*constraint_jacobian_initial_nz);
}

void Jump2DConstraintJacobianSP_api(const mxArray * const prhs[3], const mxArray
  *plhs[2])
{
  real_T (*row_index_CJ)[35];
  real_T (*col_index_CJ)[35];
  real_T iter;
  real_T NUM_X;
  real_T NUM_C;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  row_index_CJ = (real_T (*)[35])mxMalloc(sizeof(real_T [35]));
  col_index_CJ = (real_T (*)[35])mxMalloc(sizeof(real_T [35]));

  /* Marshall function inputs */
  iter = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[0]), "iter");
  NUM_X = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "NUM_X");
  NUM_C = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[2]), "NUM_C");

  /* Invoke the target function */
  Jump2DConstraintJacobianSP(iter, NUM_X, NUM_C, *row_index_CJ, *col_index_CJ);

  /* Marshall function outputs */
  plhs[0] = b_emlrt_marshallOut(*row_index_CJ);
  plhs[1] = b_emlrt_marshallOut(*col_index_CJ);
}

void Jump2DConstraintJacobian_api(const mxArray * const prhs[6], const mxArray
  *plhs[1])
{
  real_T (*constraint_jacobian_nz)[35];
  real_T (*in1)[2];
  real_T dt;
  real_T (*in3)[4];
  real_T m;
  real_T Iyy;
  real_T mu_g;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  constraint_jacobian_nz = (real_T (*)[35])mxMalloc(sizeof(real_T [35]));

  /* Marshall function inputs */
  in1 = e_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]), "in1");
  dt = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "dt");
  in3 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[2]), "in3");
  m = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[3]), "m");
  Iyy = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[4]), "Iyy");
  mu_g = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[5]), "mu_g");

  /* Invoke the target function */
  Jump2DConstraintJacobian(*in1, dt, *in3, m, Iyy, mu_g, *constraint_jacobian_nz);

  /* Marshall function outputs */
  plhs[0] = b_emlrt_marshallOut(*constraint_jacobian_nz);
}

void Jump2DConstraintsFinal_api(const mxArray * const prhs[10], const mxArray
  *plhs[1])
{
  real_T (*constraints_final)[10];
  real_T (*in1)[6];
  real_T (*in2)[4];
  real_T (*in3)[6];
  real_T (*in4)[2];
  real_T dt;
  real_T (*in6)[4];
  real_T m;
  real_T Iyy;
  real_T g;
  real_T mu_g;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  constraints_final = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]), "in1");
  in2 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[1]), "in2");
  in3 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[2]), "in3");
  in4 = e_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[3]), "in4");
  dt = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[4]), "dt");
  in6 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[5]), "in6");
  m = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[6]), "m");
  Iyy = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[7]), "Iyy");
  g = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[8]), "g");
  mu_g = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[9]), "mu_g");

  /* Invoke the target function */
  Jump2DConstraintsFinal(*in1, *in2, *in3, *in4, dt, *in6, m, Iyy, g, mu_g,
    *constraints_final);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*constraints_final);
}

void Jump2DConstraintsInitial_api(const mxArray * const prhs[10], const mxArray *
  plhs[1])
{
  real_T (*constraints_initial)[10];
  real_T (*in1)[6];
  real_T (*in2)[4];
  real_T (*in3)[6];
  real_T (*in4)[2];
  real_T dt;
  real_T (*in6)[4];
  real_T m;
  real_T Iyy;
  real_T g;
  real_T mu_g;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  constraints_initial = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]), "in1");
  in2 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[1]), "in2");
  in3 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[2]), "in3");
  in4 = e_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[3]), "in4");
  dt = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[4]), "dt");
  in6 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[5]), "in6");
  m = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[6]), "m");
  Iyy = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[7]), "Iyy");
  g = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[8]), "g");
  mu_g = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[9]), "mu_g");

  /* Invoke the target function */
  Jump2DConstraintsInitial(*in1, *in2, *in3, *in4, dt, *in6, m, Iyy, g, mu_g,
    *constraints_initial);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*constraints_initial);
}

void Jump2DConstraints_api(const mxArray * const prhs[10], const mxArray *plhs[1])
{
  real_T (*constraints)[10];
  real_T (*in1)[6];
  real_T (*in2)[4];
  real_T (*in3)[6];
  real_T (*in4)[2];
  real_T dt;
  real_T (*in6)[4];
  real_T m;
  real_T Iyy;
  real_T g;
  real_T mu_g;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  constraints = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]), "in1");
  in2 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[1]), "in2");
  in3 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[2]), "in3");
  in4 = e_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[3]), "in4");
  dt = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[4]), "dt");
  in6 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[5]), "in6");
  m = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[6]), "m");
  Iyy = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[7]), "Iyy");
  g = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[8]), "g");
  mu_g = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[9]), "mu_g");

  /* Invoke the target function */
  Jump2DConstraints(*in1, *in2, *in3, *in4, dt, *in6, m, Iyy, g, mu_g,
                    *constraints);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*constraints);
}

void Jump2DCostGradient_api(const mxArray * const prhs[6], const mxArray *plhs[1])
{
  real_T (*cost_gradient)[10];
  real_T (*in1)[6];
  real_T (*in2)[4];
  real_T (*in3)[6];
  real_T (*in4)[4];
  real_T (*in5)[6];
  real_T (*in6)[4];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  cost_gradient = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]), "in1");
  in2 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[1]), "in2");
  in3 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[2]), "in3");
  in4 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[3]), "in4");
  in5 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[4]), "in5");
  in6 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[5]), "in6");

  /* Invoke the target function */
  Jump2DCostGradient(*in1, *in2, *in3, *in4, *in5, *in6, *cost_gradient);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*cost_gradient);
}

void Jump2DCost_api(const mxArray * const prhs[6], const mxArray *plhs[1])
{
  real_T (*in1)[6];
  real_T (*in2)[4];
  real_T (*in3)[6];
  real_T (*in4)[4];
  real_T (*in5)[6];
  real_T (*in6)[4];
  real_T J;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]), "in1");
  in2 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[1]), "in2");
  in3 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[2]), "in3");
  in4 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[3]), "in4");
  in5 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[4]), "in5");
  in6 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[5]), "in6");

  /* Invoke the target function */
  J = Jump2DCost(*in1, *in2, *in3, *in4, *in5, *in6);

  /* Marshall function outputs */
  plhs[0] = d_emlrt_marshallOut(J);
}

void Jump2DInitialize_api(const mxArray * const prhs[8], const mxArray *plhs[2])
{
  real_T (*decision_vars0)[10];
  real_T (*inputs_ref0)[4];
  real_T (*in1)[6];
  real_T (*in2)[4];
  real_T (*in3)[2];
  real_T dt;
  real_T (*in5)[4];
  real_T m;
  real_T Iyy;
  real_T g;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  decision_vars0 = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));
  inputs_ref0 = (real_T (*)[4])mxMalloc(sizeof(real_T [4]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]), "in1");
  in2 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[1]), "in2");
  in3 = e_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[2]), "in3");
  dt = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[3]), "dt");
  in5 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[4]), "in5");
  m = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[5]), "m");
  Iyy = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[6]), "Iyy");
  g = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[7]), "g");

  /* Invoke the target function */
  Jump2DInitialize(*in1, *in2, *in3, dt, *in5, m, Iyy, g, *decision_vars0,
                   *inputs_ref0);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*decision_vars0);
  plhs[1] = e_emlrt_marshallOut(*inputs_ref0);
}

void Jump2DLagrangianHessianFinalSP_api(const mxArray * const prhs[2], const
  mxArray *plhs[2])
{
  real_T (*row_index_final_H)[10];
  real_T (*col_index_final_H)[10];
  real_T iter;
  real_T NUM_X;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  row_index_final_H = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));
  col_index_final_H = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));

  /* Marshall function inputs */
  iter = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[0]), "iter");
  NUM_X = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "NUM_X");

  /* Invoke the target function */
  Jump2DLagrangianHessianFinalSP(iter, NUM_X, *row_index_final_H,
    *col_index_final_H);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*row_index_final_H);
  plhs[1] = emlrt_marshallOut(*col_index_final_H);
}

void Jump2DLagrangianHessianFinal_api(const mxArray * const prhs[3], const
  mxArray *plhs[1])
{
  real_T (*lagrangian_hessian_final_nz)[10];
  real_T (*in1)[6];
  real_T (*in2)[4];
  real_T obj_factor;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  lagrangian_hessian_final_nz = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]), "in1");
  in2 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[1]), "in2");
  obj_factor = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[2]),
    "obj_factor");

  /* Invoke the target function */
  Jump2DLagrangianHessianFinal(*in1, *in2, obj_factor,
    *lagrangian_hessian_final_nz);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*lagrangian_hessian_final_nz);
}

void Jump2DLagrangianHessianInitialSP_api(const mxArray * const prhs[2], const
  mxArray *plhs[2])
{
  real_T (*row_index_final_H)[10];
  real_T (*col_index_final_H)[10];
  real_T iter;
  real_T NUM_X;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  row_index_final_H = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));
  col_index_final_H = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));

  /* Marshall function inputs */
  iter = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[0]), "iter");
  NUM_X = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "NUM_X");

  /* Invoke the target function */
  Jump2DLagrangianHessianInitialSP(iter, NUM_X, *row_index_final_H,
    *col_index_final_H);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*row_index_final_H);
  plhs[1] = emlrt_marshallOut(*col_index_final_H);
}

void Jump2DLagrangianHessianInitial_api(const mxArray * const prhs[3], const
  mxArray *plhs[1])
{
  real_T (*lagrangian_hessian_initial_nz)[10];
  real_T (*in1)[6];
  real_T (*in2)[4];
  real_T obj_factor;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  lagrangian_hessian_initial_nz = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]), "in1");
  in2 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[1]), "in2");
  obj_factor = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[2]),
    "obj_factor");

  /* Invoke the target function */
  Jump2DLagrangianHessianInitial(*in1, *in2, obj_factor,
    *lagrangian_hessian_initial_nz);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*lagrangian_hessian_initial_nz);
}

void Jump2DLagrangianHessianSP_api(const mxArray * const prhs[2], const mxArray *
  plhs[2])
{
  real_T (*row_index_H)[10];
  real_T (*col_index_H)[10];
  real_T iter;
  real_T NUM_X;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  row_index_H = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));
  col_index_H = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));

  /* Marshall function inputs */
  iter = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[0]), "iter");
  NUM_X = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "NUM_X");

  /* Invoke the target function */
  Jump2DLagrangianHessianSP(iter, NUM_X, *row_index_H, *col_index_H);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*row_index_H);
  plhs[1] = emlrt_marshallOut(*col_index_H);
}

void Jump2DLagrangianHessian_api(const mxArray * const prhs[3], const mxArray
  *plhs[1])
{
  real_T (*lagrangian_hessian_nz)[10];
  real_T (*in1)[6];
  real_T (*in2)[4];
  real_T obj_factor;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  lagrangian_hessian_nz = (real_T (*)[10])mxMalloc(sizeof(real_T [10]));

  /* Marshall function inputs */
  in1 = emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[0]), "in1");
  in2 = c_emlrt_marshallIn(&st, emlrtAlias((const mxArray *)prhs[1]), "in2");
  obj_factor = g_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[2]),
    "obj_factor");

  /* Invoke the target function */
  Jump2DLagrangianHessian(*in1, *in2, obj_factor, *lagrangian_hessian_nz);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*lagrangian_hessian_nz);
}

/* End of code generation (_coder_Jump2DBounds_api.c) */
