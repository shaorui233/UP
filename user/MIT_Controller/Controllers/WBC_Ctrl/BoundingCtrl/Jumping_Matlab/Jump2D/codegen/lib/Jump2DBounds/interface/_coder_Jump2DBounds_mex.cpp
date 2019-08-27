/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_Jump2DBounds_mex.cpp
 *
 * Code generation for function '_coder_Jump2DBounds_mex'
 *
 */

/* Include files */
#include "_coder_Jump2DBounds_api.h"
#include "_coder_Jump2DBounds_mex.h"

/* Variable Definitions */
static const char * emlrtEntryPoints[19] = { "Jump2DBounds",
  "Jump2DConstraintJacobian", "Jump2DConstraintJacobianFinal",
  "Jump2DConstraintJacobianFinalSP", "Jump2DConstraintJacobianInitial",
  "Jump2DConstraintJacobianInitialSP", "Jump2DConstraintJacobianSP",
  "Jump2DConstraints", "Jump2DConstraintsFinal", "Jump2DConstraintsInitial",
  "Jump2DCost", "Jump2DCostGradient", "Jump2DInitialize",
  "Jump2DLagrangianHessian", "Jump2DLagrangianHessianFinal",
  "Jump2DLagrangianHessianFinalSP", "Jump2DLagrangianHessianInitial",
  "Jump2DLagrangianHessianInitialSP", "Jump2DLagrangianHessianSP" };

/* Function Declarations */
static void Jump2DBounds_mexFunction(int32_T nlhs, mxArray *plhs[4], int32_T
  nrhs, const mxArray *prhs[5]);
static void Jump2DConstraints_mexFunction(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[10]);
static void Jump2DCostGradient_mexFunction(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[6]);
static void Jump2DCost_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
  const mxArray *prhs[6]);
static void Jump2DInitialize_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T
  nrhs, const mxArray *prhs[8]);
static void c_Jump2DConstraintJacobianFinal(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[6]);
static void c_Jump2DConstraintJacobianIniti(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[6]);
static void c_Jump2DConstraintJacobianSP_me(int32_T nlhs, mxArray *plhs[2],
  int32_T nrhs, const mxArray *prhs[3]);
static void c_Jump2DConstraintJacobian_mexF(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[6]);
static void c_Jump2DConstraintsFinal_mexFun(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[10]);
static void c_Jump2DConstraintsInitial_mexF(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[10]);
static void c_Jump2DLagrangianHessianFinalS(int32_T nlhs, mxArray *plhs[2],
  int32_T nrhs, const mxArray *prhs[2]);
static void c_Jump2DLagrangianHessianFinal_(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[3]);
static void c_Jump2DLagrangianHessianInitia(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[3]);
static void c_Jump2DLagrangianHessianSP_mex(int32_T nlhs, mxArray *plhs[2],
  int32_T nrhs, const mxArray *prhs[2]);
static void c_Jump2DLagrangianHessian_mexFu(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[3]);
static void d_Jump2DConstraintJacobianFinal(int32_T nlhs, mxArray *plhs[2],
  int32_T nrhs, const mxArray *prhs[3]);
static void d_Jump2DConstraintJacobianIniti(int32_T nlhs, mxArray *plhs[2],
  int32_T nrhs, const mxArray *prhs[3]);
static void d_Jump2DLagrangianHessianInitia(int32_T nlhs, mxArray *plhs[2],
  int32_T nrhs, const mxArray *prhs[2]);

/* Function Definitions */
static void Jump2DBounds_mexFunction(int32_T nlhs, mxArray *plhs[4], int32_T
  nrhs, const mxArray *prhs[5])
{
  const mxArray *inputs[5];
  const mxArray *outputs[4];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 5) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 5, 4,
                        12, "Jump2DBounds");
  }

  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 12,
                        "Jump2DBounds");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DBounds_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void Jump2DConstraints_mexFunction(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[10])
{
  const mxArray *inputs[10];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 10) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 10, 4,
                        17, "Jump2DConstraints");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 17,
                        "Jump2DConstraints");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DConstraints_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void Jump2DCostGradient_mexFunction(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[6])
{
  const mxArray *inputs[6];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 6, 4,
                        18, "Jump2DCostGradient");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 18,
                        "Jump2DCostGradient");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DCostGradient_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void Jump2DCost_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
  const mxArray *prhs[6])
{
  const mxArray *inputs[6];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 6, 4,
                        10, "Jump2DCost");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 10,
                        "Jump2DCost");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DCost_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void Jump2DInitialize_mexFunction(int32_T nlhs, mxArray *plhs[2], int32_T
  nrhs, const mxArray *prhs[8])
{
  const mxArray *inputs[8];
  const mxArray *outputs[2];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 8) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 8, 4,
                        16, "Jump2DInitialize");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 16,
                        "Jump2DInitialize");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DInitialize_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void c_Jump2DConstraintJacobianFinal(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[6])
{
  const mxArray *inputs[6];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 6, 4,
                        29, "Jump2DConstraintJacobianFinal");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 29,
                        "Jump2DConstraintJacobianFinal");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DConstraintJacobianFinal_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void c_Jump2DConstraintJacobianIniti(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[6])
{
  const mxArray *inputs[6];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 6, 4,
                        31, "Jump2DConstraintJacobianInitial");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 31,
                        "Jump2DConstraintJacobianInitial");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DConstraintJacobianInitial_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void c_Jump2DConstraintJacobianSP_me(int32_T nlhs, mxArray *plhs[2],
  int32_T nrhs, const mxArray *prhs[3])
{
  const mxArray *inputs[3];
  const mxArray *outputs[2];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        26, "Jump2DConstraintJacobianSP");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 26,
                        "Jump2DConstraintJacobianSP");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DConstraintJacobianSP_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void c_Jump2DConstraintJacobian_mexF(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[6])
{
  const mxArray *inputs[6];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 6, 4,
                        24, "Jump2DConstraintJacobian");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 24,
                        "Jump2DConstraintJacobian");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DConstraintJacobian_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void c_Jump2DConstraintsFinal_mexFun(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[10])
{
  const mxArray *inputs[10];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 10) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 10, 4,
                        22, "Jump2DConstraintsFinal");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 22,
                        "Jump2DConstraintsFinal");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DConstraintsFinal_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void c_Jump2DConstraintsInitial_mexF(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[10])
{
  const mxArray *inputs[10];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 10) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 10, 4,
                        24, "Jump2DConstraintsInitial");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 24,
                        "Jump2DConstraintsInitial");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DConstraintsInitial_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void c_Jump2DLagrangianHessianFinalS(int32_T nlhs, mxArray *plhs[2],
  int32_T nrhs, const mxArray *prhs[2])
{
  const mxArray *inputs[2];
  const mxArray *outputs[2];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4,
                        30, "Jump2DLagrangianHessianFinalSP");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 30,
                        "Jump2DLagrangianHessianFinalSP");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DLagrangianHessianFinalSP_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void c_Jump2DLagrangianHessianFinal_(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[3])
{
  const mxArray *inputs[3];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        28, "Jump2DLagrangianHessianFinal");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 28,
                        "Jump2DLagrangianHessianFinal");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DLagrangianHessianFinal_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void c_Jump2DLagrangianHessianInitia(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[3])
{
  const mxArray *inputs[3];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        30, "Jump2DLagrangianHessianInitial");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 30,
                        "Jump2DLagrangianHessianInitial");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DLagrangianHessianInitial_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void c_Jump2DLagrangianHessianSP_mex(int32_T nlhs, mxArray *plhs[2],
  int32_T nrhs, const mxArray *prhs[2])
{
  const mxArray *inputs[2];
  const mxArray *outputs[2];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4,
                        25, "Jump2DLagrangianHessianSP");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 25,
                        "Jump2DLagrangianHessianSP");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DLagrangianHessianSP_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void c_Jump2DLagrangianHessian_mexFu(int32_T nlhs, mxArray *plhs[1],
  int32_T nrhs, const mxArray *prhs[3])
{
  const mxArray *inputs[3];
  const mxArray *outputs[1];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        23, "Jump2DLagrangianHessian");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 23,
                        "Jump2DLagrangianHessian");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DLagrangianHessian_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void d_Jump2DConstraintJacobianFinal(int32_T nlhs, mxArray *plhs[2],
  int32_T nrhs, const mxArray *prhs[3])
{
  const mxArray *inputs[3];
  const mxArray *outputs[2];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        31, "Jump2DConstraintJacobianFinalSP");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 31,
                        "Jump2DConstraintJacobianFinalSP");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DConstraintJacobianFinalSP_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void d_Jump2DConstraintJacobianIniti(int32_T nlhs, mxArray *plhs[2],
  int32_T nrhs, const mxArray *prhs[3])
{
  const mxArray *inputs[3];
  const mxArray *outputs[2];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 3, 4,
                        33, "Jump2DConstraintJacobianInitialSP");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 33,
                        "Jump2DConstraintJacobianInitialSP");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DConstraintJacobianInitialSP_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

static void d_Jump2DLagrangianHessianInitia(int32_T nlhs, mxArray *plhs[2],
  int32_T nrhs, const mxArray *prhs[2])
{
  const mxArray *inputs[2];
  const mxArray *outputs[2];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 2, 4,
                        32, "Jump2DLagrangianHessianInitialSP");
  }

  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 32,
                        "Jump2DLagrangianHessianInitialSP");
  }

  /* Temporary copy for mex inputs. */
  if (0 <= nrhs - 1) {
    memcpy((void *)&inputs[0], (void *)&prhs[0], (uint32_T)(nrhs * (int32_T)
            sizeof(const mxArray *)));
  }

  /* Call the function. */
  Jump2DLagrangianHessianInitialSP_api(inputs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  Jump2DBounds_terminate();
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexAtExit(Jump2DBounds_atexit);

  /* Initialize the memory manager. */
  /* Module initialization. */
  Jump2DBounds_initialize();
  st.tls = emlrtRootTLSGlobal;

  /* Dispatch the entry-point. */
  switch (emlrtGetEntryPointIndexR2016a(&st, nrhs, prhs, emlrtEntryPoints, 19))
  {
   case 0:
    Jump2DBounds_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 1:
    c_Jump2DConstraintJacobian_mexF(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 2:
    c_Jump2DConstraintJacobianFinal(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 3:
    d_Jump2DConstraintJacobianFinal(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 4:
    c_Jump2DConstraintJacobianIniti(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 5:
    d_Jump2DConstraintJacobianIniti(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 6:
    c_Jump2DConstraintJacobianSP_me(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 7:
    Jump2DConstraints_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 8:
    c_Jump2DConstraintsFinal_mexFun(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 9:
    c_Jump2DConstraintsInitial_mexF(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 10:
    Jump2DCost_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 11:
    Jump2DCostGradient_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 12:
    Jump2DInitialize_mexFunction(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 13:
    c_Jump2DLagrangianHessian_mexFu(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 14:
    c_Jump2DLagrangianHessianFinal_(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 15:
    c_Jump2DLagrangianHessianFinalS(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 16:
    c_Jump2DLagrangianHessianInitia(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 17:
    d_Jump2DLagrangianHessianInitia(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;

   case 18:
    c_Jump2DLagrangianHessianSP_mex(nlhs, plhs, nrhs - 1, &prhs[1]);
    break;
  }
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_Jump2DBounds_mex.cpp) */
