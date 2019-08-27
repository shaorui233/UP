/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_Jump2DBounds_api.h
 *
 * Code generation for function '_coder_Jump2DBounds_api'
 *
 */

#ifndef _CODER_JUMP2DBOUNDS_API_H
#define _CODER_JUMP2DBOUNDS_API_H

/* Include files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_Jump2DBounds_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void Jump2DBounds(real_T in1[6], real_T in2[6], real_T in3[4], real_T
  in4[4], real_T in5[2], real_T decision_vars_lb[10], real_T decision_vars_ub[10],
  real_T constraints_ub[10], real_T constraints_lb[10]);
extern void Jump2DBounds_api(const mxArray * const prhs[5], const mxArray *plhs
  [4]);
extern void Jump2DBounds_atexit(void);
extern void Jump2DBounds_initialize(void);
extern void Jump2DBounds_terminate(void);
extern void Jump2DBounds_xil_terminate(void);
extern void Jump2DConstraintJacobian(real_T in1[2], real_T dt, real_T in3[4],
  real_T m, real_T Iyy, real_T mu_g, real_T constraint_jacobian_nz[35]);
extern void Jump2DConstraintJacobianFinal(real_T in1[2], real_T dt, real_T in3[4],
  real_T m, real_T Iyy, real_T mu_g, real_T constraint_jacobian_final_nz[26]);
extern void Jump2DConstraintJacobianFinalSP(real_T iter, real_T NUM_X, real_T
  NUM_C, real_T row_index_final_CJ[26], real_T col_index_final_CJ[26]);
extern void Jump2DConstraintJacobianFinalSP_api(const mxArray * const prhs[3],
  const mxArray *plhs[2]);
extern void Jump2DConstraintJacobianFinal_api(const mxArray * const prhs[6],
  const mxArray *plhs[1]);
extern void Jump2DConstraintJacobianInitial(real_T in1[2], real_T dt, real_T
  in3[4], real_T m, real_T Iyy, real_T mu_g, real_T
  constraint_jacobian_initial_nz[26]);
extern void Jump2DConstraintJacobianInitialSP(real_T iter, real_T NUM_X, real_T
  NUM_C, real_T row_index_initial_CJ[26], real_T col_index_initial_CJ[26]);
extern void Jump2DConstraintJacobianInitialSP_api(const mxArray * const prhs[3],
  const mxArray *plhs[2]);
extern void Jump2DConstraintJacobianInitial_api(const mxArray * const prhs[6],
  const mxArray *plhs[1]);
extern void Jump2DConstraintJacobianSP(real_T iter, real_T NUM_X, real_T NUM_C,
  real_T row_index_CJ[35], real_T col_index_CJ[35]);
extern void Jump2DConstraintJacobianSP_api(const mxArray * const prhs[3], const
  mxArray *plhs[2]);
extern void Jump2DConstraintJacobian_api(const mxArray * const prhs[6], const
  mxArray *plhs[1]);
extern void Jump2DConstraints(real_T in1[6], real_T in2[4], real_T in3[6],
  real_T in4[2], real_T dt, real_T in6[4], real_T m, real_T Iyy, real_T g,
  real_T mu_g, real_T constraints[10]);
extern void Jump2DConstraintsFinal(real_T in1[6], real_T in2[4], real_T in3[6],
  real_T in4[2], real_T dt, real_T in6[4], real_T m, real_T Iyy, real_T g,
  real_T mu_g, real_T constraints_final[10]);
extern void Jump2DConstraintsFinal_api(const mxArray * const prhs[10], const
  mxArray *plhs[1]);
extern void Jump2DConstraintsInitial(real_T in1[6], real_T in2[4], real_T in3[6],
  real_T in4[2], real_T dt, real_T in6[4], real_T m, real_T Iyy, real_T g,
  real_T mu_g, real_T constraints_initial[10]);
extern void Jump2DConstraintsInitial_api(const mxArray * const prhs[10], const
  mxArray *plhs[1]);
extern void Jump2DConstraints_api(const mxArray * const prhs[10], const mxArray *
  plhs[1]);
extern real_T Jump2DCost(real_T in1[6], real_T in2[4], real_T in3[6], real_T
  in4[4], real_T in5[6], real_T in6[4]);
extern void Jump2DCostGradient(real_T in1[6], real_T in2[4], real_T in3[6],
  real_T in4[4], real_T in5[6], real_T in6[4], real_T cost_gradient[10]);
extern void Jump2DCostGradient_api(const mxArray * const prhs[6], const mxArray *
  plhs[1]);
extern void Jump2DCost_api(const mxArray * const prhs[6], const mxArray *plhs[1]);
extern void Jump2DInitialize(real_T in1[6], real_T in2[4], real_T in3[2], real_T
  dt, real_T in5[4], real_T m, real_T Iyy, real_T g, real_T decision_vars0[10],
  real_T inputs_ref0[4]);
extern void Jump2DInitialize_api(const mxArray * const prhs[8], const mxArray
  *plhs[2]);
extern void Jump2DLagrangianHessian(real_T in1[6], real_T in2[4], real_T
  obj_factor, real_T lagrangian_hessian_nz[10]);
extern void Jump2DLagrangianHessianFinal(real_T in1[6], real_T in2[4], real_T
  obj_factor, real_T lagrangian_hessian_final_nz[10]);
extern void Jump2DLagrangianHessianFinalSP(real_T iter, real_T NUM_X, real_T
  row_index_final_H[10], real_T col_index_final_H[10]);
extern void Jump2DLagrangianHessianFinalSP_api(const mxArray * const prhs[2],
  const mxArray *plhs[2]);
extern void Jump2DLagrangianHessianFinal_api(const mxArray * const prhs[3],
  const mxArray *plhs[1]);
extern void Jump2DLagrangianHessianInitial(real_T in1[6], real_T in2[4], real_T
  obj_factor, real_T lagrangian_hessian_initial_nz[10]);
extern void Jump2DLagrangianHessianInitialSP(real_T iter, real_T NUM_X, real_T
  row_index_final_H[10], real_T col_index_final_H[10]);
extern void Jump2DLagrangianHessianInitialSP_api(const mxArray * const prhs[2],
  const mxArray *plhs[2]);
extern void Jump2DLagrangianHessianInitial_api(const mxArray * const prhs[3],
  const mxArray *plhs[1]);
extern void Jump2DLagrangianHessianSP(real_T iter, real_T NUM_X, real_T
  row_index_H[10], real_T col_index_H[10]);
extern void Jump2DLagrangianHessianSP_api(const mxArray * const prhs[2], const
  mxArray *plhs[2]);
extern void Jump2DLagrangianHessian_api(const mxArray * const prhs[3], const
  mxArray *plhs[1]);

#endif

/* End of code generation (_coder_Jump2DBounds_api.h) */
