/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Jump2DLagrangianHessian.h
 *
 * Code generation for function 'Jump2DLagrangianHessian'
 *
 */

#ifndef JUMP2DLAGRANGIANHESSIAN_H
#define JUMP2DLAGRANGIANHESSIAN_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "Jump2DBounds_types.h"

/* Function Declarations */
extern void Jump2DLagrangianHessian(const double in1[6], const double in2[4],
  double obj_factor, double lagrangian_hessian_nz[10]);

#endif

/* End of code generation (Jump2DLagrangianHessian.h) */
