/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Jump2DConstraintsFinal.h
 *
 * Code generation for function 'Jump2DConstraintsFinal'
 *
 */

#ifndef JUMP2DCONSTRAINTSFINAL_H
#define JUMP2DCONSTRAINTSFINAL_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "Jump2DBounds_types.h"

/* Function Declarations */
extern void Jump2DConstraintsFinal(const double in1[6], const double in2[4],
  const double in3[6], const double in4[2], double dt, const double in6[4],
  double m, double Iyy, double g, double mu_g, double constraints_final[10]);

#endif

/* End of code generation (Jump2DConstraintsFinal.h) */
