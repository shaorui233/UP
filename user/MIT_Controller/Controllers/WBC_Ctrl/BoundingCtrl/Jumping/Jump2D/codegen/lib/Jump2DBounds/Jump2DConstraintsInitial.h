/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Jump2DConstraintsInitial.h
 *
 * Code generation for function 'Jump2DConstraintsInitial'
 *
 */

#ifndef JUMP2DCONSTRAINTSINITIAL_H
#define JUMP2DCONSTRAINTSINITIAL_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "Jump2DBounds_types.h"

/* Function Declarations */
extern void Jump2DConstraintsInitial(const double in1[6], const double in2[4],
  const double in3[6], const double in4[2], double dt, const double in6[4],
  double m, double Iyy, double g, double mu_g, double constraints_initial[10]);

#endif

/* End of code generation (Jump2DConstraintsInitial.h) */
