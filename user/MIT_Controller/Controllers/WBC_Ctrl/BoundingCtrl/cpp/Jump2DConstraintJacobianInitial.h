/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Jump2DConstraintJacobianInitial.h
 *
 * Code generation for function 'Jump2DConstraintJacobianInitial'
 *
 */

#ifndef JUMP2DCONSTRAINTJACOBIANINITIAL_H
#define JUMP2DCONSTRAINTJACOBIANINITIAL_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "Jump2DBounds_types.h"

/* Function Declarations */
extern void Jump2DConstraintJacobianInitial(const double in1[2], double dt,
  const double in3[4], double m, double Iyy, double mu_g, double
  constraint_jacobian_initial_nz[26]);

#endif

/* End of code generation (Jump2DConstraintJacobianInitial.h) */
