/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * main.cpp
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
/* Include files */
#include "rt_nonfinite.h"
#include "Jump2DBounds.h"
#include "Jump2DConstraintJacobian.h"
#include "Jump2DConstraintJacobianFinal.h"
#include "Jump2DConstraintJacobianFinalSP.h"
#include "Jump2DConstraintJacobianInitial.h"
#include "Jump2DConstraintJacobianInitialSP.h"
#include "Jump2DConstraintJacobianSP.h"
#include "Jump2DConstraints.h"
#include "Jump2DConstraintsFinal.h"
#include "Jump2DConstraintsInitial.h"
#include "Jump2DCost.h"
#include "Jump2DCostGradient.h"
#include "Jump2DInitialize.h"
#include "Jump2DLagrangianHessian.h"
#include "Jump2DLagrangianHessianFinal.h"
#include "Jump2DLagrangianHessianFinalSP.h"
#include "Jump2DLagrangianHessianInitial.h"
#include "Jump2DLagrangianHessianInitialSP.h"
#include "Jump2DLagrangianHessianSP.h"
#include "main.h"
#include "Jump2DBounds_terminate.h"
#include "Jump2DBounds_initialize.h"

/* Function Declarations */
static void argInit_2x1_real_T(double result[2]);
static void argInit_4x1_real_T(double result[4]);
static void argInit_6x1_real_T(double result[6]);
static double argInit_real_T();
static void main_Jump2DBounds();
static void main_Jump2DConstraintJacobian();
static void main_Jump2DConstraintJacobianFinal();
static void main_Jump2DConstraintJacobianFinalSP();
static void main_Jump2DConstraintJacobianInitial();
static void main_Jump2DConstraintJacobianInitialSP();
static void main_Jump2DConstraintJacobianSP();
static void main_Jump2DConstraints();
static void main_Jump2DConstraintsFinal();
static void main_Jump2DConstraintsInitial();
static void main_Jump2DCost();
static void main_Jump2DCostGradient();
static void main_Jump2DInitialize();
static void main_Jump2DLagrangianHessian();
static void main_Jump2DLagrangianHessianFinal();
static void main_Jump2DLagrangianHessianFinalSP();
static void main_Jump2DLagrangianHessianInitial();
static void main_Jump2DLagrangianHessianInitialSP();
static void main_Jump2DLagrangianHessianSP();

/* Function Definitions */
static void argInit_2x1_real_T(double result[2])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 2; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static void argInit_4x1_real_T(double result[4])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 4; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static void argInit_6x1_real_T(double result[6])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 6; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = argInit_real_T();
  }
}

static double argInit_real_T()
{
  return 0.0;
}

static void main_Jump2DBounds()
{
  double dv1[6];
  double dv2[6];
  double dv3[4];
  double dv4[4];
  double dv5[2];
  double decision_vars_lb[10];
  double decision_vars_ub[10];
  double constraints_ub[10];
  double constraints_lb[10];

  /* Initialize function 'Jump2DBounds' input arguments. */
  /* Initialize function input argument 'in1'. */
  /* Initialize function input argument 'in2'. */
  /* Initialize function input argument 'in3'. */
  /* Initialize function input argument 'in4'. */
  /* Initialize function input argument 'in5'. */
  /* Call the entry-point 'Jump2DBounds'. */
  argInit_6x1_real_T(dv1);
  argInit_6x1_real_T(dv2);
  argInit_4x1_real_T(dv3);
  argInit_4x1_real_T(dv4);
  argInit_2x1_real_T(dv5);
  Jump2DBounds(dv1, dv2, dv3, dv4, dv5, decision_vars_lb, decision_vars_ub,
               constraints_ub, constraints_lb);
}

static void main_Jump2DConstraintJacobian()
{
  double dv6[2];
  double dv7[4];
  double constraint_jacobian_nz[35];

  /* Initialize function 'Jump2DConstraintJacobian' input arguments. */
  /* Initialize function input argument 'in1'. */
  /* Initialize function input argument 'in3'. */
  /* Call the entry-point 'Jump2DConstraintJacobian'. */
  argInit_2x1_real_T(dv6);
  argInit_4x1_real_T(dv7);
  Jump2DConstraintJacobian(dv6, argInit_real_T(), dv7, argInit_real_T(),
    argInit_real_T(), argInit_real_T(), constraint_jacobian_nz);
}

static void main_Jump2DConstraintJacobianFinal()
{
  double dv8[2];
  double dv9[4];
  double constraint_jacobian_final_nz[26];

  /* Initialize function 'Jump2DConstraintJacobianFinal' input arguments. */
  /* Initialize function input argument 'in1'. */
  /* Initialize function input argument 'in3'. */
  /* Call the entry-point 'Jump2DConstraintJacobianFinal'. */
  argInit_2x1_real_T(dv8);
  argInit_4x1_real_T(dv9);
  Jump2DConstraintJacobianFinal(dv8, argInit_real_T(), dv9, argInit_real_T(),
    argInit_real_T(), argInit_real_T(), constraint_jacobian_final_nz);
}

static void main_Jump2DConstraintJacobianFinalSP()
{
  double row_index_final_CJ[26];
  double col_index_final_CJ[26];

  /* Initialize function 'Jump2DConstraintJacobianFinalSP' input arguments. */
  /* Call the entry-point 'Jump2DConstraintJacobianFinalSP'. */
  Jump2DConstraintJacobianFinalSP(argInit_real_T(), argInit_real_T(),
    argInit_real_T(), row_index_final_CJ, col_index_final_CJ);
}

static void main_Jump2DConstraintJacobianInitial()
{
  double dv10[2];
  double dv11[4];
  double constraint_jacobian_initial_nz[26];

  /* Initialize function 'Jump2DConstraintJacobianInitial' input arguments. */
  /* Initialize function input argument 'in1'. */
  /* Initialize function input argument 'in3'. */
  /* Call the entry-point 'Jump2DConstraintJacobianInitial'. */
  argInit_2x1_real_T(dv10);
  argInit_4x1_real_T(dv11);
  Jump2DConstraintJacobianInitial(dv10, argInit_real_T(), dv11, argInit_real_T(),
    argInit_real_T(), argInit_real_T(), constraint_jacobian_initial_nz);
}

static void main_Jump2DConstraintJacobianInitialSP()
{
  double row_index_initial_CJ[26];
  double col_index_initial_CJ[26];

  /* Initialize function 'Jump2DConstraintJacobianInitialSP' input arguments. */
  /* Call the entry-point 'Jump2DConstraintJacobianInitialSP'. */
  Jump2DConstraintJacobianInitialSP(argInit_real_T(), argInit_real_T(),
    argInit_real_T(), row_index_initial_CJ, col_index_initial_CJ);
}

static void main_Jump2DConstraintJacobianSP()
{
  double row_index_CJ[35];
  double col_index_CJ[35];

  /* Initialize function 'Jump2DConstraintJacobianSP' input arguments. */
  /* Call the entry-point 'Jump2DConstraintJacobianSP'. */
  Jump2DConstraintJacobianSP(argInit_real_T(), argInit_real_T(), argInit_real_T(),
    row_index_CJ, col_index_CJ);
}

static void main_Jump2DConstraints()
{
  double dv12[6];
  double dv13[4];
  double dv14[6];
  double dv15[2];
  double dv16[4];
  double constraints[10];

  /* Initialize function 'Jump2DConstraints' input arguments. */
  /* Initialize function input argument 'in1'. */
  /* Initialize function input argument 'in2'. */
  /* Initialize function input argument 'in3'. */
  /* Initialize function input argument 'in4'. */
  /* Initialize function input argument 'in6'. */
  /* Call the entry-point 'Jump2DConstraints'. */
  argInit_6x1_real_T(dv12);
  argInit_4x1_real_T(dv13);
  argInit_6x1_real_T(dv14);
  argInit_2x1_real_T(dv15);
  argInit_4x1_real_T(dv16);
  Jump2DConstraints(dv12, dv13, dv14, dv15, argInit_real_T(), dv16,
                    argInit_real_T(), argInit_real_T(), argInit_real_T(),
                    argInit_real_T(), constraints);
}

static void main_Jump2DConstraintsFinal()
{
  double dv17[6];
  double dv18[4];
  double dv19[6];
  double dv20[2];
  double dv21[4];
  double constraints_final[10];

  /* Initialize function 'Jump2DConstraintsFinal' input arguments. */
  /* Initialize function input argument 'in1'. */
  /* Initialize function input argument 'in2'. */
  /* Initialize function input argument 'in3'. */
  /* Initialize function input argument 'in4'. */
  /* Initialize function input argument 'in6'. */
  /* Call the entry-point 'Jump2DConstraintsFinal'. */
  argInit_6x1_real_T(dv17);
  argInit_4x1_real_T(dv18);
  argInit_6x1_real_T(dv19);
  argInit_2x1_real_T(dv20);
  argInit_4x1_real_T(dv21);
  Jump2DConstraintsFinal(dv17, dv18, dv19, dv20, argInit_real_T(), dv21,
    argInit_real_T(), argInit_real_T(), argInit_real_T(), argInit_real_T(),
    constraints_final);
}

static void main_Jump2DConstraintsInitial()
{
  double dv22[6];
  double dv23[4];
  double dv24[6];
  double dv25[2];
  double dv26[4];
  double constraints_initial[10];

  /* Initialize function 'Jump2DConstraintsInitial' input arguments. */
  /* Initialize function input argument 'in1'. */
  /* Initialize function input argument 'in2'. */
  /* Initialize function input argument 'in3'. */
  /* Initialize function input argument 'in4'. */
  /* Initialize function input argument 'in6'. */
  /* Call the entry-point 'Jump2DConstraintsInitial'. */
  argInit_6x1_real_T(dv22);
  argInit_4x1_real_T(dv23);
  argInit_6x1_real_T(dv24);
  argInit_2x1_real_T(dv25);
  argInit_4x1_real_T(dv26);
  Jump2DConstraintsInitial(dv22, dv23, dv24, dv25, argInit_real_T(), dv26,
    argInit_real_T(), argInit_real_T(), argInit_real_T(), argInit_real_T(),
    constraints_initial);
}

static void main_Jump2DCost()
{
  double dv27[6];
  double dv28[4];
  double dv29[6];
  double dv30[4];
  double dv31[6];
  double dv32[4];
  double J;

  /* Initialize function 'Jump2DCost' input arguments. */
  /* Initialize function input argument 'in1'. */
  /* Initialize function input argument 'in2'. */
  /* Initialize function input argument 'in3'. */
  /* Initialize function input argument 'in4'. */
  /* Initialize function input argument 'in5'. */
  /* Initialize function input argument 'in6'. */
  /* Call the entry-point 'Jump2DCost'. */
  argInit_6x1_real_T(dv27);
  argInit_4x1_real_T(dv28);
  argInit_6x1_real_T(dv29);
  argInit_4x1_real_T(dv30);
  argInit_6x1_real_T(dv31);
  argInit_4x1_real_T(dv32);
  J = Jump2DCost(dv27, dv28, dv29, dv30, dv31, dv32);
}

static void main_Jump2DCostGradient()
{
  double dv33[6];
  double dv34[4];
  double dv35[6];
  double dv36[4];
  double dv37[6];
  double dv38[4];
  double cost_gradient[10];

  /* Initialize function 'Jump2DCostGradient' input arguments. */
  /* Initialize function input argument 'in1'. */
  /* Initialize function input argument 'in2'. */
  /* Initialize function input argument 'in3'. */
  /* Initialize function input argument 'in4'. */
  /* Initialize function input argument 'in5'. */
  /* Initialize function input argument 'in6'. */
  /* Call the entry-point 'Jump2DCostGradient'. */
  argInit_6x1_real_T(dv33);
  argInit_4x1_real_T(dv34);
  argInit_6x1_real_T(dv35);
  argInit_4x1_real_T(dv36);
  argInit_6x1_real_T(dv37);
  argInit_4x1_real_T(dv38);
  Jump2DCostGradient(dv33, dv34, dv35, dv36, dv37, dv38, cost_gradient);
}

static void main_Jump2DInitialize()
{
  double dv39[6];
  double dv40[4];
  double dv41[2];
  double dv42[4];
  double decision_vars0[10];
  double inputs_ref0[4];

  /* Initialize function 'Jump2DInitialize' input arguments. */
  /* Initialize function input argument 'in1'. */
  /* Initialize function input argument 'in2'. */
  /* Initialize function input argument 'in3'. */
  /* Initialize function input argument 'in5'. */
  /* Call the entry-point 'Jump2DInitialize'. */
  argInit_6x1_real_T(dv39);
  argInit_4x1_real_T(dv40);
  argInit_2x1_real_T(dv41);
  argInit_4x1_real_T(dv42);
  Jump2DInitialize(dv39, dv40, dv41, argInit_real_T(), dv42, argInit_real_T(),
                   argInit_real_T(), argInit_real_T(), decision_vars0,
                   inputs_ref0);
}

static void main_Jump2DLagrangianHessian()
{
  double dv43[6];
  double dv44[4];
  double lagrangian_hessian_nz[10];

  /* Initialize function 'Jump2DLagrangianHessian' input arguments. */
  /* Initialize function input argument 'in1'. */
  /* Initialize function input argument 'in2'. */
  /* Call the entry-point 'Jump2DLagrangianHessian'. */
  argInit_6x1_real_T(dv43);
  argInit_4x1_real_T(dv44);
  Jump2DLagrangianHessian(dv43, dv44, argInit_real_T(), lagrangian_hessian_nz);
}

static void main_Jump2DLagrangianHessianFinal()
{
  double dv45[6];
  double dv46[4];
  double lagrangian_hessian_final_nz[10];

  /* Initialize function 'Jump2DLagrangianHessianFinal' input arguments. */
  /* Initialize function input argument 'in1'. */
  /* Initialize function input argument 'in2'. */
  /* Call the entry-point 'Jump2DLagrangianHessianFinal'. */
  argInit_6x1_real_T(dv45);
  argInit_4x1_real_T(dv46);
  Jump2DLagrangianHessianFinal(dv45, dv46, argInit_real_T(),
    lagrangian_hessian_final_nz);
}

static void main_Jump2DLagrangianHessianFinalSP()
{
  double row_index_final_H[10];
  double col_index_final_H[10];

  /* Initialize function 'Jump2DLagrangianHessianFinalSP' input arguments. */
  /* Call the entry-point 'Jump2DLagrangianHessianFinalSP'. */
  Jump2DLagrangianHessianFinalSP(argInit_real_T(), argInit_real_T(),
    row_index_final_H, col_index_final_H);
}

static void main_Jump2DLagrangianHessianInitial()
{
  double dv47[6];
  double dv48[4];
  double lagrangian_hessian_initial_nz[10];

  /* Initialize function 'Jump2DLagrangianHessianInitial' input arguments. */
  /* Initialize function input argument 'in1'. */
  /* Initialize function input argument 'in2'. */
  /* Call the entry-point 'Jump2DLagrangianHessianInitial'. */
  argInit_6x1_real_T(dv47);
  argInit_4x1_real_T(dv48);
  Jump2DLagrangianHessianInitial(dv47, dv48, argInit_real_T(),
    lagrangian_hessian_initial_nz);
}

static void main_Jump2DLagrangianHessianInitialSP()
{
  double row_index_final_H[10];
  double col_index_final_H[10];

  /* Initialize function 'Jump2DLagrangianHessianInitialSP' input arguments. */
  /* Call the entry-point 'Jump2DLagrangianHessianInitialSP'. */
  Jump2DLagrangianHessianInitialSP(argInit_real_T(), argInit_real_T(),
    row_index_final_H, col_index_final_H);
}

static void main_Jump2DLagrangianHessianSP()
{
  double row_index_H[10];
  double col_index_H[10];

  /* Initialize function 'Jump2DLagrangianHessianSP' input arguments. */
  /* Call the entry-point 'Jump2DLagrangianHessianSP'. */
  Jump2DLagrangianHessianSP(argInit_real_T(), argInit_real_T(), row_index_H,
    col_index_H);
}

int main(int, const char * const [])
{
  /* Initialize the application.
     You do not need to do this more than one time. */
  Jump2DBounds_initialize();

  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_Jump2DBounds();
  main_Jump2DConstraintJacobian();
  main_Jump2DConstraintJacobianFinal();
  main_Jump2DConstraintJacobianFinalSP();
  main_Jump2DConstraintJacobianInitial();
  main_Jump2DConstraintJacobianInitialSP();
  main_Jump2DConstraintJacobianSP();
  main_Jump2DConstraints();
  main_Jump2DConstraintsFinal();
  main_Jump2DConstraintsInitial();
  main_Jump2DCost();
  main_Jump2DCostGradient();
  main_Jump2DInitialize();
  main_Jump2DLagrangianHessian();
  main_Jump2DLagrangianHessianFinal();
  main_Jump2DLagrangianHessianFinalSP();
  main_Jump2DLagrangianHessianInitial();
  main_Jump2DLagrangianHessianInitialSP();
  main_Jump2DLagrangianHessianSP();

  /* Terminate the application.
     You do not need to do this more than one time. */
  Jump2DBounds_terminate();
  return 0;
}

/* End of code generation (main.cpp) */
