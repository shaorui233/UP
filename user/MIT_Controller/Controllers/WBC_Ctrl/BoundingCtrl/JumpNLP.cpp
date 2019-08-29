#include "JumpNLP.hpp"
#include <iostream>

#ifdef IPOPT_OPTION

#include <cassert>

using namespace Ipopt;

template<typename T>
JumpNLP<T>::JumpNLP(){}


template <typename T>
bool JumpNLP<T>::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
    Index& nnz_h_lag, IndexStyleEnum& index_style){
  // Number of total decision variables for the problem
  n = NUM_DECISION_VARS * NUM_PREDICTIONS;

  // Number of total constraints for the problem
  m = NUM_CONSTRAINTS;

  // Number of nonzeros in the constraint Jacobian
  nnz_jac_g = NNZ_J_G * (NUM_PREDICTIONS - 1) + NNZ_J_G_F; // Last step only dependent on current so dynamics goes away

  // Number of nonzeros in the Hessian matrix (number calculated from MATLAB)
  nnz_h_lag = NNZ_H * (NUM_PREDICTIONS - 1) + NNZ_H_F; // Currently only for cost hessian, need constraints
  // Will definitely change possibly... It didnt change! Maybe...

  // Index style is 0-based
  index_style = C_STYLE;

  return true;
}

/** Method to return the bounds for my problem */
template <typename T>
bool JumpNLP<T>::get_bounds_info(Index n, Number* x_l, Number* x_u,
    Index m, Number* g_l, Number* g_u){
  assert(n == n_opt);
  assert(m == n_constr);

  Bounds(x_u, x_l, g_u, g_l);
  return true;
}

/** Method to return the starting point for the algorithm */
template <typename T>
bool JumpNLP<T>::get_starting_point(Index n, bool init_x, Number* x,
    bool init_z, Number* z_L, Number* z_U,
    Index m, bool init_lambda,
    Number* lambda){
  (void)n;
  (void)z_L;
  (void)z_U;
  (void)m;
  (void)lambda;

  // Here, we assume we only have starting values for x, if you code
  // your own NLP, you can provide starting values for the others if
  // you wish.
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);

  InitializeMPC(x);

  return true;
}

/** Method to return the objective value */
template <typename T>
bool JumpNLP<T>::eval_f(Index n, const Number* x, bool new_x, Number& obj_value){
  //bool JumpNLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value){
  (void)n;
  (void)new_x;

  obj_value = PredictedCost(x);

  return true;
}


/** Method to return the gradient of the objective */
template <typename T>
bool JumpNLP<T>::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f){
  (void)n;
  (void)new_x;
  PredictedCostGradient(x, grad_f);
  return true;
}

/** Method to return the constraint residuals */
template <typename T>
bool JumpNLP<T>::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g){
  (void)n;
  (void)new_x;
  (void)m;

  Constraints(x, g);

  return true;
}

/** Method to return:
 *   1) The structure of the jacobian (if "values" is NULL)
 *   2) The values of the jacobian (if "values" is not NULL)
 */
template <typename T>
bool JumpNLP<T>::eval_jac_g(Index n, const Number* x, bool new_x,
    Index m, Index nele_jac, Index* iRow, Index *jCol,
    Number* values){
  (void)n;
  (void)x;
  (void)new_x;
  (void)m;
  (void)nele_jac;

  if (values == NULL) {
    //cout << "[PRMPC] Constraint Jacobian Setup Start" << endl;

    for (int i = 0; i < NUM_PREDICTIONS; i++) {
      i_jc = i * NNZ_J_G;
      if (i < (NUM_PREDICTIONS - 1)) {
        Jump2DConstraintJacobianSP(
            i, NUM_DECISION_VARS, NUM_CONSTRAINTS, iRow + i_jc, jCol + i_jc);
      } else {
        Jump2DConstraintJacobianFinalSP(
            i, NUM_DECISION_VARS, NUM_CONSTRAINTS, iRow + i_jc, jCol + i_jc);
      }
    }
    //cout << "[PRMPC] Constraint Jacobian Setup End" << endl;

  } else {
    //cout << "[PRMPC] Constraint Jacobian Calculation Start" << endl;

    // Calculate the constraint jacobian
    ConstraintJacobian(x, values);
    //cout << "[PRMPC] Constraint Jacobian Calculation End" << endl;

  }


  return true;
}

/** Method to return:
 *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
 *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
 */
template <typename T>
bool JumpNLP<T>::eval_h(Index n, const Number* x, bool new_x,
    Number obj_factor, Index m, const Number* lambda,
    bool new_lambda, Index nele_hess, Index* iRow,
    Index* jCol, Number* values){
  (void)n;
  (void)x;
  (void)new_x;
  (void)m;
  (void)new_lambda;
  (void)nele_hess;

  if (values == NULL) {
    //cout << "[PRMPC] Hessian Setup Start" << endl;

    for (int i = 0; i < NUM_PREDICTIONS; i++) {
      i_h = i * NNZ_H;

      if (i < (NUM_PREDICTIONS - 1)) {
        Jump2DLagrangianHessianSP(i, NUM_DECISION_VARS, iRow + i_h, jCol + i_h);
      } else {
        Jump2DLagrangianHessianFinalSP(i, NUM_DECISION_VARS, iRow + i_h, jCol + i_h);
      }
    }
    //cout << "[PRMPC] Hessian Setup End" << endl;

  } else {
    //cout << "[PRMPC] Hessian Calculation Start" << endl;

    // Calculate the Hessian
    LagrangianHessian(x, values, obj_factor, lambda);
    //cout << "[PRMPC] Hessian Calculation End" << endl;

  }

  return true;
}

/** @name Solution Methods */
/** This method is called when the algorithm is complete so the TNLP can store/write the solution */
template <typename T>
void JumpNLP<T>::finalize_solution(SolverReturn status,
    //void JumpNLP::finalize_solution(SolverReturn status,
    Index n, const Number* x, const Number* z_L, const Number* z_U,
    Index m, const Number* g, const Number* lambda,
    Number obj_value, const IpoptData* ip_data,
    IpoptCalculatedQuantities* ip_cq){

    (void)status;
    (void)n;
    (void)x;
    (void)z_L;
    (void)z_U;
    (void)m;
    (void)g;
    (void)lambda;
    (void)obj_value;
    (void)ip_data;
    (void)ip_cq;

    printf("solution: %f, %f\n", x[0], x[1]);
    printf("solver return: %d\n", status);
}

template class JumpNLP<double>;
template class JumpNLP<float>;
#endif
