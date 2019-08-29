#include "JumpNLP.hpp"
#include <iostream>

#ifdef IPOPT_OPTION

#include <cassert>
using namespace Ipopt;

/**
 * Create bounds for the decision variables and the constraints.
 * Note that the first timestep states are constrained to the robot's
 * state at the beginning of the optimization.
 */
template <typename T>
void JumpNLP<T>::Bounds(double* X_ub, double* X_lb, double* constraints_ub, double* constraints_lb) {

  // Create the bounds for the prediction horizon
  for (int k = 0; k < NUM_PREDICTIONS; k++) {
    i_xd = k * NUM_STATES; // eventually get an input trajectory
    i_X = k * NUM_DECISION_VARS;
    i_c = k * NUM_CONSTRAINTS;
    i_f = k * NUM_FEET;

    // First step bounds have to be current state
    if (k == 0) {
      Jump2DBounds(
          current_state, current_state, inputs_max, inputs_min, contact_state_pred + i_f,
          X_lb, X_ub, constraints_ub, constraints_lb);

    } else {
      Jump2DBounds(
          states_max, states_min, inputs_max, inputs_min, contact_state_pred + i_f,
          X_lb + i_X, X_ub + i_X, constraints_ub + i_c, constraints_lb + i_c);
    }
  }
}


/**
 * Sets the seeded state trajectory, x_seed, and the seeded input reference
 * policy, u_seed, by calculating the reference policy, (r_ref, f_ref), for
 * all of the feet throughout the prediction horizon trajectory.
 *
 * TODO:
 *  - Add turning feed forward
 */
template <typename T>
void JumpNLP<T>::InitializeMPC(double* X_0) {

  // Set up a temporary vector to hold the current output
  double x_0[NUM_DECISION_VARS] = {0};



  // Copy the current state into the temporary variable
  for (int i = 0; i < NUM_STATES; i++) {
    x_0[i] = current_state[i];
  }


  //cout << contact_state_pred[0] << " " << " " << touchdown_pred[0] << " "  << T_stance[0] << " " << x_d[6] << endl;
  // Create the initial guess and policy seeding
  for (int k = 0; k < NUM_PREDICTIONS; k++) {

    // Start indices
    i_x = k * NUM_DECISION_VARS;
    i_x1 = (k + 1) * NUM_DECISION_VARS;
    i_u = i_x + NUM_STATES;
    i_xd = k * NUM_STATES; // eventually get an input trajectory
    i_uref = k * NUM_INPUTS;
    i_f = k * NUM_FEET;

    /*for (int i = 0; i < NUM_FEET; i++) {

      cout << gravity << " " << T_stance[0] <<" " << touchdown_pred[i_f+i]<<" " <<p_foot_0[i*3+0] << " " << p_foot_0[i*3+1] << " " << p_foot_0[i*3+2] << endl;;
      }*/
    // Set the current decision variables for the states
    for (int i = 0; i < NUM_STATES; i++) {
      X_0[i_x + i] = x_0[i];
    }

    // Mark the location at the beginning of touchdown
    for (int foot = 0; foot < NUM_FEET; foot++) {
      if (touchdown_pred[i_f + foot] == 1 || k == 0) {
        for (int i = 0; i < 2; i++) {
          x_touchdown[foot * 2 + i] = x_0[i]; // CHECK THIS... Prob in testing
        }
      }
    }

    // Iterative initialization function
    //Jump2DInitialize();
    /*for (int i = 0; i < NUM_FEET; i++) {
      cout << p_foot[i*3+0] << " " << p_foot_0[i*3+1] << " " << p_foot_0[i*3+2];
      cout << endl;
      }*/
    /*
       for (int i = 0; i < NUM_FEET; i++) {

       cout << gravity << " " << T_stance[0] <<" " << touchdown_pred[i_f+i]<<" " <<p_foot_0[i*3+0] << " " << p_foot_0[i*3+1] << " " << p_foot_0[i*3+2] << endl;;
       }
       cout<< endl;*/

    // Set the current decision variables for the inputs
    for (int i = 0; i < NUM_INPUTS; i++) {
      X_0[i_u + i] = x_0[NUM_STATES + i];
    }
  }
  /*
     cout << "initializing" << endl;
     for (int ind = 0; ind < NUM_INPUTS; ind++){
     for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++){
     cout << std::right << std::setw(13) << u_ref[ind + ind2*NUM_INPUTS];
     }
     cout << "" << endl;
     }
     cout << "\nX0" << endl;
     for (int ind = 0; ind < NUM_DECISION_VARS; ind++){
     for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++){
     cout << std::right << std::setw(13) << X_0[ind + ind2*NUM_DECISION_VARS];
     }
     cout << "" << endl;
     }
     */
  /*
     cout << "\nTD" << endl;
     for (int ind = 0; ind < NUM_FEET; ind++){
     for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++){
     cout << std::right << std::setw(13) << touchdown_pred[ind + ind2*NUM_FEET];
     }
     cout << "" << endl;
     }
     cout << "\n" << endl;*/
}

/**
 * Predicted cost over the given gait-generalized prediction horizon.
 * Calls the MPCCost function that is generated first symbolically in
 * MATLAB to produce an optimized MATLAB function, then a function is
 * created using MATLAB's codegen. This method is essentially a wrapper
 * that takes the problem data and uses this iterative cost function with
 * the known gait logic.
 *
 *
 *  J(X) = x_error'*Q*x_error + u_error'*R*u_error
 *
 *      x_error = x_d - x
 *      u_error = u_ref - u
 */
template<typename T>
double JumpNLP<T>::PredictedCost(const double * X) {

  // Initialize the predicted cost
  double cost = 0;

  // Incremental cost over the predicted gait horizon
  for (int k = 0; k < NUM_PREDICTIONS; k++) {

    // Start indices
    i_x = k * NUM_DECISION_VARS;
    i_u = i_x + NUM_STATES;
    i_xd = k * NUM_STATES; // eventually get an input trajectory
    i_uref = k * NUM_INPUTS;
    i_f = k * NUM_FEET;
    i_Q = k * NUM_STATES;
    i_R = k * NUM_INPUTS;


    //cout << x_d[2] << endl;

    // Calculate the incremental cost
    cost = cost + Jump2DCost(X + i_x, X + i_u, x_d + i_xd, u_ref + i_uref, Q + i_Q, R + i_R);
  }
  /*
     for (int ind = 0; ind < NUM_DECISION_VARS; ind++){
     for (int ind2 = 0; ind2 < NUM_PREDICTIONS; ind2++){
     cout << std::right << std::setw(13) << X[ind + ind2*NUM_DECISION_VARS];
     }
     cout << "" << endl;
     }*/

  //std::usleep(10000000);

  // Return the cost over the prediction horizon
  return cost;
}


/**
 * Predicted cost gradient over the given gait-generalized prediction horizon.
 */
template<typename T>
void JumpNLP<T>::PredictedCostGradient(const double * X, double * gradient) {

  // Incremental gradient over the predicted gait horizon
  for (int k = 0; k < NUM_PREDICTIONS; k++) {
    // Start indices
    i_x = k * NUM_DECISION_VARS;
    i_u = i_x + NUM_STATES;
    i_xd = k * NUM_STATES;
    i_uref = k * NUM_INPUTS;
    i_g = k * NUM_DECISION_VARS;
    i_f = k * NUM_FEET;
    i_Q = k * NUM_STATES;
    i_R = k * NUM_INPUTS;

    // Calculate the iterative gradient of the cost w.r.t. the decision variables
    Jump2DCostGradient(X + i_x, X + i_u, x_d + i_xd, u_ref + i_uref,
        Q + i_Q, R + i_R,
        gradient + i_g);

  }
}


/**
 * Predicted constrints over the given gait-generalized prediction horizon.
 */
template<typename T>
void JumpNLP<T>::Constraints(const double * X, double * constraints) {
  //cout << "NEW ITER" << endl;
  // Run the iterative constraints
  for (int k = 0; k < NUM_PREDICTIONS; k++) {

    // Start indices
    i_x = k * NUM_DECISION_VARS;
    i_u = i_x + NUM_STATES;
    i_f = k * NUM_FEET;
    i_c = k * NUM_CONSTRAINTS;
    i_f1 = (k + 1) * NUM_FEET;
    if (k == (NUM_PREDICTIONS - 1)) {
      // Evaluate the constraints at the final timestep
      Jump2DConstraintsFinal(X + i_x, X + i_u, current_state, 
          contact_state_pred + i_f, dt_pred[k], p_foot_0, mass, I_yy, gravity, mu,
          constraints + i_c);

    } else {
      i_x1 = (k + 1) * NUM_DECISION_VARS;
      i_u1 = i_x1 + NUM_STATES;

      // Evaluate the constraints at the timestep
      Jump2DConstraints(X + i_x, X + i_u, X + i_x1, contact_state_pred+i_f, 
          dt_pred[k], p_foot_0, mass, I_yy, gravity, mu, 
          constraints + i_c);
    }
    /*cout << X[i_x+2];
      for (int i = 0; i < NUM_FEET; i++) {
      cout << " ground: " << z_g[i] << " r's: " << X[i_u+2 + i*6] << " " ;
      }
      cout << endl;*/
    /*
       for (int i = 0; i < NUM_CONSTRAINTS; i++) {
       cout << constraints[i] << " " ;
       }
       cout << "\n\n" << endl;
       */
  }
}


/**
 * Predicted lagrangian hessian over the given gait-generalized prediction horizon.
 */
template<typename T>
void JumpNLP<T>::LagrangianHessian(const double * X, double * hessian, Number obj_factor, const Number * lambda) {

  (void)X;
  (void)lambda;

  // Incremental cost over the predicted gait horizon
  for (int k = 0; k < NUM_PREDICTIONS; k++) {

    // Start indices
    i_x = k * NUM_DECISION_VARS;
    i_u = i_x + NUM_STATES;
    i_xd = k * NUM_STATES;
    i_c = k * NUM_CONSTRAINTS;
    i_h = k * NNZ_H;
    i_f = k * NUM_FEET;
    i_Q = k * NUM_STATES;
    i_R = k * NUM_INPUTS;

    if (k == (NUM_PREDICTIONS - 1)) {
      // Calculate the iterative Hessian of the Lagrangian at the lfinal time
      Jump2DLagrangianHessianFinal(Q + i_Q, R + i_R, obj_factor, hessian + i_h);
    } else {
      // Calculate the iterative Hessian of the Lagrangian over the prediction horizon
      Jump2DLagrangianHessian(Q + i_Q, R + i_R, obj_factor, hessian + i_h);
    }
  }
}


template class JumpNLP<double>;
template class JumpNLP<float>;
#endif
