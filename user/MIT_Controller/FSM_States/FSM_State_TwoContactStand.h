#ifndef FSM_STATE_TWO_CONTACT_STAND_H
#define FSM_STATE_TWO_CONTACT_STAND_H

#include "FSM_State.h"
#include <Controllers/BalanceController/BalanceControllerVBL.hpp>
#include <Controllers/BalanceController/ReferenceGRF.hpp>

/**
 *
 */
template <typename T>
class FSM_State_TwoContactStand : public FSM_State<T> {
 public:
  FSM_State_TwoContactStand(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

  TransitionData<T> testTransition();

 private:
  //
  void get_desired_state();
  void get_model_dynamics();
  void get_foot_locations();
  void rpyToR(Mat3<float> &R, double* rpy_in);
  void eulerToQuat(double* rpy_in, double* quat_in);
  void quatToEuler(double* quat_in, double* rpy_in);

  // Keep track of the control iterations
  int iter = 0;

  // Higher Level Robot Body Controllers
  BalanceControllerVBL balanceControllerVBL; 
  ReferenceGRF refGRF;

  // LQR Weights
  double x_weights[3], xdot_weights[3], R_weights[3], omega_weights[3];
  double control_weight;

  // Actual state of the body using estimator or cheater state
  double pFeet[12], p_act[3], v_act[3], quat_act[4], rpy_act[3], O_err[3], se_xfb[13], pFeet_world[12], p_body[3];

  // Quadruped Model
  FloatingBaseModel<T> model;
  FBModelState<T> state;
  double Ig_in[3], mass_in, p_COM[3];
  Vec3<T> c_body, c_world;
  Mat18<float> H;
  DVec<T> G_ff;

  // Feet relative to COM
  Vec3<T> pFeetVec;
  Vec3<T> pFeetVecBody;

  // Desired state of the body
  double pFeet_des[12], p_des[3], v_des[3], rpy[3], omegaDes[3], pweight, convert = 3.14159/180;

  // Joint positions for legs not in contact
  Vec3<T> q_lift_leg, qd_lift_leg;
  int lift_iteration, ramp_iteration;

  // Contact Data
  double minForce, maxForce, mu_ctrl = 0.4;
  double minForces[4], maxForces[4];
  double contactStateScheduled[4] = {1, 1, 1, 1};
  Vec4<T> conPhase;

  // Control Input
  double f_ref_z[4], f_ref_world[12], fOpt[12];

  // Leg Impedance Control
  Vec3<double> impedance_kp;
  Vec3<double> impedance_kd;

  // Check to see if desired position has changed
  double p_des_prev[2] = {99.0,99.0};

  // Account for non-zero initial yaw
  double ini_yaw;
  Mat3<float> rBody_yaw; // rBody adjusted for initial yaw

  // Compare expected vs actual toqrues
  double tauOpt[12];
  Mat3<T> Jleg;



};

#endif  // FSM_STATE_TWO_CONTACT_STAND_H
