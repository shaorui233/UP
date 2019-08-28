/*============================= Two Contact Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * baland control mode.
 */

#include "FSM_State_TwoContactStand.h"
#include <Utilities/Utilities_print.h>
#include <iostream>
#include <iomanip>


/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_TwoContactStand<T>::FSM_State_TwoContactStand(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"){
  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
}

template <typename T>
void FSM_State_TwoContactStand<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Reset iteration counter
  iter = 0;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_TwoContactStand<T>::run() {
  // Count iterations
  iter++;

  // Set LQR Weights
  for (int i = 0; i < 3; i++) {
    // Manually setting weights now to avoid altering other controllers
    // x_weights[i] = this->_data->userParameters->Kp_body[i];
    // xdot_weights[i] = this->_data->userParameters->Kd_body[i];
    // R_weights[i] = this->_data->userParameters->Kp_ori[i];
    // omega_weights[i] = this->_data->userParameters->Kd_ori[i];
    x_weights[i] = 300;
    xdot_weights[i] = 50;
    R_weights[i] = 20000;
    omega_weights[i] = 150;
  }
  x_weights[2] = 100000;
  control_weight = .1;
  balanceControllerVBL.set_LQR_weights(x_weights,xdot_weights,R_weights,omega_weights,control_weight);
  refGRF.set_alpha_control(0.01);

  // Get orientation from state estimator
  for (int i = 0; i < 4; i++) {
    quat_act[i] = (double)this->_data->_stateEstimator->getResult().orientation(i);
  }

  // std::cout << std::fixed << std::setprecision(3);
  // if (iter % 2000 == 0){
  //   std::cout << "\n\nInitial yaw: " << ini_yaw << std::endl;
  //   std::cout << "State Estimator: " << quat_act[0] << ", " << quat_act[1] << ", " << quat_act[2] << ", " << quat_act[3] << ", " << std::endl;
  // }

  // Get current state from state estimator
  for (int i = 0; i < 3; i++) {
    p_act[i] = (double)this->_data->_stateEstimator->getResult().position(i);
    se_xfb[4 + i] = (double)this->_data->_stateEstimator->getResult().position(i);
    se_xfb[7 + i] = (double)this->_data->_stateEstimator->getResult().omegaBody(i);
    se_xfb[10 + i] = (double)this->_data->_stateEstimator->getResult().vBody(i);
  }

  // Convert quaternions to RPY 
  quatToEuler(quat_act, rpy_act);

  // Account for initial yaw
  if (iter < 10)
    ini_yaw = rpy_act[2];
  rpy_act[2] -= ini_yaw;

  // Convert back to quaternions w/ initial yaw accounted for
  eulerToQuat(rpy_act, quat_act);
  for (int i = 0; i < 4; i++) {
    se_xfb[i] = quat_act[i];
  }
  // if (iter % 2000 == 0)
  //   std::cout << "Modified       : " << quat_act[0] << ", " << quat_act[1] << ", " << quat_act[2] << ", " << quat_act[3] << ", " << std::endl;

  
  // Get the position of the COM in world frame & generalized gravity vector
  get_model_dynamics();

  // Use the COM pos instead of the body pos for balance control
  for (int i = 0; i < 3; i++) {
    p_body[i] = p_act[i];
    se_xfb[i+4] = p_COM[i]; 
    p_act[i] = p_COM[i];
  }

  // Get the foot locations relative to COM
  get_foot_locations();

  // Get desired state from gamepad controls
  get_desired_state();

  // Set Control Parameters
  balanceControllerVBL.set_friction(0.45);
  balanceControllerVBL.set_mass(mass_in);
  refGRF.set_mass(mass_in);
  if (this->_data->_quadruped->_robotType == RobotType::CHEETAH_3)
    balanceControllerVBL.set_inertia(0.35, 2.10, 2.10);
  else if (this->_data->_quadruped->_robotType == RobotType::MINI_CHEETAH) {
    balanceControllerVBL.set_inertia(0.025, 0.15, 0.18);
  }

  // Set minimum and maximum forces  
  if (this->_data->_quadruped->_robotType == RobotType::CHEETAH_3)
    minForce = 10;
  else if (this->_data->_quadruped->_robotType == RobotType::MINI_CHEETAH)
    minForce = 5;
  maxForce = mass_in*9.81*1.6;
  for (int leg = 0; leg < 4; leg++) {
    minForces[leg] = contactStateScheduled[leg] * minForce;
    maxForces[leg] = contactStateScheduled[leg] * maxForce;
  }

  // Compute Reference Control Inputs - only if desired xy pos has changed
  if (p_des_prev[0] != p_des[0] || p_des_prev[1] != p_des[1]) {
    refGRF.SetContactData(contactStateScheduled, minForces, maxForces);
    refGRF.updateProblemData(pFeet_des, p_des);
    refGRF.solveQP_nonThreaded(f_ref_z);
    for (int leg = 0; leg < 4; leg++)
      f_ref_world[3*leg+2] = f_ref_z[leg];
    balanceControllerVBL.set_reference_GRF(f_ref_world);
  }

  // Solve Balance control QP
  balanceControllerVBL.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
  balanceControllerVBL.SetContactData(contactStateScheduled, minForces, maxForces);
  balanceControllerVBL.updateProblemData(se_xfb, pFeet, pFeet_des, rpy, rpy_act);
  balanceControllerVBL.solveQP_nonThreaded(fOpt);
  balanceControllerVBL.publish_data_lcm();

  // Remove impedance control for all joints
  impedance_kp << 0.0, 0.0, 0.0;
  impedance_kd << 0.0, 0.0, 0.0;

  // Feed forward forces for legs in contact with the ground & PD control for legs not in contact
  qd_lift_leg << 0.0, 0.0, 0.0;
  for (int leg = 0; leg < 4; leg++) {
    if (contactStateScheduled[leg]){
      this->footFeedForwardForces.col(leg) << (T)fOpt[leg * 3], (T)fOpt[leg * 3 + 1], (T)fOpt[leg * 3 + 2];
      this->jointFeedForwardTorques.col(leg) << (T)G_ff[6+3*leg], (T)G_ff[6+3*leg+1], G_ff[6+3*leg+2];
      conPhase[leg] = 0.5;
    }

    else if (leg < 2) {
      q_lift_leg << 0, -1.52, 2.93;
      this->jointPDControl(leg, q_lift_leg, qd_lift_leg);
      conPhase[leg] = 0.0;
    }

    else {
      q_lift_leg << 0, -1.52, 2.93;
      this->jointPDControl(leg, q_lift_leg, qd_lift_leg);
      conPhase[leg] = 0.0;
    }
  
  }

  // Set the contact phase for the state estimator
  this->_data->_stateEstimator->setContactPhase(conPhase);

  // Update previous desired posiion
  for (int i = 0; i < 2; i++)
    p_des_prev[i] = p_des[i];

  // Send commands to leg controller
  for (int leg = 0; leg < 4; leg++) {
    // Impedance Control
    this->cartesianImpedanceControl(leg, this->footstepLocations.col(leg), Vec3<T>::Zero(),impedance_kp,impedance_kd);

    // Force and Joint Torque control
    this->_data->_legController->commands[leg].forceFeedForward = this->footFeedForwardForces.col(leg);
    this->_data->_legController->commands[leg].tauFeedForward = this->jointFeedForwardTorques.col(leg);
  }

}

/**
 * Uses quadruped model to find CoM position relative to body position
 * and compute generalize gravity vector
 */
template <typename T>
void FSM_State_TwoContactStand<T>::get_model_dynamics() {
  // Update state for the quadruped model using SE
  for (int i = 0; i < 3; i++){
    //state.bodyOrientation[i] = this->_data->_stateEstimator->getResult().orientation(i);
    state.bodyOrientation[i] = quat_act[i];
    state.bodyPosition[i] = this->_data->_stateEstimator->getResult().position(i);
    state.bodyVelocity[i] = this->_data->_stateEstimator->getResult().omegaBody(i);
    state.bodyVelocity[i+3] = this->_data->_stateEstimator->getResult().vBody(i);
  }
  //state.bodyOrientation[3] = this->_data->_stateEstimator->getResult().orientation(3);
  state.bodyOrientation[3] = quat_act[3];
  state.q.setZero(12);
  state.qd.setZero(12);

  for (int i = 0; i < 4; ++i) {
   state.q(3*i+0) = this->_data->_legController->datas[i].q[0];
   state.q(3*i+1) = this->_data->_legController->datas[i].q[1];
   state.q(3*i+2) = this->_data->_legController->datas[i].q[2];
   state.qd(3*i+0)= this->_data->_legController->datas[i].qd[0];
   state.qd(3*i+1)= this->_data->_legController->datas[i].qd[1];
   state.qd(3*i+2)= this->_data->_legController->datas[i].qd[2];
  }
  this->_data->_model->setState(state);
  H = this->_data->_model->massMatrix();
  G_ff = this->_data->_model->generalizedGravityForce();

  // CoM relative to body frame
  mass_in = H(3,3);
  c_body[0] = H(2,4)/mass_in;
  c_body[1] = H(0,5)/mass_in;
  c_body[2] = H(1,3)/mass_in;

  // CoM relative to world frame
  rpyToR(rBody_yaw, rpy_act);
  c_world = rBody_yaw * c_body;

  // Position of CoM in world frame
  for (int i = 0; i < 3; i++)
    p_COM[i] = p_act[i]+c_world[i]; // assumes C is expressed in world frame

}

/**
 * Gets the desired state of the robot from gamepad controls
 */
template <typename T>
void FSM_State_TwoContactStand<T>::get_desired_state() {
  // Prep state
  p_des[0] = 0.5*pFeet_world[1*3]+0.5*pFeet_world[2*3];
  p_des[1] = 0.5*pFeet_world[1*3+1]+0.5*pFeet_world[2*3+1];
  if (this->_data->_quadruped->_robotType == RobotType::CHEETAH_3)
    p_des[2] = 0.4;
  else if (this->_data->_quadruped->_robotType == RobotType::MINI_CHEETAH)
    p_des[2] = 0.225;
  for (int i = 0; i < 3; i++) {
    rpy[i] = 0.0;
    omegaDes[i] = 0.0;
    v_des[i] = 0.0;
  }

  // Lift legs after settling into prep state
  if (iter > 2000) {
    contactStateScheduled[0] = 0;
    contactStateScheduled[3] = 0;

    // To do - increase ability to change state via gamepad
    rpy[1] = this->_data->_desiredStateCommand->data.stateDes[4];
    rpy[2] = this->_data->_desiredStateCommand->data.stateDes[11];
  }

}

/**
 * Use data from leg controller and state estimator to compute positions
 * of the feet relative to the CoM
 */
template <typename T>
void FSM_State_TwoContactStand<T>::get_foot_locations() {

  for (int leg = 0; leg < 4; leg++) {
    
    // Compute vector from hip to foot (body coords)
    computeLegJacobianAndPosition(**&this->_data->_quadruped, this->_data->_legController->datas[leg].q,
                                  &this->_data->_legController->datas[leg].J, &pFeetVec, leg);
    
    // Compute vector from origin of body frame to foot (world coords)
    pFeetVecBody = rBody_yaw * (this->_data->_quadruped->getHipLocation(leg) + pFeetVec);

    // Compute vector from COM to foot (world coords)
    pFeet[leg * 3] = (double)pFeetVecBody[0]-c_world[0];
    pFeet[leg * 3 + 1] = (double)pFeetVecBody[1]-c_world[1];
    pFeet[leg * 3 + 2] = (double)pFeetVecBody[2]-c_world[2];

    // Compute vector from desired COM to foot (world coords)
    pFeet_des[leg * 3] = (double)pFeetVecBody[0]-c_world[0]+p_act[0]-p_des[0];
    pFeet_des[leg * 3 + 1] = (double)pFeetVecBody[1]-c_world[1]+p_act[1]-p_des[1];
    pFeet_des[leg * 3 + 2] = (double)pFeetVecBody[2]-c_world[2]+p_act[2]-p_des[2];

    // Locations of the feet in world coordinates
    pFeet_world[leg * 3] = (double)p_act[0]+pFeet[leg * 3];
    pFeet_world[leg * 3 + 1] = (double)p_act[1]+pFeet[leg * 3 + 1];
    pFeet_world[leg * 3 + 2] = (double)p_act[2]+pFeet[leg * 3 + 2];
  }

}

/**
 * Convert euler angles to rotation matrix
 */
template <typename T>
void FSM_State_TwoContactStand<T>::rpyToR(Mat3<float> &R, double* rpy_in) {

   Mat3<float> Rz, Ry, Rx;

   Rz.setIdentity();
   Ry.setIdentity();
   Rx.setIdentity();

   Rz(0,0) = cos(rpy_in[2]);
   Rz(0,1) = -sin(rpy_in[2]);
   Rz(1,0) = sin(rpy_in[2]);
   Rz(1,1) = cos(rpy_in[2]);

   Ry(0,0) = cos(rpy_in[1]);
   Ry(0,2) = sin(rpy_in[1]);
   Ry(2,0) = -sin(rpy_in[1]);
   Ry(2,2) = cos(rpy_in[1]);

   Rx(1,1) = cos(rpy_in[0]);
   Rx(1,2) = -sin(rpy_in[0]);
   Rx(2,1) = sin(rpy_in[0]);
   Rx(2,2) = cos(rpy_in[0]);

   R = Rz*Ry*Rx;

 }


/**
 * Convert euler angles to quaternions
 */
template <typename T>
void FSM_State_TwoContactStand<T>::eulerToQuat(double* rpy_in, double* quat_in) {

  quat_in[0] = cos(rpy_in[2]*0.5)*cos(rpy_in[1]*0.5)*cos(rpy_in[0]*0.5) + sin(rpy_in[2]*0.5)*sin(rpy_in[1]*0.5)*sin(rpy_in[0]*0.5);
  quat_in[1] = cos(rpy_in[2]*0.5)*cos(rpy_in[1]*0.5)*sin(rpy_in[0]*0.5) - sin(rpy_in[2]*0.5)*sin(rpy_in[1]*0.5)*cos(rpy_in[0]*0.5);
  quat_in[2] = sin(rpy_in[2]*0.5)*cos(rpy_in[1]*0.5)*sin(rpy_in[0]*0.5) + cos(rpy_in[2]*0.5)*sin(rpy_in[1]*0.5)*cos(rpy_in[0]*0.5);
  quat_in[3] = sin(rpy_in[2]*0.5)*cos(rpy_in[1]*0.5)*cos(rpy_in[0]*0.5) - cos(rpy_in[2]*0.5)*sin(rpy_in[1]*0.5)*sin(rpy_in[0]*0.5);

}

/**
 * Convert quaternions to euler angles
 */
template <typename T>
void FSM_State_TwoContactStand<T>::quatToEuler(double* quat_in, double* rpy_in) {
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (quat_in[0] * quat_in[1] + quat_in[2] * quat_in[3]);
  double cosr_cosp = +1.0 - 2.0 * (quat_in[1] * quat_in[1] + quat_in[2] * quat_in[2]);
  rpy_in[0] = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (quat_in[0] * quat_in[2] - quat_in[3] * quat_in[1]);
  if (fabs(sinp) >= 1)
    rpy_in[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    rpy_in[1] = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (quat_in[0] * quat_in[3] + quat_in[1] * quat_in[2]);
  double cosy_cosp = +1.0 - 2.0 * (quat_in[2] * quat_in[2] + quat_in[3] * quat_in[3]);  
  rpy_in[2] = atan2(siny_cosp, cosy_cosp);
}


/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_TwoContactStand<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_TWO_CONTACT_STAND:
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;

    case K_BOUNDING:
      this->nextStateName = FSM_StateName::BOUNDING;
      break;


    case K_PASSIVE:  // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    case K_BALANCE_STAND: 
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_TWO_CONTACT_STAND << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_TwoContactStand<T>::transition() {
  // Finish Transition
  switch (this->nextStateName) {
    case FSM_StateName::PASSIVE:  // normal
      this->transitionData.done = true;
      break;

    case FSM_StateName::BALANCE_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::LOCOMOTION:
      this->transitionData.done = true;
      break;

    case FSM_StateName::BOUNDING:
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_TwoContactStand<T>::onExit() {
  // Nothing to clean up when exiting
}



// template class FSM_State_TwoContactStand<double>;
template class FSM_State_TwoContactStand<float>;
