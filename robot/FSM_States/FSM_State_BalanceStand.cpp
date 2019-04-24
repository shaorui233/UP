/*=========================== Balance Stand ===========================*/
/**
 * FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */

#include "FSM_State_BalanceStand.h"


/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_BalanceStand<T>::FSM_State_BalanceStand(ControlFSMData<T>* _controlFSMData):
  FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_STAND, "BALANCE_STAND") {
  // Initialize GRF to 0s
  this->groundReactionForces = Mat34<T>::Zero();
}


template <typename T>
void FSM_State_BalanceStand<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;
  this->transitionDuration = 0.0;
  this->_data->_gaitScheduler->gaitData._nextGait = GaitType::STAND;
}


/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_BalanceStand<T>::run() {
  // Do nothing, all commands should begin as zeros
  BalanceStandStep();
}


/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumarated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_BalanceStand<T>::checkTransition() {
  // Get the next state
  iter++;


  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
  case K_BALANCE_STAND:
    // Normal operation for state based transitions

    // Need a working state estimator for this
    /*if (velocity > v_max) {
      // Notify the State of the upcoming next state
      this->nextStateName = FSM_StateName::LOCOMOTION;

      // Transition instantaneously to locomotion state on request
      this->transitionDuration = 0.0;

      // Set the next gait in the scheduler to
      this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;

    }*/

    // in place to show automatic non user requested transitions
    if (iter >= 2058) {
      this->nextStateName = FSM_StateName::LOCOMOTION;
      this->_data->controlParameters->control_mode = K_LOCOMOTION;
      this->transitionDuration = 0.0;
      this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT; // Or get whatever is in main_control_settings
      iter = 0;
    }
    break;

  case K_LOCOMOTION:
    // Requested change to balance stand
    this->nextStateName = FSM_StateName::LOCOMOTION;

    // Transition instantaneously to locomotion state on request
    this->transitionDuration = 0.0;

    // Set the next gait in the scheduler to
    this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;
    break;

  default:
    std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << 0 << " to " << this->_data->controlParameters->control_mode << std::endl;

  }

  // Return the next state name to the FSM
  return this->nextStateName;

}


/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
bool FSM_State_BalanceStand<T>::transition() {

  if (this->nextStateName == FSM_StateName::LOCOMOTION) {

    BalanceStandStep();

    iter++;
    if (iter >= this->transitionDuration * 1000) {
      return true;
    } else {
      return false;
    }
  }


  return true;
}



/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_BalanceStand<T>::onExit() {
  iter = 0;
}


/**
 * Calculate the commands for the leg controllers for each of the feet.
 */
template <typename T>
void FSM_State_BalanceStand<T>::BalanceStandStep() {
  //StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

  // Run the balancing controllers to get GRF and next step locations
  //runControls();

  // Reset the forces and steps to 0
  this->groundReactionForces = Mat34<T>::Zero();

  // Test to make sure we can control the robot
  for (int leg = 0; leg < 4; leg++) {
    this->groundReactionForces.col(leg) << 0.0, 0.0, -110.36;
    //groundReactionForces.col(leg) = stateEstimate.rBody * groundReactionForces.col(leg);

    this->footstepLocations.col(leg) << 0.0, 0.0, -0.65;
  }
  Vec3<T> vDes;
  vDes << 0, 0, 0;

  //std::cout << groundReactionForces << std::endl;

  // All legs are force commanded to be on the ground
  for (int leg = 0; leg < 4; leg++) {

    this->cartesianImpedanceControl(leg, this->footstepLocations.col(leg), vDes);


    this->_data->_legController->commands[leg].forceFeedForward = this->groundReactionForces.col(leg);

    // Singularity barrier calculation (maybe an overall safety checks function?)
    // TODO
  }

}

//template class FSM_State_BalanceStand<double>;
template class FSM_State_BalanceStand<float>;