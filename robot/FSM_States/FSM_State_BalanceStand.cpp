#include "FSM_State_BalanceStand.h"

template <typename T>
FSM_State_BalanceStand<T>::FSM_State_BalanceStand(ControlFSMData<T>* _controlFSMData):
  FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_STAND) {
  // Initialize GRF to 0s
  groundReactionForces = Mat34<T>::Zero();
}


template <typename T>
void FSM_State_BalanceStand<T>::onEnter() {
  // Nothing to initialize
}


template <typename T>
void FSM_State_BalanceStand<T>::run() {
  // Do nothing, all commands should begin as zeros
  this->_data->_gaitScheduler->printGaitInfo();
  this->_data->_desiredStateCommand->printStateCommandInfo();
  BalanceStandStep();
}

template <typename T>
FSM_State<T>* FSM_State_BalanceStand<T>::getNextState() {
  // Get the next state
  return this;
}


template <typename T>
void FSM_State_BalanceStand<T>::onExit() {
  // Nothing to clean up when exiting
}

/*
 * Calculate the commands for the leg controllers for each of the feet.
 */
template <typename T>
void FSM_State_BalanceStand<T>::BalanceStandStep() {
  //StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

  // Contact state logic
  //estimateContact();

  // Run the balancing controllers to get GRF and next step locations
  //runControls();

  // Reset the forces and steps to 0
  groundReactionForces = Mat34<T>::Zero();

  // Test to make sure we can control the robot
  for (int leg = 0; leg < 4; leg++) {
    groundReactionForces.col(leg) << 0.0, 0.0, -110.36;
    //groundReactionForces.col(leg) = stateEstimate.rBody * groundReactionForces.col(leg);
  }

  //std::cout << groundReactionForces << std::endl;

  // Calculate appropriate control actions for each leg to be sent out
  for (int leg = 0; leg < 4; leg++) {
    this->_data->_legController->commands[leg].forceFeedForward = groundReactionForces.col(leg);

    // Singularity barrier calculation (maybe an overall safety checks function?)
    // TODO
  }

}

//template class FSM_State_BalanceStand<double>;
template class FSM_State_BalanceStand<float>;