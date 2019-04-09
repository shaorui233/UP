/*============================ Control FSM ============================*/
/*
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

#include "ControlFSM.h"

template <typename T>
ControlFSM<T>::ControlFSM(StateEstimatorContainer<T>* _stateEstimator,
                          LegController<T>* _legController,
                          GaitScheduler<T>* _gaitScheduler,
                          DesiredStateCommand<T>* _desiredStateCommand) {
  data._stateEstimator = _stateEstimator;
  data._legController = _legController;
  data._gaitScheduler = _gaitScheduler;
  data._desiredStateCommand = _desiredStateCommand;

  // Initialize the FSM with an FSM State
  initialize();
}

template <typename T>
void ControlFSM<T>::initialize() {
  // Initialize a new FSM State with the control data
  currentState = new FSM_State_BalanceStand<T>(&data);
}


/*
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template <typename T>
void ControlFSM<T>::runFSM() {


  /*nextState = currentState->checkTransition();
  if (nextState->stateName != currentState->stateName) {
    // Transitioning
    operatingMode = FSM_OperatingMode::TRANSITIONING;
  }
  if (operatingMode == FSM_OperatingMode::NORMAL) {
    */
  // Run the iteration for the current state
  currentState->run();
  //}


}


//template class ControlFSM<double>; This should be fixed... need to make RobotController a template
template class ControlFSM<float>;