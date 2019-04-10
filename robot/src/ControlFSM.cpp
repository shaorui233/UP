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

  // Add all of the FSM States to the state list
  statesList.invalid = nullptr;
  statesList.passive = new FSM_State_Passive<T>(&data);
  statesList.jointPD = new FSM_State_JointPD<T>(&data);
  statesList.impedanceControl = new FSM_State_ImpedanceControl<T>(&data);
  statesList.balanceStand = new FSM_State_BalanceStand<T>(&data);
  statesList.locomotion = new FSM_State_Locomotion<T>(&data);

  // Initialize the FSM with an FSM State
  initialize();
}

template <typename T>
void ControlFSM<T>::initialize() {

  // Initialize a new FSM State with the control data
  currentState = statesList.balanceStand;

  // Enter the new current state cleanly
  currentState->onEnter();

  // Initialize to not be in transition
  nextState = currentState;

  // Initialize FSM mode to normal operation
  operatingMode = FSM_OperatingMode::NORMAL;
}


/*
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template <typename T>
void ControlFSM<T>::runFSM() {
  // Check the robot state for safe operation
  operatingMode = safetyCheck();

  // Run the robot control code if operating mode is not unsafe
  if (operatingMode != FSM_OperatingMode::ESTOP) {

    // Run normal controls if no transition is detected
    if (operatingMode == FSM_OperatingMode::NORMAL) {
      // Check the current state for any transition
      nextStateName = currentState->checkTransition();

      // Detect a requested transition
      if (nextStateName != currentState->stateName) {

        // Set the FSM operating mode to transitioning
        operatingMode = FSM_OperatingMode::TRANSITIONING;

        // Get the next FSM State
        nextState = getNextState(nextStateName);

        std::cout << "[CONTROL FSM] Transition initialized from " << currentState->stateString << " to " << nextState->stateString << std::endl;

      } else {
      	
        // Run the iteration for the current state normally
        currentState->run();

      }

    }

    // Run the transition code while transition is occuring
    if (operatingMode == FSM_OperatingMode::TRANSITIONING) {
      // Run the state transition
      if (currentState->transition()) {
        std::cout << "[CONTROL FSM] Transition finalizing from " << currentState->stateString << " to " << nextState->stateString << std::endl;

        // Exit the current state cleanly
        currentState->onExit();

        // Complete the transition
        currentState = nextState;

        // Enter the new current state cleanly
        currentState->onEnter();

        // Return the FSM to normal operation mode
        operatingMode = FSM_OperatingMode::NORMAL;

      }
    }

  } else {
    currentState = statesList.passive;
    currentState->onEnter();
    nextStateName = currentState->stateName;
  }

}


/*
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyCheck() {
  // Default is to return the current operating mode
  return operatingMode;

}


/*
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template <typename T>
FSM_State<T>* ControlFSM<T>::getNextState(FSM_StateName stateName) {
  switch (stateName) {

  case FSM_StateName::INVALID :
    return statesList.invalid;

  case FSM_StateName::PASSIVE :
    return statesList.passive;

  case FSM_StateName::JOINT_PD :
    return statesList.jointPD;

  case FSM_StateName::IMPEDANCE_CONTROL :
    return statesList.impedanceControl;

  case FSM_StateName::BALANCE_STAND :
    return statesList.balanceStand;

  case FSM_StateName::LOCOMOTION :
    return statesList.locomotion;

  default:
    return statesList.invalid;
  }

}


//template class ControlFSM<double>; This should be fixed... need to make RobotController a template
template class ControlFSM<float>;