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

  invalid = nullptr;
  passive = new FSM_State_Passive<T>(&data);
  jointPD = new FSM_State_JointPD<T>(&data);
  impedanceControl = new FSM_State_ImpedanceControl<T>(&data);
  balanceStand = new FSM_State_BalanceStand<T>(&data);
  locomotion = new FSM_State_Locomotion<T>(&data);

  // Initialize the FSM with an FSM State
  initialize();
}

template <typename T>
void ControlFSM<T>::initialize() {

  // Initialize a new FSM State with the control data
  currentState = balanceStand;

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

  if (operatingMode != FSM_OperatingMode::ESTOP) {

    //
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

  }


  currentState->run();
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
    return invalid;

  case FSM_StateName::PASSIVE :
    return passive;

  case FSM_StateName::JOINT_PD :
    return jointPD;

  case FSM_StateName::IMPEDANCE_CONTROL :
    return impedanceControl;

  case FSM_StateName::BALANCE_STAND :
    return balanceStand;

  case FSM_StateName::LOCOMOTION :
    return locomotion;

  default:
    return invalid;
  }

}


//template class ControlFSM<double>; This should be fixed... need to make RobotController a template
template class ControlFSM<float>;