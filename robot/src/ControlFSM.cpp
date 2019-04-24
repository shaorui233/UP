/*============================ Control FSM ============================*/
/**
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

#include "ControlFSM.h"


/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 *
 * @param _quadruped the quadruped information
 * @param _stateEstimator contains the estimated states
 * @param _legController interface to the leg controllers
 * @param _gaitScheduler controls scheduled foot contact modes
 * @param _desiredStateCommand gets the desired COM state trajectories
 * @param controlParameters passes in the control parameters from the GUI
 */
template <typename T>
ControlFSM<T>::ControlFSM(Quadruped<T>* _quadruped,
                          StateEstimatorContainer<T>* _stateEstimator,
                          LegController<T>* _legController,
                          GaitScheduler<T>* _gaitScheduler,
                          DesiredStateCommand<T>* _desiredStateCommand,
                          RobotControlParameters* controlParameters) {
  // Add the pointers to the ControlFSMData struct
  data._quadruped = _quadruped;
  data._stateEstimator = _stateEstimator;
  data._legController = _legController;
  data._gaitScheduler = _gaitScheduler;
  data._desiredStateCommand = _desiredStateCommand;
  data.controlParameters = controlParameters;

  // Initialize and add all of the FSM States to the state list
  statesList.invalid = nullptr;
  statesList.passive = new FSM_State_Passive<T>(&data);
  statesList.jointPD = new FSM_State_JointPD<T>(&data);
  statesList.impedanceControl = new FSM_State_ImpedanceControl<T>(&data);
  statesList.balanceStand = new FSM_State_BalanceStand<T>(&data);
  statesList.locomotion = new FSM_State_Locomotion<T>(&data);

  // Initialize the FSM with the Passive FSM State
  initialize();

}


/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state and Normal operation mode.
 */
template <typename T>
void ControlFSM<T>::initialize() {
  // Initialize a new FSM State with the control data
  currentState = statesList.passive;

  // Enter the new current state cleanly
  currentState->onEnter();

  // Initialize to not be in transition
  nextState = currentState;

  // Initialize FSM mode to normal operation
  operatingMode = FSM_OperatingMode::NORMAL;

}


/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template <typename T>
void ControlFSM<T>::runFSM() {
  // Check the robot state for safe operation
  operatingMode = safetyPreCheck();

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

        // Get the next FSM State by name
        nextState = getNextState(nextStateName);

        // Print transition initialized info
        printInfo(1);

      } else {
        // Run the iteration for the current state normally
        currentState->run();

      }

    }

    // Run the transition code while transition is occuring
    if (operatingMode == FSM_OperatingMode::TRANSITIONING) {
      // transitionData = currentState->transition();

      // Run the state transition
      if (currentState->transition()) { // (transitionData.transitionComplete) {

        // Exit the current state cleanly
        currentState->onExit();

        // Print finalizing transition info
        printInfo(2);

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

  // Check the robot state for safe operation
  operatingMode = safetyPostCheck();

  // Print the current state of the FSM
  printInfo(0);

}


/**
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 *
 * @return the appropriate operating mode
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPreCheck() {

  // Check for safe orientation if the current state requires it
  if (currentState->checkSafeOrientation) {
    /*if (roll >= 1.3 || pitch >= 1.3) {
      return FSM_OperatingMode::ESTOP;
    }*/
  }

  // Default is to return the current operating mode
  return operatingMode;

}


/**
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 *
 * @return the appropriate operating mode
 */
template <typename T>
FSM_OperatingMode ControlFSM<T>::safetyPostCheck() {

  // Check for safe orientation if the current state requires it
  if (false){//currentState->checkPDesFoot) {
    for (int leg = 0; leg < 4; leg++) {
      //if (data._quadruped->_robotType == RobotType::MINI_CHEETAH) {  // going to add RobotType to the Quadruped class
      //} else {
      // Keep the foot from going too far from the body in +x
      if (data._legController->commands[leg].pDes(0) > 0.5) {  // 0.5 change to data._quadruped->maxLegLength*cos(maxAngle)
        data._legController->commands[leg].pDes(0) = 0.5;

      }

      // Keep the foot from going too far from the body in -x
      if (data._legController->commands[leg].pDes(0) < -0.5) {
        data._legController->commands[leg].pDes(0) = -0.5;

      }

      // Keep the foot from going too far from the body in +y
      if (data._legController->commands[leg].pDes(1) > 0.5) {
        data._legController->commands[leg].pDes(1) = 0.5;

      }

      // Keep the foot from going too far from the body in -y
      if (data._legController->commands[leg].pDes(1) < -0.5) {
        data._legController->commands[leg].pDes(1) = -0.5;

      }

      // Keep the leg under the motor module
      if (data._legController->commands[leg].pDes(2) > -0.2) {
        data._legController->commands[leg].pDes(2) = -0.2;

      }

      // Keep the foot within the kinematic limits
      if (data._legController->commands[leg].pDes(2) < -data._quadruped->_maxLegLength) { // 0.7 change to data._quadruped->maxLegLength // add maxLegLength
        data._legController->commands[leg].pDes(2) = -data._quadruped->_maxLegLength;

      }
    }

  }

// Default is to return the current operating mode
  return operatingMode;

}


/**
 * Returns the approptiate next FSM State when requested.
 *
 * @param  next requested enumerated state name
 * @return next FSM state
 */
template <typename T>
FSM_State<T>* ControlFSM<T>::getNextState(FSM_StateName stateName) {
  // Choose the correct FSM State by enumerated state name
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


/**
 * Prints Control FSM info at regular intervals and on important events
 * such as transition initializations and finalizations. Separate function
 * to not clutter the actual code.
 *
 * @param printing mode option for regular or an event
 */
template <typename T>
void ControlFSM<T>::printInfo(int opt) {
  switch (opt) {
  case 0 :  // Normal printing case at regular intervals
    // Increment printing iteration
    printIter++;

    // Print at requested frequency
    if (printIter == printNum) {
      std::cout << "[CONTROL FSM] Printing FSM Info...\n";
      std::cout << "---------------------------------------------------------\n";
      if (operatingMode == FSM_OperatingMode::NORMAL) {
        std::cout << "Operating Mode: NORMAL in " << currentState->stateString << "\n";

      } else if (operatingMode == FSM_OperatingMode::TRANSITIONING) {
        std::cout << "Operating Mode: TRANSITIONING from " << currentState->stateString << " to " << nextState->stateString << "\n";

      } else if (operatingMode == FSM_OperatingMode::ESTOP) {
        std::cout << "Operating Mode: ESTOP\n";

      }
      std::cout << "Gait Type: " << data._gaitScheduler->gaitData.gaitName << "\n";
      std::cout << std::endl;

      // Reset iteration counter
      printIter = 0;
    }

    // Print robot info about the robot's status
    //data._gaitScheduler->printGaitInfo();
    //data._desiredStateCommand->printStateCommandInfo();

    break;

  case 1 :  // Initializing FSM State transition
    std::cout << "[CONTROL FSM] Transition initialized from " << currentState->stateString << " to " << nextState->stateString << "\n" << std::endl;

    break;

  case 2 :  // Finalizing FSM State transition
    std::cout << "[CONTROL FSM] Transition finalizing from " << currentState->stateString << " to " << nextState->stateString << "\n" << std::endl;

    break;
  }
}


//template class ControlFSM<double>; This should be fixed... need to make RobotController a template
template class ControlFSM<float>;