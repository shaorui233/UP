/*============================ Locomotion =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include "FSM_State_Locomotion.h"


/**
 * Constructor for the FSM State that passes in state specific info to 
 * the generif FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData):
  FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION") {
  // Initialize GRF and footstep locations to 0s
  groundReactionForces = Mat34<T>::Zero();
  footstepLocations = Mat34<T>::Zero();
}


template <typename T>
void FSM_State_Locomotion<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;
}


/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Locomotion<T>::run() {
  // Call the locomotion control logic for this iteration
  LocomotionControlStep();

  // Print some information about the current iteration
  //this->_data->_gaitScheduler->printGaitInfo();
  //this->_data->_desiredStateCommand->printStateCommandInfo();
}


/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumarated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Locomotion<T>::checkTransition() {
  // Get the next state
  iter++;
  if (iter > 2500) {
    this->nextStateName = FSM_StateName::BALANCE_STAND;
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
bool FSM_State_Locomotion<T>::transition() {
  // Get the next state

  if (this->nextStateName == FSM_StateName::BALANCE_STAND) {
    // Call the locomotion control logic for this iteration
    LocomotionControlStep();

    iter++;
    if (iter > 5000) {
      return true;
    } else {
      return false;
    }
  } else {

  }

  return true;
}


/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Locomotion<T>::onExit() {
  // Nothing to clean up when exiting
  iter = 0;
}

/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
template <typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep() {
  //StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

  // Contact state logic
  //estimateContact();

  // Run the balancing controllers to get GRF and next step locations
  //runControls();

  // Reset the forces and steps to 0
  groundReactionForces = Mat34<T>::Zero();
  footstepLocations = Mat34<T>::Zero();

  // Test to make sure we can control the robot these will be calculated by the controllers
  for (int leg = 0; leg < 4; leg++) {
    groundReactionForces.col(leg) << 0.0, 0.0, -220.36;
    //groundReactionForces.col(leg) = stateEstimate.rBody * groundReactionForces.col(leg);

    footstepLocations.col(leg) << 0.1, 0, -0.35;
  }

  //std::cout << groundReactionForces << std::endl;

  // Calculate appropriate control actions for each leg to be sent out
  for (int leg = 0; leg < 4; leg++) {

    // The actual contact logic should come from the contact estimator later rather than schedule
    if (this->_data->_gaitScheduler->gaitData.contactStateScheduled(leg)) {
      // Leg is in contact
      //std::cout << "[CONTROL] Leg " << leg << " is in stance" << std::endl;

      // Impedance control for the stance leg
      //stanceLegImpedanceControl(leg);

      // Stance leg Ground Reaction Force command
      this->_data->_legController->commands[leg].forceFeedForward = groundReactionForces.col(leg);

    } else if (!this->_data->_gaitScheduler->gaitData.contactStateScheduled(leg)) {
      // Leg is not in contact
      //std::cout << "[CONTROL] Leg " << leg << " is in swing" << std::endl;
      this->_data->_legController->commands[leg].pDes = footstepLocations.col(leg);

      // Create the cartesian P gain matrix
      Mat3<float> kpMat;
      kpMat << 500, 0, 0,
            0, 500, 0,
            0, 0, 500;
      this->_data->_legController->commands[leg].kpCartesian = kpMat;

      // Create the cartesian D gain matrix
      Mat3<float> kdMat;
      kdMat << 10, 0, 0,
            0, 10, 0,
            0, 0, 10;
      this->_data->_legController->commands[leg].kdCartesian = kdMat;
      // Swing leg trajectory
      // TODO

      // Feedforward torques for swing leg tracking
      // TODO

    } else {
      std::cout << "[CONTROL ERROR] Undefined scheduled contact state\n" << std::endl;
    }

    // Singularity barrier calculation (maybe an overall safety checks function?)
    // TODO
  }

}

//template class FSM_State_Locomotion<double>;
template class FSM_State_Locomotion<float>;