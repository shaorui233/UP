#include "FSM_State_Locomotion.h"

template <typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(ControlFSMData<T>* _controlFSMData):
  FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION) {
  // Initialize GRF and footstep locations to 0s
  groundReactionForces = Mat34<T>::Zero();
  footstepLocations = Mat34<T>::Zero();
}


template <typename T>
void FSM_State_Locomotion<T>::onEnter() {
  // Nothing to initialize
}


template <typename T>
void FSM_State_Locomotion<T>::run() {
  // Do nothing, all commands should begin as zeros
  this->_data->_gaitScheduler->printGaitInfo();
  this->_data->_desiredStateCommand->printStateCommandInfo();
  LocomotionControlStep();
}

template <typename T>
FSM_State<T>* FSM_State_Locomotion<T>::getNextState() {
  // Get the next state
  return this;
}


template <typename T>
void FSM_State_Locomotion<T>::onExit() {
  // Nothing to clean up when exiting
}

/*
 * Calculate the commands for the leg controllers for each of the feet.
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

    footstepLocations.col(leg) << 0.3, 0.1, 0.45;
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