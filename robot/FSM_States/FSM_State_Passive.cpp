/*============================== Passive ==============================*/
/**
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

#include "FSM_State_Passive.h"

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Passive<T>::FSM_State_Passive(ControlFSMData<T>* _controlFSMData):
  FSM_State<T>(_controlFSMData, FSM_StateName::PASSIVE, "PASSIVE") {
  // Do nothing
}

template <typename T>
void FSM_State_Passive<T>::onEnter() {
  // Nothing to initialize

}


/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Passive<T>::run() {
  // Do nothing, all commands should begin as zeros

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
FSM_StateName FSM_State_Passive<T>::checkTransition() {
  this->nextStateName = this->stateName;
  iter++;
  if (iter >= 2987) {
    this->nextStateName = FSM_StateName::JOINT_PD;
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
bool FSM_State_Passive<T>::transition() {
  // Get the next state
  return true;
}


/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Passive<T>::onExit() {
  // Nothing to clean up when exiting
}



//template class FSM_State_Passive<double>;
template class FSM_State_Passive<float>;