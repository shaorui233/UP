/*============================== Passive ==============================*/
/*
 * FSM State that calls no controls. Meant to be a safe state where the
 * robot should not do anything as all commands will be set to 0.
 */

#include "FSM_State_Passive.h"

template <typename T>
FSM_State_Passive<T>::FSM_State_Passive(ControlFSMData<T>* _controlFSMData):
  FSM_State<T>(_controlFSMData, FSM_StateName::PASSIVE) {
  // Do nothing
}

template <typename T>
void FSM_State_Passive<T>::onEnter() {
  // NOthing to initialize
}


/*
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Passive<T>::run() {
  // Do nothing, all commands should begin as zeros

  // Print some information about the current iteration
  this->_data->_gaitScheduler->printGaitInfo();
  this->_data->_desiredStateCommand->printStateCommandInfo();
}


/*
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 */
template <typename T>
FSM_State<T>* FSM_State_Passive<T>::getNextState() {
  // Get the next state
  return this;
}


/*
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Passive<T>::onExit() {
  // Nothing to clean up when exiting
}



template class FSM_State_Passive<double>;
template class FSM_State_Passive<float>;