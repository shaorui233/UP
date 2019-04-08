#include "FSM_State_DoNothing.h"
/*
template <typename T>
FSM_State_DoNothing<T>::FSM_State_DoNothing(ControlFSMData<T>* _controlFSMDataIn)//:
  //FSM_State(ControlFSMData<T>* _controlFSMDataIn) {
{  	// Do nothing here
}
*/

template <typename T>
void FSM_State_DoNothing<T>::onEnter() {
  // NOthing to initialize
}


template <typename T>
void FSM_State_DoNothing<T>::run() {
  // Do nothing, all commands should begin as zeros
  this->_controlFSMData->_gaitScheduler->printGaitInfo();
  this->_controlFSMData->_desiredStateCommand->printStateCommandInfo();
}

template <typename T>
FSM_State<T>* FSM_State_DoNothing<T>::getNextState() {
  // Get the next state
  return this;
}


template <typename T>
void FSM_State_DoNothing<T>::onExit() {
  // Nothing to clean up when exiting
}



template class FSM_State_DoNothing<double>;
template class FSM_State_DoNothing<float>;