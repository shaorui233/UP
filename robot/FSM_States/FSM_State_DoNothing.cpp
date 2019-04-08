#include "FSM_State_DoNothing.h"

template <typename T>
FSM_State_DoNothing<T>::FSM_State_DoNothing(ControlFSMData<T>* _controlFSMData):
  FSM_State(ControlFSMData<T>* _controlFSMData) {
  	// Do nothing here
}


template <typename T>
FSM_State_DoNothing<T>::onEnter() {
	// NOthing to initialize
}


template <typename T>
FSM_State_DoNothing<T>::run() {
	// Do nothing, all commands should begin as zeros
}

template <typename T>
FSM_State_DoNothing<T>::getNextState() {
	// Get the next state
}


template <typename T>
FSM_State_DoNothing<T>::onExit() {
	// Nothing to clean up when exiting
}



template class FSM_State_DoNothing<double>;
template class FSM_State_DoNothing<float>;