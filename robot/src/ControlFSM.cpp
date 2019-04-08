#include "ControlFSM.h"



template <typename T>
void ControlFSM<T>::initialize() {
  currentState = new FSM_State_DoNothing<T>();//(&data);
  currentState->_controlFSMData = &data;
  currentStateName = FSM_StateName::DO_NOTHING;
  nextStateName = currentStateName;
}


template <typename T>
void ControlFSM<T>::runFSM() {

  currentState->run();

}


template class ControlFSM<double>;
template class ControlFSM<float>;