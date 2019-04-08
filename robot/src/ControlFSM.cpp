#include "ControlFSM.h"



template <typename T>
void ControlFSM<T>::initialize() {
  currentState = new FSM_State_Locomotion<T>();//(&data);
  currentState->_data = &data;
  currentStateName = FSM_StateName::DO_NOTHING;
  nextStateName = currentStateName;
}


template <typename T>
void ControlFSM<T>::runFSM() {


  /*nextState = currentState->checkTransition();
  if (nextState->stateName != currentState->stateName) {
    // Transitioning
    operatingMode = FSM_OperatingMode::TRANSITIONING;
  }
  if (operatingMode == FSM_OperatingMode::NORMAL) {
    */
    currentState->run();
  //}


}


//template class ControlFSM<double>;
template class ControlFSM<float>;