#include "ControlFSM.h"



template <typename T>
void ControlFSM<T>::initialize() {
  // Initialize a new FSM State with the control data
  currentState = new FSM_State_BalanceStand<T>(&data);
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
  // Run the iteration for the current state
  currentState->run();
  //}


}


//template class ControlFSM<double>; This should be fixed... need to make RobotController a template
template class ControlFSM<float>;