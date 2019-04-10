/*========================= Impedance Control =========================*/
/**
 * FSM State that allows PD Impedance control in cartesian space for
 * each of the legs.
 */

#include "FSM_State_ImpedanceControl.h"


/**
 * Constructor for the FSM State that passes in state specific info to 
 * the generif FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_ImpedanceControl<T>::FSM_State_ImpedanceControl(ControlFSMData<T>* _controlFSMData):
  FSM_State<T>(_controlFSMData, FSM_StateName::IMPEDANCE_CONTROL, "IMPEDANCE_CONTROL") {
  // Do nothing here yet
}


template <typename T>
void FSM_State_ImpedanceControl<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;
}


/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_ImpedanceControl<T>::run() {
  // Do nothing, all commands should begin as zeros
  this->_data->_gaitScheduler->printGaitInfo();
  this->_data->_desiredStateCommand->printStateCommandInfo();
}


/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumarated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_ImpedanceControl<T>::checkTransition() {
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
bool FSM_State_ImpedanceControl<T>::transition() {
  // Get the next state
  return true;
}


/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_ImpedanceControl<T>::onExit() {
  // Nothing to clean up when exiting
}


//template class FSM_State_ImpedanceControl<double>;
template class FSM_State_ImpedanceControl<float>;