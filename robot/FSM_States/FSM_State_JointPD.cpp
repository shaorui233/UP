/*============================= Joint PD ==============================*/
/**
 * FSM State that allows PD control of the joints.
 */

#include "FSM_State_JointPD.h"


/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_JointPD<T>::FSM_State_JointPD(ControlFSMData<T>* _controlFSMData):
  FSM_State<T>(_controlFSMData, FSM_StateName::JOINT_PD, "JOINT_PD") {
  // Do nothing here yet
}


template <typename T>
void FSM_State_JointPD<T>::onEnter() {
  // Nothing to initialize
}


/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_JointPD<T>::run() {
  Vec3<T> qDes;
  qDes << 0, -1.052, 2.63;
  Vec3<T> qdDes;
  qdDes << 0, 0, 0;
  this->jointPDControl(0, qDes, qdDes);
  this->jointPDControl(1, qDes, qdDes);
  this->jointPDControl(2, qDes, qdDes);
  this->jointPDControl(3, qDes, qdDes);
  // Do nothing, all commands should begin as zeros
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
FSM_StateName FSM_State_JointPD<T>::checkTransition() {
  this->nextStateName = this->stateName;

  iter++;
  if (iter >= 2345) {
  	this->nextStateName = FSM_StateName::BALANCE_STAND;
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
bool FSM_State_JointPD<T>::transition() {
  // Get the next state
  return true;
}


/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_JointPD<T>::onExit() {
  // Nothing to clean up when exiting
}


//template class FSM_State_JointPD<double>;
template class FSM_State_JointPD<float>;