#ifndef FSM_State_H
#define FSM_State_H

#include <stdio.h>

#include "../include/ControlFSMData.h"

/*
 * Enumerate all of the FSM states so we can keep track of them.
 */
enum class FSM_StateName {
  INVALID,
  PASSIVE,
  JOINT_PD,
  IMPEDANCE_CONTROL,
  BALANCE_STAND,
  LOCOMOTION
};


/*
 *
 */
template <typename T>
class FSM_State {
public:
  FSM_State(ControlFSMData<T>* _controlFSMData,
            FSM_StateName stateNameIn,
            std::string stateStringIn);

  // Behavior to be carried out when entering a state
  virtual void onEnter() { }

  // Run the normal behavior for the state
  virtual void run() { }

  // Manages state specific transitions
  virtual FSM_StateName checkTransition() { return FSM_StateName::INVALID; }

  //
  virtual bool transition() { return false; }

  // Behavior to be carried out when exiting a state
  virtual void onExit() { }

  // Holds all of the relevant control data
  ControlFSMData<T>* _data;

  // The enumerated name of the current state
  FSM_StateName stateName;

  // The enumerated name of the next state
  FSM_StateName nextStateName;

  // State name string
  std::string stateString;

  // Save the time transition starts
  T tStartTransition = 0;

private:


};

#endif // FSM_State_H
