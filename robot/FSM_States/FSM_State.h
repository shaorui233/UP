#ifndef FSM_State_H
#define FSM_State_H

#include <string>

#include "../include/ControlFSMData.h"
#include <stdio.h>

/*
 * Enumerate all of the FSM states so we can keep track of them
 */
enum class FSM_StateName {
  INVALID,
  DO_NOTHING,
  BALANCE_STAND,
  LOCOMOTION
};


/*
 *
 */
template <typename T>
class FSM_State {
public:
  FSM_State(ControlFSMData<T>* _controlFSMData, FSM_StateName stateNameIn);

  // Behavior to be carried out when entering a state
  virtual void onEnter() { }

  // Run the normal behavior for the state
  virtual void run() { }

  //
  virtual FSM_State<T>* getNextState() { return 0; }

  // Behavior to be carried out when exiting a state
  virtual void onExit() { }

  // Holds all of the relevant control data
  ControlFSMData<T>* _data;

  // The enumerated name of the current state
  FSM_StateName stateName;
private:


};

#endif // FSM_State_H
