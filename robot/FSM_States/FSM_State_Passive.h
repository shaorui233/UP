#ifndef FSM_STATE_PASSIVE_H
#define FSM_STATE_PASSIVE_H

#include "FSM_State.h"

/*
 *
 */
template <typename T>
class FSM_State_Passive: public FSM_State<T> {
public:
  FSM_State_Passive(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Manages state specific transitions
  FSM_State<T>* getNextState();

  // Behavior to be carried out when exiting a state
  void onExit();

private:

};

#endif // FSM_STATE_PASSIVE_H
