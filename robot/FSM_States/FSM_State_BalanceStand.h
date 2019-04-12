#ifndef FSM_STATE_BALANCESTAND_H
#define FSM_STATE_BALANCESTAND_H

#include "FSM_State.h"

/**
 *
 */
template <typename T>
class FSM_State_BalanceStand: public FSM_State<T> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FSM_State_BalanceStand(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  bool transition();

  // Behavior to be carried out when exiting a state
  void onExit();

private:
  // Keep track of the control iterations
  int iter = 0;

  // Ground reaction forces for the stance feet to be calculated by the controllers
  Mat34<T> groundReactionForces;
  
  // Next footstep location for the swing feet
  Mat34<T> footstepLocations;

  // Parses contact specific controls to the leg controller
  void BalanceStandStep();

};

#endif // FSM_STATE_BALANCESTAND_H
