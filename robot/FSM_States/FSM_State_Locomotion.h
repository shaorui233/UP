#ifndef FSM_STATE_LOCOMOTION_H
#define FSM_STATE_LOCOMOTION_H

#include "FSM_State.h"

/*
 *
 */
template <typename T>
class FSM_State_Locomotion: public FSM_State<T> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //FSM_State_Locomotion(ControlFSMData<T>* _controlFSMDataIn);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  //
  FSM_State<T>* getNextState();

  // Behavior to be carried out when exiting a state
  void onExit();

  // Parses contact specific controls to the leg controller
  void LocomotionControlStep();

private:

  // Ground reaction forces for the stance feet to be calculated by the controllers
  Mat34<T> groundReactionForces;

  // Next footstep location for the swing feet
  Mat34<T> footstepLocations;

};

#endif // FSM_STATE_LOCOMOTION_H
