#ifndef CONTROLFSM_H
#define CONTROLFSM_H

#include <iostream>

// Contains all of the control related data
#include "ControlFSMData.h"

// FSM States
#include "../FSM_States/FSM_State.h"
#include "../FSM_States/FSM_State_Passive.h"
#include "../FSM_States/FSM_State_BalanceStand.h"
#include "../FSM_States/FSM_State_Locomotion.h"


/*
 * Enumerate all of the operating modes
 */
enum class FSM_OperatingMode {
  NORMAL,
  TRANSITIONING,
  ESTOP,
};


/*
 * Control FSM handles the FSM states from a higher level
 */
template <typename T>
class ControlFSM {
public:
  ControlFSM(StateEstimatorContainer<T>* _stateEstimator,
             LegController<T>* _legController,
             GaitScheduler<T>* _gaitScheduler,
             DesiredStateCommand<T>* _desiredStateCommand);

  // Initializes the Control FSM instance
  void initialize();

  // Runs the FSM logic and handles the state transitions and normal runs
  void runFSM();

  // Contains all of the control related data
  ControlFSMData<T> data;

  // The current FSM State of the robot
  FSM_State<T>* currentState;

  // The next FSM State that the robot will transition to
  FSM_State<T>* nextState;

private:


};



#endif // CONTROLFSM_H