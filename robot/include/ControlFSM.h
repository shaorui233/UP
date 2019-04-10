#ifndef CONTROLFSM_H
#define CONTROLFSM_H

#include <iostream>

// Contains all of the control related data
#include "ControlFSMData.h"

// FSM States
#include "../FSM_States/FSM_State.h"
#include "../FSM_States/FSM_State_Passive.h"
#include "../FSM_States/FSM_State_JointPD.h"
#include "../FSM_States/FSM_State_ImpedanceControl.h"
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

  // Gets the next FSM_State from the list of created states when requested
  FSM_State<T>* getNextState(FSM_StateName stateName);

  // Contains all of the control related data
  ControlFSMData<T> data;

  // The current FSM State of the robot
  FSM_State<T>* currentState;

  // The next FSM State that the robot will transition to
  FSM_State<T>* nextState;

  // The name of the next FSM State
  FSM_StateName nextStateName;

private:

  // Operating mode of the FSM
  FSM_OperatingMode operatingMode;


  FSM_State<T>* invalid;
  FSM_State<T>* passive;
  FSM_State<T>* jointPD;
  FSM_State<T>* impedanceControl;
  FSM_State<T>* balanceStand;
  FSM_State<T>* locomotion;
};



#endif // CONTROLFSM_H