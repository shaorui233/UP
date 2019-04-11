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


/**
 * Enumerate all of the operating modes
 */
enum class FSM_OperatingMode {
  NORMAL,
  TRANSITIONING,
  ESTOP,
};


/**
 *
 */
template <typename T>
struct FSM_StatesList {
  FSM_State<T>* invalid;
  FSM_State_Passive<T>* passive;
  FSM_State_JointPD<T>* jointPD;
  FSM_State_ImpedanceControl<T>* impedanceControl;
  FSM_State_BalanceStand<T>* balanceStand;
  FSM_State_Locomotion<T>* locomotion;
};



/**
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

  //
  FSM_OperatingMode safetyCheck();

  // Gets the next FSM_State from the list of created states when requested
  FSM_State<T>* getNextState(FSM_StateName stateName);

  // Prints the current FSM status
  void printInfo(int opt);

  // Contains all of the control related data
  ControlFSMData<T> data;

  // Holds all of the FSM States
  FSM_StatesList<T> statesList;

  // The current FSM State of the robot
  FSM_State<T>* currentState;

  // The next FSM State that the robot will transition to
  FSM_State<T>* nextState;

  // The name of the next FSM State
  FSM_StateName nextStateName;

private:
  // Operating mode of the FSM
  FSM_OperatingMode operatingMode;

  // Choose how often to print info, every N iterations
  int printNum = 5000; // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 5001;  // make larger than printNum to not print

};



#endif // CONTROLFSM_H