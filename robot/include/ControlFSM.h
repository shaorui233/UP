#ifndef CONTROLFSM_H
#define CONTROLFSM_H

#include <iostream>
#include "../FSM_States/FSM_State.h"
#include "../FSM_States/FSM_State_DoNothing.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Controllers/LegController.h"
#include "Controllers/GaitScheduler.h"
#include "Controllers/DesiredStateCommand.h"
#include "ControlFSMData.h"


/*
 * Enumerate all of the operating modes
 */
enum class FSM_OperatingMode {
  NORMAL,
  TRANSITIONING,
  ESTOP,
};

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
 * Control FSM handles the FSM states from a higher level
 */
template <typename T>
class ControlFSM {
public:
  ControlFSM(StateEstimatorContainer<T>* _stateEstimator,
             LegController<T>* _legController,
             GaitScheduler<T>* _gaitScheduler,
             DesiredStateCommand<T>* _desiredStateCommand) {
    data._stateEstimator = _stateEstimator;
    data._legController = _legController;
    data._gaitScheduler = _gaitScheduler;
    data._desiredStateCommand = _desiredStateCommand;
  }

  void initialize();

  void runFSM();

  ControlFSMData<T> data;

  FSM_State<T>* currentState;
  FSM_State<T>* nextState;
  FSM_StateName currentStateName;
  FSM_StateName nextStateName;


private:


};



#endif // CONTROLFSM_H