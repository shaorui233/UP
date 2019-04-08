#ifndef FSM_State_H
#define FSM_State_H

#include <string>

#include "../include/ControlFSMData.h"
#include <stdio.h>


/*
 *
 */
template <typename T>
class FSM_State {
public:
  //FSM_State();//ControlFSMData<T>* _controlFSMDataIn);//: _controlFSMData(_controlFSMData) { }//hardware_interface* hw_i, FSM_StateName stateName, control_fsm* fsm);

  // Behavior to be carried out when entering a state
  virtual void onEnter() { }

  // Run the normal behavior for the state
  virtual void run() { }

  //
  virtual FSM_State<T>* getNextState() { return 0; }

  // Behavior to be carried out when exiting a state
  virtual void onExit() { }

  //hardware_interface* hw_i;
  //FSM_StateName stateName;
  //RobotController* _robotController;
  ControlFSMData<T>* _controlFSMData;
private:


};

#endif // FSM_State_H
