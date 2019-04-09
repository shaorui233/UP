#include "FSM_State.h"


template <typename T>
FSM_State<T>::FSM_State(ControlFSMData<T>* _controlFSMData,
                        FSM_StateName stateNameIn):
  _data(_controlFSMData),
  stateName(stateNameIn) {
  printf("[FSM_State] Initialized FSM state");// %s\n",name.c_str());
}



template class FSM_State<double>;
template class FSM_State<float>;