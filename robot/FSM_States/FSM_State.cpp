#include "FSM_State.h"


template <typename T>
FSM_State<T>::FSM_State(ControlFSMData<T>* _controlFSMData,
                        FSM_StateName stateNameIn,
                        std::string stateStringIn):
  _data(_controlFSMData),
  stateName(stateNameIn),
  stateString(stateStringIn) {
  printf("[FSM_State] Initialized FSM state\n");// %s\n",name.c_str());
}



template class FSM_State<double>;
template class FSM_State<float>;