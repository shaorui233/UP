#include "FSM_State.h"

/*
template <typename T>
FSM_State<T>::FSM_State(ControlFSMData<T>* _controlFSMDataIn):
  _controlFSMData(_controlFSMDataIn) {
  printf("[FSM_State] Initialized FSM state");// %s\n",name.c_str());
}
*/


template class FSM_State<double>;
template class FSM_State<float>;