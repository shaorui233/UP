#include "FSM_State.h"

/**
 * Constructor for the FSM State class.
 *
 * @param _controlFSMData holds all of the relevant control data
 * @param stateNameIn the enumerated state name
 * @param stateStringIn the string name of the current FSM state
 */
template <typename T>
FSM_State<T>::FSM_State(ControlFSMData<T>* _controlFSMData,
                        FSM_StateName stateNameIn,
                        std::string stateStringIn):
  _data(_controlFSMData),
  stateName(stateNameIn),
  stateString(stateStringIn) {
  std::cout << "[FSM_State] Initialized FSM state: " <<  stateStringIn << std::endl;
}



template class FSM_State<double>;
template class FSM_State<float>;