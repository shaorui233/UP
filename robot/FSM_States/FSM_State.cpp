#include "FSM_State.h"
#include <stdio.h>

FSM_State::FSM_State(hardware_interface* hw_i,  string name,control_fsm* fsm):
    hw_i(hw_i),
    fsm(fsm),
    name(name)
{
    printf("[FSM_State] Initialized FSM state %s\n",name.c_str());
}
