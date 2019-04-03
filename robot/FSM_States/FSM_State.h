#ifndef FSM_State_H
#define FSM_State_H

#include "../hardware_interface.h"
#include <string>

#include "../ControlFSM.h"

using std::string;
class FSM_State
{
public:
    FSM_State(hardware_interface* hw_i,string name,control_fsm* fsm);
    virtual FSM_State* get_next_state() { return 0;}
    virtual void run_state() { }
    virtual void cleanup_state() { }
    hardware_interface* hw_i;
    string name;
    control_fsm* fsm;
private:


};

#endif // FSM_State_H
