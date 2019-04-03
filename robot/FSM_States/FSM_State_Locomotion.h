#ifndef FSM_STATE_LOCOMOTION_H
#define FSM_STATE_LOCOMOTION_H

#include "../hardware_interface.h"
#include "../Controllers/FootSwing.h"
#include "../Controllers/LegController/leg_controller.h"
#include "../control_fsm.h"
#include "fsm_state.h"
#include "../Controllers/convexMPC/convexMPC_interface.h"
#include "../../cpp_dynamics/spatial_utilities.h"
#include "../Gait.h"

class fsm_state_locomotion: public fsm_state
{
public:
    fsm_state_locomotion(hardware_interface* hw_i,string name,control_fsm* fsm);
    void run_state();
    fsm_state* get_next_state();
    void cleanup_state();
private:

};

#endif // FSM_STATE_LOCOMOTION_H
