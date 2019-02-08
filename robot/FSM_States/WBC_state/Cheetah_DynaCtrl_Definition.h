#ifndef Cheetah_DYNACORE_CONTROL_DEFINITION
#define Cheetah_DYNACORE_CONTROL_DEFINITION

#include <Configuration.h>
#include <Dynamics/Quadruped.h>
#include <Controllers/LegController.h>

#define CheetahConfigPath THIS_COM"FSM_States/WBC_state/CheetahTestConfig/"

template <typename T>
class Cheetah_Data{
    public:
        T ang_vel[3];
        T body_ori[4];
        T jpos[cheetah::num_act_joint];
        T jvel[cheetah::num_act_joint];
        bool foot_contact[4];
};

#endif
