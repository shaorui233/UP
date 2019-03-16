#ifndef Cheetah_DYNACORE_CONTROL_DEFINITION
#define Cheetah_DYNACORE_CONTROL_DEFINITION

#include <Configuration.h>
#include <Dynamics/Quadruped.h>
#include <Controllers/LegController.h>

#define CheetahConfigPath THIS_COM"robot/WBC_States/config/"

template <typename T>
class Cheetah_Data{
    public:
        T ang_vel[3];
        T body_ori[4];
        T jpos[cheetah::num_act_joint];
        T jvel[cheetah::num_act_joint];
        bool foot_contact[4];
        T dir_command[2];
        T ori_command[3];
};

template <typename T>
class Cheetah_Extra_Data{
    public:
        int num_step;
        T loc_x[20]; //Large enough number
        T loc_y[20];
        T loc_z[20];

        T ori_roll[20];
        T ori_pitch[20];
        T ori_yaw[20];

        int num_path_pt;
        T path_x[100];
        T path_y[100];
        T path_z[100];
};


#endif
