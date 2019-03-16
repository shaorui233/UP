#ifndef STATE_PROVIDER_Cheetah
#define STATE_PROVIDER_Cheetah

#include <cppTypes.h>
#include "Cheetah_DynaCtrl_Definition.h"

template <typename T>
class StateProvider{
    public:
        static StateProvider<T>* getStateProvider();
        ~StateProvider(){}

        DVec<T> _Q;
        DVec<T> _Qdot;

        size_t _contact_pt[cheetah::num_leg];
        size_t _num_contact;
        Vec3<T> _local_frame_global_pos;

        T _curr_time;
        T _dir_command[2];
        T _ori_command[3];

        int _num_step;
    private:
        StateProvider();
};


#endif
