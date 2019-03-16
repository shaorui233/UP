#include "StateProvider.hpp"

template<typename T>
StateProvider<T>* StateProvider<T>::getStateProvider(){
    static StateProvider state_provider_;
    return &state_provider_;
}

template<typename T>
StateProvider<T>::StateProvider():
    _Q(cheetah::num_q),
    _Qdot(cheetah::dim_config),
    _num_contact(2),
    _curr_time(0.),
    _num_step(-1)
{
    _contact_pt[0] = linkID::FL;
    _contact_pt[1] = linkID::HR;
    _Q.setZero();
    _Qdot.setZero();

    _local_frame_global_pos.setZero();
    _dir_command[0] = 0.;
    _dir_command[1] = 0.;

    _ori_command[0] = 0.;
    _ori_command[1] = 0.;
    _ori_command[2] = 0.;
}

template class StateProvider<double> ;
template class StateProvider<float> ;

