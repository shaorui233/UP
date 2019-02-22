#include "Cheetah_StateProvider.hpp"

template<typename T>
Cheetah_StateProvider<T>* Cheetah_StateProvider<T>::getStateProvider(){
    static Cheetah_StateProvider state_provider_;
    return &state_provider_;
}

template<typename T>
Cheetah_StateProvider<T>::Cheetah_StateProvider():
    Q_(cheetah::num_q),
    Qdot_(cheetah::dim_config),
    _jpos_des_pre(cheetah::num_act_joint),
    _num_contact(2),
    curr_time_(0.),
    _num_step(-1),
    _opt_play(false)
{
    _contact_pt[0] = linkID::FL;
    _contact_pt[1] = linkID::HR;
    Q_.setZero();
    Qdot_.setZero();
    _local_frame_global_pos.setZero();
    _dir_command[0] = 0.;
    _dir_command[1] = 0.;

    _body_target.setZero();
}

template class Cheetah_StateProvider<double> ;
template class Cheetah_StateProvider<float> ;

