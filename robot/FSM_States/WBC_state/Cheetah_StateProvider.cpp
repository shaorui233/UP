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

    _ori_command[0] = 0.;
    _ori_command[1] = 0.;
    _ori_command[2] = 0.;

    _target_ori_command.setZero();
    _YawRot.setIdentity();

    _body_target.setZero();
}
template<typename T>
void Cheetah_StateProvider<T>::UpdateYawTargetRot(const T & yaw){
    _YawRot(0, 0) = cos(yaw); _YawRot(0, 1) = -sin(yaw); _YawRot(0, 2) = 0.;
    _YawRot(1, 0) = sin(yaw); _YawRot(1, 1) = cos(yaw);  _YawRot(1, 2) = 0.;
    _YawRot(2, 0) = 0.;       _YawRot(2, 1) = 0.;        _YawRot(2, 2) = 1.;
}

template<typename T>
void Cheetah_StateProvider<T>::UpdateExtraData(Cheetah_Extra_Data<T> * ext_data){
    //printf("foot_step list size: %lu\n", _foot_step_list.size());
    for(size_t i(0); i<_foot_step_list.size(); ++i){
        ext_data->loc_x[2*i] = _foot_step_list[i][0]; 
        ext_data->loc_x[2*i + 1] = _foot_step_list[i][3]; 

        ext_data->loc_y[2*i] = _foot_step_list[i][1]; 
        ext_data->loc_y[2*i + 1] = _foot_step_list[i][4]; 
        
        ext_data->loc_z[2*i] = _foot_step_list[i][2]; 
        ext_data->loc_z[2*i + 1] = _foot_step_list[i][5]; 
     }
    ext_data->num_step = _num_step;
}

template class Cheetah_StateProvider<double> ;
template class Cheetah_StateProvider<float> ;

