#include "JPosCtrl.hpp"

#include <WBC_state/Cheetah_StateProvider.hpp>
#include <WBC_state/ContactSet/FixedBodyContact.hpp>

#include <WBLC/WBLC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utilities/utilities.h>

    template <typename T>
JPosCtrl<T>::JPosCtrl(const FloatingBaseModel<T>* robot):Controller<T>(robot),
    Kp_(cheetah::num_act_joint),
    Kd_(cheetah::num_act_joint),
    des_jpos_(cheetah::num_act_joint),
    des_jvel_(cheetah::num_act_joint),
    des_jacc_(cheetah::num_act_joint),
    b_set_target_(false),
    end_time_(1000.0),
    dim_contact_(0),
    ctrl_start_time_(0.),
    _jpos_target(cheetah::num_act_joint),
    _move_to_target(true)
{

    contact_ = new FixedBodyContact<T>();

    Ctrl::contact_list_.push_back(contact_);
    dim_contact_ = Ctrl::contact_list_[0]->getDim();

    wblc_ = new WBLC<T>(cheetah::dim_config, Ctrl::contact_list_);
    wblc_data_ = new WBLC_ExtraData<T>();

    wblc_data_->W_qddot_ = DVec<T>::Constant(cheetah::dim_config, 100.0);
    wblc_data_->W_rf_ = DVec<T>::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = DVec<T>::Constant(dim_contact_, 1000.0);

    // torque limit default setting
    wblc_data_->tau_min_ = DVec<T>::Constant(cheetah::num_act_joint, -150.);
    wblc_data_->tau_max_ = DVec<T>::Constant(cheetah::num_act_joint, 150.);

    sp_ = Cheetah_StateProvider<T>::getStateProvider();

    printf("[Config JPos Control] Constructed\n");
}

template <typename T>
JPosCtrl<T>::~JPosCtrl(){
    delete wblc_;
    delete wblc_data_;

    typename std::vector<ContactSpec<T>*>::iterator iter2 
        = Ctrl::contact_list_.begin();

    while(iter2 < Ctrl::contact_list_.end()){
        delete (*iter2);
        ++iter2;
    }
    Ctrl::contact_list_.clear();
}

template <typename T>
void JPosCtrl<T>::OneStep(void* _cmd){
    Ctrl::_PreProcessing_Command();
    Ctrl::state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    DVec<T> gamma = DVec<T>::Zero(cheetah::num_act_joint);

    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for(size_t leg(0); leg<cheetah::num_leg; ++leg){
        for(size_t jidx(0); jidx<cheetah::num_leg_joint; ++jidx){
            ((LegControllerCommand<T>*)_cmd)[leg].tauFeedForward[jidx] 
                = gamma[cheetah::num_leg_joint * leg + jidx];

            ((LegControllerCommand<T>*)_cmd)[leg].qDes[jidx] = 
                des_jpos_[cheetah::num_leg_joint * leg + jidx];

            ((LegControllerCommand<T>*)_cmd)[leg].qdDes[jidx] = 
                des_jvel_[cheetah::num_leg_joint * leg + jidx];
        }
    }
    Ctrl::_PostProcessing_Command();
}

template <typename T>
void JPosCtrl<T>::_compute_torque_wblc(DVec<T> & gamma){
    // WBLC
    wblc_->UpdateSetting(Ctrl::A_, Ctrl::Ainv_, Ctrl::coriolis_, Ctrl::grav_);
    DVec<T> des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - Ctrl::robot_sys_->_state.q)
        + Kd_.cwiseProduct(des_jvel_ - Ctrl::robot_sys_->_state.qd);

    //pretty_print(des_jacc_cmd, std::cout, "des jacc");
    wblc_->MakeWBLC_Torque(des_jacc_cmd, gamma, wblc_data_);
}

template <typename T>
void JPosCtrl<T>::_task_setup(){
    des_jpos_ = jpos_ini_;
    des_jvel_.setZero();
    des_jacc_.setZero();

    if(_move_to_target){
        for(size_t i(0); i<cheetah::num_act_joint; ++i){
            des_jpos_[i] = smooth_change(jpos_ini_[i], _jpos_target[i], end_time_,
                    Ctrl::state_machine_time_);
            des_jvel_[i] = smooth_change_vel(jpos_ini_[i], _jpos_target[i], end_time_,
                    Ctrl::state_machine_time_);
            des_jacc_[i] = smooth_change_acc(jpos_ini_[i], _jpos_target[i], end_time_,
                    Ctrl::state_machine_time_);
        }
    }else{
        des_jpos_ = _jpos_target;
    }
}

template <typename T>
void JPosCtrl<T>::_contact_setup(){
    typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::contact_list_.begin();
    while(iter < Ctrl::contact_list_.end()){
        (*iter)->UpdateContactSpec();
        ++iter;
    }
}

template <typename T>
void JPosCtrl<T>::FirstVisit(){
    jpos_ini_ = Ctrl::robot_sys_->_state.q;
    ctrl_start_time_ = sp_->curr_time_;
}

template <typename T>
void JPosCtrl<T>::LastVisit(){}

template <typename T>
bool JPosCtrl<T>::EndOfPhase(){
    if(Ctrl::state_machine_time_ > end_time_){
        return true;
    }
    return false;
}

template <typename T>
void JPosCtrl<T>::CtrlInitialization(const std::string & category_name){
    ParamHandler handler(_test_file_name);
    handler.getValue<T>(category_name, "move_time", end_time_);
    handler.getBoolean(category_name, "moving_to_target", _move_to_target);
}

template <typename T>
void JPosCtrl<T>::SetTestParameter(const std::string & test_file){
    ParamHandler handler(test_file);
    _test_file_name = test_file;

    std::vector<T> tmp_vec;
    handler.getVector<T>("target_jpos", tmp_vec);
    for(size_t i(0); i<cheetah::num_act_joint; ++i) _jpos_target[i] = tmp_vec[i];

    b_set_target_ = true;

    // Feedback Gain
    handler.getVector<T>("Kp", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        Kp_[i] = tmp_vec[i];
    }
    handler.getVector<T>("Kd", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        Kd_[i] = tmp_vec[i];
    }
}


template class JPosCtrl<double>;
template class JPosCtrl<float>;

