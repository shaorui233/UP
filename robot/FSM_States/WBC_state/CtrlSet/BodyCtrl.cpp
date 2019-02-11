#include "BodyCtrl.hpp"

#include <WBC_state/Cheetah_StateProvider.hpp>
#include <WBC_state/Cheetah_DynaCtrl_Definition.h>
#include <WBC_state/ContactSet/SingleContact.hpp>
#include <WBC_state/TaskSet/LinkPosTask.hpp>
#include <WBC_state/TaskSet/BodyOriTask.hpp>
#include <WBC_state/TaskSet/BodyPosTask.hpp>

#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <ParamHandler/ParamHandler.hpp>

template <typename T>
BodyCtrl<T>::BodyCtrl(const FloatingBaseModel<T>* robot):Controller<T>(robot),
    Kp_(cheetah::num_act_joint),
    Kd_(cheetah::num_act_joint),
    des_jpos_(cheetah::num_act_joint),
    des_jvel_(cheetah::num_act_joint),
    des_jacc_(cheetah::num_act_joint),
    b_set_height_target_(false),
    end_time_(1000.0),
    dim_contact_(0),
    ctrl_start_time_(0.)
{
    body_pos_task_ = new BodyPosTask<T>(Ctrl::robot_sys_);
    body_ori_task_ = new BodyOriTask<T>(Ctrl::robot_sys_);

    Ctrl::task_list_.push_back(body_ori_task_);
    Ctrl::task_list_.push_back(body_pos_task_);

   
    fr_contact_ = new SingleContact<T>(Ctrl::robot_sys_, linkID::FR);
    fl_contact_ = new SingleContact<T>(Ctrl::robot_sys_, linkID::FL);
    hr_contact_ = new SingleContact<T>(Ctrl::robot_sys_, linkID::HR);
    hl_contact_ = new SingleContact<T>(Ctrl::robot_sys_, linkID::HL);

    Ctrl::contact_list_.push_back(fr_contact_);
    Ctrl::contact_list_.push_back(fl_contact_);
    Ctrl::contact_list_.push_back(hr_contact_);
    Ctrl::contact_list_.push_back(hl_contact_);

    kin_wbc_ = new KinWBC<T>(cheetah::dim_config);
    wblc_ = new WBLC<T>(cheetah::dim_config, Ctrl::contact_list_);
    wblc_data_ = new WBLC_ExtraData<T>();
 
    for(size_t i(0); i<Ctrl::contact_list_.size(); ++i){
        dim_contact_ += Ctrl::contact_list_[i]->getDim();
    }

    wblc_data_->W_qddot_ = DVec<T>::Constant(cheetah::dim_config, 100.0);
    wblc_data_->W_rf_ = DVec<T>::Constant(dim_contact_, 1.);
    wblc_data_->W_xddot_ = DVec<T>::Constant(dim_contact_, 1000.0);

    int idx_offset(0);
    for(size_t i(0); i<Ctrl::contact_list_.size(); ++i){
        wblc_data_->W_rf_[idx_offset + Ctrl::contact_list_[i]->getFzIndex()]= 0.01;
        idx_offset += Ctrl::contact_list_[i]->getDim();
    }

    // torque limit default setting
    wblc_data_->tau_min_ = DVec<T>::Constant(cheetah::num_act_joint, -150.);
    wblc_data_->tau_max_ = DVec<T>::Constant(cheetah::num_act_joint, 150.);

    _sp = Cheetah_StateProvider<T>::getStateProvider();

    printf("[Body Control] Constructed\n");
}

template <typename T>
BodyCtrl<T>::~BodyCtrl(){
    delete wblc_;
    delete wblc_data_;
    delete kin_wbc_;

    typename std::vector<Task<T>*>::iterator iter = Ctrl::task_list_.begin();
    while(iter < Ctrl::task_list_.end()){
        delete (*iter);
        ++iter;
    }
    Ctrl::task_list_.clear();

    typename std::vector<ContactSpec<T>*>::iterator iter2 = Ctrl::contact_list_.begin();
    while(iter2 < Ctrl::contact_list_.end()){
        delete (*iter2);
        ++iter2;
    }
    Ctrl::contact_list_.clear();
}

template <typename T>
void BodyCtrl<T>::OneStep(void* _cmd){
    Ctrl::_PreProcessing_Command();
    Ctrl::state_machine_time_ = _sp->curr_time_ - ctrl_start_time_;

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
void BodyCtrl<T>::_compute_torque_wblc(DVec<T> & gamma){
    // WBLC
    wblc_->UpdateSetting(Ctrl::A_, Ctrl::Ainv_, Ctrl::coriolis_, Ctrl::grav_);
    DVec<T> des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - Ctrl::robot_sys_->_state.q)
        + Kd_.cwiseProduct(des_jvel_ - Ctrl::robot_sys_->_state.qd);

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, 
            gamma, wblc_data_);
}

template <typename T>
void BodyCtrl<T>::_task_setup(){
    des_jpos_ = jpos_ini_;
    des_jvel_.setZero();
    des_jacc_.setZero();

    // Calculate IK for a desired height and orientation.
    T body_height_cmd;
    if(b_set_height_target_) body_height_cmd = target_body_height_;
    else{ printf("No Height Command\n"); exit(0); }
    
    DVec<T> vel_des(3); vel_des.setZero();
    DVec<T> acc_des(3); acc_des.setZero();
    //Vec3<T> des_pos = ini_body_pos_;
    //Vec3<T> des_pos = Ctrl::robot_sys_->_state.bodyPosition;
    Vec3<T> des_pos = _sp->_body_target;
    des_pos[2] = body_height_cmd;
    body_pos_task_->UpdateTask(&(des_pos), vel_des, acc_des);

    // Set Desired Orientation
    Quat<T> des_quat; des_quat.setZero();
    T theta(0.);
    T amp_rot(0.);
    T omega_ang(0.5*2.*M_PI);
    theta = amp_rot*sin(omega_ang * Ctrl::state_machine_time_);
    // pitch
    des_quat[0] = cos(theta/2.);
    des_quat[2] = sin(theta/2.);
    //des_quat[0] = 1.;

    DVec<T> ang_vel_des(body_ori_task_->getDim()); ang_vel_des.setZero();
    DVec<T> ang_acc_des(body_ori_task_->getDim()); ang_acc_des.setZero();
    body_ori_task_->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);

    kin_wbc_->FindConfiguration(_sp->Q_,
            Ctrl::task_list_, Ctrl::contact_list_, 
            des_jpos_, des_jvel_, des_jacc_);
}

template <typename T>
void BodyCtrl<T>::_contact_setup(){
    typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::contact_list_.begin();
    while(iter < Ctrl::contact_list_.end()){
        (*iter)->UpdateContactSpec();
        ++iter;
    }
}

template <typename T>
void BodyCtrl<T>::FirstVisit(){
    jpos_ini_ = Ctrl::robot_sys_->_state.q;
    ctrl_start_time_ = _sp->curr_time_;
    ini_body_pos_ = Ctrl::robot_sys_->_state.bodyPosition;
}

template <typename T>
void BodyCtrl<T>::LastVisit(){}

template <typename T>
bool BodyCtrl<T>::EndOfPhase(){
    if(Ctrl::state_machine_time_ > end_time_){
        return true;
    }
    return false;
}

template <typename T>
void BodyCtrl<T>::CtrlInitialization(const std::string & setting_file_name){
    jpos_ini_ = Ctrl::robot_sys_->_state.q;

    ini_body_pos_ = Ctrl::robot_sys_->_state.bodyPosition;
    ParamHandler handler(CheetahConfigPath + setting_file_name + ".yaml");

    std::vector<T> tmp_vec;
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

template <typename T>
void BodyCtrl<T>::SetTestParameter(const std::string & test_file){
    ParamHandler handler(test_file);
    if(handler.getValue<T>("body_height", target_body_height_)){
        b_set_height_target_ = true;
    }
    handler.getValue<T>("stance_time", end_time_);
}

template class BodyCtrl<double>;
template class BodyCtrl<float>;

