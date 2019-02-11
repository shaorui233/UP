#include "FullContactTransCtrl.hpp"

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
FullContactTransCtrl<T>::FullContactTransCtrl(const FloatingBaseModel<T>* robot):
    Controller<T>(robot),
    Kp_(cheetah::num_act_joint),
    Kd_(cheetah::num_act_joint),
    des_jpos_(cheetah::num_act_joint),
    des_jvel_(cheetah::num_act_joint),
    des_jacc_(cheetah::num_act_joint),
    b_set_height_target_(false),
    end_time_(100.),
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

    sp_ = Cheetah_StateProvider<T>::getStateProvider();

    printf("[Full Contact Transition Ctrl] Constructed\n");
}

template <typename T>
FullContactTransCtrl<T>::~FullContactTransCtrl(){
    delete wblc_;
    delete kin_wbc_;
    delete wblc_data_;

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
void FullContactTransCtrl<T>::OneStep(void* _cmd){
    Ctrl::_PreProcessing_Command();
    Ctrl::state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    //pretty_print(Ctrl::robot_sys_->_pGC[linkID::FR], std::cout, "FR");
    //pretty_print(Ctrl::robot_sys_->_pGC[linkID::FL], std::cout, "FL");
    //pretty_print(Ctrl::robot_sys_->_pGC[linkID::HR], std::cout, "HR");
    //pretty_print(Ctrl::robot_sys_->_pGC[linkID::HL], std::cout, "HL");
 
    //pretty_print(Ctrl::robot_sys_->_Jc[linkID::FR], std::cout, "Jc 0");
    //pretty_print(Ctrl::robot_sys_->_Jc[linkID::FL], std::cout, "Jc 1");
    //pretty_print(Ctrl::robot_sys_->_Jc[linkID::HL], std::cout, "Jc 2");
    //pretty_print(Ctrl::robot_sys_->_Jc[linkID::HR], std::cout, "Jc 3");

    DVec<T> gamma;
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
void FullContactTransCtrl<T>::_compute_torque_wblc(DVec<T> & gamma){
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
void FullContactTransCtrl<T>::_task_setup(){
    des_jpos_ = ini_jpos_;
    des_jvel_.setZero();
    des_jacc_.setZero();
    
    // Calculate IK for a desired height and orientation.
    if(!b_set_height_target_) { printf("No Height Command\n"); exit(0); }
    DVec<T> vel_des(3); vel_des.setZero();
    DVec<T> acc_des(3); acc_des.setZero();
    T alpha = (Ctrl::state_machine_time_/end_time_);
    Vec3<T> des_pos = (1.-alpha) * ini_body_pos_ + alpha * sp_->_body_target;
    des_pos[2] = ini_body_pos_[2] + 
        Ctrl::state_machine_time_/end_time_ * (target_body_height_ - ini_body_pos_[2]);

    body_pos_task_->UpdateTask(&(des_pos), vel_des, acc_des);

    // Set Desired Orientation
    Quat<T> des_quat; des_quat.setZero();
    des_quat[0] = 1.;

    DVec<T> ang_vel_des(body_ori_task_->getDim()); ang_vel_des.setZero();
    DVec<T> ang_acc_des(body_ori_task_->getDim()); ang_acc_des.setZero();
    body_ori_task_->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);

    kin_wbc_->FindConfiguration(sp_->Q_, 
            Ctrl::task_list_, Ctrl::contact_list_, 
            des_jpos_, des_jvel_, des_jacc_);

    //pretty_print(des_jpos_, std::cout, "des_jpos");
    //pretty_print(des_jvel_, std::cout, "des_jvel");
    //pretty_print(des_jacc_, std::cout, "des_jacc");
}

template <typename T>
void FullContactTransCtrl<T>::_contact_setup(){
    typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::contact_list_.begin();
    while(iter < Ctrl::contact_list_.end()){
        ((SingleContact<T>*)(*iter))->setMaxFz(
        min_rf_z_ + Ctrl::state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_) );
        (*iter)->UpdateContactSpec();
        ++iter;
    }
}

template <typename T>
void FullContactTransCtrl<T>::FirstVisit(){
    ini_jpos_ = Ctrl::robot_sys_->_state.q;
    ctrl_start_time_ = sp_->curr_time_;
    ini_body_pos_ = Ctrl::robot_sys_->_state.bodyPosition;
}

template <typename T>
void FullContactTransCtrl<T>::LastVisit(){
    // printf("[ContactTransBody] End\n");
}

template <typename T>
bool FullContactTransCtrl<T>::EndOfPhase(){
    if(Ctrl::state_machine_time_ > end_time_){
        return true;
    }
    return false;
}

template <typename T>
void FullContactTransCtrl<T>::CtrlInitialization(const std::string & setting_file_name){
    ParamHandler handler(CheetahConfigPath + setting_file_name + ".yaml");
    handler.getValue<T>("max_rf_z", max_rf_z_);
    handler.getValue<T>("min_rf_z", min_rf_z_);

    // Feedback Gain
    std::vector<T> tmp_vec;
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
void FullContactTransCtrl<T>::SetTestParameter(
        const std::string & test_file){
    ParamHandler handler(test_file);
    if(handler.getValue<T>("body_height", target_body_height_)){
        b_set_height_target_ = true;
    }
    handler.getValue<T>("body_lifting_time", end_time_);

}
template class FullContactTransCtrl<double>;
template class FullContactTransCtrl<float>;
