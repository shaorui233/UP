#include "TwoLegSwingCtrl.hpp"

#include <WBC_state/Cheetah_StateProvider.hpp>
#include <WBC_state/Cheetah_DynaCtrl_Definition.h>
#include <WBC_state/ContactSet/SingleContact.hpp>
#include <WBC_state/TaskSet/LinkPosTask.hpp>
#include <WBC_state/TaskSet/BodyOriTask.hpp>
#include <WBC_state/TaskSet/BodyPosTask.hpp>

#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utilities/utilities.h>


template <typename T>
TwoLegSwingCtrl<T>::TwoLegSwingCtrl(const FloatingBaseModel<T>* robot, 
        size_t cp1, size_t cp2):
    Controller<T>(robot),
    _cp1(cp1), _cp2(cp2),
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

    _cp_pos_task1 = new LinkPosTask<T>(Ctrl::robot_sys_, _cp1);
    _cp_pos_task2 = new LinkPosTask<T>(Ctrl::robot_sys_, _cp2);
    Ctrl::task_list_.push_back(_cp_pos_task1);
    Ctrl::task_list_.push_back(_cp_pos_task2);

    _foot_pos_ini1.setZero();
    _foot_pos_des1.setZero();
    _foot_vel_des1 = DVec<T>::Zero(3);
    _foot_acc_des1 = DVec<T>::Zero(3);

    _foot_pos_ini2.setZero();
    _foot_pos_des2.setZero();
    _foot_vel_des2 = DVec<T>::Zero(3);
    _foot_acc_des2 = DVec<T>::Zero(3);

    fr_contact_ = new SingleContact<T>(Ctrl::robot_sys_, linkID::FR);
    fl_contact_ = new SingleContact<T>(Ctrl::robot_sys_, linkID::FL);
    hr_contact_ = new SingleContact<T>(Ctrl::robot_sys_, linkID::HR);
    hl_contact_ = new SingleContact<T>(Ctrl::robot_sys_, linkID::HL);

    Ctrl::contact_list_.push_back(fr_contact_);
    Ctrl::contact_list_.push_back(fl_contact_);
    Ctrl::contact_list_.push_back(hr_contact_);
    Ctrl::contact_list_.push_back(hl_contact_);

    _kin_contact_list.push_back(fr_contact_);
    _kin_contact_list.push_back(fl_contact_);
    _kin_contact_list.push_back(hr_contact_);
    _kin_contact_list.push_back(hl_contact_);


    kin_wbc_ = new KinWBC<T>(cheetah::dim_config);
    wblc_ = new WBLC<T>(cheetah::dim_config, Ctrl::contact_list_);
    wblc_data_ = new WBLC_ExtraData<T>();

    for(size_t i(0); i<Ctrl::contact_list_.size(); ++i){
        dim_contact_ += Ctrl::contact_list_[i]->getDim();
    }

    wblc_data_->W_qddot_ = DVec<T>::Constant(cheetah::dim_config, Weight::qddot_relax);
    wblc_data_->W_rf_ = DVec<T>::Constant(dim_contact_, Weight::tan_small);
    wblc_data_->W_xddot_ = DVec<T>::Constant(dim_contact_, Weight::foot_big);


    int idx_offset(0);
    for(size_t i(0); i<Ctrl::contact_list_.size(); ++i){
        wblc_data_->W_rf_[idx_offset + Ctrl::contact_list_[i]->getFzIndex()]= Weight::nor_small;
        idx_offset += Ctrl::contact_list_[i]->getDim();
    }

    // torque limit default setting
    wblc_data_->tau_min_ = DVec<T>::Constant(cheetah::num_act_joint, -150.);
    wblc_data_->tau_max_ = DVec<T>::Constant(cheetah::num_act_joint, 150.);

    if(_cp1 == linkID::HL || _cp2 == linkID::HL){
        _SetContact(3, 0.0001, Weight::tan_big, Weight::nor_big, Weight::foot_small);
        _kin_contact_list.erase(_kin_contact_list.begin() + 3);
    }
    if(_cp1 == linkID::HR || _cp2 == linkID::HR){
        _SetContact(2, 0.0001, Weight::tan_big, Weight::nor_big, Weight::foot_small);
        _kin_contact_list.erase(_kin_contact_list.begin() + 2);
    }
    if(_cp1 == linkID::FL || _cp2 == linkID::FL){
        _SetContact(1, 0.0001, Weight::tan_big, Weight::nor_big, Weight::foot_small);
        _kin_contact_list.erase(_kin_contact_list.begin() + 1);
    }
    if(_cp1 == linkID::FR || _cp2 == linkID::FR){
        _SetContact(0, 0.0001, Weight::tan_big, Weight::nor_big, Weight::foot_small);
        _kin_contact_list.erase(_kin_contact_list.begin());
    }

    _sp = Cheetah_StateProvider<T>::getStateProvider();

    _dir_command[0] = 0.;
    _dir_command[1] = 0.;
    printf("[Two Leg Swing Ctrl] Constructed\n");
}

template <typename T>
void TwoLegSwingCtrl<T>::_SetContact(const size_t & cp_idx, 
        const T & upper_lim, const T & rf_weight, const T & rf_weight_z, const T & foot_weight){

    ((SingleContact<T>*)Ctrl::contact_list_[cp_idx])->setMaxFz(upper_lim);
    for(size_t i(0); i<3; ++i){
        wblc_data_->W_rf_[i + 3*cp_idx] = rf_weight;
        wblc_data_->W_xddot_[i + 3*cp_idx] = foot_weight;
    }
    wblc_data_->W_rf_[2 + 3*cp_idx] = rf_weight_z;
}


template <typename T>
TwoLegSwingCtrl<T>::~TwoLegSwingCtrl(){
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
void TwoLegSwingCtrl<T>::OneStep(void* _cmd){
    Ctrl::_PreProcessing_Command();
    Ctrl::state_machine_time_ = _sp->curr_time_ - ctrl_start_time_;

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
void TwoLegSwingCtrl<T>::_compute_torque_wblc(DVec<T> & gamma){
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
void TwoLegSwingCtrl<T>::_task_setup(){
    des_jpos_ = ini_jpos_;
    des_jvel_.setZero();
    des_jacc_.setZero();

    // Calculate IK for a desired height and orientation.
    if(!b_set_height_target_) { printf("No Height Command\n"); exit(0); }
    DVec<T> vel_des(3); vel_des.setZero();
    DVec<T> acc_des(3); acc_des.setZero();
    Vec3<T> des_pos; des_pos.setZero();
    for(size_t i(0); i<2; ++i){
        des_pos[i] = smooth_change(_ini_body_target[i], 
                _sp->_body_target[i], end_time_, Ctrl::state_machine_time_);
        vel_des[i] = smooth_change_vel(_ini_body_target[i], 
                _sp->_body_target[i], end_time_, Ctrl::state_machine_time_);
        acc_des[i] = smooth_change_acc(_ini_body_target[i], 
                _sp->_body_target[i], end_time_, Ctrl::state_machine_time_);

        //des_pos[i] = smooth_change(_ini_body_pos[i], (T)0., end_time_, Ctrl::state_machine_time_);
        //vel_des[i] = smooth_change_vel(_ini_body_pos[i], (T)0., end_time_, Ctrl::state_machine_time_);
        //acc_des[i] = smooth_change_acc(_ini_body_pos[i], (T)0., end_time_, Ctrl::state_machine_time_);
     }
    des_pos[2] = target_body_height_;

    body_pos_task_->UpdateTask(&(des_pos), vel_des, acc_des);

    // Set Desired Orientation
    Quat<T> des_quat; des_quat.setZero();
    des_quat[0] = 1.;

    DVec<T> ang_vel_des(body_ori_task_->getDim()); ang_vel_des.setZero();
    DVec<T> ang_acc_des(body_ori_task_->getDim()); ang_acc_des.setZero();
    body_ori_task_->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);

    // set Foot trajectory
    _GetSinusoidalSwingTrajectory(_foot_pos_ini1, _target_loc1, Ctrl::state_machine_time_,
            _foot_pos_des1, _foot_vel_des1, _foot_acc_des1);
    _GetSinusoidalSwingTrajectory(_foot_pos_ini2, _target_loc2, Ctrl::state_machine_time_,
            _foot_pos_des2, _foot_vel_des2, _foot_acc_des2);

    _cp_pos_task1->UpdateTask(&(_foot_pos_des1), _foot_vel_des1, _foot_acc_des1);
    _cp_pos_task2->UpdateTask(&(_foot_pos_des2), _foot_vel_des2, _foot_acc_des2);

    kin_wbc_->FindConfiguration(_sp->Q_, 
            Ctrl::task_list_, _kin_contact_list, 
            des_jpos_, des_jvel_, des_jacc_);

    //pretty_print(des_jpos_, std::cout, "des_jpos");
    //pretty_print(des_jvel_, std::cout, "des_jvel");
    //pretty_print(des_jacc_, std::cout, "des_jacc");
}

template<typename T>
void TwoLegSwingCtrl<T>::_GetSinusoidalSwingTrajectory(
        const Vec3<T> & ini, const Vec3<T> & fin, const T & t, 
        Vec3<T> & pos_des, DVec<T> & vel_des, DVec<T> & acc_des){
    
    for (size_t i(0); i<2; ++i){
        pos_des[i] = smooth_change(ini[i], fin[i], end_time_, t);
        vel_des[i] = smooth_change_vel(ini[i], fin[i], end_time_, t);
        acc_des[i] = smooth_change_acc(ini[i], fin[i], end_time_, t);
    }
    // for Z (height)
    double amp(_swing_height/2.);
    double omega ( 2.*M_PI /end_time_ );

    pos_des[2] = ini[2] + amp * (1-cos(omega * t));
    vel_des[2] = amp * omega * sin(omega * t);
    acc_des[2] = amp * omega * omega * cos(omega * t);
}

template <typename T>
void TwoLegSwingCtrl<T>::_contact_setup(){
    typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::contact_list_.begin();
    while(iter < Ctrl::contact_list_.end()){
        (*iter)->UpdateContactSpec();
        ++iter;
    }
}

template <typename T>
void TwoLegSwingCtrl<T>::FirstVisit(){
    ini_jpos_ = Ctrl::robot_sys_->_state.q;
    ctrl_start_time_ = _sp->curr_time_;
    _ini_body_pos = Ctrl::robot_sys_->_state.bodyPosition;

    _foot_pos_ini1 = Ctrl::robot_sys_->_pGC[_cp1]; 
    _foot_pos_ini2 = Ctrl::robot_sys_->_pGC[_cp2]; 

    _target_loc1 = _default_target_foot_loc_1;
    _target_loc2 = _default_target_foot_loc_2;
    
    _target_loc1 += _sp->_local_frame_global_pos;
    _target_loc2 += _sp->_local_frame_global_pos;
    _sp->_body_target = _sp->_local_frame_global_pos;
    
    //_target_loc1.head(2) += Ctrl::robot_sys_->_state.bodyPosition.head(2);
    //_target_loc2.head(2) += Ctrl::robot_sys_->_state.bodyPosition.head(2);


    _dir_command[0] = -0.22 * _sp->_dir_command[0];
    _dir_command[1] = 0.08 * _sp->_dir_command[1];

    _ini_body_target = _sp->_body_target;
    _sp->_body_target[0] += 0.4 *_dir_command[0];
    _sp->_body_target[1] += 0.6 * _dir_command[1];

    if(_sp->_dir_command[0] > 0.){
        _target_loc1[0] += 0.6*_dir_command[0];
        _target_loc2[0] += _dir_command[0];
    }else{
        _target_loc1[0] += _dir_command[0];
        _target_loc2[0] += 0.6*_dir_command[0];
    }

    _target_loc1[1] += _dir_command[1];
    _target_loc2[1] += _dir_command[1];

    //pretty_print(_target_loc1, std::cout, "target loc 1");
    //pretty_print(_target_loc2, std::cout, "target loc 2");
}

template <typename T>
void TwoLegSwingCtrl<T>::LastVisit(){
    _sp->_jpos_des_pre = des_jpos_;
    // printf("[LegSwingBody] End\n");
}

template <typename T>
bool TwoLegSwingCtrl<T>::EndOfPhase(){
    if(Ctrl::state_machine_time_ > end_time_){
        return true;
    }
    return false;
}

template <typename T>
void TwoLegSwingCtrl<T>::CtrlInitialization(const std::string & setting_file_name){
    ParamHandler handler(CheetahConfigPath + setting_file_name + ".yaml");

    // Feedback Gain
    std::vector<T> tmp_vec;

    handler.getVector<T>("default_target_foot_location_1", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        _default_target_foot_loc_1[i] = tmp_vec[i];
    }
    handler.getVector<T>("default_target_foot_location_2", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        _default_target_foot_loc_2[i] = tmp_vec[i];
    }

    handler.getValue<T>("swing_height", _swing_height);

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
void TwoLegSwingCtrl<T>::SetTestParameter(const std::string & test_file){
    ParamHandler handler(test_file);
    if(handler.getValue<T>("body_height", target_body_height_)){
        b_set_height_target_ = true;
    }
    handler.getValue<T>("swing_time", end_time_);
}

template class TwoLegSwingCtrl<double>;
template class TwoLegSwingCtrl<float>;
