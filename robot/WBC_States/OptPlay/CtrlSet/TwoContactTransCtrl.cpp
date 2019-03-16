#include "TwoContactTransCtrl.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <WBC_States/common/ContactSet/SingleContact.hpp>
#include <WBC_States/common/TaskSet/LinkPosTask.hpp>
#include <WBC_States/common/TaskSet/BodyOriTask.hpp>
#include <WBC_States/common/TaskSet/BodyPosTask.hpp>
#include <WBC_States/OptPlay/OptInterpreter.hpp>

#include <WBC/WBLC/KinWBC.hpp>
#include <WBC/WBLC/WBLC.hpp>
#include <ParamHandler/ParamHandler.hpp>


template <typename T>
TwoContactTransCtrl<T>::TwoContactTransCtrl(const FloatingBaseModel<T>* robot, 
        size_t cp1, size_t cp2, int transit_dir):
    Controller<T>(robot),
    _cp1(cp1), _cp2(cp2), _transit_dir(transit_dir),
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
    body_pos_task_ = new BodyPosTask<T>(Ctrl::_robot_sys);
    body_ori_task_ = new BodyOriTask<T>(Ctrl::_robot_sys);

    Ctrl::_task_list.push_back(body_ori_task_);
    Ctrl::_task_list.push_back(body_pos_task_);

    // Pushback sequence is important !!
    fr_contact_ = new SingleContact<T>(Ctrl::_robot_sys, linkID::FR);
    fl_contact_ = new SingleContact<T>(Ctrl::_robot_sys, linkID::FL);
    hr_contact_ = new SingleContact<T>(Ctrl::_robot_sys, linkID::HR);
    hl_contact_ = new SingleContact<T>(Ctrl::_robot_sys, linkID::HL);

    Ctrl::_contact_list.push_back(fr_contact_);
    Ctrl::_contact_list.push_back(fl_contact_);
    Ctrl::_contact_list.push_back(hr_contact_);
    Ctrl::_contact_list.push_back(hl_contact_);

    kin_wbc_ = new KinWBC<T>(cheetah::dim_config);
    wblc_ = new WBLC<T>(cheetah::dim_config, Ctrl::_contact_list);
    wblc_data_ = new WBLC_ExtraData<T>();

    for(size_t i(0); i<Ctrl::_contact_list.size(); ++i){
        dim_contact_ += Ctrl::_contact_list[i]->getDim();
    }

    wblc_data_->W_qddot_ = DVec<T>::Constant(cheetah::dim_config, Weight::qddot_relax);
    wblc_data_->W_rf_ = DVec<T>::Constant(dim_contact_, Weight::tan_small);
    wblc_data_->W_xddot_ = DVec<T>::Constant(dim_contact_, Weight::foot_big);


    int idx_offset(0);
    for(size_t i(0); i<Ctrl::_contact_list.size(); ++i){
        wblc_data_->W_rf_[idx_offset + Ctrl::_contact_list[i]->getFzIndex()] =
            Weight::nor_small;
        idx_offset += Ctrl::_contact_list[i]->getDim();
    }

    // torque limit default setting
    wblc_data_->tau_min_ = DVec<T>::Constant(cheetah::num_act_joint, -150.);
    wblc_data_->tau_max_ = DVec<T>::Constant(cheetah::num_act_joint, 150.);

    _sp = StateProvider<T>::getStateProvider();

    printf("[Two Contact Transition Ctrl] Constructed\n");
}

template <typename T>
TwoContactTransCtrl<T>::~TwoContactTransCtrl(){
    delete wblc_;
    delete kin_wbc_;
    delete wblc_data_;

    typename std::vector<Task<T>*>::iterator iter = Ctrl::_task_list.begin();
    while(iter < Ctrl::_task_list.end()){
        delete (*iter);
        ++iter;
    }
    Ctrl::_task_list.clear();

    typename std::vector<ContactSpec<T>*>::iterator iter2 = Ctrl::_contact_list.begin();
    while(iter2 < Ctrl::_contact_list.end()){
        delete (*iter2);
        ++iter2;
    }
    Ctrl::_contact_list.clear();
}

template <typename T>
void TwoContactTransCtrl<T>::OneStep(void* _cmd){
    Ctrl::_PreProcessing_Command();
    Ctrl::_state_machine_time = _sp->_curr_time - ctrl_start_time_;

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
void TwoContactTransCtrl<T>::_compute_torque_wblc(DVec<T> & gamma){
    // WBLC
    wblc_->UpdateSetting(Ctrl::_A, Ctrl::_Ainv, Ctrl::_coriolis, Ctrl::_grav);
    DVec<T> des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - Ctrl::_robot_sys->_state.q)
        + Kd_.cwiseProduct(des_jvel_ - Ctrl::_robot_sys->_state.qd);

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, 
            gamma, wblc_data_);

   //pretty_print(wblc_data_->Fr_, std::cout, "reaction force");
}

template <typename T>
void TwoContactTransCtrl<T>::_task_setup(){
    des_jpos_ = Ctrl::_robot_sys->_state.q;
    des_jvel_.setZero();
    des_jacc_.setZero();
    
    // Calculate IK for a desired height and orientation.
    Vec3<T> pos_des; 
    DVec<T> vel_des(3); vel_des.setZero();
    DVec<T> acc_des(3); acc_des.setZero();

    OptInterpreter<T>::getOptInterpreter()->updateBodyTarget(
            _sp->_curr_time, pos_des, vel_des, acc_des);

   body_pos_task_->UpdateTask(&(pos_des), vel_des, acc_des);

    // Set Desired Orientation
    Vec3<T> rpy_des;
    OptInterpreter<T>::getOptInterpreter()->updateBodyOriTarget(
            _sp->_curr_time, rpy_des);
    Quat<T> des_quat; des_quat.setZero();
    Mat3<T> Rot = rpyToRotMat(rpy_des);
    Eigen::Quaternion<T> eigen_quat(Rot.transpose());
    des_quat[0] = eigen_quat.w();
    des_quat[1] = eigen_quat.x();
    des_quat[2] = eigen_quat.y();
    des_quat[3] = eigen_quat.z();

    DVec<T> ang_vel_des(body_ori_task_->getDim()); ang_vel_des.setZero();
    DVec<T> ang_acc_des(body_ori_task_->getDim()); ang_acc_des.setZero();
    body_ori_task_->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);

    kin_wbc_->FindConfiguration(_sp->_Q, 
            Ctrl::_task_list, Ctrl::_contact_list, 
            des_jpos_, des_jvel_, des_jacc_);

    //if(_transit_dir<0){
        //T alpha(Ctrl::_state_machine_time/end_time_); //0->1
        //des_jpos_ = alpha * des_jpos_ + (1.-alpha) * _sp->_jpos_des_pre;
    //}
    //pretty_print(des_jpos_, std::cout, "des_jpos");
    //pretty_print(des_jvel_, std::cout, "des_jvel");
    //pretty_print(des_jacc_, std::cout, "des_jacc");
}

template <typename T>
void TwoContactTransCtrl<T>::_contact_setup(){
    T alpha = 0.5 * (1-cos(M_PI * Ctrl::_state_machine_time/end_time_)); // 0 -> 1
    T upper_lim(100.);
    T rf_weight(100.);
    T rf_weight_z(100.);
    T foot_weight(1000.);

    if(_transit_dir > 0){ // Decrease reaction force & Increase full acceleration
        upper_lim = max_rf_z_ + alpha*(min_rf_z_ - max_rf_z_);
        rf_weight   = (1.-alpha)*Weight::tan_small  + alpha*Weight::tan_big;
        rf_weight_z = (1.-alpha)*Weight::nor_small + alpha*Weight::nor_big;
        foot_weight = (1.-alpha)*Weight::foot_big   + alpha*Weight::foot_small;
    } else {
        upper_lim = min_rf_z_ + alpha*(max_rf_z_ - min_rf_z_);
        rf_weight   = (1.-alpha)*Weight::tan_big  + alpha*Weight::tan_small;
        rf_weight_z = (1.-alpha)*Weight::nor_big + alpha*Weight::nor_small;
        foot_weight = (1.-alpha)*Weight::foot_small + alpha*Weight::foot_big;
    }

    if(_cp1 == linkID::FR || _cp2 == linkID::FR){
        _SetContact(0, upper_lim, rf_weight, rf_weight_z, foot_weight);
    }
    if(_cp1 == linkID::FL || _cp2 == linkID::FL){
        _SetContact(1, upper_lim, rf_weight, rf_weight_z, foot_weight);
    }

    if(_cp1 == linkID::HR || _cp2 == linkID::HR){
        _SetContact(2, upper_lim, rf_weight, rf_weight_z, foot_weight);
    }

    if(_cp1 == linkID::HL || _cp2 == linkID::HL){
        _SetContact(3, upper_lim, rf_weight, rf_weight_z, foot_weight);
    }


    typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::_contact_list.begin();
    while(iter < Ctrl::_contact_list.end()){
       (*iter)->UpdateContactSpec();
        ++iter;
    }
}

template <typename T>
void TwoContactTransCtrl<T>::_SetContact(const size_t & cp_idx, 
        const T & upper_lim, const T & rf_weight, const T & rf_weight_z, const T & foot_weight){

    ((SingleContact<T>*)Ctrl::_contact_list[cp_idx])->setMaxFz(upper_lim);
    for(size_t i(0); i<3; ++i){
        wblc_data_->W_rf_[i + 3*cp_idx] = rf_weight;
        wblc_data_->W_xddot_[i + 3*cp_idx] = foot_weight;
    }
        wblc_data_->W_rf_[2 + 3*cp_idx] = rf_weight_z;
}

template <typename T>
void TwoContactTransCtrl<T>::FirstVisit(){
    ctrl_start_time_ = _sp->_curr_time;
    ini_body_pos_ = Ctrl::_robot_sys->_state.bodyPosition;
}

template <typename T>
void TwoContactTransCtrl<T>::LastVisit(){
    // printf("[ContactTransBody] End\n");
}

template <typename T>
bool TwoContactTransCtrl<T>::EndOfPhase(){
    if(Ctrl::_state_machine_time > end_time_){
        return true;
    }
    return false;
}

template <typename T>
void TwoContactTransCtrl<T>::CtrlInitialization(const std::string & category_name){
    //ParamHandler handler(CheetahConfigPath + setting_file_name + ".yaml");
    ParamHandler handler(_test_file_name);
    handler.getValue<T>(category_name, "max_rf_z", max_rf_z_);
    handler.getValue<T>(category_name, "min_rf_z", min_rf_z_);
}

template <typename T>
void TwoContactTransCtrl<T>::SetTestParameter(const std::string & test_file){
    _test_file_name = test_file;
    ParamHandler handler(_test_file_name);
    if(handler.getValue<T>("body_height", _body_height_cmd)){
        b_set_height_target_ = true;
    }
    handler.getValue<T>("transition_time", end_time_);

    // Feedback Gain
    std::vector<T> tmp_vec;
    handler.getVector<T>("Kp", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        Kp_[i] = tmp_vec[i];
    }
    handler.getVector<T>("Kd", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        Kd_[i] = tmp_vec[i];
    }}

template class TwoContactTransCtrl<double>;
template class TwoContactTransCtrl<float>;
