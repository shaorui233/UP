#include "WBLC_FullContactCtrl.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <WBC_States/common/ContactSet/SingleContact.hpp>
#include <WBC_States/common/TaskSet/LinkPosTask.hpp>
#include <WBC_States/common/TaskSet/BodyOriTask.hpp>
#include <WBC_States/common/TaskSet/BodyPosTask.hpp>

#include <WBC/WBLC/KinWBC.hpp>
#include <WBC/WBLC/WBLC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <WBC_States/WBLCTrot/WBLCTrotTest.hpp>


template <typename T>
WBLC_FullContactCtrl<T>::WBLC_FullContactCtrl(
        WBLCTrotTest<T>* trot_test, const FloatingBaseModel<T>* robot):Controller<T>(robot),
    _trot_test(trot_test),
    _Kp(cheetah::num_act_joint),
    _Kd(cheetah::num_act_joint),
    _des_jpos(cheetah::num_act_joint),
    _des_jvel(cheetah::num_act_joint),
    _des_jacc(cheetah::num_act_joint),
    _end_time(1000.0),
    _dim_contact(0),
    _ctrl_start_time(0.)
{
    _body_pos_task = new BodyPosTask<T>(Ctrl::_robot_sys);
    _body_ori_task = new BodyOriTask<T>(Ctrl::_robot_sys);

    Ctrl::_task_list.push_back(_body_ori_task);
    Ctrl::_task_list.push_back(_body_pos_task);

   
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
        _dim_contact += Ctrl::_contact_list[i]->getDim();
    }

    wblc_data_->W_qddot_ = DVec<T>::Constant(cheetah::dim_config, Weight::qddot_relax);
    wblc_data_->W_qddot_.head(6) = DVec<T>::Constant(6, Weight::qddot_relax_virtual);
    wblc_data_->W_rf_ = DVec<T>::Constant(_dim_contact, Weight::tan_small);
    wblc_data_->W_xddot_ = DVec<T>::Constant(_dim_contact, Weight::foot_big);

    int idx_offset(0);
    for(size_t i(0); i<Ctrl::_contact_list.size(); ++i){
        wblc_data_->W_rf_[idx_offset + Ctrl::_contact_list[i]->getFzIndex()]= Weight::nor_small;
        idx_offset += Ctrl::_contact_list[i]->getDim();
    }

    // torque limit default setting
    wblc_data_->tau_min_ = DVec<T>::Constant(cheetah::num_act_joint, -150.);
    wblc_data_->tau_max_ = DVec<T>::Constant(cheetah::num_act_joint, 150.);

    _sp = StateProvider<T>::getStateProvider();

    printf("[Body Control] Constructed\n");
}

template <typename T>
WBLC_FullContactCtrl<T>::~WBLC_FullContactCtrl(){
    delete wblc_;
    delete wblc_data_;
    delete kin_wbc_;
    delete _param_handler;

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
void WBLC_FullContactCtrl<T>::OneStep(void* _cmd){
    Ctrl::_PreProcessing_Command();
    Ctrl::_state_machine_time = _sp->_curr_time - _ctrl_start_time;

    DVec<T> gamma = DVec<T>::Zero(cheetah::num_act_joint);
    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for(size_t leg(0); leg<cheetah::num_leg; ++leg){
        for(size_t jidx(0); jidx<cheetah::num_leg_joint; ++jidx){
            ((LegControllerCommand<T>*)_cmd)[leg].tauFeedForward[jidx] 
                = gamma[cheetah::num_leg_joint * leg + jidx];

            ((LegControllerCommand<T>*)_cmd)[leg].qDes[jidx] = 
                _des_jpos[cheetah::num_leg_joint * leg + jidx];

            ((LegControllerCommand<T>*)_cmd)[leg].qdDes[jidx] = 
                _des_jvel[cheetah::num_leg_joint * leg + jidx];
        }
    }
    Ctrl::_PostProcessing_Command();
}

template <typename T>
void WBLC_FullContactCtrl<T>::_compute_torque_wblc(DVec<T> & gamma){
    // WBLC
    wblc_->UpdateSetting(Ctrl::_A, Ctrl::_Ainv, Ctrl::_coriolis, Ctrl::_grav);
    DVec<T> des_jacc_cmd = _des_jacc 
        + _Kp.cwiseProduct(_des_jpos - Ctrl::_robot_sys->_state.q)
        + _Kd.cwiseProduct(_des_jvel - Ctrl::_robot_sys->_state.qd);

    wblc_data_->_des_jacc_cmd = des_jacc_cmd;
    wblc_->MakeTorque(gamma, wblc_data_);
    //pretty_print(gamma, std::cout, "gamma");
}

template <typename T>
void WBLC_FullContactCtrl<T>::_task_setup(){
    _des_jpos.setZero();
    _des_jvel.setZero();
    _des_jacc.setZero();

    // Calculate IK for a desired height and orientation.
    Vec3<T> pos_des; pos_des.setZero();
    DVec<T> vel_des(3); vel_des.setZero();
    DVec<T> acc_des(3); acc_des.setZero();
    Vec3<T> rpy_des; rpy_des.setZero(); 
    DVec<T> ang_vel_des(_body_ori_task->getDim()); ang_vel_des.setZero();

    for(size_t i(0); i<3; ++i){
        pos_des[i] = _trot_test->_body_pos[i];
        vel_des[i] = _trot_test->_body_vel[i];
        acc_des[i] = _trot_test->_body_acc[i];

        rpy_des[i] = _trot_test->_body_ori_rpy[i];
        // TODO : Frame must coincide. Currently, it's not
        ang_vel_des[i] = _trot_test->_body_ang_vel[i];
    }

    _body_pos_task->UpdateTask(&(pos_des), vel_des, acc_des);

    // Set Desired Orientation
    Quat<T> des_quat; des_quat.setZero();

    Mat3<T> Rot = rpyToRotMat(rpy_des);
    Eigen::Quaternion<T> eigen_quat(Rot.transpose());
    des_quat[0] = eigen_quat.w();
    des_quat[1] = eigen_quat.x();
    des_quat[2] = eigen_quat.y();
    des_quat[3] = eigen_quat.z();


    DVec<T> ang_acc_des(_body_ori_task->getDim()); ang_acc_des.setZero();
    _body_ori_task->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);

    kin_wbc_->FindConfiguration(_sp->_Q,
            Ctrl::_task_list, Ctrl::_contact_list, 
            _des_jpos, _des_jvel, _des_jacc);
}

template <typename T>
void WBLC_FullContactCtrl<T>::_contact_setup(){
    typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::_contact_list.begin();
    while(iter < Ctrl::_contact_list.end()){
        (*iter)->UpdateContactSpec();
        ++iter;
    }
}

template <typename T>
void WBLC_FullContactCtrl<T>::FirstVisit(){
    _ctrl_start_time = _sp->_curr_time;
    ini_body_pos_ = Ctrl::_robot_sys->_state.bodyPosition;
}

template <typename T>
void WBLC_FullContactCtrl<T>::LastVisit(){}

template <typename T>
bool WBLC_FullContactCtrl<T>::EndOfPhase(){
    if(Ctrl::_state_machine_time > (_end_time-2.*Test<T>::dt)){
        return true;
    }
    return false;
}

template <typename T>
void WBLC_FullContactCtrl<T>::CtrlInitialization(const std::string & category_name){
    (void)category_name;
    ini_body_pos_ = Ctrl::_robot_sys->_state.bodyPosition;
}

template <typename T>
void WBLC_FullContactCtrl<T>::SetTestParameter(const std::string & test_file){
    _param_handler = new ParamHandler(test_file);
    if(_param_handler->getValue<T>("body_height", target_body_height_)){
    }
    _param_handler->getValue<T>("stance_time", _end_time);

    std::vector<T> tmp_vec;
    // Feedback Gain
    _param_handler->getVector<T>("Kp", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        _Kp[i] = tmp_vec[i];
    }
    _param_handler->getVector<T>("Kd", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        _Kd[i] = tmp_vec[i];
    }
    // torque limit default setting
    _param_handler->getVector<T>("tau_lim", tmp_vec);
    wblc_data_->tau_min_ = DVec<T>::Constant(cheetah::num_act_joint, tmp_vec[0]);
    wblc_data_->tau_max_ = DVec<T>::Constant(cheetah::num_act_joint, tmp_vec[1]);

}

template class WBLC_FullContactCtrl<double>;
template class WBLC_FullContactCtrl<float>;
