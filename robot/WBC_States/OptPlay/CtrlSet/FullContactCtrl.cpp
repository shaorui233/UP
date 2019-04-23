#include "FullContactCtrl.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <WBC_States/common/ContactSet/SingleContact.hpp>
#include <WBC_States/common/TaskSet/LinkPosTask.hpp>
#include <WBC_States/common/TaskSet/BodyOriTask.hpp>
#include <WBC_States/common/TaskSet/BodyPosTask.hpp>

#include <WBC_States/OptPlay/OptInterpreter.hpp>

#include <WBC/WBLC/KinWBC.hpp>
#include <WBC/WBLC/WBLC.hpp>

template <typename T>
FullContactCtrl<T>::FullContactCtrl(const FloatingBaseModel<T>* robot):Controller<T>(robot),
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

   
    _fr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FR);
    _fl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FL);
    _hr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HR);
    _hl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HL);

    Ctrl::_contact_list.push_back(_fr_contact);
    Ctrl::_contact_list.push_back(_fl_contact);
    Ctrl::_contact_list.push_back(_hr_contact);
    Ctrl::_contact_list.push_back(_hl_contact);

    _kin_wbc = new KinWBC<T>(cheetah::dim_config);
    _wblc = new WBLC<T>(cheetah::dim_config, Ctrl::_contact_list);
    _wblc_data = new WBLC_ExtraData<T>();
 
    for(size_t i(0); i<Ctrl::_contact_list.size(); ++i){
        _dim_contact += Ctrl::_contact_list[i]->getDim();
    }

    _wblc_data->W_qddot_ = DVec<T>::Constant(cheetah::dim_config, 100.0);
    _wblc_data->W_rf_ = DVec<T>::Constant(_dim_contact, 1.);
    _wblc_data->W_xddot_ = DVec<T>::Constant(_dim_contact, 1000.0);

    int idx_offset(0);
    for(size_t i(0); i<Ctrl::_contact_list.size(); ++i){
        _wblc_data->W_rf_[idx_offset + Ctrl::_contact_list[i]->getFzIndex()]= 0.01;
        idx_offset += Ctrl::_contact_list[i]->getDim();
    }

    // torque limit default setting
    _wblc_data->tau_min_ = DVec<T>::Constant(cheetah::num_act_joint, -150.);
    _wblc_data->tau_max_ = DVec<T>::Constant(cheetah::num_act_joint, 150.);

    _sp = StateProvider<T>::getStateProvider();

    printf("[Body Control] Constructed\n");
}

template <typename T>
FullContactCtrl<T>::~FullContactCtrl(){
    delete _wblc;
    delete _wblc_data;
    delete _kin_wbc;
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
void FullContactCtrl<T>::OneStep(void* _cmd){
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

            ((LegControllerCommand<T>*)_cmd)[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
            ((LegControllerCommand<T>*)_cmd)[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
        }
    }
    Ctrl::_PostProcessing_Command();
}

template <typename T>
void FullContactCtrl<T>::_compute_torque_wblc(DVec<T> & gamma){
    // WBLC
    _wblc->UpdateSetting(Ctrl::_A, Ctrl::_Ainv, Ctrl::_coriolis, Ctrl::_grav);
    DVec<T> des_jacc_cmd = _des_jacc 
        + _Kp.cwiseProduct(_des_jpos - Ctrl::_robot_sys->_state.q)
        + _Kd.cwiseProduct(_des_jvel - Ctrl::_robot_sys->_state.qd);

    _wblc_data->_des_jacc_cmd = des_jacc_cmd;
    _wblc->MakeTorque(gamma, _wblc_data);
}

template <typename T>
void FullContactCtrl<T>::_task_setup(){
    _des_jpos.setZero();
    _des_jvel.setZero();
    _des_jacc.setZero();

    // Calculate IK for a desired height and orientation.
    Vec3<T> pos_des; pos_des.setZero();
    DVec<T> vel_des(3); vel_des.setZero();
    DVec<T> acc_des(3); acc_des.setZero();

    T run_time = _sp->_curr_time;
    if(_sp->_num_step<0){
        run_time = 0.;
    }
    OptInterpreter<T>::getOptInterpreter()->updateBodyTarget(
            run_time, pos_des, vel_des, acc_des);
    _body_pos_task->UpdateTask(&(pos_des), vel_des, acc_des);


    // Set Desired Orientation
    Vec3<T> rpy_des;
    OptInterpreter<T>::getOptInterpreter()->updateBodyOriTarget(
            run_time, rpy_des);

    Quat<T> des_quat; des_quat.setZero();

    Mat3<T> Rot = rpyToRotMat(rpy_des);
    Eigen::Quaternion<T> eigen_quat(Rot.transpose());
    des_quat[0] = eigen_quat.w();
    des_quat[1] = eigen_quat.x();
    des_quat[2] = eigen_quat.y();
    des_quat[3] = eigen_quat.z();

    DVec<T> ang_vel_des(_body_ori_task->getDim()); ang_vel_des.setZero();
    DVec<T> ang_acc_des(_body_ori_task->getDim()); ang_acc_des.setZero();
    _body_ori_task->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);

    //printf("run time: %f\n", run_time);
    //pretty_print(pos_des, std::cout, "pos");
    //pretty_print(rpy_des, std::cout, "rpy");
    _kin_wbc->FindConfiguration(_sp->_Q,
            Ctrl::_task_list, Ctrl::_contact_list, 
            _des_jpos, _des_jvel, _des_jacc);
}

template <typename T>
void FullContactCtrl<T>::_contact_setup(){
    typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::_contact_list.begin();
    while(iter < Ctrl::_contact_list.end()){
        (*iter)->UpdateContactSpec();
        ++iter;
    }
}

template <typename T>
void FullContactCtrl<T>::FirstVisit(){
    _ctrl_start_time = _sp->_curr_time;
}

template <typename T>
void FullContactCtrl<T>::LastVisit(){}

template <typename T>
bool FullContactCtrl<T>::EndOfPhase(){
    if(Ctrl::_state_machine_time > _end_time){
        return true;
    }
    return false;
}

template <typename T>
void FullContactCtrl<T>::CtrlInitialization(const std::string & category_name){
    (void)category_name;
}

template <typename T>
void FullContactCtrl<T>::SetTestParameter(const std::string & test_file){
    _param_handler = new ParamHandler(test_file);
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
    // Joint level feedback gain
    _param_handler->getVector<T>("Kp_joint", _Kp_joint);
    _param_handler->getVector<T>("Kd_joint", _Kd_joint);

}

template class FullContactCtrl<double>;
template class FullContactCtrl<float>;

