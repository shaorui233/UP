#include "BoundingCtrl.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <WBC_States/common/ContactSet/SingleContact.hpp>
#include <WBC_States/common/TaskSet/BodyPostureTask.hpp>

#include <WBC/WBIC/WBIC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <WBC_States/Bounding/BoundingTest.hpp>

template <typename T>
BoundingCtrl<T>::BoundingCtrl(
        BoundingTest<T>* bounding_test, const FloatingBaseModel<T>* robot):Controller<T>(robot),
    _bounding_test(bounding_test),
    _des_jpos(cheetah::num_act_joint),
    _des_jvel(cheetah::num_act_joint),
    _end_time(1000.0),
    _dim_contact(0),
    _ctrl_start_time(0.)
{
    _wbic = new WBIC<T>(cheetah::dim_config);
    _wbic_data = new WBIC_ExtraData<T>();

    // Task
    _body_posture_task = new BodyPostureTask<T>(Ctrl::_robot_sys);
    _wbic_data->_task_list.clear();
    _wbic_data->_task_list.push_back(_body_posture_task);

    // Constraints
    _fr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FR);
    _fl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FL);
    _hr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HR);
    _hl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HL);

    _wbic_data->_contact_list.clear();
    _wbic_data->_contact_list.push_back(_fr_contact);
    _wbic_data->_contact_list.push_back(_fl_contact);
    _wbic_data->_contact_list.push_back(_hr_contact);
    _wbic_data->_contact_list.push_back(_hl_contact);

    for(size_t i(0); i<_wbic_data->_contact_list.size(); ++i){
        _dim_contact += _wbic_data->_contact_list[i]->getDim();
    }

    _wbic_data->_W_floating = DVec<T>::Constant(6, Weight::qddot_relax);
    _wbic_data->_W_rf = DVec<T>::Constant(_dim_contact, Weight::tan_small);

    _sp = StateProvider<T>::getStateProvider();

    printf("[Bounding Control] Constructed\n");
}

template <typename T>
void BoundingCtrl<T>::OneStep(void* _cmd){
    Ctrl::_PreProcessing_Command();
    Ctrl::_state_machine_time = _sp->_curr_time - _ctrl_start_time;

    DVec<T> gamma = DVec<T>::Zero(cheetah::num_act_joint);
    _contact_setup();
    _task_setup();
    _compute_torque_wbic(gamma);

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
void BoundingCtrl<T>::_compute_torque_wbic(DVec<T> & gamma){
    // WBIC
    _wbic->UpdateSetting(Ctrl::_A, Ctrl::_Ainv, Ctrl::_coriolis, Ctrl::_grav);
    _wbic->MakeTorque(gamma, _wbic_data);
}

template <typename T>
void BoundingCtrl<T>::_task_setup(){
    Vec3<T> rpy_des; rpy_des.setZero();

    DVec<T> pos_des(7); pos_des.setZero();
    DVec<T> vel_des(6); vel_des.setZero();
    DVec<T> acc_des(6); acc_des.setZero();

    for(size_t i(0); i<3; ++i){
        rpy_des[i] = _bounding_test->_body_ori_rpy[i];
        // TODO : Frame must coincide. Currently, it's not
        vel_des[i] = _bounding_test->_body_ang_vel[i];

        pos_des[i + 4] = _bounding_test->_body_pos[i];
        vel_des[i + 3] = _bounding_test->_body_vel[i];
        acc_des[i + 3] = _bounding_test->_body_acc[i];
    }
    pos_des[6] = _target_body_height;
    // Orientation
    Mat3<T> Rot = rpyToRotMat(rpy_des);
    Eigen::Quaternion<T> eigen_quat(Rot.transpose());
    pos_des[0] = eigen_quat.w();
    pos_des[1] = eigen_quat.x();
    pos_des[2] = eigen_quat.y();
    pos_des[3] = eigen_quat.z();

    _body_posture_task->UpdateTask(&(pos_des), vel_des, acc_des);
}

template <typename T>
void BoundingCtrl<T>::_contact_setup(){
    typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::_contact_list.begin();
    while(iter < Ctrl::_contact_list.end()){
        (*iter)->UpdateContactSpec();
        ++iter;
    }
}

template <typename T>
void BoundingCtrl<T>::FirstVisit(){
    _ctrl_start_time = _sp->_curr_time;
}

template <typename T>
void BoundingCtrl<T>::LastVisit(){}

template <typename T>
bool BoundingCtrl<T>::EndOfPhase(){
    if(Ctrl::_state_machine_time > (_end_time-2.*Test<T>::dt)){
        return true;
    }
    return false;
}

template <typename T>
void BoundingCtrl<T>::CtrlInitialization(const std::string & category_name){
    (void)category_name;
}

template <typename T>
void BoundingCtrl<T>::SetTestParameter(const std::string & test_file){
    _param_handler = new ParamHandler(test_file);
    std::vector<T> tmp_vec;
    if(_param_handler->getValue<T>("body_height", _target_body_height)){
    }
    _param_handler->getValue<T>("stance_time", _end_time);

    _param_handler->getVector<T>("body_posture_Kp", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){ 
        ((BodyPostureTask<T>*)_body_posture_task)->_Kp[i] = tmp_vec[i]; 
    }
    _param_handler->getVector<T>("body_posture_Kd", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){ 
        ((BodyPostureTask<T>*)_body_posture_task)->_Kd[i] = tmp_vec[i]; 
    }
    // Joint level feedback gain
    if(!_param_handler->getVector<T>("Kp_joint", _Kp_joint)){
        printf("no Kp joint setting\n");
        exit(0);
    }
    _param_handler->getVector<T>("Kd_joint", _Kd_joint);

}


template <typename T>
BoundingCtrl<T>::~BoundingCtrl(){
    delete _wbic;
    delete _wbic_data;
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


template class BoundingCtrl<double>;
template class BoundingCtrl<float>;
