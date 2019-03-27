#include "WBDCVM_TwoContactTransCtrl.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <WBC_States/common/ContactSet/SingleContact.hpp>
#include <WBC_States/common/TaskSet/BodyPostureTask.hpp>

#include <WBC/WBDC/WBDC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <WBC_States/WBDCTrot/WBDCTrotTest.hpp>

template <typename T>
WBDCVM_TwoContactTransCtrl<T>::WBDCVM_TwoContactTransCtrl(
        WBDCTrotTest<T> * test, const FloatingBaseModel<T>* robot, 
        size_t cp1, size_t cp2, int transit_dir):
    Controller<T>(robot),
    _trot_test(test),
    _cp1(cp1), _cp2(cp2), _transit_dir(transit_dir),
    _des_jpos(cheetah::num_act_joint),
    _des_jvel(cheetah::num_act_joint),
    _end_time(100.),
    _dim_contact(0),
    _ctrl_start_time(0.)
{
    _body_posture_task = new BodyPostureTask<T>(Ctrl::_robot_sys);

    Ctrl::_task_list.push_back(_body_posture_task);

    // Pushback sequence is important !!
    _fr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FR);
    _fl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FL);
    _hr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HR);
    _hl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HL);

    Ctrl::_contact_list.push_back(_fr_contact);
    Ctrl::_contact_list.push_back(_fl_contact);
    Ctrl::_contact_list.push_back(_hr_contact);
    Ctrl::_contact_list.push_back(_hl_contact);

    _wbdc = new WBDC<T>(cheetah::dim_config, Ctrl::_contact_list, Ctrl::_task_list);
    _wbdc_data = new WBDC_ExtraData<T>();

    for(size_t i(0); i<Ctrl::_contact_list.size(); ++i){
        _dim_contact += Ctrl::_contact_list[i]->getDim();
    }
    _wbdc_data->_W_contact = DVec<T>::Constant(_dim_contact, Weight::foot_big);
    _wbdc_data->_W_task = DVec<T>::Constant(_body_posture_task->getDim(), Weight::qddot_relax);
    _wbdc_data->_W_rf = DVec<T>::Constant(_dim_contact, Weight::tan_small);

    int idx_offset(0);
    for(size_t i(0); i<Ctrl::_contact_list.size(); ++i){
        _wbdc_data->_W_rf[idx_offset + Ctrl::_contact_list[i]->getFzIndex()] =
            Weight::nor_small;
        idx_offset += Ctrl::_contact_list[i]->getDim();
    }

    _wbdc_data->_contact_pt_acc = DVec<T>::Zero(_dim_contact);
    _sp = StateProvider<T>::getStateProvider();

    printf("[Two Contact Transition Ctrl] Constructed\n");
}

template <typename T>
WBDCVM_TwoContactTransCtrl<T>::~WBDCVM_TwoContactTransCtrl(){
    delete _wbdc;
    delete _wbdc_data;

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
void WBDCVM_TwoContactTransCtrl<T>::OneStep(void* _cmd){
    Ctrl::_PreProcessing_Command();
    Ctrl::_state_machine_time = _sp->_curr_time - _ctrl_start_time;

    DVec<T> gamma;
    _contact_setup();
    _task_setup();
    _compute_torque_wbdc(gamma);

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
void WBDCVM_TwoContactTransCtrl<T>::_compute_torque_wbdc(DVec<T> & gamma){
    // WBDC
    _wbdc->UpdateSetting(Ctrl::_A, Ctrl::_Ainv, Ctrl::_coriolis, Ctrl::_grav);
    _wbdc->MakeTorque(gamma, _wbdc_data);

    _trot_test->_vm_qdot += _wbdc_data->_qddot * _trot_test->dt;
    _trot_test->_vm_q.head(cheetah::dim_config) += _trot_test->_vm_qdot * _trot_test->dt;

    _des_jpos = _trot_test->_vm_q.segment(6, cheetah::num_act_joint);
    _des_jvel = _trot_test->_vm_qdot.segment(6, cheetah::num_act_joint);

   //pretty_print(_wbdc_data->Fr_, std::cout, "reaction force");
}

template <typename T>
void WBDCVM_TwoContactTransCtrl<T>::_task_setup(){
    Vec3<T> rpy_des; rpy_des.setZero();

    DVec<T> pos_des(7); pos_des.setZero();
    DVec<T> vel_des(6); vel_des.setZero();
    DVec<T> acc_des(6); acc_des.setZero();

    for(size_t i(0); i<3; ++i){
        rpy_des[i] = _trot_test->_body_ori_rpy[i];
        // TODO : Frame must coincide. Currently, it's not
        vel_des[i] = _trot_test->_body_ang_vel[i];

        pos_des[i + 4] = _trot_test->_body_pos[i];
        vel_des[i + 3] = _trot_test->_body_vel[i];
        acc_des[i + 3] = _trot_test->_body_acc[i];
    }
    Mat3<T> Rot = rpyToRotMat(rpy_des);
    Eigen::Quaternion<T> eigen_quat(Rot.transpose());
    pos_des[0] = eigen_quat.w();
    pos_des[1] = eigen_quat.x();
    pos_des[2] = eigen_quat.y();
    pos_des[3] = eigen_quat.z();

    _body_posture_task->UpdateTask(&(pos_des), vel_des, acc_des);
}

template <typename T>
void WBDCVM_TwoContactTransCtrl<T>::_contact_setup(){
    T alpha = 0.5 * (1-cos(M_PI * Ctrl::_state_machine_time/_end_time)); // 0 -> 1
    T upper_lim, rf_weight, rf_weight_z, foot_weight;

    if(_transit_dir > 0){ // Decrease reaction force & Increase full acceleration
        upper_lim = _max_rf_z + alpha*(_min_rf_z - _max_rf_z);
        rf_weight   = (1.-alpha)*Weight::tan_small  + alpha*Weight::tan_big;
        rf_weight_z = (1.-alpha)*Weight::nor_small + alpha*Weight::nor_big;
        foot_weight = (1.-alpha)*Weight::foot_big   + alpha*Weight::foot_small;
    } else {
        upper_lim = _min_rf_z + alpha*(_max_rf_z - _min_rf_z);
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
void WBDCVM_TwoContactTransCtrl<T>::_SetContact(const size_t & cp_idx, 
        const T & upper_lim, const T & rf_weight, const T & rf_weight_z, const T & foot_weight){

    ((SingleContact<T>*)Ctrl::_contact_list[cp_idx])->setMaxFz(upper_lim);
    for(size_t i(0); i<3; ++i){
        _wbdc_data->_W_rf[i + 3*cp_idx] = rf_weight;
        _wbdc_data->_W_contact[i + 3*cp_idx] = foot_weight;
    }
    _wbdc_data->_W_rf[2 + 3*cp_idx] = rf_weight_z;
}

template <typename T>
void WBDCVM_TwoContactTransCtrl<T>::FirstVisit(){
    _ctrl_start_time = _sp->_curr_time;
    _ini_body_pos = Ctrl::_robot_sys->_state.bodyPosition;
}

template <typename T>
void WBDCVM_TwoContactTransCtrl<T>::LastVisit(){
    // printf("[ContactTransBody] End\n");
}

template <typename T>
bool WBDCVM_TwoContactTransCtrl<T>::EndOfPhase(){
    if(Ctrl::_state_machine_time > (_end_time-2.*Test<T>::dt)){
        return true;
    }
    return false;
}

template <typename T>
void WBDCVM_TwoContactTransCtrl<T>::CtrlInitialization(const std::string & category_name){
    //ParamHandler handler(CheetahConfigPath + setting_file_name + ".yaml");
    ParamHandler handler(_test_file_name);
    handler.getValue<T>(category_name, "max_rf_z", _max_rf_z);
    handler.getValue<T>(category_name, "min_rf_z", _min_rf_z);
}

template <typename T>
void WBDCVM_TwoContactTransCtrl<T>::SetTestParameter(const std::string & test_file){
    _test_file_name = test_file;
    ParamHandler handler(_test_file_name);
    handler.getValue<T>("body_height", _body_height_cmd);
    handler.getValue<T>("transition_time", _end_time);
}


template class WBDCVM_TwoContactTransCtrl<double>;
template class WBDCVM_TwoContactTransCtrl<float>;
