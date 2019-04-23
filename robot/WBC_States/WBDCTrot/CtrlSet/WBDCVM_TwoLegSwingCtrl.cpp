#include "WBDCVM_TwoLegSwingCtrl.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <WBC_States/common/ContactSet/SingleContact.hpp>
#include <WBC_States/common/TaskSet/LinkPosTask.hpp>
#include <WBC_States/common/TaskSet/BodyPostureTask.hpp>

#include <WBC/WBDC/WBDC.hpp>
#include <WBC/WBDC/WBDC_Full.hpp>
#include <Utilities/utilities.h>
#include <WBC_States/WBDCTrot/WBDCTrotTest.hpp>

template <typename T>
WBDCVM_TwoLegSwingCtrl<T>::WBDCVM_TwoLegSwingCtrl(
        WBDCTrotTest<T> * test, const FloatingBaseModel<T>* robot, 
        size_t cp1, size_t cp2):
    Controller<T>(robot),
    _trot_test(test),
    _cp1(cp1), _cp2(cp2),
    _des_jpos(cheetah::num_act_joint),
    _des_jvel(cheetah::num_act_joint),
    _end_time(100.),
    _dim_contact(0),
    _ctrl_start_time(0.)
{
    _body_posture_task = new BodyPostureTask<T>(Ctrl::_robot_sys);
    Ctrl::_task_list.push_back(_body_posture_task);

    //_cp_pos_task1 = new LinkPosTask<T>(Ctrl::_robot_sys, _cp1, false);
    //_cp_pos_task2 = new LinkPosTask<T>(Ctrl::_robot_sys, _cp2, false);
    _cp_pos_task1 = new LinkPosTask<T>(Ctrl::_robot_sys, _cp1);
    _cp_pos_task2 = new LinkPosTask<T>(Ctrl::_robot_sys, _cp2);

    //Ctrl::_task_list.push_back(_cp_pos_task1);
    //Ctrl::_task_list.push_back(_cp_pos_task2);

    _foot_pos_ini1.setZero();
    _foot_pos_des1.setZero();
    _foot_vel_des1 = DVec<T>::Zero(3);
    _foot_acc_des1 = DVec<T>::Zero(3);

    _foot_pos_ini2.setZero();
    _foot_pos_des2.setZero();
    _foot_vel_des2 = DVec<T>::Zero(3);
    _foot_acc_des2 = DVec<T>::Zero(3);

    _fr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FR);
    _fl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FL);
    _hr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HR);
    _hl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HL);

    Ctrl::_contact_list.push_back(_fr_contact);
    Ctrl::_contact_list.push_back(_fl_contact);
    Ctrl::_contact_list.push_back(_hr_contact);
    Ctrl::_contact_list.push_back(_hl_contact);

    _wbdc = new WBDC<T>(cheetah::dim_config, Ctrl::_contact_list, Ctrl::_task_list);
    //_wbdc = new WBDC_Full<T>(cheetah::dim_config, Ctrl::_contact_list, Ctrl::_task_list);
    _wbdc_data = new WBDC_ExtraData<T>();

    for(size_t i(0); i<Ctrl::_contact_list.size(); ++i){
        _dim_contact += Ctrl::_contact_list[i]->getDim();
    }

    _wbdc_data->_W_contact = DVec<T>::Constant(_dim_contact, Weight::foot_big);
    _wbdc_data->_W_task = DVec<T>::Constant(_body_posture_task->getDim(), Weight::qddot_relax);
    _wbdc_data->_W_rf = DVec<T>::Constant(_dim_contact, Weight::tan_small);

    int idx_offset(0);
    for(size_t i(0); i<Ctrl::_contact_list.size(); ++i){
        _wbdc_data->_W_rf[idx_offset + Ctrl::_contact_list[i]->getFzIndex()]= Weight::nor_small;
        idx_offset += Ctrl::_contact_list[i]->getDim();
    }

    
    if(_cp1 == linkID::HL || _cp2 == linkID::HL){
        _SetContact(3, 0.0001, Weight::tan_big, Weight::nor_big, Weight::foot_small);
    }
    if(_cp1 == linkID::HR || _cp2 == linkID::HR){
        _SetContact(2, 0.0001, Weight::tan_big, Weight::nor_big, Weight::foot_small);
    }
    if(_cp1 == linkID::FL || _cp2 == linkID::FL){
        _SetContact(1, 0.0001, Weight::tan_big, Weight::nor_big, Weight::foot_small);
    }
    if(_cp1 == linkID::FR || _cp2 == linkID::FR){
        _SetContact(0, 0.0001, Weight::tan_big, Weight::nor_big, Weight::foot_small);
    }

    _wbdc_data->_contact_pt_acc = DVec<T>::Zero(_dim_contact);
    _sp = StateProvider<T>::getStateProvider();

    printf("[Two Leg Swing Ctrl] Constructed\n");
}

template <typename T>
void WBDCVM_TwoLegSwingCtrl<T>::_SetContact(const size_t & cp_idx, 
        const T & upper_lim, const T & rf_weight, const T & rf_weight_z, const T & foot_weight){

    (void)foot_weight;
    ((SingleContact<T>*)Ctrl::_contact_list[cp_idx])->setMaxFz(upper_lim);
    for(size_t i(0); i<3; ++i){
        _wbdc_data->_W_rf[i + 3*cp_idx] = rf_weight;
    }
    _wbdc_data->_W_rf[2 + 3*cp_idx] = rf_weight_z;
}


template <typename T>
WBDCVM_TwoLegSwingCtrl<T>::~WBDCVM_TwoLegSwingCtrl(){
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
void WBDCVM_TwoLegSwingCtrl<T>::OneStep(void* _cmd){
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

            //((LegControllerCommand<T>*)_cmd)[leg].qDes[jidx] = 
                //_des_jpos[cheetah::num_leg_joint * leg + jidx];

                //((LegControllerCommand<T>*)_cmd)[leg].qdDes[jidx] = 
                //_des_jvel[cheetah::num_leg_joint * leg + jidx];

            ((LegControllerCommand<T>*)_cmd)[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
            ((LegControllerCommand<T>*)_cmd)[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
        }
    }
    Ctrl::_PostProcessing_Command();
}

template <typename T>
void WBDCVM_TwoLegSwingCtrl<T>::_compute_torque_wbdc(DVec<T> & gamma){
    // WBDC
    _wbdc->UpdateSetting(Ctrl::_A, Ctrl::_Ainv, Ctrl::_coriolis, Ctrl::_grav);
    _wbdc->MakeTorque(gamma, _wbdc_data);

    //_des_jvel.setZero();
    //pretty_print(_wbdc_data->Fr_, std::cout, "Fr");
}

template <typename T>
void WBDCVM_TwoLegSwingCtrl<T>::_task_setup(){
    Vec3<T> rpy_des; rpy_des.setZero();

    DVec<T> pos_des(7); 
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
    pos_des[6] = _target_body_height;
   
    // Orientation
    Mat3<T> Rot = rpyToRotMat(rpy_des);
    Eigen::Quaternion<T> eigen_quat(Rot.transpose());
    pos_des[0] = eigen_quat.w();
    pos_des[1] = eigen_quat.x();
    pos_des[2] = eigen_quat.y();
    pos_des[3] = eigen_quat.z();

    _body_posture_task->UpdateTask(&(pos_des), vel_des, acc_des);

    // set Foot trajectory
    _GetSinusoidalSwingTrajectory(_foot_pos_ini1, _target_loc1, Ctrl::_state_machine_time, 
            _foot_pos_des1, _foot_vel_des1, _foot_acc_des1);
    _GetSinusoidalSwingTrajectory(_foot_pos_ini2, _target_loc2, Ctrl::_state_machine_time, 
            _foot_pos_des2, _foot_vel_des2, _foot_acc_des2);

    //_GetBsplineSwingTrajectory(Ctrl::_state_machine_time, _foot_traj_1,
            //_foot_pos_des1, _foot_vel_des1, _foot_acc_des1);
    //_GetBsplineSwingTrajectory(Ctrl::_state_machine_time, _foot_traj_2, 
            //_foot_pos_des2, _foot_vel_des2, _foot_acc_des2);


    // Capture Point
    if(false){
    //if(true){
        Vec3<T> global_body_vel;
        Vec3<T> local_body_vel;
        for(size_t i(0);i<3; ++i){
            local_body_vel[i] = Ctrl::_robot_sys->_state.bodyVelocity[i+3];// Local
        }
        Quat<T> quat = Ctrl::_robot_sys->_state.bodyOrientation;
        Mat3<T> Rot_curr = ori::quaternionToRotationMatrix(quat);
        global_body_vel = Rot_curr.transpose()*local_body_vel;

        for(size_t i(0); i<2; ++i){

            _foot_pos_des1[i] += sqrt(_target_body_height/9.81) * 
                (global_body_vel[i] - _trot_test->_body_vel[i]);
            _foot_pos_des2[i] += sqrt(_target_body_height/9.81) * 
                (global_body_vel[i] - _trot_test->_body_vel[i]);

        }
    }
    _cp_pos_task1->UpdateTask(&(_foot_pos_des1), _foot_vel_des1, _foot_acc_des1);
    _cp_pos_task2->UpdateTask(&(_foot_pos_des2), _foot_vel_des2, _foot_acc_des2);

    DVec<T> op_cmd; 
    _cp_pos_task1->getCommand(op_cmd);
    _updateContactAcc(_cp1, op_cmd);

    _cp_pos_task2->getCommand(op_cmd);
    _updateContactAcc(_cp2, op_cmd);
}

template<typename T>
void WBDCVM_TwoLegSwingCtrl<T>::_updateContactAcc(const size_t & cp_idx, const DVec<T>& cmd){
    // Warning) The index depends on contact list sequence
    if(cp_idx == linkID::FR) _wbdc_data->_contact_pt_acc.segment(0, 3) = cmd;
    else if(cp_idx == linkID::FL) _wbdc_data->_contact_pt_acc.segment(3, 3) = cmd;
    else if(cp_idx == linkID::HR) _wbdc_data->_contact_pt_acc.segment(6, 3) = cmd;
    else if(cp_idx == linkID::HL) _wbdc_data->_contact_pt_acc.segment(9, 3) = cmd;
    else{ printf("[Two Leg Swing] Invalid contact idx\n"); }
}
template<typename T>
void WBDCVM_TwoLegSwingCtrl<T>::_GetSinusoidalSwingTrajectory(
        const Vec3<T> & ini, const Vec3<T> & fin, const T & t, 
        Vec3<T> & pos_des, DVec<T> & vel_des, DVec<T> & acc_des){
    
    for (size_t i(0); i<2; ++i){
        pos_des[i] = smooth_change(ini[i], fin[i], _end_time, t);
        vel_des[i] = smooth_change_vel(ini[i], fin[i], _end_time, t);
        acc_des[i] = smooth_change_acc(ini[i], fin[i], _end_time, t);
    }
    // for Z (height)
    double amp(_swing_height/2.);
    double omega ( 2.*M_PI /_end_time );

    pos_des[2] = ini[2] + amp * (1-cos(omega * t));
    vel_des[2] = amp * omega * sin(omega * t);
    acc_des[2] = amp * omega * omega * cos(omega * t);
}

template <typename T>
void WBDCVM_TwoLegSwingCtrl<T>::_contact_setup(){
    typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::_contact_list.begin();
    while(iter < Ctrl::_contact_list.end()){
        (*iter)->UpdateContactSpec();
        ++iter;
    }
}

template <typename T>
void WBDCVM_TwoLegSwingCtrl<T>::FirstVisit(){
    _ctrl_start_time = _sp->_curr_time;
    _ini_body_pos = Ctrl::_robot_sys->_state.bodyPosition;

    _foot_pos_ini1 = Ctrl::_robot_sys->_pGC[_cp1]; 
    _foot_pos_ini2 = Ctrl::_robot_sys->_pGC[_cp2]; 

    _target_loc1 = _default_target_foot_loc_1;
    _target_loc2 = _default_target_foot_loc_2;

    Vec3<T> cmd_vel = _trot_test->_body_vel;
    Vec3<T> next_body_pos;
    next_body_pos = _ini_body_pos + cmd_vel*_step_time; 
    Mat3<T> Rot = rpyToRotMat(_trot_test->_body_ori_rpy);

    computeFootLoc(Rot.transpose(), _default_target_foot_loc_1, _step_time, next_body_pos, 
            cmd_vel, 
            _trot_test->_body_ang_vel, _target_loc1);

    computeFootLoc(Rot.transpose(), _default_target_foot_loc_2, _step_time, next_body_pos, 
            cmd_vel, 
            _trot_test->_body_ang_vel, _target_loc2);

    //_SetBspline(_foot_pos_ini1, _target_loc1, _foot_traj_1);
    //_SetBspline(_foot_pos_ini2, _target_loc2, _foot_traj_2);

    //pretty_print(Rot, std::cout, "Rot");
    //pretty_print(_trot_test->_body_pos, std::cout, "commanded body_pos");
    //pretty_print(next_body_pos, std::cout, "nx body_pos");
    //pretty_print(_trot_test->_body_vel, std::cout, "body vel");
    //pretty_print(_trot_test->_body_ang_vel, std::cout, "body ang vel");
    //pretty_print(_target_loc1, std::cout, "target loc 1");
    //pretty_print(_target_loc2, std::cout, "target loc 2");
}


template<typename T>
void WBDCVM_TwoLegSwingCtrl<T>::computeFootLoc(const Mat3<T> & Rot, const Vec3<T> & shoulder, 
        const T& step_time, 
        const Vec3<T> & body_pos, const Vec3<T> & body_vel, 
        const Vec3<T> & body_ang_vel, Vec3<T> & foot_loc){

    foot_loc = body_pos + Rot * shoulder
        +  step_time/2. *(body_vel + body_ang_vel.cross(Rot * shoulder));
    foot_loc += _target_body_height/ 9.81 * body_vel.cross(body_ang_vel);
    foot_loc[2] = 0.;
}

template<typename T>
void WBDCVM_TwoLegSwingCtrl<T>::_SetBspline(const Vec3<T> & st_pos, const Vec3<T> & des_pos, 
        BS_Basic<double, 3, 3, 1, 2, 2> & spline){
    // Trajectory Setup
    double init[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];
    Vec3<T> middle_pos;

    middle_pos = (st_pos + des_pos)/2.;
    middle_pos[2] = _swing_height + std::max(st_pos[2], des_pos[2]);

    // Initial and final position & velocity & acceleration
    for(int i(0); i<3; ++i){
        // Initial
        init[i] = st_pos[i];
        init[i+3] = 0.;
        init[i+6] = 0.;
        // Final
        fin[i] = des_pos[i];
        fin[i+3] = 0.;
        fin[i+6] = 0.;
        // Middle
        middle_pt[0][i] = middle_pos[i];
    }
    // TEST
    //fin[5] = -0.05;
    fin[8] = 5.;
    spline.SetParam(init, fin, middle_pt, _end_time);

    delete [] *middle_pt;
    delete [] middle_pt;    
}

template<typename T>
void WBDCVM_TwoLegSwingCtrl<T>::_GetBsplineSwingTrajectory(const T & t, 
        BS_Basic<double, 3, 3, 1, 2, 2> & spline,
        Vec3<T> & pos_des, DVec<T> & vel_des, DVec<T> & acc_des){

    double pos[3];
    double vel[3];
    double acc[3];
    
    spline.getCurvePoint(t, pos);
    spline.getCurveDerPoint(t, 1, vel);
    spline.getCurveDerPoint(t, 2, acc);

    for(int i(0); i<3; ++i){
        pos_des[i] = pos[i];
        vel_des[i] = vel[i];
        acc_des[i] = acc[i];
    }
}
 

template <typename T>
void WBDCVM_TwoLegSwingCtrl<T>::LastVisit(){
    // printf("[LegSwingBody] End\n");
}

template <typename T>
bool WBDCVM_TwoLegSwingCtrl<T>::EndOfPhase(){
    if(Ctrl::_state_machine_time > (_end_time-2.*Test<T>::dt)){
        return true;
    }
    return false;
}

template <typename T>
void WBDCVM_TwoLegSwingCtrl<T>::CtrlInitialization(const std::string & category_name){
    ParamHandler handler(_test_file_name);
    std::vector<T> tmp_vec;
    _param_handler->getVector<T>(category_name, "default_target_foot_location_1", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        _default_target_foot_loc_1[i] = tmp_vec[i];
    }
    _param_handler->getVector<T>(category_name, "default_target_foot_location_2", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        _default_target_foot_loc_2[i] = tmp_vec[i];
    }
    _param_handler->getValue<T>(category_name, "swing_height", _swing_height);

    _param_handler->getVector<T>(category_name, "landing_offset", tmp_vec);
    //pretty_print(tmp_vec, "landing offset");
    for(size_t i(0); i<3; ++i) _landing_offset[i] = tmp_vec[i];

    _param_handler->getVector<T>(category_name, "foot_Kp", tmp_vec);
    for(size_t i(0); i<3; ++i) { 
        ((LinkPosTask<T>*)_cp_pos_task1)->_Kp[i] = tmp_vec[i]; 
        ((LinkPosTask<T>*)_cp_pos_task2)->_Kp[i] = tmp_vec[i]; 
    }

    _param_handler->getVector<T>(category_name, "foot_Kd", tmp_vec);
    for(size_t i(0); i<3; ++i) { 
        ((LinkPosTask<T>*)_cp_pos_task1)->_Kd[i] = tmp_vec[i]; 
        ((LinkPosTask<T>*)_cp_pos_task2)->_Kd[i] = tmp_vec[i]; 
    }
}


template <typename T>
void WBDCVM_TwoLegSwingCtrl<T>::SetTestParameter(const std::string & test_file){
    _param_handler = new ParamHandler(test_file);
    _param_handler->getValue<T>("body_height", _target_body_height);
    _param_handler->getValue<T>("swing_time", _end_time);

    std::vector<T> tmp_vec;
    _param_handler->getVector<T>("body_posture_Kp", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){ 
        ((BodyPostureTask<T>*)_body_posture_task)->_Kp[i] = tmp_vec[i]; 
    }
    _param_handler->getVector<T>("body_posture_Kd", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){ 
        ((BodyPostureTask<T>*)_body_posture_task)->_Kd[i] = tmp_vec[i]; 
    }

    _step_time = 0.;
    _step_time += _end_time;
    
    T tmp_value;
    _param_handler->getValue<T>("transition_time", tmp_value);
    _step_time += tmp_value;
    _step_time += tmp_value;
    
    _param_handler->getValue<T>("stance_time", tmp_value);
    _step_time += tmp_value;
    
    // Joint level feedback gain
    _param_handler->getVector<T>("Kp_joint", _Kp_joint);
    _param_handler->getVector<T>("Kd_joint", _Kd_joint);

}

template class WBDCVM_TwoLegSwingCtrl<double>;
template class WBDCVM_TwoLegSwingCtrl<float>;
