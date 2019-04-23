#include "WBLC_TwoLegSwingCtrl.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <WBC_States/common/ContactSet/SingleContact.hpp>
#include <WBC_States/common/TaskSet/LinkPosTask.hpp>
#include <WBC_States/common/TaskSet/BodyOriTask.hpp>
#include <WBC_States/common/TaskSet/BodyPosTask.hpp>

#include <WBC/WBLC/KinWBC.hpp>
#include <WBC/WBLC/WBLC.hpp>
#include <Utilities/utilities.h>
#include <WBC_States/WBLCTrot/WBLCTrotTest.hpp>

template <typename T>
WBLC_TwoLegSwingCtrl<T>::WBLC_TwoLegSwingCtrl(WBLCTrotTest<T> * test, const FloatingBaseModel<T>* robot, 
        size_t cp1, size_t cp2):
    Controller<T>(robot),
    _trot_test(test),
    _cp1(cp1), _cp2(cp2),
    Kp_(cheetah::num_act_joint),
    Kd_(cheetah::num_act_joint),
    des_jpos_(cheetah::num_act_joint),
    des_jvel_(cheetah::num_act_joint),
    des_jacc_(cheetah::num_act_joint),
    _end_time(100.),
    dim_contact_(0),
    ctrl_start_time_(0.)
{
    _body_pos_task = new BodyPosTask<T>(Ctrl::_robot_sys);
    _body_ori_task = new BodyOriTask<T>(Ctrl::_robot_sys);
    Ctrl::_task_list.push_back(_body_ori_task);
    Ctrl::_task_list.push_back(_body_pos_task);

    _cp_pos_task1 = new LinkPosTask<T>(Ctrl::_robot_sys, _cp1, false);
    _cp_pos_task2 = new LinkPosTask<T>(Ctrl::_robot_sys, _cp2, false);
    Ctrl::_task_list.push_back(_cp_pos_task1);
    Ctrl::_task_list.push_back(_cp_pos_task2);

    _foot_pos_ini1.setZero();
    _foot_pos_des1.setZero();
    _foot_vel_des1 = DVec<T>::Zero(3);
    _foot_acc_des1 = DVec<T>::Zero(3);

    _foot_pos_ini2.setZero();
    _foot_pos_des2.setZero();
    _foot_vel_des2 = DVec<T>::Zero(3);
    _foot_acc_des2 = DVec<T>::Zero(3);

    fr_contact_ = new SingleContact<T>(Ctrl::_robot_sys, linkID::FR);
    fl_contact_ = new SingleContact<T>(Ctrl::_robot_sys, linkID::FL);
    hr_contact_ = new SingleContact<T>(Ctrl::_robot_sys, linkID::HR);
    hl_contact_ = new SingleContact<T>(Ctrl::_robot_sys, linkID::HL);

    Ctrl::_contact_list.push_back(fr_contact_);
    Ctrl::_contact_list.push_back(fl_contact_);
    Ctrl::_contact_list.push_back(hr_contact_);
    Ctrl::_contact_list.push_back(hl_contact_);

    _kin_contact_list.push_back(fr_contact_);
    _kin_contact_list.push_back(fl_contact_);
    _kin_contact_list.push_back(hr_contact_);
    _kin_contact_list.push_back(hl_contact_);


    kin_wbc_ = new KinWBC<T>(cheetah::dim_config);
    wblc_ = new WBLC<T>(cheetah::dim_config, Ctrl::_contact_list);
    wblc_data_ = new WBLC_ExtraData<T>();

    for(size_t i(0); i<Ctrl::_contact_list.size(); ++i){
        dim_contact_ += Ctrl::_contact_list[i]->getDim();
    }

    wblc_data_->W_qddot_ = DVec<T>::Constant(cheetah::dim_config, Weight::qddot_relax);
    wblc_data_->W_qddot_.head(6) = DVec<T>::Constant(6, Weight::qddot_relax_virtual);
    wblc_data_->W_rf_ = DVec<T>::Constant(dim_contact_, Weight::tan_small);
    wblc_data_->W_xddot_ = DVec<T>::Constant(dim_contact_, Weight::foot_big);


    int idx_offset(0);
    for(size_t i(0); i<Ctrl::_contact_list.size(); ++i){
        wblc_data_->W_rf_[idx_offset + Ctrl::_contact_list[i]->getFzIndex()]= Weight::nor_small;
        idx_offset += Ctrl::_contact_list[i]->getDim();
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

    _sp = StateProvider<T>::getStateProvider();

    _dir_command[0] = 0.;
    _dir_command[1] = 0.;
    printf("[WBLC_Two Leg Swing Ctrl] Constructed\n");
}

template <typename T>
void WBLC_TwoLegSwingCtrl<T>::_SetContact(const size_t & cp_idx, 
        const T & upper_lim, const T & rf_weight, const T & rf_weight_z, const T & foot_weight){

    ((SingleContact<T>*)Ctrl::_contact_list[cp_idx])->setMaxFz(upper_lim);
    for(size_t i(0); i<3; ++i){
        wblc_data_->W_rf_[i + 3*cp_idx] = rf_weight;
        wblc_data_->W_xddot_[i + 3*cp_idx] = foot_weight;
    }
    wblc_data_->W_rf_[2 + 3*cp_idx] = rf_weight_z;
}


template <typename T>
WBLC_TwoLegSwingCtrl<T>::~WBLC_TwoLegSwingCtrl(){
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
void WBLC_TwoLegSwingCtrl<T>::OneStep(void* _cmd){
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
            ((LegControllerCommand<T>*)_cmd)[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
            ((LegControllerCommand<T>*)_cmd)[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
        }
    }
    Ctrl::_PostProcessing_Command();
}

template <typename T>
void WBLC_TwoLegSwingCtrl<T>::_compute_torque_wblc(DVec<T> & gamma){
    // WBLC
    wblc_->UpdateSetting(Ctrl::_A, Ctrl::_Ainv, Ctrl::_coriolis, Ctrl::_grav);
    DVec<T> des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - Ctrl::_robot_sys->_state.q)
        + Kd_.cwiseProduct(des_jvel_ - Ctrl::_robot_sys->_state.qd);

    wblc_data_->_des_jacc_cmd = des_jacc_cmd;
    wblc_->MakeTorque(gamma, wblc_data_);

    //pretty_print(wblc_data_->Fr_, std::cout, "Fr");
}

template <typename T>
void WBLC_TwoLegSwingCtrl<T>::_task_setup(){
    des_jpos_.setZero();
    des_jvel_.setZero();
    des_jacc_.setZero();

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
    des_quat = ori::rpyToQuat(rpy_des);

    DVec<T> ang_acc_des(_body_ori_task->getDim()); ang_acc_des.setZero();
    _body_ori_task->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);

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
    //if(false){
    if(true){
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

    kin_wbc_->FindConfiguration(_sp->_Q, 
            Ctrl::_task_list, _kin_contact_list, 
            des_jpos_, des_jvel_, des_jacc_);

    //pretty_print(des_jpos_, std::cout, "des_jpos");
    //pretty_print(des_jvel_, std::cout, "des_jvel");
    //pretty_print(des_jacc_, std::cout, "des_jacc");
}

template<typename T>
void WBLC_TwoLegSwingCtrl<T>::_GetSinusoidalSwingTrajectory(
        const Vec3<T> & ini, const Vec3<T> & fin, const T & t, 
        Vec3<T> & pos_des, DVec<T> & vel_des, DVec<T> & acc_des){
    
    for (size_t i(0); i<2; ++i){
        pos_des[i] = smooth_change(ini[i], fin[i], _end_time, t);
        vel_des[i] = smooth_change_vel(ini[i], fin[i], _end_time, t);
        acc_des[i] = smooth_change_acc(ini[i], fin[i], _end_time, t);
    }
    // for Z (height)
    T amp(_swing_height/2.);
    T omega ( 2.*M_PI /_end_time );

    pos_des[2] = ini[2] + amp * (1-cos(omega * t));
    vel_des[2] = amp * omega * sin(omega * t);
    acc_des[2] = amp * omega * omega * cos(omega * t);
}

template <typename T>
void WBLC_TwoLegSwingCtrl<T>::_contact_setup(){
    typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::_contact_list.begin();
    while(iter < Ctrl::_contact_list.end()){
        (*iter)->UpdateContactSpec();
        ++iter;
    }
}

template <typename T>
void WBLC_TwoLegSwingCtrl<T>::FirstVisit(){
    ctrl_start_time_ = _sp->_curr_time;
    _ini_body_pos = Ctrl::_robot_sys->_state.bodyPosition;

    _foot_pos_ini1 = Ctrl::_robot_sys->_pGC[_cp1]; 
    _foot_pos_ini2 = Ctrl::_robot_sys->_pGC[_cp2]; 

    _target_loc1 = _trot_test->_front_foot_loc;
    _target_loc2 = _trot_test->_hind_foot_loc;

    Vec3<T> cmd_vel = _trot_test->_body_vel;
    // TEST
    //SVec<T> curr_body_vel = Ctrl::_robot_sys->_state.bodyVelocity;
    //cmd_vel = curr_body_vel.tail(3);
    //Vec3<T> next_body_pos = _ini_body_pos + cmd_vel*_step_time; 
    Vec3<T> next_body_pos = _ini_body_pos + cmd_vel*_end_time; 
    //Vec3<T> next_body_pos = _trot_test->_body_pos + cmd_vel*_step_time; 
    Mat3<T> Rot = rpyToRotMat(_trot_test->_body_ori_rpy);
    computeFootLoc(Rot.transpose(), _default_target_foot_loc_1, _step_time, next_body_pos, 
            cmd_vel, 
            _trot_test->_body_ang_vel, _target_loc1);

    computeFootLoc(Rot.transpose(), _default_target_foot_loc_2, _step_time, next_body_pos, 
            cmd_vel, 
            _trot_test->_body_ang_vel, _target_loc2);

    _target_loc1 += _landing_offset;
    _target_loc2 += _landing_offset;


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
void WBLC_TwoLegSwingCtrl<T>::_SetBspline(const Vec3<T> & st_pos, const Vec3<T> & des_pos, 
        BS_Basic<T, 3, 3, 1, 2, 2> & spline){
    // Trajectory Setup
    T init[9];
    T fin[9];
    T** middle_pt = new T*[1];
    middle_pt[0] = new T[3];
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
void WBLC_TwoLegSwingCtrl<T>::_GetBsplineSwingTrajectory(const T & t, 
        BS_Basic<T, 3, 3, 1, 2, 2> & spline,
        Vec3<T> & pos_des, DVec<T> & vel_des, DVec<T> & acc_des){

    T pos[3];
    T vel[3];
    T acc[3];
    
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
void WBLC_TwoLegSwingCtrl<T>::LastVisit(){
    _trot_test->_jpos_des_pre = des_jpos_;
    // printf("[LegSwingBody] End\n");
}

template <typename T>
bool WBLC_TwoLegSwingCtrl<T>::EndOfPhase(){
    if(Ctrl::_state_machine_time > (_end_time-2.*Test<T>::dt)){
        return true;
    }
    return false;
}

template <typename T>
void WBLC_TwoLegSwingCtrl<T>::CtrlInitialization(const std::string & category_name){
    std::vector<T> tmp_vec;
    _param_handler->getVector<T>(category_name, "default_target_foot_location_1", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        _default_target_foot_loc_1[i] = tmp_vec[i];
    }
    _param_handler->getVector<T>(category_name, "default_target_foot_location_2", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        _default_target_foot_loc_2[i] = tmp_vec[i];
    }
    //pretty_print(tmp_vec, "default target foot");
    _param_handler->getValue<T>(category_name, "swing_height", _swing_height);

    _param_handler->getVector<T>(category_name, "landing_offset", tmp_vec);
    //pretty_print(tmp_vec, "landing offset");
    for(size_t i(0); i<3; ++i) _landing_offset[i] = tmp_vec[i];
}


template <typename T>
void WBLC_TwoLegSwingCtrl<T>::SetTestParameter(const std::string & test_file){
    _param_handler = new ParamHandler(test_file);
    _param_handler->getValue<T>("body_height", _target_body_height);
    _param_handler->getValue<T>("swing_time", _end_time);

    // Feedback Gain
    std::vector<T> tmp_vec;
    _param_handler->getVector<T>("Kp", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        Kp_[i] = tmp_vec[i];
    }
    _param_handler->getVector<T>("Kd", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        Kd_[i] = tmp_vec[i];
    }
    // Joint level feedback gain
    _param_handler->getVector<T>("Kp_joint", _Kp_joint);
    _param_handler->getVector<T>("Kd_joint", _Kd_joint);

    // Feedback gain for kinematic tasks
    _param_handler->getVector<T>("Kp_body_pos_kin", tmp_vec);
    for(size_t i(0); i<_body_pos_task->getDim(); ++i){
        ((BodyPosTask<T>*)_body_pos_task)->_Kp_kin[i] = tmp_vec[i];
    }
    _param_handler->getVector<T>("Kp_body_ori_kin", tmp_vec);
    for(size_t i(0); i<_body_ori_task->getDim(); ++i){
        ((BodyOriTask<T>*)_body_ori_task)->_Kp_kin[i] = tmp_vec[i];
    }

    _step_time = 0.;
    _step_time += _end_time;
    
    T tmp_value;
    _param_handler->getValue<T>("transition_time", tmp_value);
    _step_time += tmp_value;
    _step_time += tmp_value;
    
    _param_handler->getValue<T>("stance_time", tmp_value);
    _step_time += tmp_value;

    _param_handler->getValue<T>("step_time_ratio", tmp_value);
    _step_time *= tmp_value;
    // torque limit default setting
    _param_handler->getVector<T>("tau_lim", tmp_vec);
    wblc_data_->tau_min_ = DVec<T>::Constant(cheetah::num_act_joint, tmp_vec[0]);
    wblc_data_->tau_max_ = DVec<T>::Constant(cheetah::num_act_joint, tmp_vec[1]);

}

template<typename T>
void WBLC_TwoLegSwingCtrl<T>::computeFootLoc(const Mat3<T> & Rot, const Vec3<T> & shoulder, 
        const T& step_time, 
        const Vec3<T> & body_pos, const Vec3<T> & body_vel, 
        const Vec3<T> & body_ang_vel, Vec3<T> & foot_loc){

    foot_loc = body_pos + Rot * shoulder
        +  step_time/2. *(body_vel + body_ang_vel.cross(Rot * shoulder));
    foot_loc += _target_body_height/ 9.81 * body_vel.cross(body_ang_vel);
    foot_loc[2] = 0.;
}

template class WBLC_TwoLegSwingCtrl<double>;
template class WBLC_TwoLegSwingCtrl<float>;
