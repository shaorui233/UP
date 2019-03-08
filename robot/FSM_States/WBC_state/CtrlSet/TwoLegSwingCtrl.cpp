#include "TwoLegSwingCtrl.hpp"

#include <WBC_state/Cheetah_StateProvider.hpp>
#include <WBC_state/Cheetah_DynaCtrl_Definition.h>
#include <WBC_state/ContactSet/SingleContact.hpp>
#include <WBC_state/TaskSet/LinkPosTask.hpp>
#include <WBC_state/TaskSet/BodyOriTask.hpp>
#include <WBC_state/TaskSet/BodyPosTask.hpp>
#include <WBC_state/OptInterpreter.hpp>

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

    _cp_pos_task1 = new LinkPosTask<T>(Ctrl::robot_sys_, _cp1, false);
    _cp_pos_task2 = new LinkPosTask<T>(Ctrl::robot_sys_, _cp2, false);
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

    //pretty_print(wblc_data_->Fr_, std::cout, "Fr");
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
    Mat3<T> Rot ;
    if(_sp->_opt_play){
        OptInterpreter<T>::getOptInterpreter()->updateBodyTarget(
                _sp->curr_time_, _sp->_body_target, vel_des, acc_des);
        OptInterpreter<T>::getOptInterpreter()->updateBodyOriTarget(
                _sp->curr_time_, _sp->_target_ori_command);
        Rot = rpyToRotMat(_sp->_target_ori_command);

    }else{
        _sp->_body_target[2] = _body_height_cmd;
        Vec3<T> curr_rpy;
        for(size_t i(0); i<3; ++i){
            curr_rpy[i] = smooth_change(_prev_ori_command[i], _sp->_target_ori_command[i], 
                    end_time_, Ctrl::state_machine_time_);
        }
        Rot = rpyToRotMat(curr_rpy);
    }
    Vec3<T> des_pos = _sp->_body_target;
    body_pos_task_->UpdateTask(&(des_pos), vel_des, acc_des);

    // Set Desired Orientation
    Quat<T> des_quat; des_quat.setZero();
    Eigen::Quaternion<T> eigen_quat(Rot.transpose());
    des_quat[0] = eigen_quat.w();
    des_quat[1] = eigen_quat.x();
    des_quat[2] = eigen_quat.y();
    des_quat[3] = eigen_quat.z();

    DVec<T> ang_vel_des(body_ori_task_->getDim()); ang_vel_des.setZero();
    DVec<T> ang_acc_des(body_ori_task_->getDim()); ang_acc_des.setZero();
    body_ori_task_->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);

    // set Foot trajectory
    //_GetSinusoidalSwingTrajectory(_foot_pos_ini1, _target_loc1, Ctrl::state_machine_time_, 
            //_foot_pos_des1, _foot_vel_des1, _foot_acc_des1);
    //_GetSinusoidalSwingTrajectory(_foot_pos_ini2, _target_loc2, Ctrl::state_machine_time_, 
            //_foot_pos_des2, _foot_vel_des2, _foot_acc_des2);

    _GetBsplineSwingTrajectory(Ctrl::state_machine_time_, _foot_traj_1,
            _foot_pos_des1, _foot_vel_des1, _foot_acc_des1);
    _GetBsplineSwingTrajectory(Ctrl::state_machine_time_, _foot_traj_2, 
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


    if(_sp->_opt_play){
        for(int i(0); i<3; ++i){
        _target_loc1[i] = 
            OptInterpreter<T>::getOptInterpreter()->_foot_step_list[_sp->_num_step][i];
        _target_loc2[i] = 
            OptInterpreter<T>::getOptInterpreter()->_foot_step_list[_sp->_num_step][i+3];
        }
    }else{
        Vec3<T> body_target; body_target.setZero();
        //_target_loc1 += _sp->_local_frame_global_pos;
        //_target_loc2 += _sp->_local_frame_global_pos;
        _sp->_body_target = _sp->_local_frame_global_pos;

        _prev_ori_command = _sp->_target_ori_command;
        for(size_t i(0); i<3; ++i) 
            _sp->_target_ori_command[i] += 0.1*_sp->_ori_command[i];

        _sp->UpdateYawTargetRot(_sp->_target_ori_command[2]);
        _dir_command[0] = 0.10 * _sp->_dir_command[0];
        _dir_command[1] = 0.03 * _sp->_dir_command[1];

        _ini_body_target = _sp->_body_target;
        body_target[0] += 0.4 *_dir_command[0];
        body_target[1] += 0.6 * _dir_command[1];

        if(_sp->_dir_command[0] > 0.){
            _target_loc1[0] += _dir_command[0];
            _target_loc2[0] += 0.4*_dir_command[0];
        }else{
            _target_loc1[0] += 0.4 * _dir_command[0];
            _target_loc2[0] += _dir_command[0];
        }

        _target_loc1[1] += _dir_command[1];
        _target_loc2[1] += _dir_command[1];
        _sp->_body_target += _sp->_YawRot * body_target;
        _target_loc1 = (_sp->_YawRot*_target_loc1 + _sp->_body_target);
        _target_loc2 = (_sp->_YawRot*_target_loc2 + _sp->_body_target);
    }

    _target_loc1 += _landing_offset;
    _target_loc2 += _landing_offset;
    
    _SetBspline(_foot_pos_ini1, _target_loc1, _foot_traj_1);
    _SetBspline(_foot_pos_ini2, _target_loc2, _foot_traj_2);

    //pretty_print(_target_loc1, std::cout, "target loc 1");
    //pretty_print(_target_loc2, std::cout, "target loc 2");
}

template<typename T>
void TwoLegSwingCtrl<T>::_SetBspline(const Vec3<T> & st_pos, const Vec3<T> & des_pos, 
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
    spline.SetParam(init, fin, middle_pt, end_time_);

    delete [] *middle_pt;
    delete [] middle_pt;    
}

template<typename T>
void TwoLegSwingCtrl<T>::_GetBsplineSwingTrajectory(const T & t, 
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
void TwoLegSwingCtrl<T>::CtrlInitialization(const std::string & category_name){
    ParamHandler handler(_test_file_name);
    std::vector<T> tmp_vec;
    
    handler.getVector<T>(category_name, "default_target_foot_location_1", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        _default_target_foot_loc_1[i] = tmp_vec[i];
    }
    handler.getVector<T>(category_name, "default_target_foot_location_2", tmp_vec);
    for(size_t i(0); i<tmp_vec.size(); ++i){
        _default_target_foot_loc_2[i] = tmp_vec[i];
    }
    handler.getValue<T>(category_name, "swing_height", _swing_height);

    handler.getVector<T>(category_name, "landing_offset", tmp_vec);
    //pretty_print(tmp_vec, "landing offset");
    for(size_t i(0); i<3; ++i) _landing_offset[i] = tmp_vec[i];
}


template <typename T>
void TwoLegSwingCtrl<T>::SetTestParameter(const std::string & test_file){
    _test_file_name = test_file;
    ParamHandler handler(_test_file_name);
    if(handler.getValue<T>("body_height", _body_height_cmd)){
        b_set_height_target_ = true;
    }
    handler.getValue<T>("swing_time", end_time_);

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

template class TwoLegSwingCtrl<double>;
template class TwoLegSwingCtrl<float>;
