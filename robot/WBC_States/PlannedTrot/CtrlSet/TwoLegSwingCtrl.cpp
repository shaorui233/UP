#include "TwoLegSwingCtrl.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <WBC_States/common/ContactSet/SingleContact.hpp>
#include <WBC_States/common/TaskSet/LinkPosTask.hpp>
#include <WBC_States/common/TaskSet/BodyOriTask.hpp>
#include <WBC_States/common/TaskSet/BodyPosTask.hpp>

#include <WBC/WBLC/KinWBC.hpp>
#include <WBC/WBLC/WBLC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utilities/utilities.h>
#include <WBC_States/PlannedTrot/PlannedTrotTest.hpp>

namespace trot{  
template <typename T>
TwoLegSwingCtrl<T>::TwoLegSwingCtrl(PlannedTrotTest<T> * test, const FloatingBaseModel<T>* robot, 
        size_t cp1, size_t cp2):
    Controller<T>(robot),
    _trot_test(test),
    _cp1(cp1), _cp2(cp2),
    Kp_(cheetah::num_act_joint),
    Kd_(cheetah::num_act_joint),
    des_jpos_(cheetah::num_act_joint),
    des_jvel_(cheetah::num_act_joint),
    des_jacc_(cheetah::num_act_joint),
    end_time_(100.),
    dim_contact_(0),
    ctrl_start_time_(0.)
{
    body_pos_task_ = new BodyPosTask<T>(Ctrl::_robot_sys);
    body_ori_task_ = new BodyOriTask<T>(Ctrl::_robot_sys);
    Ctrl::_task_list.push_back(body_ori_task_);
    Ctrl::_task_list.push_back(body_pos_task_);

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
    printf("[Two Leg Swing Ctrl] Constructed\n");
}

template <typename T>
void TwoLegSwingCtrl<T>::_SetContact(const size_t & cp_idx, 
        const T & upper_lim, const T & rf_weight, const T & rf_weight_z, const T & foot_weight){

    ((SingleContact<T>*)Ctrl::_contact_list[cp_idx])->setMaxFz(upper_lim);
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
void TwoLegSwingCtrl<T>::OneStep(void* _cmd){
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
void TwoLegSwingCtrl<T>::_compute_torque_wblc(DVec<T> & gamma){
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
void TwoLegSwingCtrl<T>::_task_setup(){
    des_jpos_.setZero();
    des_jvel_.setZero();
    des_jacc_.setZero();

    // Calculate IK for a desired height and orientation.
    Vec3<T> pos_des; pos_des.setZero();
    DVec<T> vel_des(3); vel_des.setZero();
    DVec<T> acc_des(3); acc_des.setZero();
    Vec3<T> rpy_des; rpy_des.setZero();
    DVec<T> ang_vel_des(body_ori_task_->getDim()); ang_vel_des.setZero();
    
    for(size_t i(0); i<3; ++i){
        pos_des[i] = _trot_test->_body_pos[i];
        vel_des[i] = _trot_test->_body_vel[i];
        acc_des[i] = _trot_test->_body_acc[i];

        rpy_des[i] = _trot_test->_body_ori_rpy[i];
        // TODO : Frame must coincide. Currently, it's not
        ang_vel_des[i] = _trot_test->_body_ang_vel[i];
    }

    body_pos_task_->UpdateTask(&(pos_des), vel_des, acc_des);

    // Set Desired Orientation
    Quat<T> des_quat; des_quat.setZero();
    Mat3<T> Rot = rpyToRotMat(rpy_des);
    Eigen::Quaternion<T> eigen_quat(Rot.transpose());
    des_quat[0] = eigen_quat.w();
    des_quat[1] = eigen_quat.x();
    des_quat[2] = eigen_quat.y();
    des_quat[3] = eigen_quat.z();

    DVec<T> ang_acc_des(body_ori_task_->getDim()); ang_acc_des.setZero();
    body_ori_task_->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);

    // set Foot trajectory
    //_GetSinusoidalSwingTrajectory(_foot_pos_ini1, _target_loc1, Ctrl::_state_machine_time, 
            //_foot_pos_des1, _foot_vel_des1, _foot_acc_des1);
    //_GetSinusoidalSwingTrajectory(_foot_pos_ini2, _target_loc2, Ctrl::_state_machine_time, 
            //_foot_pos_des2, _foot_vel_des2, _foot_acc_des2);

    _GetBsplineSwingTrajectory(Ctrl::_state_machine_time, _foot_traj_1,
            _foot_pos_des1, _foot_vel_des1, _foot_acc_des1);
    _GetBsplineSwingTrajectory(Ctrl::_state_machine_time, _foot_traj_2, 
            _foot_pos_des2, _foot_vel_des2, _foot_acc_des2);

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
    typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::_contact_list.begin();
    while(iter < Ctrl::_contact_list.end()){
        (*iter)->UpdateContactSpec();
        ++iter;
    }
}

template <typename T>
void TwoLegSwingCtrl<T>::FirstVisit(){
    ctrl_start_time_ = _sp->_curr_time;
    _ini_body_pos = Ctrl::_robot_sys->_state.bodyPosition;

    _foot_pos_ini1 = Ctrl::_robot_sys->_pGC[_cp1]; 
    _foot_pos_ini2 = Ctrl::_robot_sys->_pGC[_cp2]; 

    _target_loc1 = _trot_test->_front_foot_loc;
    _target_loc2 = _trot_test->_hind_foot_loc;

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
    _trot_test->_jpos_des_pre = des_jpos_;
    // printf("[LegSwingBody] End\n");
}

template <typename T>
bool TwoLegSwingCtrl<T>::EndOfPhase(){
    if(Ctrl::_state_machine_time > (end_time_-2.*Test<T>::dt)){
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
    //pretty_print(tmp_vec, "default target foot");
    handler.getValue<T>(category_name, "swing_height", _swing_height);

    handler.getVector<T>(category_name, "landing_offset", tmp_vec);
    //pretty_print(tmp_vec, "landing offset");
    for(size_t i(0); i<3; ++i) _landing_offset[i] = tmp_vec[i];
}


template <typename T>
void TwoLegSwingCtrl<T>::SetTestParameter(const std::string & test_file){
    _test_file_name = test_file;
    ParamHandler handler(_test_file_name);
    handler.getValue<T>("body_height", _body_height_cmd);
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
};
