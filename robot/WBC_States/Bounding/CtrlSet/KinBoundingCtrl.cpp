#include "KinBoundingCtrl.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <WBC_States/common/ContactSet/SingleContact.hpp>
#include <WBC_States/common/TaskSet/BodyOriTask.hpp>
#include <WBC_States/common/TaskSet/BodyPosTask.hpp>
#include <WBC_States/common/TaskSet/JPosTask.hpp>
#include <WBC_States/Bounding/TaskSet/BodyXYTask.hpp>
#include <WBC_States/Bounding/TaskSet/LegHeightTask.hpp>
#include <WBC_States/Bounding/TaskSet/LocalPosTask.hpp>
#include <WBC_States/Bounding/TaskSet/SelectedJointTask.hpp>
#include <WBC_States/Bounding/TaskSet/BodyRyRzTask.hpp>
#include <WBC_States/Bounding/TaskSet/LocalRollTask.hpp>
#include <WBC_States/Bounding/TaskSet/LocalHeadPosTask.hpp>
#include <WBC_States/Bounding/TaskSet/LocalTailPosTask.hpp>

#include <WBC/WBLC/KinWBC.hpp>
#include <WBC/WBIC/WBIC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <WBC_States/Bounding/BoundingTest.hpp>
#include <Utilities/save_file.h>

template <typename T>
KinBoundingCtrl<T>::KinBoundingCtrl(
    BoundingTest<T>* bounding_test, const FloatingBaseModel<T>* robot):Controller<T>(robot),
  _bounding_test(bounding_test),
  _end_time(1000.0),
  _dim_contact(0),
  _ctrl_start_time(0.),
  _fr_foot_vel(3),
  _fr_foot_acc(3),
  _fl_foot_vel(3),
  _fl_foot_acc(3),
  _hr_foot_vel(3),
  _hr_foot_acc(3),
  _hl_foot_vel(3),
  _hl_foot_acc(3),
  _Fr_des(12),
  _Fr_result(12),
  _des_jpos(cheetah::num_act_joint),
  _des_jvel(cheetah::num_act_joint),
  _des_jacc(cheetah::num_act_joint)
{
  // Start from front swing & hind stance
  _b_first_stance = true;
  _b_front_swing = true;
  _b_hind_swing = false;

  //_b_front_swing = false;
  //_b_hind_swing = true;

  _fr_foot_vel.setZero();
  _fr_foot_acc.setZero();

  _fl_foot_vel.setZero();
  _fl_foot_acc.setZero();

  _hr_foot_vel.setZero();
  _hr_foot_acc.setZero();

  _hl_foot_vel.setZero();
  _hl_foot_acc.setZero();

  _jpos_task = new JPosTask<T>(Ctrl::_robot_sys);
  _local_roll_task = new LocalRollTask<T>(Ctrl::_robot_sys);
  _body_ori_task = new BodyOriTask<T>(Ctrl::_robot_sys);
  _body_ryrz_task = new BodyRyRzTask<T>(Ctrl::_robot_sys);

  _fr_foot_local_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::FR, linkID::FR_abd);
  _fl_foot_local_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::FL, linkID::FL_abd);
  _hr_foot_local_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::HR, linkID::HR_abd);
  _hl_foot_local_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::HL, linkID::HL_abd);

  //_fr_leg_height_task = new LegHeightTask<T>(Ctrl::_robot_sys, linkID::FR, linkID::FR_abd);
  //_fl_leg_height_task = new LegHeightTask<T>(Ctrl::_robot_sys, linkID::FL, linkID::FL_abd);
  //_hr_leg_height_task = new LegHeightTask<T>(Ctrl::_robot_sys, linkID::HR, linkID::HR_abd);
  //_hl_leg_height_task = new LegHeightTask<T>(Ctrl::_robot_sys, linkID::HL, linkID::HL_abd);

  _local_head_pos_task = new LocalHeadPosTask<T>(Ctrl::_robot_sys);
  _local_tail_pos_task = new LocalTailPosTask<T>(Ctrl::_robot_sys);

  _fr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FR);
  _fl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FL);
  _hr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HR);
  _hl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HL);

  _kin_wbc = new KinWBC<T>(cheetah::dim_config);

  Ctrl::_task_list.push_back(_jpos_task);
  _wbic = new WBIC<T>(cheetah::dim_config, &(Ctrl::_contact_list), &(Ctrl::_task_list));

  _wbic_data = new WBIC_ExtraData<T>();
  _wbic_data->_W_floating = DVec<T>::Constant(6, 5.);
  _sp = StateProvider<T>::getStateProvider();

  _folder_name = "/robot/WBC_States/sim_data/";
  create_folder(_folder_name);

  _Fr_des.setZero();
  _Fr_result.setZero();
  printf("[Kinematics Bounding Control] Constructed\n");
}

template<typename T>
void KinBoundingCtrl<T>::_ContactUpdate(){
  T fr_z = Ctrl::_robot_sys->_pGC[linkID::FR][2];
  T fl_z = Ctrl::_robot_sys->_pGC[linkID::FL][2];
  T hr_z = Ctrl::_robot_sys->_pGC[linkID::HR][2];
  T hl_z = Ctrl::_robot_sys->_pGC[linkID::HL][2];

  T threshold(0.001);
  if( (fr_z < threshold) && (fl_z < threshold) ){
    _b_front_contact = true;
  }else {
    _b_front_contact = false;
  }
  if( (hr_z < threshold) && (hl_z < threshold) ){
    _b_hind_contact = true;
  }else {
    _b_hind_contact = false;
  }
}

template <typename T>
void KinBoundingCtrl<T>::_StatusCheck(){
  _ContactUpdate();

  //T K_time(0.3);
  if(_b_front_swing && (_front_time > 0.5 * _swing_time)){
    // Check Contact && TEST
    //if(_b_front_contact){
    if(_front_time > _front_swing_time){
      _b_front_swing = false;
      _front_start_time = _sp->_curr_time;

      //_front_previous_swing = _front_time;
      //_front_current_stance = 
        //_stance_time -
        //K_time* (_hind_previous_stance + _aerial_duration - 0.5*_default_gait_period);

      //printf("front, hind swing: %d, %d\n", _b_front_swing, _b_hind_swing);
      //printf("front time, hind time: %f, %f\n", _front_time, _hind_time); 
      //printf("front stance start (aerial, stance): %f, %f\n", 
      //_aerial_duration, _front_current_stance);
      
      _front_time = 0.;

      //T apex = 
        //_total_mass * 9.81 * (_swing_time + _front_current_stance)/
        //(2. * 2.0*0.7*_front_current_stance);

      //_front_z_impulse.setCurve(apex, _front_current_stance);
      //_front_previous_stance = _front_current_stance;

      _ini_front_body = 
        0.5*Ctrl::_robot_sys->_pGC[linkID::FR_abd] 
        + 0.5*Ctrl::_robot_sys->_pGC[linkID::FL_abd]
        - 0.5*Ctrl::_robot_sys->_pGC[linkID::FR]
        - 0.5*Ctrl::_robot_sys->_pGC[linkID::FL];

      _front_swing_time = _swing_time - 2.*Test<T>::dt;
    }
  }

  if(_b_hind_swing && (_hind_time > 0.5 * _swing_time)){
    // Check Contact && TEST
    //if(_b_hind_contact){
    if(_hind_time > _swing_time - 2.*Test<T>::dt){ // Switch to stance
      _b_hind_swing = false;
      _hind_start_time = _sp->_curr_time;
      
      //_hind_previous_swing = _hind_time;
      //_hind_current_stance = 
        //_stance_time - 
        //K_time* (_front_previous_stance + _aerial_duration - 0.5*_default_gait_period);

      //printf("front, hind swing: %d, %d\n", _b_front_swing, _b_hind_swing);
      //printf("front time, hind time: %f, %f\n", _front_time, _hind_time); 
      //printf("hind stance start (aerial, stance): %f, %f\n", 
          //_aerial_duration, _hind_current_stance);

      _hind_time = 0.;

      //T apex = 
        //_total_mass * 9.81 * (_swing_time + _hind_current_stance)/
        //(2. * 2.0*0.7*_hind_current_stance);

      //_hind_z_impulse.setCurve(apex, _hind_current_stance);
      //_hind_previous_stance = _hind_current_stance;

      _ini_hind_body = 
        0.5*Ctrl::_robot_sys->_pGC[linkID::FR_abd] 
        + 0.5*Ctrl::_robot_sys->_pGC[linkID::FL_abd]
        - 0.5*Ctrl::_robot_sys->_pGC[linkID::FR]
        - 0.5*Ctrl::_robot_sys->_pGC[linkID::FL];
    }
  }

  T scale(1.5);
  T offset_front(0.03);
  T offset_hind(-0.03);
  // If stance time is over switch to swing
  if((!_b_front_swing) && (_front_time > (_front_current_stance - 2.*Test<T>::dt))){
    _b_front_swing = true;

    _ini_fr = Ctrl::_robot_sys->_pGC[linkID::FR] - Ctrl::_robot_sys->_pGC[linkID::FR_abd];
    _ini_fl = Ctrl::_robot_sys->_pGC[linkID::FL] - Ctrl::_robot_sys->_pGC[linkID::FL_abd];

    _fin_fr = _bounding_test->_body_vel * _stance_time/2. * scale;
    _fin_fl = _bounding_test->_body_vel * _stance_time/2. * scale;

    _ini_fr[0] += offset_front;
    _ini_fl[0] += offset_front;

    _fin_fr[0] += offset_front;
    _fin_fl[0] += offset_front;
     //pretty_print(_bounding_test->_body_vel, std::cout, "body vel cmd ");
    //pretty_print(_ini_fr, std::cout, "ini fr");
    //pretty_print(_ini_fl, std::cout, "ini fl");
    //pretty_print(_fin_fr, std::cout, "fin fr");
    //pretty_print(_fin_fl, std::cout, "fin fl");

    _front_start_time = _sp->_curr_time;
    _front_time = 0.;
  }
  // If stance time is over switch to swing
  if((!_b_hind_swing) && (_hind_time > (_hind_current_stance - 2.*Test<T>::dt))){
    _b_hind_swing = true;

    _ini_hr = Ctrl::_robot_sys->_pGC[linkID::HR] - Ctrl::_robot_sys->_pGC[linkID::HR_abd];
    _ini_hl = Ctrl::_robot_sys->_pGC[linkID::HL] - Ctrl::_robot_sys->_pGC[linkID::HL_abd];

    _fin_hr = _bounding_test->_body_vel * _stance_time/2. * scale;
    _fin_hl = _bounding_test->_body_vel * _stance_time/2. * scale;

    _ini_hr[0] += offset_hind;
    _ini_hl[0] += offset_hind;
    _fin_hr[0] += offset_hind;
    _fin_hl[0] += offset_hind;
 
    _hind_start_time = _sp->_curr_time;
    _hind_time = 0.;
  }
  //printf("front, hind swing: %d, %d\n", _b_front_swing, _b_hind_swing);
}

template <typename T>
void KinBoundingCtrl<T>::_setupTaskAndContactList(){
  if(_b_front_swing){
    _kin_task_list.push_back(_fl_foot_local_task);
    _kin_task_list.push_back(_fr_foot_local_task);
  }else{
    _kin_task_list.push_back(_local_head_pos_task);
    //_kin_task_list.push_back(_fr_leg_height_task);
    //_kin_task_list.push_back(_fl_leg_height_task);

    Ctrl::_contact_list.push_back(_fr_contact);
    Ctrl::_contact_list.push_back(_fl_contact);

    _kin_contact_list.push_back(_fr_contact);
    _kin_contact_list.push_back(_fl_contact);
  }

  if(_b_hind_swing){
    _kin_task_list.push_back(_hr_foot_local_task);
    _kin_task_list.push_back(_hl_foot_local_task);
  }else{
    _kin_task_list.push_back(_local_tail_pos_task);
    //_kin_task_list.push_back(_hr_leg_height_task);
    //_kin_task_list.push_back(_hl_leg_height_task);

    Ctrl::_contact_list.push_back(_hr_contact);
    Ctrl::_contact_list.push_back(_hl_contact);

    _kin_contact_list.push_back(_hr_contact);
    _kin_contact_list.push_back(_hl_contact);
  }
}

template <typename T>
void KinBoundingCtrl<T>::OneStep(void* _cmd){
  Ctrl::_PreProcessing_Command();

  // Initialize all
  Ctrl::_contact_list.clear();
  Ctrl::_task_list.clear();
  _kin_contact_list.clear();
  _kin_task_list.clear();
  // Update Time
  Ctrl::_state_machine_time = _sp->_curr_time - _ctrl_start_time;

  _front_time = _sp->_curr_time - _front_start_time;
  _hind_time = _sp->_curr_time - _hind_start_time;

  // Update Current Stance and Swing Status
  _StatusCheck();

  if((!_b_front_swing) || (!_b_hind_swing)){
    _aerial_duration = 0.;
    //_kin_task_list.push_back(_body_ori_task);
    _kin_task_list.push_back(_local_roll_task);
    _kin_task_list.push_back(_body_ryrz_task);
  }else{
    _aerial_duration += Test<T>::dt;
  }

  _setupTaskAndContactList();

  DVec<T> gamma = DVec<T>::Zero(cheetah::num_act_joint);
  _contact_update();
  _body_task_setup();
  _leg_task_setup();

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

  // File Save (Please disable when doing hardware tests)
  if(false){
  //if(true){
    saveValue(Ctrl::_state_machine_time, _folder_name, "time");
    saveValue(_b_hind_contact, _folder_name, "hind_contact");
    saveValue(_b_front_contact, _folder_name, "front_contact");

    saveVector(_fr_foot_pos, _folder_name, "fr_foot_pos_cmd");
    saveVector(_fr_foot_vel, _folder_name, "fr_foot_vel_cmd");
    saveVector(_fr_foot_acc, _folder_name, "fr_foot_acc_cmd");

    saveVector(_fl_foot_pos, _folder_name, "fl_foot_pos_cmd");
    saveVector(_fl_foot_vel, _folder_name, "fl_foot_vel_cmd");
    saveVector(_fl_foot_acc, _folder_name, "fl_foot_acc_cmd");

    saveVector(_hr_foot_pos, _folder_name, "hr_foot_pos_cmd");
    saveVector(_hr_foot_vel, _folder_name, "hr_foot_vel_cmd");
    saveVector(_hr_foot_acc, _folder_name, "hr_foot_acc_cmd");

    saveVector(_hl_foot_pos, _folder_name, "hl_foot_pos_cmd");
    saveVector(_hl_foot_vel, _folder_name, "hl_foot_vel_cmd");
    saveVector(_hl_foot_acc, _folder_name, "hl_foot_acc_cmd");

    saveVector(_fr_foot_local_task->getPosError(), _folder_name, "fr_foot_err");
    saveVector(_fl_foot_local_task->getPosError(), _folder_name, "fl_foot_err");
    saveVector(_hr_foot_local_task->getPosError(), _folder_name, "hr_foot_err");
    saveVector(_hl_foot_local_task->getPosError(), _folder_name, "hl_foot_err");

    saveVector(_des_jpos, _folder_name, "jpos_cmd");
    saveVector(_des_jvel, _folder_name, "jvel_cmd");
    saveVector(_des_jacc, _folder_name, "jacc_cmd");

    saveVector(_sp->_Q, _folder_name, "config");
    saveVector(_sp->_Qdot, _folder_name, "qdot");

    _Fr_des = DVec<T>::Zero(12);
    _Fr_result = DVec<T>::Zero(12);

    if((!_b_front_swing)&&(!_b_hind_swing)){ // Full Stance
      _Fr_des = _wbic_data->_Fr_des;
      _Fr_result = _wbic_data->_Fr;
    }else if(!_b_front_swing){ // Front Stance
      _Fr_des.head(6) = _wbic_data->_Fr_des;
      _Fr_result.head(6) = _wbic_data->_Fr;
    }else if(!_b_hind_swing){ // Hind Stance
      _Fr_des.tail(6) = _wbic_data->_Fr_des;
      _Fr_result.tail(6) = _wbic_data->_Fr;
    }
    saveVector(_Fr_des, _folder_name, "Fr_des");
    saveVector(_Fr_result, _folder_name, "Fr_result");
  }
}

template <typename T>
void KinBoundingCtrl<T>::_compute_torque_wbic(DVec<T> & gamma){

  _kin_wbc->FindConfiguration(_sp->_Q, 
      _kin_task_list, _kin_contact_list, 
      _des_jpos, _des_jvel, _des_jacc);

  _jpos_task->UpdateTask(&(_des_jpos), _des_jvel, _des_jacc);

  // WBIC
  _wbic->UpdateSetting(Ctrl::_A, Ctrl::_Ainv, Ctrl::_coriolis, Ctrl::_grav);
  _wbic->MakeTorque(gamma, _wbic_data);
}

template <typename T>
void KinBoundingCtrl<T>::_body_task_setup(){
  Vec3<T> rpy_des; rpy_des.setZero();
  DVec<T> vel_des(3); vel_des.setZero();
  DVec<T> acc_des(3); acc_des.setZero();

  // Orientation
  Quat<T> quat_des = ori::rpyToQuat(rpy_des);
  //_body_ori_task->UpdateTask(&(quat_des), vel_des, acc_des);

  // Body Ry Rz (pitch and yaw)
  vel_des.resize(2); vel_des.setZero();
  acc_des.resize(2); acc_des.setZero();
  _body_ryrz_task->UpdateTask(&(quat_des), vel_des, acc_des);

  // Local Roll
  T roll_cmd(0.);
  vel_des.resize(1); vel_des.setZero();
  acc_des.resize(1); acc_des.setZero();
  _local_roll_task->UpdateTask(&(roll_cmd), vel_des, acc_des);

  // Local Head
  Vec3<T> pos_des; pos_des.setZero();
  pos_des[2] = _target_leg_height;
  vel_des.resize(3); vel_des.setZero();
  acc_des.resize(3); acc_des.setZero();
 
  if(!_b_front_swing){ // Stance
    pos_des[0] = _ini_front_body[0] + _bounding_test->_body_vel[0]*_front_time;
    vel_des[0] = _bounding_test->_body_vel[0];
    _local_head_pos_task->UpdateTask(&pos_des, vel_des, acc_des);
  }
  if(!_b_hind_swing){ // Stance
    pos_des[0] = _ini_hind_body[0] + _bounding_test->_body_vel[0]*_hind_time;
    vel_des[0] = _bounding_test->_body_vel[0];
    _local_tail_pos_task->UpdateTask(&pos_des, vel_des, acc_des);
  }
}

template<typename T>
void KinBoundingCtrl<T>::_leg_task_setup(){
  // for Z (height)
  T amp(0.05/2.);
  T omega ( 2.*M_PI /_swing_time);
  T t;

  _fr_foot_pos.setZero();
  _fl_foot_pos.setZero(); 
  _hr_foot_pos.setZero(); 
  _hl_foot_pos.setZero(); 


  //T leg_height = -_target_leg_height;
  DVec<T> vel_des(3); vel_des.setZero();
  DVec<T> acc_des(3); acc_des.setZero();

  if(_b_front_swing){
    t = _front_time;
    if(_front_time > _swing_time - 2. * Test<T>::dt){ t = 0.; }

    // FR X
    _fr_foot_pos[0] = smooth_change(_ini_fr[0], _fin_fr[0], _swing_time, t);
    _fr_foot_vel[0] = smooth_change_vel(_ini_fr[0], _fin_fr[0], _swing_time, t);
    _fr_foot_acc[0] = smooth_change_acc(_ini_fr[0], _fin_fr[0], _swing_time, t);
    // FR Y
    _fr_foot_pos[1] = -0.05;
    // FR Z
    _fr_foot_pos[2] = -_target_leg_height + amp * (1.-cos(omega*t));  
    _fr_foot_vel[2] = amp * omega * sin(omega*t);  
    _fr_foot_acc[2] = amp * omega * omega * sin(omega*t);  

    // FL X
    _fl_foot_pos[0] = smooth_change(_ini_fl[0], _fin_fl[0], _swing_time, t);
    _fl_foot_vel[0] = smooth_change_vel(_ini_fl[0], _fin_fl[0], _swing_time, t);
    _fl_foot_acc[0] = smooth_change_acc(_ini_fl[0], _fin_fl[0], _swing_time, t);
    // FL Y
    _fl_foot_pos[1] = 0.05;
    // FL Z
    _fl_foot_pos[2] = -_target_leg_height + amp * (1.-cos(omega*t));  
    _fl_foot_vel[2] = amp * omega * sin(omega*t);  
    _fl_foot_acc[2] = amp * omega * omega * sin(omega*t);  

    _fr_foot_local_task->UpdateTask(&(_fr_foot_pos), _fr_foot_vel, _fr_foot_acc);
    _fl_foot_local_task->UpdateTask(&(_fl_foot_pos), _fl_foot_vel, _fl_foot_acc);
  }else{ // Front Leg Stance

    //DVec<T> leg_vel_des(1); leg_vel_des.setZero();
    //DVec<T> leg_acc_des(1); leg_acc_des.setZero();

    //_fr_leg_height_task->UpdateTask(&(leg_height), leg_vel_des, leg_acc_des);
    //_fl_leg_height_task->UpdateTask(&(leg_height), leg_vel_des, leg_acc_des);

    _wbic_data->_W_rf = DVec<T>::Constant(6, 1.0);

    _wbic_data->_Fr_des = DVec<T>::Zero(6);
    _wbic_data->_W_rf[2] = 50.0;
    _wbic_data->_W_rf[5] = 50.0;

    _wbic_data->_Fr_des[2] = _front_z_impulse.getValue(_front_time);
    _wbic_data->_Fr_des[5] = _front_z_impulse.getValue(_front_time);
  }

  if(_b_hind_swing){
    t = _hind_time;
    if(_hind_time > _swing_time - 2. * Test<T>::dt){ t = 0.; }

    // HR X
    _hr_foot_pos[0] = smooth_change(_ini_hr[0], _fin_hr[0], _swing_time, t);
    _hr_foot_vel[0] = smooth_change_vel(_ini_hr[0], _fin_hr[0], _swing_time, t);
    _hr_foot_acc[0] = smooth_change_acc(_ini_hr[0], _fin_hr[0], _swing_time, t);
    // HR Y
    _hr_foot_pos[1] = -0.05;
    // HR Z
    _hr_foot_pos[2] = -_target_leg_height + amp * (1.-cos(omega*t));  
    _hr_foot_vel[2] = amp * omega * sin(omega*t);  
    _hr_foot_acc[2] = amp * omega * omega * sin(omega*t);  
    
    // HL X
    _hl_foot_pos[0] = smooth_change(_ini_hl[0], _fin_hl[0], _swing_time, t);
    _hl_foot_vel[0] = smooth_change_vel(_ini_hl[0], _fin_hl[0], _swing_time, t);
    _hl_foot_acc[0] = smooth_change_acc(_ini_hl[0], _fin_hl[0], _swing_time, t);
    // HL Y
    _hl_foot_pos[1] = 0.05;
    // HL Z
    _hl_foot_pos[2] = -_target_leg_height + amp * (1.-cos(omega*t));  
    _hl_foot_vel[2] = amp * omega * sin(omega*t);  
    _hl_foot_acc[2] = amp * omega * omega * sin(omega*t);  

    _hr_foot_local_task->UpdateTask(&(_hr_foot_pos), _hr_foot_vel, _hr_foot_acc);
    _hl_foot_local_task->UpdateTask(&(_hl_foot_pos), _hl_foot_vel, _hl_foot_acc);

  }else{
    //DVec<T> leg_vel_des(1); leg_vel_des.setZero();
    //DVec<T> leg_acc_des(1); leg_acc_des.setZero();

    //_hr_leg_height_task->UpdateTask(&(leg_height), leg_vel_des, leg_acc_des);
    //_hl_leg_height_task->UpdateTask(&(leg_height), leg_vel_des, leg_acc_des);


    if(_b_front_swing){ // Hind stance only
      _wbic_data->_Fr_des = DVec<T>::Zero(6);
      _wbic_data->_W_rf = DVec<T>::Constant(6, 1.0);
      _wbic_data->_W_rf[2] = 50.0;
      _wbic_data->_W_rf[5] = 50.0;

      _wbic_data->_Fr_des[2] = _hind_z_impulse.getValue(_hind_time);
      _wbic_data->_Fr_des[5] = _hind_z_impulse.getValue(_hind_time);
    }else{ // Front and Hind stance
      _wbic_data->_Fr_des = DVec<T>::Zero(12);
      _wbic_data->_W_rf = DVec<T>::Constant(12, 1.0);
      _wbic_data->_W_rf[2] = 50.0;
      _wbic_data->_W_rf[5] = 50.0;
      _wbic_data->_W_rf[8] = 50.0;
      _wbic_data->_W_rf[11] = 50.0;

      _wbic_data->_Fr_des[2] = _front_z_impulse.getValue(_front_time);
      _wbic_data->_Fr_des[5] = _front_z_impulse.getValue(_front_time);
      _wbic_data->_Fr_des[8] = _hind_z_impulse.getValue(_hind_time);
      _wbic_data->_Fr_des[11] = _hind_z_impulse.getValue(_hind_time);
    }
  }
}

template <typename T>
void KinBoundingCtrl<T>::_contact_update(){
  typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::_contact_list.begin();
  while(iter < Ctrl::_contact_list.end()){
    (*iter)->UpdateContactSpec();
    ++iter;
  }

  iter = _kin_contact_list.begin();
  while(iter < _kin_contact_list.end()){
    (*iter)->UpdateContactSpec();
    ++iter;
  }
}

template <typename T>
void KinBoundingCtrl<T>::FirstVisit(){
  _total_mass = Ctrl::_robot_sys->getMassMatrix()(5,5);
  T apex = _total_mass * 9.81 * (_swing_time + _stance_time)/(2. * 2.0*0.7*_stance_time);

  //printf("Bounding apex: %f\n", apex);
  _front_z_impulse.setCurve(apex, _stance_time);
  _hind_z_impulse.setCurve(apex, _stance_time);

  _default_gait_period = _swing_time + _stance_time;
  _front_swing_time = _default_gait_period/2. - 2. * Test<T>::dt;

  _front_previous_stance = _stance_time;
  _front_previous_swing = _swing_time;
  _front_current_stance = _stance_time;

  _hind_previous_stance = _stance_time;
  _hind_previous_swing = _swing_time;
  _hind_current_stance = _stance_time;

  _aerial_duration = 0.;

  _fr_abduction_pos =
    Ctrl::_robot_sys->_pGC[linkID::FR_abd] -
    Ctrl::_robot_sys->_pGC[linkID::FR];

  _fl_abduction_pos =
    Ctrl::_robot_sys->_pGC[linkID::FL_abd] -
    Ctrl::_robot_sys->_pGC[linkID::FL];

  _hr_abduction_pos =
    Ctrl::_robot_sys->_pGC[linkID::HR_abd] -
    Ctrl::_robot_sys->_pGC[linkID::HR];

  _hl_abduction_pos =
    Ctrl::_robot_sys->_pGC[linkID::HL_abd] -
    Ctrl::_robot_sys->_pGC[linkID::HL];

  _ini_fr = Ctrl::_robot_sys->_pGC[linkID::FR] - Ctrl::_robot_sys->_pGC[linkID::FR_abd];
  _ini_fl = Ctrl::_robot_sys->_pGC[linkID::FL] - Ctrl::_robot_sys->_pGC[linkID::FL_abd];

  _fin_fr = _ini_fr + _bounding_test->_body_vel * _stance_time/2.;
  _fin_fl = _ini_fl + _bounding_test->_body_vel * _stance_time/2.;

  _ini_hr = Ctrl::_robot_sys->_pGC[linkID::HR] - Ctrl::_robot_sys->_pGC[linkID::HR_abd];
  _ini_hl = Ctrl::_robot_sys->_pGC[linkID::HL] - Ctrl::_robot_sys->_pGC[linkID::HL_abd];

  _fin_hr = _ini_hr + _bounding_test->_body_vel * _stance_time/2.;
  _fin_hl = _ini_hl + _bounding_test->_body_vel * _stance_time/2.;



  _front_start_time = _sp->_curr_time;
  _hind_start_time = _sp->_curr_time;

  _ctrl_start_time = _sp->_curr_time;
}

template <typename T>
void KinBoundingCtrl<T>::LastVisit(){}

template <typename T>
bool KinBoundingCtrl<T>::EndOfPhase(){
  if(Ctrl::_state_machine_time > (_end_time-2.*Test<T>::dt)){
    return true;
  }
  return false;
}

template <typename T>
void KinBoundingCtrl<T>::CtrlInitialization(const std::string & category_name){
  (void)category_name;
}

template <typename T>
void KinBoundingCtrl<T>::SetTestParameter(const std::string & test_file){
  _param_handler = new ParamHandler(test_file);
  std::vector<T> tmp_vec;
  if(!_param_handler->getValue<T>("leg_height", _target_leg_height)){
    printf("[BoundingInitiate] No leg height\n");
  }

  _param_handler->getValue<T>("swing_time", _swing_time);
  _param_handler->getValue<T>("default_stance_time", _stance_time);
  _param_handler->getValue<T>("bounding_time", _end_time);

  //_param_handler->getVector<T>("Kp_ori", tmp_vec);
  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = tmp_vec[i]; 
  //}
  //_param_handler->getVector<T>("Kd_ori", tmp_vec);
  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = tmp_vec[i]; 
  //}

  //_param_handler->getVector<T>("Kp_ryrz", tmp_vec);
  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((BodyRyRzTask<T>*)_body_ryrz_task)->_Kp[i] = tmp_vec[i]; 
  //}
  //_param_handler->getVector<T>("Kd_ryrz", tmp_vec);
  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((BodyRyRzTask<T>*)_body_ryrz_task)->_Kd[i] = tmp_vec[i]; 
  //}

  _param_handler->getVector<T>("Kp", tmp_vec);
  for(size_t i(0); i<tmp_vec.size(); ++i){ 
    ((JPosTask<T>*)_jpos_task)->_Kp[i] = tmp_vec[i]; 
  }
  _param_handler->getVector<T>("Kd", tmp_vec);
  for(size_t i(0); i<tmp_vec.size(); ++i){ 
    ((JPosTask<T>*)_jpos_task)->_Kd[i] = tmp_vec[i]; 
  }

  //T tmp_value;
  //_param_handler->getValue<T>("Kp_leg_height", tmp_value);
  //((LegHeightTask<T>*)_fr_leg_height_task)->_Kp[0] = tmp_value;
  //((LegHeightTask<T>*)_fl_leg_height_task)->_Kp[0] = tmp_value;
  //((LegHeightTask<T>*)_hr_leg_height_task)->_Kp[0] = tmp_value;
  //((LegHeightTask<T>*)_hl_leg_height_task)->_Kp[0] = tmp_value;

  //_param_handler->getValue<T>("Kd_leg_height", tmp_value);
  //((LegHeightTask<T>*)_fr_leg_height_task)->_Kd[0] = tmp_value;
  //((LegHeightTask<T>*)_fl_leg_height_task)->_Kd[0] = tmp_value;
  //((LegHeightTask<T>*)_hr_leg_height_task)->_Kd[0] = tmp_value;
  //((LegHeightTask<T>*)_hl_leg_height_task)->_Kd[0] = tmp_value;

  // Local Roll
  //_param_handler->getValue<T>("Kp_roll", tmp_value);
  //((LocalRollTask<T>*)_local_roll_task)->_Kp[0] = tmp_value; 
  //_param_handler->getValue<T>("Kd_roll", tmp_value);
  //((LocalRollTask<T>*)_local_roll_task)->_Kd[0] = tmp_value; 


  // Joint level feedback gain
  if(!_param_handler->getVector<T>("Kp_joint", _Kp_joint)){
    printf("no Kp joint setting\n");
    exit(0);
  }
  _param_handler->getVector<T>("Kd_joint", _Kd_joint);
}


template <typename T>
KinBoundingCtrl<T>::~KinBoundingCtrl(){
  delete _kin_wbc;
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


template class KinBoundingCtrl<double>;
template class KinBoundingCtrl<float>;
