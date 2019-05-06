#include "BoundingCtrl.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <WBC_States/common/ContactSet/SingleContact.hpp>
#include <WBC_States/common/TaskSet/BodyOriTask.hpp>
#include <WBC_States/common/TaskSet/BodyPosTask.hpp>
#include <WBC_States/Bounding/TaskSet/BodyXYTask.hpp>
#include <WBC_States/Bounding/TaskSet/LegHeightTask.hpp>
#include <WBC_States/Bounding/TaskSet/LocalPosTask.hpp>
#include <WBC_States/Bounding/TaskSet/SelectedJointTask.hpp>
#include <WBC_States/Bounding/TaskSet/BodyRyRzTask.hpp>
#include <WBC_States/Bounding/TaskSet/LocalRollTask.hpp>

#include <WBC/WBIC/WBIC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <WBC_States/Bounding/BoundingTest.hpp>
#include <Utilities/save_file.h>

template <typename T>
BoundingCtrl<T>::BoundingCtrl(
    BoundingTest<T>* bounding_test, const FloatingBaseModel<T>* robot):Controller<T>(robot),
  _bounding_test(bounding_test),
  _des_jpos(cheetah::num_act_joint),
  _des_jvel(cheetah::num_act_joint),
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
  _Fr_result(12)
{
  _fr_foot_vel.setZero();
  _fr_foot_acc.setZero();

  _fl_foot_vel.setZero();
  _fl_foot_acc.setZero();

  _hr_foot_vel.setZero();
  _hr_foot_acc.setZero();

  _hl_foot_vel.setZero();
  _hl_foot_acc.setZero();
   // Start from front swing & hind stance
  _b_front_swing = true;
  _b_hind_swing = false;

  std::vector<int> joint_list(4);
  joint_list[0] = 0;
  joint_list[1] = 3;
  joint_list[2] = 6;
  joint_list[3] = 9;

  _local_roll_task = new LocalRollTask<T>(Ctrl::_robot_sys);
  _abd_joint_task = new SelectedJointTask<T>(Ctrl::_robot_sys, joint_list);
  _body_ori_task = new BodyOriTask<T>(Ctrl::_robot_sys);
  _body_ryrz_task = new BodyRyRzTask<T>(Ctrl::_robot_sys);

  _fr_foot_local_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::FR, linkID::FR_abd);
  _fl_foot_local_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::FL, linkID::FL_abd);
  _hr_foot_local_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::HR, linkID::HR_abd);
  _hl_foot_local_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::HL, linkID::HL_abd);

  //_fr_abduction_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::FR_abd, linkID::FR);
  //_fl_abduction_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::FL_abd, linkID::FL);
  //_hr_abduction_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::HR_abd, linkID::HR);
  //_hl_abduction_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::HL_abd, linkID::HL);

  _fr_leg_height_task = new LegHeightTask<T>(Ctrl::_robot_sys, linkID::FR, linkID::FR_abd);
  _fl_leg_height_task = new LegHeightTask<T>(Ctrl::_robot_sys, linkID::FL, linkID::FL_abd);
  _hr_leg_height_task = new LegHeightTask<T>(Ctrl::_robot_sys, linkID::HR, linkID::HR_abd);
  _hl_leg_height_task = new LegHeightTask<T>(Ctrl::_robot_sys, linkID::HL, linkID::HL_abd);

  _fr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FR);
  _fl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FL);
  _hr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HR);
  _hl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HL);

  Ctrl::_contact_list.clear();
  Ctrl::_contact_list.push_back(_hr_contact);
  Ctrl::_contact_list.push_back(_hl_contact);

  _wbic = new WBIC<T>(cheetah::dim_config, &(Ctrl::_contact_list), &(Ctrl::_task_list));
  _wbic_data = new WBIC_ExtraData<T>();

  _wbic_data->_W_floating = DVec<T>::Constant(6, 50.);
  //_wbic_data->_W_floating[1] = 1.0;
  _wbic_data->_W_floating[3] = 1.0;
  _wbic_data->_W_floating[4] = 1.0;
  _wbic_data->_W_floating[5] = 1.0;
  _sp = StateProvider<T>::getStateProvider();

  _folder_name = "/robot/WBC_States/sim_data/";
  create_folder(_folder_name);

  _Fr_des.setZero();
  _Fr_result.setZero();
  printf("[Bounding Control] Constructed\n");
}
template <typename T>
void BoundingCtrl<T>::_StatusCheck(){

  //printf("fr foot: %f, %f, %f\n", 
      //Ctrl::_robot_sys->_pGC[linkID::FR][0], 
      //Ctrl::_robot_sys->_pGC[linkID::FR][1], 
      //Ctrl::_robot_sys->_pGC[linkID::FR][2]);

  //printf("fl foot: %f, %f, %f\n", 
      //Ctrl::_robot_sys->_pGC[linkID::FL][0], 
      //Ctrl::_robot_sys->_pGC[linkID::FL][1], 
      //Ctrl::_robot_sys->_pGC[linkID::FL][2]);

  //printf("hr foot: %f, %f, %f\n", 
      //Ctrl::_robot_sys->_pGC[linkID::HR][0], 
      //Ctrl::_robot_sys->_pGC[linkID::HR][1], 
      //Ctrl::_robot_sys->_pGC[linkID::HR][2]);

  //printf("hl foot: %f, %f, %f\n", 
      //Ctrl::_robot_sys->_pGC[linkID::HL][0], 
      //Ctrl::_robot_sys->_pGC[linkID::HL][1], 
      //Ctrl::_robot_sys->_pGC[linkID::HL][2]);

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


  if(_b_front_swing && (_front_time > 0.5 * _swing_time)){
    // Check Contact && TEST
    if(_b_front_contact){
      _b_front_swing = false;
      _front_start_time = _sp->_curr_time;
      _front_time = 0.;
    }
  }
  if(_b_hind_swing && (_hind_time > 0.5 * _swing_time)){
    // Check Contact && TEST
    if(_b_hind_contact){
      _b_hind_swing = false;
      _hind_start_time = _sp->_curr_time;
      _hind_time = 0.;
    }
  }

  // If stance time is over switch to swing
  if((!_b_front_swing) && (_front_time > (_stance_time - 2.*Test<T>::dt))){
    _b_front_swing = true;
    _front_start_time = _sp->_curr_time;
    _front_time = 0.;
  }
  // If stance time is over switch to swing
  if((!_b_hind_swing) && (_hind_time > (_stance_time - 2.*Test<T>::dt))){
    _b_hind_swing = true;
    _hind_start_time = _sp->_curr_time;
    _hind_time = 0.;
  }
  //printf("front, hind swing: %d, %d\n", _b_front_swing, _b_hind_swing);
}

template <typename T>
void BoundingCtrl<T>::_setupTaskAndContactList(){
  if(_b_front_swing){
    Ctrl::_task_list.push_back(_fr_foot_local_task);
    Ctrl::_task_list.push_back(_fl_foot_local_task);
  }else{
    Ctrl::_contact_list.push_back(_fr_contact);
    Ctrl::_contact_list.push_back(_fl_contact);

    //Ctrl::_task_list.push_back(_fr_abduction_task);
    //Ctrl::_task_list.push_back(_fl_abduction_task);
    Ctrl::_task_list.push_back(_fr_leg_height_task);
    Ctrl::_task_list.push_back(_fl_leg_height_task);
   }

  if(_b_hind_swing){
    Ctrl::_task_list.push_back(_hr_foot_local_task);
    Ctrl::_task_list.push_back(_hl_foot_local_task);
  }else{
    Ctrl::_contact_list.push_back(_hr_contact);
    Ctrl::_contact_list.push_back(_hl_contact);

    //Ctrl::_task_list.push_back(_hr_abduction_task);
    //Ctrl::_task_list.push_back(_hl_abduction_task);
    Ctrl::_task_list.push_back(_hr_leg_height_task);
    Ctrl::_task_list.push_back(_hl_leg_height_task);
   }
}

template <typename T>
void BoundingCtrl<T>::OneStep(void* _cmd){
  Ctrl::_PreProcessing_Command();

  // Initialize all
  Ctrl::_contact_list.clear();
  Ctrl::_task_list.clear();
  // Update Time
  Ctrl::_state_machine_time = _sp->_curr_time - _ctrl_start_time;

  _front_time = _sp->_curr_time - _front_start_time;
  _hind_time = _sp->_curr_time - _hind_start_time;

  // Update Current Stance and Swing Status
  _StatusCheck();

  //Ctrl::_task_list.push_back(_abd_joint_task);
  //Ctrl::_task_list.push_back(_body_ori_task);
  if((!_b_front_swing) || (!_b_hind_swing)){
  Ctrl::_task_list.push_back(_body_ori_task);
    //Ctrl::_task_list.push_back(_local_roll_task);
    //Ctrl::_task_list.push_back(_body_ryrz_task);
  }
  //Ctrl::_task_list.push_back(_body_xy_task);
 
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
      ((LegControllerCommand<T>*)_cmd)[leg].kpJoint(jidx, jidx) = 0.;
      ((LegControllerCommand<T>*)_cmd)[leg].kdJoint(jidx, jidx) = 0.;
    }
    ((LegControllerCommand<T>*)_cmd)[leg].qDes[0] = 0.;
    ((LegControllerCommand<T>*)_cmd)[leg].qdDes[0] = 0.;
    ((LegControllerCommand<T>*)_cmd)[leg].kpJoint(0, 0) = _Kp_joint[0];
    ((LegControllerCommand<T>*)_cmd)[leg].kdJoint(0, 0) = _Kd_joint[0];
  }

  Ctrl::_PostProcessing_Command();

  // File Save (Please disable when doing hardware tests)
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
  //static int count(0);
  //++count;
  //if(count > 50){
  //exit(0);
  //}
}

template <typename T>
void BoundingCtrl<T>::_compute_torque_wbic(DVec<T> & gamma){
  // WBIC
  _wbic->UpdateSetting(Ctrl::_A, Ctrl::_Ainv, Ctrl::_coriolis, Ctrl::_grav);
  _wbic->MakeTorque(gamma, _wbic_data);
}

template <typename T>
void BoundingCtrl<T>::_body_task_setup(){
  Vec3<T> rpy_des; rpy_des.setZero();
  DVec<T> vel_des(3); vel_des.setZero();
  DVec<T> acc_des(3); acc_des.setZero();

  // Orientation
  Quat<T> quat_des = ori::rpyToQuat(rpy_des);
  _body_ori_task->UpdateTask(&(quat_des), vel_des, acc_des);
  
  // Body Ry Rz (pitch and yaw)
  vel_des.resize(2); vel_des.setZero();
  acc_des.resize(2); acc_des.setZero();
  _body_ryrz_task->UpdateTask(&(quat_des), vel_des, acc_des);


  DVec<T> jpos(4); jpos.setZero();
  DVec<T> jvel(4); jvel.setZero();
  DVec<T> jacc(4); jacc.setZero();
  _abd_joint_task->UpdateTask(&(jpos), jvel, jacc);
  
  // Local Roll
  T roll_cmd(0.);
  vel_des.resize(1); vel_des.setZero();
  acc_des.resize(1); acc_des.setZero();
  _local_roll_task->UpdateTask(&(roll_cmd), vel_des, acc_des);


  // (X, Y)
  //DVec<T> pos_des(2); pos_des.setZero();
  //vel_des.resize(2); vel_des.setZero();
  //acc_des.resize(2); acc_des.setZero();
  //_body_xy_task->UpdateTask(&(pos_des), vel_des, acc_des);
}

template<typename T>
void BoundingCtrl<T>::_leg_task_setup(){
  // for Z (height)
  T amp(0.03/2.);
  T omega ( 2.*M_PI /_swing_time);
  T t;

  Vec3<T> fr_abd_pos = _fr_abduction_pos;
  Vec3<T> fl_abd_pos = _fl_abduction_pos;
  Vec3<T> hr_abd_pos = _hr_abduction_pos;
  Vec3<T> hl_abd_pos = _hl_abduction_pos;

  //fr_abd_pos[1] = 0.05;
  //fl_abd_pos[1] = -0.05;
  //hr_abd_pos[1] = 0.05;
  //hl_abd_pos[1] = -0.05;

  fr_abd_pos[2] = _target_leg_height;
  fl_abd_pos[2] = _target_leg_height;
  hr_abd_pos[2] = _target_leg_height;
  hl_abd_pos[2] = _target_leg_height;

  _fr_foot_pos.head(2) = -fr_abd_pos.head(2);
  _fl_foot_pos.head(2) = -fl_abd_pos.head(2);
  _hr_foot_pos.head(2) = -hr_abd_pos.head(2);
  _hl_foot_pos.head(2) = -hl_abd_pos.head(2);

  T leg_height = -_target_leg_height;
  DVec<T> vel_des(3); vel_des.setZero();
  DVec<T> acc_des(3); acc_des.setZero();

  if(_b_front_swing){
    t = _front_time;
    if(_front_time > _swing_time - 2. * Test<T>::dt){ t = 0.; }

    _fr_foot_pos[1] = -0.0;
    _fr_foot_pos[2] = -_target_leg_height + amp * (1.-cos(omega*t));  
    _fr_foot_vel[2] = amp * omega * sin(omega*t);  
    _fr_foot_acc[2] = amp * omega * omega * sin(omega*t);  

    _fl_foot_pos[1] = 0.0;
    _fl_foot_pos[2] = -_target_leg_height + amp * (1.-cos(omega*t));  
    _fl_foot_vel[2] = amp * omega * sin(omega*t);  
    _fl_foot_acc[2] = amp * omega * omega * sin(omega*t);  

    _fr_foot_local_task->UpdateTask(&(_fr_foot_pos), _fr_foot_vel, _fr_foot_acc);
    _fl_foot_local_task->UpdateTask(&(_fl_foot_pos), _fl_foot_vel, _fl_foot_acc);
  }else{ // Front Leg Stance
    //_fr_abduction_task->UpdateTask(&(fr_abd_pos), vel_des, acc_des);
    //_fl_abduction_task->UpdateTask(&(fl_abd_pos), vel_des, acc_des);

    DVec<T> leg_vel_des(1); leg_vel_des.setZero();
    DVec<T> leg_acc_des(1); leg_acc_des.setZero();

    _fr_leg_height_task->UpdateTask(&(leg_height), leg_vel_des, leg_acc_des);
    _fl_leg_height_task->UpdateTask(&(leg_height), leg_vel_des, leg_acc_des);


    _wbic_data->_W_rf = DVec<T>::Constant(6, 1.0);
    _wbic_data->_W_rf[2] = 2.0;
    _wbic_data->_W_rf[5] = 2.0;
    _wbic_data->_Fr_des = DVec<T>::Zero(6);
    _wbic_data->_Fr_des[2] = _z_impulse.getValue(_front_time);
    _wbic_data->_Fr_des[5] = _z_impulse.getValue(_front_time);
  }

  if(_b_hind_swing){
    t = _hind_time;
    if(_hind_time > _swing_time - 2. * Test<T>::dt){ t = 0.; }

    _hr_foot_pos[1] = -0.0;
    _hr_foot_pos[2] = -_target_leg_height + amp * (1.-cos(omega*t));  
    _hr_foot_vel[2] = amp * omega * sin(omega*t);  
    _hr_foot_acc[2] = amp * omega * omega * sin(omega*t);  

    _hl_foot_pos[1] = 0.0;
    _hl_foot_pos[2] = -_target_leg_height + amp * (1.-cos(omega*t));  
    _hl_foot_vel[2] = amp * omega * sin(omega*t);  
    _hl_foot_acc[2] = amp * omega * omega * sin(omega*t);  

    _hr_foot_local_task->UpdateTask(&(_hr_foot_pos), _hr_foot_vel, _hr_foot_acc);
    _hl_foot_local_task->UpdateTask(&(_hl_foot_pos), _hl_foot_vel, _hl_foot_acc);

  }else{
    //_hr_abduction_task->UpdateTask(&(hr_abd_pos), vel_des, acc_des);
    //_hl_abduction_task->UpdateTask(&(hl_abd_pos), vel_des, acc_des);

    DVec<T> leg_vel_des(1); leg_vel_des.setZero();
    DVec<T> leg_acc_des(1); leg_acc_des.setZero();

    _hr_leg_height_task->UpdateTask(&(leg_height), leg_vel_des, leg_acc_des);
    _hl_leg_height_task->UpdateTask(&(leg_height), leg_vel_des, leg_acc_des);


    if(_b_front_swing){ // Hind stance only
      _wbic_data->_Fr_des = DVec<T>::Zero(6);
      _wbic_data->_W_rf = DVec<T>::Constant(6, 1.0);
      _wbic_data->_W_rf[2] = 2.0;
      _wbic_data->_W_rf[5] = 2.0;
      _wbic_data->_Fr_des[2] = _z_impulse.getValue(_hind_time);
      _wbic_data->_Fr_des[5] = _z_impulse.getValue(_hind_time);
    }else{ // Front and Hind stance
      _wbic_data->_Fr_des = DVec<T>::Zero(12);
      _wbic_data->_W_rf = DVec<T>::Constant(12, 1.0);

      _wbic_data->_W_rf[2] = 2.0;
      _wbic_data->_W_rf[5] = 2.0;
      _wbic_data->_W_rf[8] = 2.0;
      _wbic_data->_W_rf[11] = 2.0;
      
      _wbic_data->_Fr_des[2] = _z_impulse.getValue(_front_time);
      _wbic_data->_Fr_des[5] = _z_impulse.getValue(_front_time);
      _wbic_data->_Fr_des[8] = _z_impulse.getValue(_hind_time);
      _wbic_data->_Fr_des[11] = _z_impulse.getValue(_hind_time);
    }
  }
}

template <typename T>
void BoundingCtrl<T>::_contact_update(){
  typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::_contact_list.begin();
  while(iter < Ctrl::_contact_list.end()){
    (*iter)->UpdateContactSpec();
    ++iter;
  }
}

template <typename T>
void BoundingCtrl<T>::FirstVisit(){
  T total_mass = Ctrl::_robot_sys->getMassMatrix()(5,5);
  T apex = total_mass * 9.81 * (_swing_time + _stance_time)/(2. * 2.0*0.7*_stance_time);

  //printf("Bounding apex: %f\n", apex);
  _z_impulse.setCurve(apex, _stance_time);

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


  _front_start_time = _sp->_curr_time;
  _hind_start_time = _sp->_curr_time;

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
  if(!_param_handler->getValue<T>("leg_height", _target_leg_height)){
    printf("[BoundingInitiate] No leg height\n");
  }

  _param_handler->getValue<T>("swing_time", _swing_time);
  _param_handler->getValue<T>("default_stance_time", _stance_time);
  _param_handler->getValue<T>("bounding_time", _end_time);

  //_param_handler->getVector<T>("Kp_xy_body", tmp_vec);
  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((BodyXYTask<T>*)_body_xy_task)->_Kp[i] = tmp_vec[i]; 
  //}
  //_param_handler->getVector<T>("Kd_xy_body", tmp_vec);
  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((BodyXYTask<T>*)_body_xy_task)->_Kd[i] = tmp_vec[i]; 
  //}

  _param_handler->getVector<T>("Kp_ori", tmp_vec);
  for(size_t i(0); i<tmp_vec.size(); ++i){ 
    ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = tmp_vec[i]; 
  }
  _param_handler->getVector<T>("Kd_ori", tmp_vec);
  for(size_t i(0); i<tmp_vec.size(); ++i){ 
    ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = tmp_vec[i]; 
  }

  _param_handler->getVector<T>("Kp_ryrz", tmp_vec);
  for(size_t i(0); i<tmp_vec.size(); ++i){ 
    ((BodyRyRzTask<T>*)_body_ryrz_task)->_Kp[i] = tmp_vec[i]; 
  }
  _param_handler->getVector<T>("Kd_ryrz", tmp_vec);
  for(size_t i(0); i<tmp_vec.size(); ++i){ 
    ((BodyRyRzTask<T>*)_body_ryrz_task)->_Kd[i] = tmp_vec[i]; 
  }



  _param_handler->getVector<T>("Kp_foot", tmp_vec);
  for(size_t i(0); i<tmp_vec.size(); ++i){ 
    ((LocalPosTask<T>*)_fr_foot_local_task)->_Kp[i] = tmp_vec[i]; 
  }
  for(size_t i(0); i<tmp_vec.size(); ++i){ 
    ((LocalPosTask<T>*)_fl_foot_local_task)->_Kp[i] = tmp_vec[i]; 
  }
  for(size_t i(0); i<tmp_vec.size(); ++i){ 
    ((LocalPosTask<T>*)_hr_foot_local_task)->_Kp[i] = tmp_vec[i]; 
  }
  for(size_t i(0); i<tmp_vec.size(); ++i){ 
    ((LocalPosTask<T>*)_hl_foot_local_task)->_Kp[i] = tmp_vec[i]; 
  }

  _param_handler->getVector<T>("Kd_foot", tmp_vec);
  for(size_t i(0); i<tmp_vec.size(); ++i){ 
    ((LocalPosTask<T>*)_fr_foot_local_task)->_Kd[i] = tmp_vec[i]; 
  }
  for(size_t i(0); i<tmp_vec.size(); ++i){ 
    ((LocalPosTask<T>*)_hr_foot_local_task)->_Kd[i] = tmp_vec[i]; 
  }


  // Abduction
  //_param_handler->getVector<T>("Kp_abd", tmp_vec);
  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((LocalPosTask<T>*)_fr_abduction_task)->_Kp[i] = tmp_vec[i]; 
  //}
  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((LocalPosTask<T>*)_fl_abduction_task)->_Kp[i] = tmp_vec[i]; 
  //}
  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((LocalPosTask<T>*)_hr_abduction_task)->_Kp[i] = tmp_vec[i]; 
  //}
  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((LocalPosTask<T>*)_hl_abduction_task)->_Kp[i] = tmp_vec[i]; 
  //}

  //_param_handler->getVector<T>("Kd_abd", tmp_vec);
  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((LocalPosTask<T>*)_fr_abduction_task)->_Kd[i] = tmp_vec[i]; 
  //}
  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((LocalPosTask<T>*)_fl_abduction_task)->_Kd[i] = tmp_vec[i]; 
  //}
  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((LocalPosTask<T>*)_hr_abduction_task)->_Kd[i] = tmp_vec[i]; 
  //}

  //for(size_t i(0); i<tmp_vec.size(); ++i){ 
    //((LocalPosTask<T>*)_hl_abduction_task)->_Kd[i] = tmp_vec[i]; 
  //}


  T tmp_value;
  _param_handler->getValue<T>("Kp_leg_height", tmp_value);
  ((LegHeightTask<T>*)_fr_leg_height_task)->_Kp[0] = tmp_value;
  ((LegHeightTask<T>*)_fl_leg_height_task)->_Kp[0] = tmp_value;
  ((LegHeightTask<T>*)_hr_leg_height_task)->_Kp[0] = tmp_value;
  ((LegHeightTask<T>*)_hl_leg_height_task)->_Kp[0] = tmp_value;

  _param_handler->getValue<T>("Kd_leg_height", tmp_value);
  ((LegHeightTask<T>*)_fr_leg_height_task)->_Kd[0] = tmp_value;
  ((LegHeightTask<T>*)_fl_leg_height_task)->_Kd[0] = tmp_value;
  ((LegHeightTask<T>*)_hr_leg_height_task)->_Kd[0] = tmp_value;
  ((LegHeightTask<T>*)_hl_leg_height_task)->_Kd[0] = tmp_value;
  
  _param_handler->getValue<T>("Kp_abd_joint", tmp_value);
  for(size_t i(0); i<_abd_joint_task->getDim(); ++i)
    ((SelectedJointTask<T>*)_abd_joint_task)->_Kp[i] = tmp_value;

  _param_handler->getValue<T>("Kd_abd_joint", tmp_value);
  for(size_t i(0); i<_abd_joint_task->getDim(); ++i)
    ((SelectedJointTask<T>*)_abd_joint_task)->_Kd[i] = tmp_value;
  
  // Local Roll
  _param_handler->getValue<T>("Kp_roll", tmp_value);
  ((LocalRollTask<T>*)_local_roll_task)->_Kp[0] = tmp_value; 
  _param_handler->getValue<T>("Kd_roll", tmp_value);
  ((LocalRollTask<T>*)_local_roll_task)->_Kd[0] = tmp_value; 



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
