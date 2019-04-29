#include "BoundingInitiateCtrl.hpp"

#include <WBC_States/StateProvider.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <WBC_States/common/ContactSet/SingleContact.hpp>
#include <WBC_States/common/TaskSet/BodyOriTask.hpp>
#include <WBC_States/common/TaskSet/BodyPosTask.hpp>
#include <WBC_States/Bounding/TaskSet/BodyXYTask.hpp>
#include <WBC_States/Bounding/TaskSet/LegHeightTask.hpp>
#include <WBC_States/Bounding/TaskSet/LocalPosTask.hpp>
#include <WBC_States/Bounding/TaskSet/SelectedJointTask.hpp>
#include <WBC_States/Bounding/TaskSet/LocalRollTask.hpp>

#include <WBC/WBIC/WBIC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <WBC_States/Bounding/BoundingTest.hpp>

template <typename T>
BoundingInitiateCtrl<T>::BoundingInitiateCtrl(
    const FloatingBaseModel<T>* robot):Controller<T>(robot),
  _des_jpos(cheetah::num_act_joint),
  _des_jvel(cheetah::num_act_joint),
  _end_time(1000.0),
  _dim_contact(0),
  _ctrl_start_time(0.)
{
  std::vector<int> joint_list(4);
  joint_list[0] = 0;
  joint_list[1] = 3;
  joint_list[2] = 6;
  joint_list[3] = 9;

  _abd_joint_task = new SelectedJointTask<T>(Ctrl::_robot_sys, joint_list);
  _body_ori_task = new BodyOriTask<T>(Ctrl::_robot_sys);
  _local_roll_task = new LocalRollTask<T>(Ctrl::_robot_sys);
  //_body_xy_task = new BodyXYTask<T>(Ctrl::_robot_sys);
  _body_pos_task = new BodyPosTask<T>(Ctrl::_robot_sys);

  Ctrl::_task_list.clear();
  Ctrl::_task_list.push_back(_local_roll_task);
  //Ctrl::_task_list.push_back(_abd_joint_task);
  //Ctrl::_task_list.push_back(_body_ori_task);
  //Ctrl::_task_list.push_back(_body_xy_task);
  //Ctrl::_task_list.push_back(_body_pos_task);

  _fr_leg_height_task = new LegHeightTask<T>(Ctrl::_robot_sys, linkID::FR, linkID::FR_abd);
  _fl_leg_height_task = new LegHeightTask<T>(Ctrl::_robot_sys, linkID::FL, linkID::FL_abd);
  _hr_leg_height_task = new LegHeightTask<T>(Ctrl::_robot_sys, linkID::HR, linkID::HR_abd);
  _hl_leg_height_task = new LegHeightTask<T>(Ctrl::_robot_sys, linkID::HL, linkID::HL_abd);

  Ctrl::_task_list.push_back(_fr_leg_height_task);
  Ctrl::_task_list.push_back(_fl_leg_height_task);
  Ctrl::_task_list.push_back(_hr_leg_height_task);
  Ctrl::_task_list.push_back(_hl_leg_height_task);


  //_fr_abduction_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::FR_abd, linkID::FR);
  //_fl_abduction_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::FL_abd, linkID::FL);
  //_hr_abduction_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::HR_abd, linkID::HR);
  //_hl_abduction_task = new LocalPosTask<T>(Ctrl::_robot_sys, linkID::HL_abd, linkID::HL);

  //Ctrl::_task_list.push_back(_fr_abduction_task);
  //Ctrl::_task_list.push_back(_fl_abduction_task);
  //Ctrl::_task_list.push_back(_hr_abduction_task);
  //Ctrl::_task_list.push_back(_hl_abduction_task);

  _fr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FR);
  _fl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::FL);
  _hr_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HR);
  _hl_contact = new SingleContact<T>(Ctrl::_robot_sys, linkID::HL);

  Ctrl::_contact_list.clear();
  Ctrl::_contact_list.push_back(_fr_contact);
  Ctrl::_contact_list.push_back(_fl_contact);
  Ctrl::_contact_list.push_back(_hr_contact);
  Ctrl::_contact_list.push_back(_hl_contact);

  for(size_t i(0); i < Ctrl::_contact_list.size(); ++i){
    _dim_contact += Ctrl::_contact_list[i]->getDim();
  }
  _wbic = new WBIC<T>(cheetah::dim_config, &(Ctrl::_contact_list), &(Ctrl::_task_list));
  _wbic_data = new WBIC_ExtraData<T>();


  _wbic_data->_W_floating = DVec<T>::Constant(6, 1.);
  _wbic_data->_W_rf = DVec<T>::Constant(_dim_contact, 1.);
  _wbic_data->_Fr_des = DVec<T>::Zero(_dim_contact);
  _wbic_data->_Fr_des[2] = 20;
  _wbic_data->_Fr_des[5] = 20;
  _wbic_data->_Fr_des[8] = 20;
  _wbic_data->_Fr_des[11] = 20;

  _sp = StateProvider<T>::getStateProvider();

  printf("[Bounding Initialiating Control] Constructed\n");
}

template <typename T>
void BoundingInitiateCtrl<T>::OneStep(void* _cmd){
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
      ((LegControllerCommand<T>*)_cmd)[leg].kpJoint(jidx, jidx) = 0.;
      ((LegControllerCommand<T>*)_cmd)[leg].kdJoint(jidx, jidx) = 0.;
    }
  }
  Ctrl::_PostProcessing_Command();
}

template <typename T>
void BoundingInitiateCtrl<T>::_compute_torque_wbic(DVec<T> & gamma){
  // WBIC
  _wbic->UpdateSetting(Ctrl::_A, Ctrl::_Ainv, Ctrl::_coriolis, Ctrl::_grav);
  _wbic->MakeTorque(gamma, _wbic_data);
}

template <typename T>
void BoundingInitiateCtrl<T>::_task_setup(){
  Vec3<T> rpy_des; rpy_des.setZero();
  DVec<T> vel_des(3); vel_des.setZero();
  DVec<T> acc_des(3); acc_des.setZero();

  // Orientation
  Quat<T> quat_des = ori::rpyToQuat(rpy_des);
  _body_ori_task->UpdateTask(&(quat_des), vel_des, acc_des);

  // (X, Y)
  //DVec<T> pos_des(2); pos_des.setZero();
  //vel_des.resize(2); vel_des.setZero();
  //acc_des.resize(2); acc_des.setZero();
  //_body_xy_task->UpdateTask(&(pos_des), vel_des, acc_des);

  // Abduction
  //Vec3<T> abd_pos_des; abd_pos_des.setZero();
  //abd_pos_des[2] = _target_leg_height;

  //_fr_abduction_task->UpdateTask(&(_ini_fr_abduction_pos), vel_des, acc_des);
  //abd_pos_des[1] = -0.18;
  //_fl_abduction_task->UpdateTask(&(abd_pos_des), vel_des, acc_des);
  //abd_pos_des[1] = 0.18;
  //_hr_abduction_task->UpdateTask(&(abd_pos_des), vel_des, acc_des);
  //abd_pos_des[1] = -0.18;
  //_hl_abduction_task->UpdateTask(&(abd_pos_des), vel_des, acc_des);

  
  DVec<T> jpos(4); jpos.setZero();
  DVec<T> jvel(4); jvel.setZero();
  DVec<T> jacc(4); jacc.setZero();
  _abd_joint_task->UpdateTask(&(jpos), jvel, jacc);
  

  // Foot height
  T foot_height(-_target_leg_height);
  vel_des.resize(1); vel_des.setZero();
  acc_des.resize(1); acc_des.setZero();

  _fr_leg_height_task->UpdateTask(&(foot_height), vel_des, acc_des);
  _fl_leg_height_task->UpdateTask(&(foot_height), vel_des, acc_des);
  _hr_leg_height_task->UpdateTask(&(foot_height), vel_des, acc_des);
  _hl_leg_height_task->UpdateTask(&(foot_height), vel_des, acc_des);

  _wbic_data->_Fr_des[2] = 2. * _z_impulse.getValue(Ctrl::_state_machine_time);
  _wbic_data->_Fr_des[5] = 2. * _z_impulse.getValue(Ctrl::_state_machine_time);
 
  // TEST: Body pos
  Vec3<T> body_pos_des;body_pos_des.setZero();
  body_pos_des[2] = _target_leg_height;
  vel_des.resize(3); vel_des.setZero();
  acc_des.resize(3); acc_des.setZero();

  _body_pos_task->UpdateTask(&(body_pos_des), vel_des, acc_des);
 
  // Local Roll
  T roll_cmd(0.);
  vel_des.resize(1); vel_des.setZero();
  acc_des.resize(1); acc_des.setZero();
  _local_roll_task->UpdateTask(&roll_cmd, vel_des, acc_des);

}

template <typename T>
void BoundingInitiateCtrl<T>::_contact_setup(){
  typename std::vector<ContactSpec<T> *>::iterator iter = Ctrl::_contact_list.begin();
  while(iter < Ctrl::_contact_list.end()){
    (*iter)->UpdateContactSpec();
    ++iter;
  }
}

template <typename T>
void BoundingInitiateCtrl<T>::FirstVisit(){
  //pretty_print(Ctrl::_robot_sys->getMassMatrix(), std::cout, "Mass");
  T total_mass = Ctrl::_robot_sys->getMassMatrix()(5,5);
  T apex = total_mass * 9.81 * _end_time/(2.0*0.7*_stance_time);

  printf("apex: %f\n", apex);
  _z_impulse.setCurve(apex, _stance_time);

  _ini_fr_abduction_pos =
    Ctrl::_robot_sys->_pGC[linkID::FR_abd] -
    Ctrl::_robot_sys->_pGC[linkID::FR];

  _ctrl_start_time = _sp->_curr_time;
}

template <typename T>
void BoundingInitiateCtrl<T>::LastVisit(){}

template <typename T>
bool BoundingInitiateCtrl<T>::EndOfPhase(){
  if(Ctrl::_state_machine_time > (_end_time-2.*Test<T>::dt)){
    return true;
  }
  return false;
}

template <typename T>
void BoundingInitiateCtrl<T>::CtrlInitialization(const std::string & category_name){
  (void)category_name;
}

template <typename T>
void BoundingInitiateCtrl<T>::SetTestParameter(const std::string & test_file){
  _param_handler = new ParamHandler(test_file);
  std::vector<T> tmp_vec;
  if(!_param_handler->getValue<T>("leg_height", _target_leg_height)){
    printf("[BoundingInitiate] No leg height\n");
  }
  _param_handler->getValue<T>("swing_time", _swing_time);
  _param_handler->getValue<T>("default_stance_time", _stance_time);
  _end_time = (_swing_time + _stance_time)/2.;

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


  // Joint level feedback gain
  if(!_param_handler->getVector<T>("Kp_joint", _Kp_joint)){
    printf("no Kp joint setting\n");
    exit(0);
  }
}


template <typename T>
BoundingInitiateCtrl<T>::~BoundingInitiateCtrl(){
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


template class BoundingInitiateCtrl<double>;
template class BoundingInitiateCtrl<float>;
