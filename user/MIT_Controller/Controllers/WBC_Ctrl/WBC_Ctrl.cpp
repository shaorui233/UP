#include "WBC_Ctrl.hpp"
#include <WBC_Ctrl/TaskSet/BodyOriTask.hpp>
#include <WBC_Ctrl/TaskSet/BodyPosTask.hpp>
#include <WBC_Ctrl/TaskSet/LinkPosTask.hpp>

#include <WBC_Ctrl/ContactSet/SingleContact.hpp>

#include <Dynamics/Quadruped.h>
#include <Utilities/Utilities_print.h>


template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(FloatingBaseModel<T> model):
  _full_config(cheetah::num_act_joint + 7),
  _tau_ff(cheetah::num_act_joint),
  _des_jpos(cheetah::num_act_joint),
  _des_jvel(cheetah::num_act_joint),
  _des_jacc(cheetah::num_act_joint),
_wbcLCM(getLcmUrl(255))
{
  _iter = 0;
  _full_config.setZero();

  _model = model;
  _body_pos_task = new BodyPosTask<T>(&_model);
  _body_ori_task = new BodyOriTask<T>(&_model);


  _foot_contact[0] = new SingleContact<T>(&_model, linkID::FR);
  _foot_contact[1] = new SingleContact<T>(&_model, linkID::FL);
  _foot_contact[2] = new SingleContact<T>(&_model, linkID::HR);
  _foot_contact[3] = new SingleContact<T>(&_model, linkID::HL);

  _foot_task[0] = new LinkPosTask<T>(&_model, linkID::FR);
  _foot_task[1] = new LinkPosTask<T>(&_model, linkID::FL);
  _foot_task[2] = new LinkPosTask<T>(&_model, linkID::HR);
  _foot_task[3] = new LinkPosTask<T>(&_model, linkID::HL);

  _kin_wbc = new KinWBC<T>(cheetah::dim_config);
  _wbic = new WBIC<T>(cheetah::dim_config, &(_contact_list), &(_task_list));

  _wbic_data = new WBIC_ExtraData<T>();
  _wbic_data->_W_floating = DVec<T>::Constant(6, 0.1);
  _wbic_data->_W_rf = DVec<T>::Constant(12, 1.);

  _Kp_joint.resize(cheetah::num_leg_joint, 5.);
  _Kd_joint.resize(cheetah::num_leg_joint, 1.5);

  //_Kp_joint.resize(cheetah::num_leg_joint, 3.);
  //_Kd_joint.resize(cheetah::num_leg_joint, 0.5);


  //_Kp_joint.resize(cheetah::num_leg_joint, 0.);
  //_Kd_joint.resize(cheetah::num_leg_joint, 0.);

  _state.q = DVec<T>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<T>::Zero(cheetah::num_act_joint);
}

template<typename T>
WBC_Ctrl<T>::~WBC_Ctrl(){
}

template<typename T>
void WBC_Ctrl<T>::run(
    const Vec3<T> & pBody_des, const Vec3<T> & vBody_des, const Vec3<T> & aBody_des,
    const Vec3<T> & pBody_RPY_des, const Vec3<T> & vBody_Ori_des, 
    const Vec3<T>* pFoot_des, const Vec3<T>* vFoot_des, const Vec3<T>* aFoot_des,
    const Vec3<T>* Fr_des, const Vec4<T> & contact_state,
    ControlFSMData<T> & data){

  _ParameterSetup(data.userParameters);
  ++_iter;
  // First visit
  static bool first_visit(true);
  if(first_visit){
    _quat_ini = data._stateEstimator->getResult().orientation;
    _body_des_ini = pBody_des;
    first_visit = false;
  }

  // Update Model
  _UpdateModel(data._stateEstimator->getResult(), data._legController->datas);

  // Wash out the previous setup
  _CleanUp();

  // Task & Contact Set
  Vec3<T> zero_vec3; zero_vec3.setZero();

  Quat<T> quat_des = ori::rpyToQuat(pBody_RPY_des);
  //pretty_print(pBody_RPY_des, std::cout, "rpy_des");
  _body_ori_task->UpdateTask(&quat_des, vBody_Ori_des, zero_vec3);
  _body_pos_task->UpdateTask(&pBody_des, vBody_des, aBody_des);

  _task_list.push_back(_body_ori_task);
  _task_list.push_back(_body_pos_task);

  for(size_t leg(0); leg<4; ++leg){
    if(contact_state[leg] > 0.){ // Contact
      _foot_contact[leg]->setRFDesired((DVec<T>)Fr_des[leg]);
      _foot_contact[leg]->UpdateContactSpec();
      _contact_list.push_back(_foot_contact[leg]);

    }else{ // No Contact (swing)
      _foot_task[leg]->UpdateTask(&pFoot_des[leg], vFoot_des[leg], aFoot_des[leg]);
      _task_list.push_back(_foot_task[leg]);
    }
  }

  // WBC Computation
  _ComputeWBC();

  // Update Leg Command
  _UpdateLegCMD(data._legController->commands);

  //_print_summary();
  //pretty_print(contact_state, std::cout, "contact_state");
  //pretty_print(pBody_RPY_des, std::cout, "RPY_des");

  _LCM_PublishData(pBody_des, vBody_des, quat_des, 
      pFoot_des, vFoot_des, aFoot_des, Fr_des, contact_state);
}
template<typename T>
void WBC_Ctrl<T>::_ParameterSetup(const MIT_UserParameters* param){
  _wbic_data->_W_floating = DVec<T>::Constant(6, param->wbc_base_Fr_weight);

  for(size_t i(0); i<3; ++i){
    ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = param->Kp_body[i];
    ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = param->Kp_body[i];

    ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = param->Kp_ori[i];
    ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = param->Kp_ori[i];

    _Kp_joint[i] = param->Kp_joint[i];
    _Kd_joint[i] = param->Kd_joint[i];
  }
}

template<typename T>
void WBC_Ctrl<T>::_LCM_PublishData(
    const Vec3<T> & pBody_des, const Vec3<T> & vBody_des, 
    const Quat<T> & quat_des, 
    const Vec3<T>* pFoot_des, const Vec3<T>* vFoot_des, const Vec3<T>* aFoot_des,
    const Vec3<T>* Fr_des, const Vec4<T> & contact_state)
{
  (void)contact_state;
  int iter(0);
  for(size_t leg(0); leg<4; ++leg){
    _Fr_result[leg].setZero();
    if(contact_state[leg]>0.){
      for(size_t i(0); i<3; ++i){
        _Fr_result[leg][i] = _wbic_data->_Fr[3*iter + i];
      }
      ++iter;
    }
  }

  for(size_t i(0); i<3; ++i){
    _wbc_data_lcm.foot_pos[i] = _model._pGC[linkID::FR][i];
    _wbc_data_lcm.foot_vel[i] = _model._vGC[linkID::FR][i];

    _wbc_data_lcm.foot_pos[i + 3] = _model._pGC[linkID::FL][i];
    _wbc_data_lcm.foot_vel[i + 3] = _model._vGC[linkID::FL][i];

    _wbc_data_lcm.foot_pos[i + 6] = _model._pGC[linkID::HR][i];
    _wbc_data_lcm.foot_vel[i + 6] = _model._vGC[linkID::HR][i];

    _wbc_data_lcm.foot_pos[i + 9] = _model._pGC[linkID::HL][i];
    _wbc_data_lcm.foot_vel[i + 9] = _model._vGC[linkID::HL][i];


    for(size_t leg(0); leg<4; ++leg){
      _wbc_data_lcm.Fr_des[3*leg + i] = Fr_des[leg][i];
      _wbc_data_lcm.Fr[3*leg + i] = _Fr_result[leg][i];

      _wbc_data_lcm.foot_pos_cmd[3*leg + i] = pFoot_des[leg][i];
      _wbc_data_lcm.foot_vel_cmd[3*leg + i] = vFoot_des[leg][i];
      _wbc_data_lcm.foot_acc_cmd[3*leg + i] = aFoot_des[leg][i];

      _wbc_data_lcm.jpos_cmd[3*leg + i] = _des_jpos[3*leg + i];
      _wbc_data_lcm.jvel_cmd[3*leg + i] = _des_jvel[3*leg + i];
      _wbc_data_lcm.jacc_cmd[3*leg + i] = _des_jacc[3*leg + i];

      _wbc_data_lcm.jpos[3*leg + i] = _state.q[3*leg + i];
      _wbc_data_lcm.jvel[3*leg + i] = _state.qd[3*leg + i];
    }

    _wbc_data_lcm.body_pos_cmd[i] = pBody_des[i];
    _wbc_data_lcm.body_vel_cmd[i] = vBody_des[i];
    _wbc_data_lcm.body_ori_cmd[i] = quat_des[i];

    _wbc_data_lcm.body_pos[i] = _state.bodyPosition[i];
    _wbc_data_lcm.body_vel[i] = _state.bodyVelocity[i+3];
    _wbc_data_lcm.body_ori[i] = _state.bodyOrientation[i];
  }
  _wbc_data_lcm.body_ori_cmd[3] = quat_des[3];
  _wbc_data_lcm.body_ori[3] = _state.bodyOrientation[3];

  _wbcLCM.publish("wbc_lcm_data", &_wbc_data_lcm);
}

template <typename T>
void WBC_Ctrl<T>::_print_summary() {
  pretty_print(_tau_ff, std::cout, "tau_ff");
  pretty_print(_des_jpos, std::cout, "des_jpos");
  pretty_print(_des_jvel, std::cout, "des_jvel");
}


template <typename T>
void WBC_Ctrl<T>::_ComputeWBC() {
  _kin_wbc->FindConfiguration(_full_config, _task_list, _contact_list,
                              _des_jpos, _des_jvel, _des_jacc);

  // WBIC
  _wbic->UpdateSetting(_A, _Ainv, _coriolis, _grav);
  _wbic->MakeTorque(_tau_ff, _wbic_data);
}


template<typename T>
void WBC_Ctrl<T>::_UpdateLegCMD(LegControllerCommand<T> * cmd){
  for (size_t leg(0); leg < cheetah::num_leg; ++leg) {
    cmd[leg].zero();
    for (size_t jidx(0); jidx < cheetah::num_leg_joint; ++jidx) {
      cmd[leg].tauFeedForward[jidx] = _tau_ff[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].qDes[jidx] = _des_jpos[cheetah::num_leg_joint * leg + jidx];
      cmd[leg].qdDes[jidx] = _des_jvel[cheetah::num_leg_joint * leg + jidx];

      cmd[leg].kpJoint(jidx, jidx) = _Kp_joint[jidx];
      cmd[leg].kdJoint(jidx, jidx) = _Kd_joint[jidx];
    }
  }
}

template<typename T>
void WBC_Ctrl<T>::_UpdateModel(const StateEstimate<T> & state_est, 
    const LegControllerData<T> * leg_data){

  _state.bodyOrientation = state_est.orientation;
  //_state.bodyOrientation = ori::rpyToQuat(state_est.rpy);
  _state.bodyPosition = state_est.position;
  for(size_t i(0); i<3; ++i){
    _state.bodyVelocity[i] = state_est.omegaBody[i];
    _state.bodyVelocity[i+3] = state_est.vBody[i];

    for(size_t leg(0); leg<4; ++leg){
      _state.q[3*leg + i] = leg_data[leg].q[i];
      _state.qd[3*leg + i] = leg_data[leg].qd[i];

      _full_config[3*leg + i + 6] = _state.q[3*leg + i];
    }
  }
  _model.setState(_state);

  //_model.forwardKinematics();
  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();

  _A = _model.getMassMatrix();
  _grav = _model.getGravityForce();
  _coriolis = _model.getCoriolisForce();
  _Ainv = _A.inverse();

  //if(_iter%100 ==0){
  //pretty_print(_state.bodyVelocity, std::cout, "body vel");
  //}
}

template<typename T>
void WBC_Ctrl<T>::_CleanUp(){
  _contact_list.clear();
  _task_list.clear();
}


template class WBC_Ctrl<float>;
template class WBC_Ctrl<double>;
