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
  _des_jacc(cheetah::num_act_joint)
{
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
  _wbic_data->_W_floating = DVec<T>::Constant(6, 5.);
  _wbic_data->_W_rf = DVec<T>::Constant(12, 1.);

  _Kp_joint.resize(cheetah::num_leg_joint, 3.0);
  _Kd_joint.resize(cheetah::num_leg_joint, 1.0);

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

  // Update Model
  _UpdateModel(data._stateEstimator->getResult(), data._legController->datas);

  // Wash out the previous setup
  _CleanUp();

  // Task & Contact Set
  Vec3<T> empty_vec3; empty_vec3.setZero();

  Quat<T> quat_des = ori::rpyToQuat(pBody_RPY_des);
  _body_ori_task->UpdateTask(&quat_des, vBody_Ori_des, empty_vec3);
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
  _state.bodyPosition = state_est.position;
  for(size_t i(0); i<3; ++i){
    _state.bodyVelocity[i] = state_est.vBody[i];
    _state.bodyVelocity[i+3] = state_est.omegaBody[i];

    for(size_t leg(0); leg<4; ++leg){
      _state.q[3*leg + i] = leg_data[leg].q[i];
      _state.qd[3*leg + i] = leg_data[leg].qd[i];

      _full_config[3*leg + i + 6] = _state.q[3*leg + i];
    }
  }
  _model.setState(_state);

  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();

  _A = _model.getMassMatrix();
  _grav = _model.getGravityForce();
  _coriolis = _model.getCoriolisForce();
  _Ainv = _A.inverse();
}

template<typename T>
void WBC_Ctrl<T>::_CleanUp(){
  _contact_list.clear();
  _task_list.clear();
}


template class WBC_Ctrl<float>;
template class WBC_Ctrl<double>;
