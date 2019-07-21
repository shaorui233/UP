#include "LocomotionCtrl.hpp"
#include <WBC_Ctrl/ContactSet/SingleContact.hpp>
#include <WBC_Ctrl/TaskSet/BodyOriTask.hpp>
#include <WBC_Ctrl/TaskSet/BodyPosTask.hpp>
#include <WBC_Ctrl/TaskSet/LinkPosTask.hpp>

template<typename T>
LocomotionCtrl<T>::LocomotionCtrl(FloatingBaseModel<T> model):
  WBC_Ctrl<T>(model)
{
  _body_pos_task = new BodyPosTask<T>(&(WBCtrl::_model));
  _body_ori_task = new BodyOriTask<T>(&(WBCtrl::_model));


  _foot_contact[0] = new SingleContact<T>(&(WBCtrl::_model), linkID::FR);
  _foot_contact[1] = new SingleContact<T>(&(WBCtrl::_model), linkID::FL);
  _foot_contact[2] = new SingleContact<T>(&(WBCtrl::_model), linkID::HR);
  _foot_contact[3] = new SingleContact<T>(&(WBCtrl::_model), linkID::HL);

  _foot_task[0] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::FR);
  _foot_task[1] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::FL);
  _foot_task[2] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::HR);
  _foot_task[3] = new LinkPosTask<T>(&(WBCtrl::_model), linkID::HL);
}

template<typename T>
LocomotionCtrl<T>::~LocomotionCtrl(){
  delete _body_pos_task;
  delete _body_ori_task;

  for(size_t i (0); i<4; ++i){
    delete _foot_contact[i];
    delete _foot_task[i];
  }
}

template<typename T>
void LocomotionCtrl<T>::_ContactTaskUpdate(void* input, ControlFSMData<T> & data){
  _input_data = static_cast<LocomotionCtrlData<T>* >(input);

  _ParameterSetup(data.userParameters);
  
  // Wash out the previous setup
  _CleanUp();

  // Task & Contact Set
  Vec3<T> zero_vec3; zero_vec3.setZero();

  Quat<T> quat_des = ori::rpyToQuat(_input_data->pBody_RPY_des);
  _body_ori_task->UpdateTask(&quat_des, _input_data->vBody_Ori_des, zero_vec3);
  _body_pos_task->UpdateTask(
      &(_input_data->pBody_des), 
      _input_data->vBody_des, 
      _input_data->aBody_des);

  WBCtrl::_task_list.push_back(_body_ori_task);
  WBCtrl::_task_list.push_back(_body_pos_task);

  for(size_t leg(0); leg<4; ++leg){
    if(_input_data->contact_state[leg] > 0.){ // Contact
      _foot_contact[leg]->setRFDesired((DVec<T>)(_input_data->Fr_des[leg]));
      _foot_contact[leg]->UpdateContactSpec();
      WBCtrl::_contact_list.push_back(_foot_contact[leg]);

    }else{ // No Contact (swing)
      _foot_task[leg]->UpdateTask(
          &(_input_data->pFoot_des[leg]), 
          _input_data->vFoot_des[leg], 
          _input_data->aFoot_des[leg]);
      WBCtrl::_task_list.push_back(_foot_task[leg]);
    }
  }

  _LCM_PublishData();
}

template<typename T>
void LocomotionCtrl<T>::_ParameterSetup(const MIT_UserParameters* param){
  WBCtrl::_wbic_data->_W_floating = DVec<T>::Constant(6, param->wbc_base_Fr_weight);

  for(size_t i(0); i<3; ++i){
    ((BodyPosTask<T>*)_body_pos_task)->_Kp[i] = param->Kp_body[i];
    ((BodyPosTask<T>*)_body_pos_task)->_Kd[i] = param->Kp_body[i];

    ((BodyOriTask<T>*)_body_ori_task)->_Kp[i] = param->Kp_ori[i];
    ((BodyOriTask<T>*)_body_ori_task)->_Kd[i] = param->Kp_ori[i];

    WBCtrl::_Kp_joint[i] = param->Kp_joint[i];
    WBCtrl::_Kd_joint[i] = param->Kd_joint[i];
  }
}


template<typename T>
void LocomotionCtrl<T>::_CleanUp(){
  WBCtrl::_contact_list.clear();
  WBCtrl::_task_list.clear();
}

template<typename T>
void LocomotionCtrl<T>::_LCM_PublishData() {

  //int iter(0);
  //for(size_t leg(0); leg<4; ++leg){
    //_Fr_result[leg].setZero();
    //if(contact_state[leg]>0.){
      //for(size_t i(0); i<3; ++i){
        //_Fr_result[leg][i] = _wbic_data->_Fr[3*iter + i];
      //}
      //++iter;
    //}
  //}

  //for(size_t i(0); i<3; ++i){
    //_wbc_data_lcm.foot_pos[i] = _model._pGC[linkID::FR][i];
    //_wbc_data_lcm.foot_vel[i] = _model._vGC[linkID::FR][i];

    //_wbc_data_lcm.foot_pos[i + 3] = _model._pGC[linkID::FL][i];
    //_wbc_data_lcm.foot_vel[i + 3] = _model._vGC[linkID::FL][i];

    //_wbc_data_lcm.foot_pos[i + 6] = _model._pGC[linkID::HR][i];
    //_wbc_data_lcm.foot_vel[i + 6] = _model._vGC[linkID::HR][i];

    //_wbc_data_lcm.foot_pos[i + 9] = _model._pGC[linkID::HL][i];
    //_wbc_data_lcm.foot_vel[i + 9] = _model._vGC[linkID::HL][i];


    //for(size_t leg(0); leg<4; ++leg){
      //_wbc_data_lcm.Fr_des[3*leg + i] = Fr_des[leg][i];
      //_wbc_data_lcm.Fr[3*leg + i] = _Fr_result[leg][i];

      //_wbc_data_lcm.foot_pos_cmd[3*leg + i] = pFoot_des[leg][i];
      //_wbc_data_lcm.foot_vel_cmd[3*leg + i] = vFoot_des[leg][i];
      //_wbc_data_lcm.foot_acc_cmd[3*leg + i] = aFoot_des[leg][i];

      //_wbc_data_lcm.jpos_cmd[3*leg + i] = _des_jpos[3*leg + i];
      //_wbc_data_lcm.jvel_cmd[3*leg + i] = _des_jvel[3*leg + i];
      //_wbc_data_lcm.jacc_cmd[3*leg + i] = _des_jacc[3*leg + i];

      //_wbc_data_lcm.jpos[3*leg + i] = _state.q[3*leg + i];
      //_wbc_data_lcm.jvel[3*leg + i] = _state.qd[3*leg + i];
    //}

    //_wbc_data_lcm.body_pos_cmd[i] = pBody_des[i];
    //_wbc_data_lcm.body_vel_cmd[i] = vBody_des[i];
    //_wbc_data_lcm.body_ori_cmd[i] = quat_des[i];

    //_wbc_data_lcm.body_pos[i] = _state.bodyPosition[i];
    //_wbc_data_lcm.body_vel[i] = _state.bodyVelocity[i+3];
    //_wbc_data_lcm.body_ori[i] = _state.bodyOrientation[i];
  //}
  //_wbc_data_lcm.body_ori_cmd[3] = quat_des[3];
  //_wbc_data_lcm.body_ori[3] = _state.bodyOrientation[3];

  WBCtrl::_wbcLCM.publish("wbc_lcm_data", &(WBCtrl::_wbc_data_lcm) );
}

template class LocomotionCtrl<float>;
template class LocomotionCtrl<double>;
