#include "WBC_Ctrl.hpp"
#include <WBC_Ctrl/TaskSet/BodyOriTask.hpp>
#include <WBC_Ctrl/TaskSet/BodyPosTask.hpp>
#include <WBC_Ctrl/TaskSet/LinkPosTask.hpp>

#include <WBC_Ctrl/ContactSet/SingleContact.hpp>


template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(FloatingBaseModel<T> model){
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

  printf("WBC_Ctrl\n");
  (void)data;
  _CleanUp();

  // Task & Contact Set
  Vec3<T> empty_vec3; empty_vec3.setZero();

  _body_pos_task->UpdateTask(&pBody_des, vBody_des, aBody_des);
  _body_ori_task->UpdateTask(&pBody_RPY_des, vBody_Ori_des, empty_vec3);

  for(size_t leg(0); leg<4; ++leg){
    if(contact_state[leg] > 0.){
      _foot_contact[leg]->setRFDesired((DVec<T>)Fr_des[leg]);
      _foot_contact[leg]->UpdateContactSpec();
      _contact_list.push_back(_foot_contact[leg]);

    }else{ // No Contact (swing)
      _foot_task[leg]->UpdateTask(&pFoot_des[leg], vFoot_des[leg], aFoot_des[leg]);
      _task_list.push_back(_foot_task[leg]);
    }
  }
}

template<typename T>
void WBC_Ctrl<T>::_CleanUp(){
  _contact_list.clear();
  _task_list.clear();
}


template class WBC_Ctrl<float>;
template class WBC_Ctrl<double>;
