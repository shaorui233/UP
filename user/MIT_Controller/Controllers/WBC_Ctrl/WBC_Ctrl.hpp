#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include <FSM_States/ControlFSMData.h>
#include <Dynamics/FloatingBaseModel.h>
#include <Dynamics/Quadruped.h>
#include "cppTypes.h"

template <typename T> class ContactSpec;
template <typename T> class Task;
template <typename T> class KinWBC;
template <typename T> class WBIC;
template <typename T> class WBIC_ExtraData;


template<typename T>
class WBC_Ctrl{
  public:
    WBC_Ctrl(FloatingBaseModel<T> model);
    ~WBC_Ctrl();

    void run(
        const Vec3<T> & pBody_des, const Vec3<T> & vBody_des, const Vec3<T> & aBody_des,
        const Vec3<T> & pBody_RPY_des, const Vec3<T> & vBody_Ori_des, 
        const Vec3<T>* pFoot_des, const Vec3<T>* vFoot_des, const Vec3<T>* aFoot_des,
        const Vec3<T>* Fr_des, const Vec4<T> & contact_state,
        ControlFSMData<T> & data);

  private:
    void _CleanUp();

    FloatingBaseModel<T> _model;
    Task<T>* _body_pos_task;
    Task<T>* _body_ori_task;

    Task<T>* _foot_task[4];
    ContactSpec<T>* _foot_contact[4];

    std::vector<ContactSpec<T> * > _contact_list;
    std::vector<Task<T> * > _task_list;
};
#endif
