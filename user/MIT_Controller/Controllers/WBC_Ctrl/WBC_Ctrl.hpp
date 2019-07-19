#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include <FSM_States/ControlFSMData.h>
#include <Dynamics/FloatingBaseModel.h>
#include <Dynamics/Quadruped.h>
#include "cppTypes.h"
#include <WBC/WBIC/WBIC.hpp>
#include <WBC/WBLC/KinWBC.hpp>

#include <lcm-cpp.hpp>
#include "wbc_test_data_t.hpp"

class MIT_UserParameters;

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
    void _UpdateModel(const StateEstimate<T> & state_est, const LegControllerData<T> * leg_data);
    void _UpdateLegCMD(LegControllerCommand<T> * cmd);
    void _ComputeWBC();
    void _print_summary();
    void _LCM_PublishData(
        const Vec3<T> & pBody_des, const Vec3<T> & vBody_des, const Quat<T> & quat_des, 
        const Vec3<T>* pFoot_des, const Vec3<T>* vFoot_des, const Vec3<T>* aFoot_des,
        const Vec3<T>* Fr_des, const Vec4<T> & contact_state);
    void _ParameterSetup(const MIT_UserParameters* param);

    KinWBC<T>* _kin_wbc;
    WBIC<T>* _wbic;
    WBIC_ExtraData<T>* _wbic_data;
    Vec3<T> _Fr_result[4];

    FloatingBaseModel<T> _model;
    Task<T>* _body_pos_task;
    Task<T>* _body_ori_task;

    Task<T>* _foot_task[4];
    ContactSpec<T>* _foot_contact[4];

    std::vector<ContactSpec<T> * > _contact_list;
    std::vector<Task<T> * > _task_list;

    Quat<T> _quat_ini;
    Vec3<T> _body_des_ini;

    DMat<T> _A;
    DMat<T> _Ainv;
    DVec<T> _grav;
    DVec<T> _coriolis;

    FBModelState<T> _state;

    DVec<T> _full_config;
    DVec<T> _tau_ff;
    DVec<T> _des_jpos;
    DVec<T> _des_jvel;
    DVec<T> _des_jacc;

    long long _iter;

    std::vector<T> _Kp_joint, _Kd_joint;

  lcm::LCM _wbcLCM;
  wbc_test_data_t _wbc_data_lcm;
};
#endif
