#ifndef BACKFLIP_CTRL
#define BACKFLIP_CTRL

#include <ParamHandler/ParamHandler.hpp>
#include <Controllers/BackFlip/DataReader.hpp>
#include <Dynamics/FloatingBaseModel.h>
#include <Controllers/LegController.h>

template <typename T>
class BackFlipCtrl {
 public:
  BackFlipCtrl(const FloatingBaseModel<T>, DataReader*, float _dt);
  ~BackFlipCtrl();

  void OneStep(float _curr_time, LegControllerCommand<T>* command);
  void FirstVisit(float _curr_time);
  void LastVisit();
  bool EndOfPhase(LegControllerData<T>* data);

  void CtrlInitialization(const std::string& category_name);
  void SetTestParameter(const std::string& test_file);

  DMat<T> _A;
  DMat<T> _Ainv;
  DVec<T> _grav;
  DVec<T> _coriolis;

 protected:
  DataReader* _data_reader;

  DVec<T> _Kp, _Kd;
  DVec<T> _des_jpos;
  DVec<T> _des_jvel;
  DVec<T> _jtorque;

  T dt;

  std::vector<T> _Kp_joint, _Kd_joint;

  bool _b_set_height_target;
  T _end_time = 5.5;
  int _dim_contact;

  void _update_joint_command();

  T _ctrl_start_time;
  T _q_knee_max;
  T _qdot_knee_max;

  T _state_machine_time;

  ParamHandler* _param_handler;
  FloatingBaseModel<T> _model;

  int current_iteration, pre_mode_count;

  void _PreProcessing_Command() {
    _A = _model.getMassMatrix();
    _grav = _model.getGravityForce();
    _coriolis = _model.getCoriolisForce();
    _Ainv = _A.inverse();
  }

  void _PostProcessing_Command() {
    // for(size_t i(0); i<_task_list.size(); ++i){ _task_list[i]->UnsetTask(); }
    // for(size_t i(0); i<_contact_list.size(); ++i){
    // _contact_list[i]->UnsetContact(); }
  }

};

#endif
