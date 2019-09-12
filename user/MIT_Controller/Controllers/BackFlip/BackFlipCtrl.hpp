#ifndef BACKFLIP_CTRL
#define BACKFLIP_CTRL

#include <Controllers/BackFlip/DataReader.hpp>
#include <Dynamics/FloatingBaseModel.h>
#include <Controllers/LegController.h>

template <typename T>
class BackFlipCtrl {
 public:
  BackFlipCtrl(DataReader*, float _dt);
  ~BackFlipCtrl();

  void OneStep(float _curr_time, bool b_preparation, LegControllerCommand<T>* command);
  void FirstVisit(float _curr_time);
  void LastVisit();
  bool EndOfPhase(LegControllerData<T>* data);
  void SetParameter();

 protected:
  DataReader* _data_reader;

  DVec<T> _Kp, _Kd;
  DVec<T> _des_jpos;
  DVec<T> _des_jvel;
  DVec<T> _jtorque;

  T dt;

  std::vector<T> _Kp_joint, _Kd_joint;

  bool _b_BackFlipPreparation = false;

  bool _b_set_height_target;
  T _end_time = 5.5;
  int _dim_contact;

  void _update_joint_command();

  T _ctrl_start_time;
  T _q_knee_max = 2.0;
  T _qdot_knee_max = 2.0;

  T _state_machine_time;

  int _key_pt_step = 1;
  int current_iteration, pre_mode_count;
};

#endif
