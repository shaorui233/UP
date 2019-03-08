#ifndef STATE_PROVIDER_Cheetah
#define STATE_PROVIDER_Cheetah

#include <cppTypes.h>
#include "Cheetah_DynaCtrl_Definition.h"

template <typename T>
class Cheetah_StateProvider{
public:
  static Cheetah_StateProvider<T>* getStateProvider();
  ~Cheetah_StateProvider(){}

  void UpdateYawTargetRot(const T & yaw);
  void UpdateExtraData(Cheetah_Extra_Data<T> * ext_data);
  vectorAligned< DVec<T> > _foot_step_list;

  DVec<T> Q_;
  DVec<T> Qdot_;
  DVec<T> jpos_ini_;
  DVec<T> _jpos_des_pre;
  DVec<T> des_jpos_prev_;

  size_t _contact_pt[cheetah::num_leg];
  size_t _num_contact;
  Vec3<T> _local_frame_global_pos;

  T curr_time_;

  T _dir_command[2];
  T _ori_command[3];
  Vec3<T> _target_ori_command;
  Vec3<T> _body_target;

  Mat3<T> _YawRot;

  // For optimization replay
  int _num_step;
  bool _opt_play;
  DVec<T> upcoming_step_pos;
private:
  Cheetah_StateProvider();
};


#endif
