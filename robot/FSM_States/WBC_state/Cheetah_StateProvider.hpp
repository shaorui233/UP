#ifndef STATE_PROVIDER_Cheetah
#define STATE_PROVIDER_Cheetah

#include <cppTypes.h>
#include "Cheetah_DynaCtrl_Definition.h"

template <typename T>
class Cheetah_StateProvider{
public:
  static Cheetah_StateProvider<T>* getStateProvider();
  ~Cheetah_StateProvider(){}

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
  Vec3<T> _body_target;
private:
  Cheetah_StateProvider();
};


#endif
