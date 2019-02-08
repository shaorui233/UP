#ifndef STATE_PROVIDER_Cheetah
#define STATE_PROVIDER_Cheetah

#include <cppTypes.h>
#include <Configuration.h>

template <typename T>
class Cheetah_StateProvider{
public:
  static Cheetah_StateProvider<T>* getStateProvider();
  ~Cheetah_StateProvider(){}

  DVec<T> Q_;
  DVec<T> Qdot_;
  DVec<T> jpos_ini_;
  DVec<T> des_jpos_prev_;

  T curr_time_;
private:
  Cheetah_StateProvider();
};


#endif
