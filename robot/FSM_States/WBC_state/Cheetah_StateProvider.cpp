#include "Cheetah_StateProvider.hpp"
#include "Cheetah_DynaCtrl_Definition.h"

template<typename T>
Cheetah_StateProvider<T>* Cheetah_StateProvider<T>::getStateProvider(){
    static Cheetah_StateProvider state_provider_;
    return &state_provider_;
}

template<typename T>
Cheetah_StateProvider<T>::Cheetah_StateProvider():
                                Q_(cheetah::num_q),
                                Qdot_(cheetah::dim_config),
                                curr_time_(0.)
{
  Q_.setZero();
  Qdot_.setZero();
}

template class Cheetah_StateProvider<double> ;
template class Cheetah_StateProvider<float> ;

