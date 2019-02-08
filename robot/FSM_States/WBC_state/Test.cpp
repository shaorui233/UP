#include "Test.hpp"

template <typename T>
Test<T>::Test():b_first_visit_(true){}

template <typename T>
Test<T>::~Test(){}

template <typename T>
void Test<T>::getCommand(void* command){
  if(b_first_visit_){
    state_list_[phase_]->FirstVisit();
    b_first_visit_ = false;
  }

  state_list_[phase_]->OneStep(command);

  if(state_list_[phase_]->EndOfPhase()){
    state_list_[phase_]->LastVisit();
    phase_ = _NextPhase(phase_);
    b_first_visit_ = true;
  }
}

template class Test<double>;
template class Test<float>;
