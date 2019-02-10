#include "BodyCtrlTest.hpp"

#include <WBC_state/CtrlSet/BodyCtrl.hpp>
#include <WBC_state/CtrlSet/FullContactTransCtrl.hpp>
#include <WBC_state/Cheetah_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

template <typename T>
BodyCtrlTest<T>::BodyCtrlTest(const FloatingBaseModel<T>* robot):Test<T>(){
    //phase_ = BodyCtrlPhase::BDCTRL_body_ctrl;
    Test<T>::phase_ = BodyCtrlPhase::BDCTRL_body_up_ctrl;
    Test<T>::state_list_.clear();

    body_up_ctrl_ = new FullContactTransCtrl<T>(robot);
    body_ctrl_ = new BodyCtrl<T>(robot);
    
    Test<T>::state_list_.push_back(body_up_ctrl_);
    Test<T>::state_list_.push_back(body_ctrl_);

    _SettingParameter();

    printf("[Body Position Control Test] Constructed\n");
}

template <typename T>
BodyCtrlTest<T>::~BodyCtrlTest(){
  for(size_t i(0); i<Test<T>::state_list_.size(); ++i){
    delete Test<T>::state_list_[i];
  }
}

template <typename T>
void BodyCtrlTest<T>::TestInitialization(){
  // Yaml file name
  body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
  body_ctrl_->CtrlInitialization("CTRL_stance");
}

template <typename T>
int BodyCtrlTest<T>::_NextPhase(const int & phase){
  int next_phase = phase + 1;
  printf("next phase: %i\n", next_phase);
  if (next_phase == NUM_BDCTRL_PHASE) {
    return BodyCtrlPhase::BDCTRL_body_ctrl;
  }
  else return next_phase;
}

template <typename T>
void BodyCtrlTest<T>::_SettingParameter(){
    typename std::vector< Controller<T> *>::iterator iter 
        = Test<T>::state_list_.begin();

    while(iter < Test<T>::state_list_.end()){
        (*iter)->SetTestParameter(
                CheetahConfigPath"TEST_body_ctrl.yaml");
        ++iter;
    }
}

template class BodyCtrlTest<double>;
template class BodyCtrlTest<float>;
