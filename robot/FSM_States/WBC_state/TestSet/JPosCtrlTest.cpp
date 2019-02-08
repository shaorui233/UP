#include "JPosCtrlTest.hpp"

#include <WBC_state/CtrlSet/JPosCtrl.hpp>
#include <WBC_state/Cheetah_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

template <typename T>
JPosCtrlTest<T>::JPosCtrlTest(const FloatingBaseModel<T>* robot):Test<T>(){
    Test<T>::phase_ = JPosCtrlPhase::JPCTRL_move_to_target;
    Test<T>::state_list_.clear();

    ini_jpos_ctrl_ = new JPosCtrl<T>(robot);
    jpos_swing_ = new JPosCtrl<T>(robot);
    
    Test<T>::state_list_.push_back(ini_jpos_ctrl_);
    Test<T>::state_list_.push_back(jpos_swing_);

    _SettingParameter();

    printf("[Joint Position Control Test] Constructed\n");
}

template <typename T>
JPosCtrlTest<T>::~JPosCtrlTest(){
  for(size_t i(0); i<Test<T>::state_list_.size(); ++i){
    delete Test<T>::state_list_[i];
  }
}

template <typename T>
void JPosCtrlTest<T>::TestInitialization(){
  // Yaml file name
  ini_jpos_ctrl_->CtrlInitialization("CTRL_jpos_move_to_target");
  jpos_swing_->CtrlInitialization("CTRL_jpos_swing");
}

template <typename T>
int JPosCtrlTest<T>::_NextPhase(const int & phase){
  int next_phase = phase + 1;
  printf("next phase: %i\n", next_phase);
  if (next_phase == NUM_JPCTRL_PHASE) {
    return JPosCtrlPhase::JPCTRL_swing;
  }
  else return next_phase;
}

template <typename T>
void JPosCtrlTest<T>::_SettingParameter(){
  ParamHandler handler(CheetahConfigPath"TEST_jpos_ctrl.yaml");
  T tmp;
  std::vector<T> tmp_vec;
  // Stance Time
  handler.getValue<T>("ini_jpos_move_time", tmp);
  ((JPosCtrl<T>*)ini_jpos_ctrl_)->setMovingTime(tmp);
  handler.getValue<T>("swing_time", tmp);
  ((JPosCtrl<T>*)jpos_swing_)->setMovingTime(tmp);

  handler.getVector<T>("target_jpos", tmp_vec);
  ((JPosCtrl<T>*)ini_jpos_ctrl_)->setTargetPos(tmp_vec);
  ((JPosCtrl<T>*)jpos_swing_)->setTargetPos(tmp_vec);
}

template class JPosCtrlTest<double>;
template class JPosCtrlTest<float>;
