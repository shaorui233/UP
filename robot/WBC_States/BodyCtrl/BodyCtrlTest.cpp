#include "BodyCtrlTest.hpp"

#include <WBC_States/BodyCtrl/BodyPostureCtrl.hpp>
#include <WBC_States/common/CtrlSet/FullContactTransCtrl.hpp>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

template <typename T>
BodyCtrlTest<T>::BodyCtrlTest(FloatingBaseModel<T>* robot, const RobotType & type):
    Test<T>(robot, type){

    //_phase = BodyCtrlPhase::BDCTRL_body_ctrl;
    TEST::_phase = BodyCtrlPhase::BDCTRL_body_up_ctrl;
    TEST::_state_list.clear();

    body_up_ctrl_ = new FullContactTransCtrl<T>(robot);
    body_ctrl_ = new BodyPostureCtrl<T>(robot);
    
    TEST::_state_list.push_back(body_up_ctrl_);
    TEST::_state_list.push_back(body_ctrl_);

    _SettingParameter();
    printf("[Body Position Control Test] Constructed\n");
}

template <typename T>
BodyCtrlTest<T>::~BodyCtrlTest(){
  for(size_t i(0); i<TEST::_state_list.size(); ++i){
    delete TEST::_state_list[i];
  }
}

template <typename T>
void BodyCtrlTest<T>::_TestInitialization(){
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
        = Test<T>::_state_list.begin();

    while(iter < Test<T>::_state_list.end()){
        if(TEST::_robot_type == RobotType::CHEETAH_3){
            (*iter)->SetTestParameter(
                    CheetahConfigPath"TEST_body_ctrl_cheetah3.yaml");
        }else if(TEST::_robot_type == RobotType::MINI_CHEETAH){
            (*iter)->SetTestParameter(
                    CheetahConfigPath"TEST_body_ctrl_mini_cheetah.yaml");
        }else{
            printf("[Body Ctrl Test] Invalid robot type\n");
        }
        ++iter;
    }
}

template<typename T>
void BodyCtrlTest<T>::_UpdateExtraData(Cheetah_Extra_Data<T> * ext_data){
    ext_data->num_step = 0;
    ext_data->num_path_pt = 0;
}


template class BodyCtrlTest<double>;
template class BodyCtrlTest<float>;
