#ifndef BODY_CONTROL_TEST_Cheetah
#define BODY_CONTROL_TEST_Cheetah

#include <WBC_state/Test.hpp>
#include <Dynamics/FloatingBaseModel.h>


enum BodyCtrlPhase{
  BDCTRL_body_up_ctrl = 0,
  BDCTRL_body_ctrl = 1,
  NUM_BDCTRL_PHASE
};

template <typename T>
class BodyCtrlTest: public Test<T>{
public:
  BodyCtrlTest(const FloatingBaseModel<T>* );
  virtual ~BodyCtrlTest();
  virtual void TestInitialization();

protected:
  virtual int _NextPhase(const int & phase);
  void _SettingParameter();
  
  Controller<T>* body_up_ctrl_;
  Controller<T>* body_ctrl_;

};

#endif
