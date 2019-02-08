#ifndef JPOS_CTR_TEST_H
#define JPOS_CTR_TEST_H

#include <WBC_state/Test.hpp>
#include <Dynamics/FloatingBaseModel.h>

enum JPosCtrlPhase{
  JPCTRL_move_to_target = 0,
  JPCTRL_swing = 1,
  NUM_JPCTRL_PHASE
};

template <typename T>
class JPosCtrlTest: public Test<T>{
    public:
        JPosCtrlTest(const FloatingBaseModel<T>* );
        virtual ~JPosCtrlTest();
        virtual void TestInitialization();

    protected:
        virtual int _NextPhase(const int & phase);
        void _SettingParameter();

        Controller<T>* ini_jpos_ctrl_;
        Controller<T>* jpos_swing_;

};


#endif
