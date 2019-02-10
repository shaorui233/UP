#ifndef TROT_TEST_Cheetah
#define TROT_TEST_Cheetah

#include <WBC_state/Test.hpp>
#include <Dynamics/FloatingBaseModel.h>

template <typename T> class Cheetah_StateProvider;

namespace TrotPhase{
  constexpr int lift_up = 0;
  constexpr int full_contact_1 = 1;
  constexpr int frhl_swing_start_trans = 2;
  constexpr int frhl_swing = 3;
  constexpr int frhl_swing_end_trans = 4;
  constexpr int full_contact_2 = 5;
  constexpr int flhr_swing_start_trans = 6;
  constexpr int flhr_swing = 7;
  constexpr int flhr_swing_end_trans = 8;
  constexpr int NUM_TROT_PHASE = 9;
};


template <typename T>
class TrotTest: public Test<T>{
public:
  TrotTest(const FloatingBaseModel<T>* );
  virtual ~TrotTest();
  virtual void TestInitialization();

protected:
  virtual int _NextPhase(const int & phase);
  void _SettingParameter();
  
  Controller<T>* body_up_ctrl_;
  Controller<T>* body_ctrl_;

  // Front Right and Hind Left leg swing
  Controller<T>* frhl_swing_start_trans_ctrl_;
  Controller<T>* frhl_swing_ctrl_;
  Controller<T>* frhl_swing_end_trans_ctrl_;

  // Front Left and Hind Right leg swing
  Controller<T>* flhr_swing_start_trans_ctrl_;
  Controller<T>* flhr_swing_ctrl_;
  Controller<T>* flhr_swing_end_trans_ctrl_;

  const FloatingBaseModel<T> * _robot_sys;
  Cheetah_StateProvider<T>* _sp;
};

#endif
