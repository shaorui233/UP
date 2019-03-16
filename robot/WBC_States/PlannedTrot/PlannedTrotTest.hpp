#ifndef PLANNED_TROT_TEST_Cheetah
#define PLANNED_TROT_TEST_Cheetah

#include <WBC_States/Test.hpp>
#include <Dynamics/Quadruped.h>

template <typename T> class StateProvider;

namespace PlannedTrotPhase{
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
class PlannedTrotTest: public Test<T>{
public:
  PlannedTrotTest(FloatingBaseModel<T>* , const RobotType& );
  virtual ~PlannedTrotTest();

  DVec<T> _jpos_des_pre;
protected:
  virtual void _TestInitialization();
  virtual int _NextPhase(const int & phase);
  virtual void _UpdateExtraData(Cheetah_Extra_Data<T> * ext_data);

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

  StateProvider<T>* _sp;
};

#endif
