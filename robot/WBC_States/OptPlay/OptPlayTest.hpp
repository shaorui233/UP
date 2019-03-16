#ifndef OPT_TEST_Cheetah
#define OPT_TEST_Cheetah

#include <WBC_States/Test.hpp>
#include <Dynamics/Quadruped.h>

template <typename T> class StateProvider;

namespace OptPlayPhase{
  constexpr int stance_wait = 0;
  constexpr int full_contact_1 = 1;
  constexpr int frhl_swing_start_trans = 2;
  constexpr int frhl_swing = 3;
  constexpr int frhl_swing_end_trans = 4;
  constexpr int full_contact_2 = 5;
  constexpr int flhr_swing_start_trans = 6;
  constexpr int flhr_swing = 7;
  constexpr int flhr_swing_end_trans = 8;
  constexpr int NUM_OPT_PHASE = 9;
};


template <typename T>
class OptPlayTest: public Test<T>{
public:
  OptPlayTest(FloatingBaseModel<T>* robot, const RobotType & robot_type);
  virtual ~OptPlayTest();

protected:
  virtual void _TestInitialization();
  virtual void _UpdateExtraData(Cheetah_Extra_Data<T> * ext_data);
  virtual int _NextPhase(const int & phase);

  void _SettingParameter();

  Controller<T>* body_ctrl_stay_;
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

  int _max_num_step;

};

#endif
