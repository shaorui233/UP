#ifndef FRONTJUMP_CTRL
#define FRONTJUMP_CTRL

#include "DataReader.hpp"
#include "DataReadCtrl.hpp"
#include <Dynamics/FloatingBaseModel.h>
#include <Controllers/LegController.h>

template <typename T>
class FrontJumpCtrl : public DataReadCtrl<T> {
 public:
  FrontJumpCtrl(DataReader*, float _dt);
  virtual ~FrontJumpCtrl();

  virtual void OneStep(float _curr_time, bool b_preparation, LegControllerCommand<T>* command);

 protected:
  void _update_joint_command();
  std::vector<T> _Kp_joint_front, _Kd_joint_front;
};

#endif
