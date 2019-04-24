#ifndef BODY_XY_POS_TASK
#define BODY_XY_POS_TASK

// (X, Y, Z)
#include <WBC/Task.hpp>

template <typename T> class FloatingBaseModel;

template <typename T>
class BodyXYTask: public Task<T>{
public:
  BodyXYTask(const FloatingBaseModel<T>*);
  virtual ~BodyXYTask();

  DVec<T> _Kp_kin;
  DVec<T> _Kp, _Kd;
protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(void* pos_des,
                              const DVec<T> & vel_des,
                              const DVec<T> & acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate(){ return true; }

  const FloatingBaseModel<T>* _robot_sys;
};

#endif
