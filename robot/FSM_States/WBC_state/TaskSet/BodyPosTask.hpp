#ifndef KINEMATICS_BODY_POS_TASK
#define KINEMATICS_BODY_POS_TASK

// (X, Y, Z)
#include <Task.hpp>

template <typename T> class FloatingBaseModel;

template <typename T>
class BodyPosTask: public Task<T>{
public:
  BodyPosTask(const FloatingBaseModel<T>*);
  virtual ~BodyPosTask();

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

  const FloatingBaseModel<T>* robot_sys_;
};

#endif
