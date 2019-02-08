#ifndef BODY_ORIENTATION_TASK
#define BODY_ORIENTATION_TASK
// (Rx, Ry, Rz)
#include <Task.hpp>

template <typename T> class FloatingBaseModel;

template<typename T>
class BodyOriTask: public Task<T>{
public:
  BodyOriTask(const FloatingBaseModel<T>*);
  virtual ~BodyOriTask();

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

  int link_idx_;
  bool virtual_depend_; 
  const FloatingBaseModel<T>* robot_sys_;
};

#endif
