#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cppTypes.h>
#include <Dynamics/FloatingBaseModel.h>
#include <Task.hpp>
#include <ContactSpec.hpp>

#define Ctrl Controller<T>

namespace Weight{
    constexpr float tan_big = 5.;
    constexpr float tan_small = 1.;
    constexpr float nor_big = 0.5;
    constexpr float nor_small = 0.01;
    constexpr float foot_big = 1000.;
    constexpr float foot_small = 0.001;
    constexpr float qddot_relax = 100.;
}


template <typename T>
class Controller{
public:
  Controller(const FloatingBaseModel<T>* robot):robot_sys_(robot),state_machine_time_(0.){}
  virtual ~Controller(){}

  virtual void OneStep(void* command) = 0;
  virtual void FirstVisit() = 0;
  virtual void LastVisit() = 0;
  virtual bool EndOfPhase() = 0;
  virtual void CtrlInitialization(const std::string & setting_file_name) = 0;
  virtual void SetTestParameter(const std::string & test_file) = 0;

protected:
  void _DynConsistent_Inverse(const DMat<T> & J, DMat<T> & Jinv){
      DMat<T> Jtmp(J * Ainv_ * J.transpose());
      Jinv = Ainv_ * J.transpose() * Jtmp.inverse();
  }

  void _PreProcessing_Command(){
      A_ = robot_sys_->getMassMatrix();
      grav_ = robot_sys_->getGravityForce();
      coriolis_ = robot_sys_->getCoriolisForce();
      Ainv_ = A_.inverse();
  }

  void _PostProcessing_Command(){
      for(size_t i(0); i<task_list_.size(); ++i){ task_list_[i]->UnsetTask(); }
      for(size_t i(0); i<contact_list_.size(); ++i){ contact_list_[i]->UnsetContact(); }
  }

  const FloatingBaseModel<T>* robot_sys_;

  DMat<T> A_;
  DMat<T> Ainv_;
  DVec<T> grav_;
  DVec<T> coriolis_;

  std::vector<Task<T>*> task_list_;
  std::vector<ContactSpec<T>*> contact_list_;

  T state_machine_time_;
};

#endif
