#ifndef Cheetah_SINGLE_CONTACT
#define Cheetah_SINGLE_CONTACT

#include <ContactSpec.hpp>
#include <Dynamics/FloatingBaseModel.h>

template<typename T>
class SingleContact: public ContactSpec<T>{
public:
  SingleContact(const FloatingBaseModel<T>* robot, int contact_pt);
  virtual ~SingleContact();

  void setMaxFz(T max_fz){ max_Fz_ = max_fz; }
protected:
  T max_Fz_;
  int contact_pt_;
  int dim_U_;

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  const FloatingBaseModel<T>* robot_sys_;
};

#endif
