#ifndef CONTACT_SPEC
#define CONTACT_SPEC

#include <cppTypes.h>

#define Contact ContactSpec<T>

template <typename T>
class ContactSpec{
public:
  ContactSpec(size_t dim):dim_contact_(dim), b_set_contact_(false){
      idx_Fz_ = dim -1;
  }
  virtual ~ContactSpec(){}

  size_t getDim() const { return dim_contact_; }
  size_t getDimRFConstraint() const { return Uf_.rows(); }
  size_t getFzIndex() const { return idx_Fz_; }

  void getContactJacobian(DMat<T> & Jc){ Jc = Jc_; }
  void getJcDotQdot(DVec<T> & JcDotQdot) { JcDotQdot = JcDotQdot_; }
  void UnsetContact(){ b_set_contact_ = false; }

  void getRFConstraintMtx(DMat<T> & Uf){ Uf = Uf_; }
  void getRFConstraintVec(DVec<T> & ieq_vec){ ieq_vec = ieq_vec_; }

  bool UpdateContactSpec(){
      _UpdateJc();
      _UpdateJcDotQdot();
      _UpdateUf();
      _UpdateInequalityVector();
      b_set_contact_ = true;
      return true;
  }

protected:
  virtual bool _UpdateJc() = 0;
  virtual bool _UpdateJcDotQdot() = 0;
  virtual bool _UpdateUf() = 0;
  virtual bool _UpdateInequalityVector() = 0;
  
  int idx_Fz_;
  DMat<T> Uf_;
  DVec<T> ieq_vec_;

  DMat<T> Jc_;
  DVec<T> JcDotQdot_;
  size_t dim_contact_;
  bool b_set_contact_;
};
#endif
