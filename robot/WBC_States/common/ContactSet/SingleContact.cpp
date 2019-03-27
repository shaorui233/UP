#include "SingleContact.hpp"
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <Utilities/Utilities_print.h>

// [ Fx, Fy, Fz ]
template <typename T>
SingleContact<T>::SingleContact(const FloatingBaseModel<T>* robot, int pt):
    ContactSpec<T>(3),
    max_Fz_(700.),
    contact_pt_(pt),
    dim_U_(6)
{
    Contact::idx_Fz_ = 2;
    robot_sys_ = robot;
    Contact::Jc_ = DMat<T>(Contact::dim_contact_, cheetah::dim_config);
    Contact::Uf_ = DMat<T>::Zero(dim_U_, Contact::dim_contact_);

    T mu (0.4);


    Contact::Uf_(0, 2) = 1.;

    Contact::Uf_(1, 0) = 1.; Contact::Uf_(1, 2) = mu;
    Contact::Uf_(2, 0) = -1.; Contact::Uf_(2, 2) = mu;

    Contact::Uf_(3, 1) = 1.; Contact::Uf_(3, 2) = mu;
    Contact::Uf_(4, 1) = -1.; Contact::Uf_(4, 2) = mu;

    // Upper bound of normal force
    Contact::Uf_(5,2) = -1.;

}

template <typename T>
SingleContact<T>::~SingleContact(){  }

template <typename T>
bool SingleContact<T>::_UpdateJc(){
    Contact::Jc_ = robot_sys_->_Jc[contact_pt_];
    Contact::Jc_.block(0, 3, 3, 3).setIdentity();
    //pretty_print(Contact::Jc_, std::cout, "Jc");
    return true;
}

template <typename T>
bool SingleContact<T>::_UpdateJcDotQdot(){
    Contact::JcDotQdot_ = DVec<T>::Zero(Contact::dim_contact_);
    return true;
}

template <typename T>
bool SingleContact<T>::_UpdateUf(){
    return true;
}

template <typename T>
bool SingleContact<T>::_UpdateInequalityVector(){
    Contact::ieq_vec_ = DVec<T>::Zero(dim_U_);
    Contact::ieq_vec_[5] = -max_Fz_;
    return true;
}

template class SingleContact<double>;
template class SingleContact<float>;
