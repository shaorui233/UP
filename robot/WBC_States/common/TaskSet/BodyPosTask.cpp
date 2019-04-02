#include "BodyPosTask.hpp"
// (X, Y, Z)
#include <Configuration.h>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <Utilities/Utilities_print.h>
#include <Dynamics/FloatingBaseModel.h>

template <typename T>
BodyPosTask<T>::BodyPosTask(const FloatingBaseModel<T>* robot):
    Task<T>(3),
    _robot_sys(robot)
{
    TK::Jt_ = DMat<T>::Zero(TK::dim_task_, cheetah::dim_config);
    TK::Jt_.block(0, 3, 3, 3).setIdentity();
    TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);
}

template <typename T>
BodyPosTask<T>::~BodyPosTask(){}

template <typename T>
bool BodyPosTask<T>::_UpdateCommand(void* pos_des,
        const DVec<T> & vel_des,
        const DVec<T> & acc_des){
    Vec3<T>* pos_cmd = (Vec3<T>*)pos_des;
    Vec3<T> link_pos = _robot_sys->_state.bodyPosition;
    
    // X, Y, Z
    for(int i(0); i<3; ++i){
        TK::pos_err_[i] = (*pos_cmd)[i] - link_pos[i];
        TK::vel_des_[i] = vel_des[i];
        TK::acc_des_[i] = acc_des[i];
    }
    //Quat<T> quat = _robot_sys->_state.bodyOrientation;
    //Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
    //TK::pos_err_ = Rot * TK::pos_err_;
    //TK::vel_des_ = Rot * TK::vel_des_;
    //TK::acc_des_ = Rot * TK::acc_des_;

     //printf("[Body Pos Task]\n");
    //pretty_print(acc_des, std::cout, "acc_des");
    //pretty_print(TK::pos_err_, std::cout, "pos_err_");
    //pretty_print(*pos_cmd, std::cout, "pos cmd");
    //pretty_print(TK::Jt_, std::cout, "Jt");

    return true;
}

template <typename T>
bool BodyPosTask<T>::_UpdateTaskJacobian(){
    Quat<T> quat = _robot_sys->_state.bodyOrientation;
    Mat3<T> Rot = ori::quaternionToRotationMatrix(quat);
    TK::Jt_.block(0,3, 3,3) = Rot.transpose();
    //TK::Jt_.block(0,3, 3,3) = Rot;
    //pretty_print(TK::Jt_, std::cout, "Jt");
    //TK::Jt_.block(0,3, 3,3) = Rot*TK::Jt_.block(0,3,3,3);
     return true;
}

template <typename T>
bool BodyPosTask<T>::_UpdateTaskJDotQdot(){
    return true;
}

template class BodyPosTask<double>;
template class BodyPosTask<float>;
