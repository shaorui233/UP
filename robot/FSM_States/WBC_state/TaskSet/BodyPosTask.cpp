#include "BodyPosTask.hpp"
// (X, Y, Z)
#include <Configuration.h>
#include <WBC_state/Cheetah_DynaCtrl_Definition.h>
#include <Utilities/Utilities_print.h>
#include <Dynamics/FloatingBaseModel.h>

template <typename T>
BodyPosTask<T>::BodyPosTask(const FloatingBaseModel<T>* robot):
    Task<T>(3),
    robot_sys_(robot)
{
    TK::Jt_ = DMat<T>::Zero(TK::dim_task_, cheetah::dim_config);
    // TEST
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
    Vec3<T> link_pos = robot_sys_->_state.bodyPosition;
    
    // X, Y, Z
    for(int i(0); i<3; ++i){
        TK::pos_err_[i] = (*pos_cmd)[i] - link_pos[i];
        TK::vel_des_[i] = vel_des[i];
        TK::acc_des_[i] = acc_des[i];
    }
    //printf("[Body Pos Task]\n");
    //dynacore::pretty_print(acc_des, std::cout, "acc_des");
    //dynacore::pretty_print(pos_err_, std::cout, "pos_err_");
    //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
    //dynacore::pretty_print(Jt_, std::cout, "Jt");

    return true;
}

template <typename T>
bool BodyPosTask<T>::_UpdateTaskJacobian(){
    //TK::Jt_ = robot_sys_->_J[0].block(3,0, 3, cheetah::dim_config);
    return true;
}

template <typename T>
bool BodyPosTask<T>::_UpdateTaskJDotQdot(){
    //TK::JtDotQdot_ = robot_sys_->_Jdqd[0].tail(3);
    return true;
}

template class BodyPosTask<double>;
template class BodyPosTask<float>;
