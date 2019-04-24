#include "FootLocalPosTask.hpp"
// (X, Y, Z)
#include <Configuration.h>
#include <WBC_States/Cheetah_DynaCtrl_Definition.h>
#include <Utilities/Utilities_print.h>
#include <Dynamics/FloatingBaseModel.h>


template <typename T>
FootLocalPosTask<T>::FootLocalPosTask(
        const FloatingBaseModel<T>* robot, int link_idx, int local_frame_idx):
    Task<T>(3),
    _robot_sys(robot), _link_idx(link_idx), _local_frame_idx(local_frame_idx)
{
    TK::Jt_ = DMat<T>::Zero(TK::dim_task_, cheetah::dim_config);
    TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);
    
    _Kp_kin = DVec<T>::Constant(TK::dim_task_, 1.);
    _Kp = DVec<T>::Constant(TK::dim_task_, 50.);
    _Kd = DVec<T>::Constant(TK::dim_task_, 3.);
}

template <typename T>
FootLocalPosTask<T>::~FootLocalPosTask(){}

template <typename T>
bool FootLocalPosTask<T>::_UpdateCommand(void* pos_des,
        const DVec<T> & vel_des,
        const DVec<T> & acc_des){
    Vec3<T>* pos_cmd = (Vec3<T>*)pos_des;
    Vec3<T> link_pos, local_pos, local_vel;

    link_pos = _robot_sys->_pGC[_link_idx];
    Vec3<T> offset; offset.setZero();
    _robot_sys->getPositionVelocity(_local_frame_idx, offset, local_pos, local_vel);

    // X, Y
    for(size_t i(0); i<TK::dim_task_; ++i){
        TK::pos_err_[i] = (*pos_cmd)[i] - (link_pos[i] - local_pos[i]);
        TK::vel_des_[i] = vel_des[i];
        TK::acc_des_[i] = acc_des[i];
    }
    
    // Op acceleration command
    for(size_t i(0); i<TK::dim_task_; ++i){
        TK::op_cmd_[i] = _Kp[i] * TK::pos_err_[i]
            + _Kd[i] * (TK::vel_des_[i] - (_robot_sys->_vGC[_link_idx][i] - local_vel[i]))
            + TK::acc_des_[i];
    }

    //printf("[Link Pos Task]\n");
    //pretty_print(acc_des, std::cout, "acc_des");
    //pretty_print(TK::pos_err_, std::cout, "pos_err_");
    //pretty_print(*pos_cmd, std::cout, "pos cmd");
    //pretty_print(TK::op_cmd_, std::cout, "op cmd");
    // TEST
    //TK::op_cmd_.setZero();
    //pretty_print(TK::Jt_, std::cout, "Jt");

    return true;
}

template <typename T>
bool FootLocalPosTask<T>::_UpdateTaskJacobian(){
    TK::Jt_ = _robot_sys->_Jc[_link_idx];
    TK::Jt_.block(0,0, 3, 6) = DMat<T>::Zero(3,6);
    return true;
}

template <typename T>
bool FootLocalPosTask<T>::_UpdateTaskJDotQdot(){
    //TK::JtDotQdot_ = _robot_sys->_Jcdqd[_link_idx];
    return true;
}

template class FootLocalPosTask<double>;
template class FootLocalPosTask<float>;
