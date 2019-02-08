#include "LinkPosTask.hpp"
// (X, Y, Z)
#include <Configuration.h>
#include <WBC_state/Cheetah_DynaCtrl_Definition.h>
#include <Utilities/Utilities_print.h>
#include <Dynamics/FloatingBaseModel.h>


template <typename T>
LinkPosTask<T>::LinkPosTask(const FloatingBaseModel<T>* robot, int link_idx, bool virtual_depend):
    Task<T>(3),
    robot_sys_(robot), link_idx_(link_idx), virtual_depend_(virtual_depend)
{
    TK::Jt_ = DMat<T>::Zero(TK::dim_task_, cheetah::dim_config);
    TK::JtDotQdot_ = DVec<T>::Zero(TK::dim_task_);
}

template <typename T>
LinkPosTask<T>::~LinkPosTask(){}

template <typename T>
bool LinkPosTask<T>::_UpdateCommand(void* pos_des,
        const DVec<T> & vel_des,
        const DVec<T> & acc_des){
    Vec3<T>* pos_cmd = (Vec3<T>*)pos_des;
    Vec3<T> link_pos;

    link_pos = robot_sys_->_pGC[link_idx_];
    
    // X, Y, Z
    for(int i(0); i<3; ++i){
        TK::pos_err_[i] = (*pos_cmd)[i] - link_pos[i];
        TK::vel_des_[i] = vel_des[i];
        TK::acc_des_[i] = acc_des[i];
    }
    //printf("[Link Pos Task]\n");
    //dynacore::pretty_print(acc_des, std::cout, "acc_des");
    //dynacore::pretty_print(pos_err_, std::cout, "pos_err_");
    //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
    //dynacore::pretty_print(Jt_, std::cout, "Jt");

    return true;
}

template <typename T>
bool LinkPosTask<T>::_UpdateTaskJacobian(){
    TK::Jt_ = robot_sys_->_Jc[link_idx_].block(3,0, 3, cheetah::dim_config);
    if(!virtual_depend_){
        TK::Jt_.block(0,0, 3, 6) = DMat<T>::Zero(3,6);
    }
    return true;
}

template <typename T>
bool LinkPosTask<T>::_UpdateTaskJDotQdot(){
    TK::JtDotQdot_ = robot_sys_->_Jcdqd[link_idx_].tail(3);
    return true;
}

template class LinkPosTask<double>;
template class LinkPosTask<float>;
