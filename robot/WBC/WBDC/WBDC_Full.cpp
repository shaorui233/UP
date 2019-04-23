#include "WBDC_Full.hpp"
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include <Utilities/Timer.h>

template<typename T> 
WBDC_Full<T>::WBDC_Full(size_t num_qdot, 
        const std::vector<ContactSpec<T> * > & contact_list, 
        const std::vector<Task<T> * > & task_list): 
    WBC<T>(num_qdot), _b_first_visit(true){

    _Sf = DMat<T>::Zero(6, WB::num_qdot_);
    _Sf.block(0,0, 6, 6).setIdentity();

    // Dimension 
    _dim_rf = 0;
    _dim_Uf = 0;
    for(size_t i(0); i<(contact_list).size(); ++i){
        _contact_list.push_back(contact_list[i]);
        _dim_rf += (_contact_list)[i]->getDim();
        _dim_Uf += (_contact_list)[i]->getDimRFConstraint();
    }

    _dim_first_task = task_list[0]->getDim();
    for(size_t i(0);i<(task_list).size(); ++i){
        _task_list.push_back(task_list[i]);
    }
    // xddot_contact, first_task, reaction force
    _dim_opt = _dim_rf + _dim_first_task + _dim_rf; 
    _dim_eq_cstr = 6;

    // Matrix Setting
    G.resize(_dim_opt, _dim_opt); 
    g0.resize(_dim_opt);
    CE.resize(_dim_opt, _dim_eq_cstr);
    ce0.resize(_dim_eq_cstr);
    CI.resize(_dim_opt, _dim_Uf);
    ci0.resize(_dim_Uf);

    // Eigen Matrix Setting
    _dyn_CE = DMat<T>(_dim_eq_cstr, _dim_opt);
    _dyn_ce0 = DVec<T>(_dim_eq_cstr);
    _dyn_CI = DMat<T>(_dim_Uf, _dim_opt); _dyn_CI.setZero();
    _dyn_ci0 = DVec<T>(_dim_Uf);

    _Jc = DMat<T>(_dim_rf, WB::num_qdot_);
    _JcDotQdot = DVec<T>(_dim_rf);
    _Uf = DMat<T>(_dim_Uf, _dim_rf); _Uf.setZero();
    _Uf_ieq_vec = DVec<T>(_dim_Uf);
 
    _Ja = DMat<T>(_dim_rf + _dim_first_task, WB::num_qdot_);
    _JaDotQdot = DVec<T>(_dim_rf + _dim_first_task);
    _xa_ddot = DVec<T>(_dim_rf + _dim_first_task); 


    for(size_t i(0); i<_dim_opt; ++i){
        for(size_t j(0); j<_dim_opt; ++j){
            G[i][j] = 0.;
        }
        g0[i] = 0.;
    }
    _eye = DMat<T>::Identity(WB::num_qdot_, WB::num_qdot_);
}

template<typename T> 
void WBDC_Full<T>::MakeTorque(
        DVec<T> & cmd,
        void* extra_input){

    if(!WB::b_updatesetting_) { printf("[Wanning] WBDC_Full setting is not done\n"); }
    if(extra_input) _data = static_cast<WBDC_ExtraData<T>* >(extra_input);
    _SetCost();

    // Contact Setting 
    _ContactBuilding();
    // Set inequality constraints
    _SetInEqualityConstraint();

    // First Task Check
    Task<T>* task = _task_list[0];
    DMat<T> Jt, JaBar;
    DVec<T> JtDotQdot, xddot;

    if(!task->IsTaskSet()){ printf("1st task is not set!\n"); exit(0); }
    task->getTaskJacobian(Jt);
    task->getTaskJacobianDotQdot(JtDotQdot);
    task->getCommand(xddot);

    _Ja.block(0,0, _dim_rf, WB::num_qdot_) = _Jc;
    _Ja.block(_dim_rf,0, _dim_first_task, WB::num_qdot_) = Jt;
    _JaDotQdot.head(_dim_rf) = _JcDotQdot;
    _JaDotQdot.tail(_dim_first_task) = JtDotQdot;

    WB::_WeightedInverse(_Ja, WB::Ainv_, JaBar);

    _xa_ddot.head(_dim_rf) = _data->_contact_pt_acc;
    _xa_ddot.tail(_dim_first_task) = xddot;

    // Optimization
    // Set equality constraints
    _dyn_CE.block(0,0, _dim_eq_cstr, _dim_first_task + _dim_rf) = _Sf * WB::A_ * JaBar;
    _dyn_CE.block(0, _dim_first_task + _dim_rf, _dim_eq_cstr, _dim_rf) = -_Sf * _Jc.transpose();
    _dyn_ce0 = _Sf * (WB::A_ * JaBar * (_xa_ddot - _JaDotQdot) + WB::cori_ + WB::grav_);
    
    for(size_t i(0); i< _dim_eq_cstr; ++i){
        for(size_t j(0); j<_dim_opt; ++j){
            CE[j][i] = _dyn_CE(i,j);
        }
        ce0[i] = _dyn_ce0[i];
    }
    // Optimization
    T f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);

    (void)f;
    DVec<T> delta(_dim_rf + _dim_first_task);
    for(size_t i(0); i<_dim_rf + _dim_first_task; ++i) { delta[i] = z[i]; }
    DVec<T> qddot_pre = JaBar * (_xa_ddot + delta - _JaDotQdot);

    // First Qddot is found
    // Stack The last Task
    if(_task_list.size() > 1){
        DMat<T> JtBar, JtPre;
        DMat<T> Npre = _eye - JaBar * _Ja;

        for(size_t i(1); i<_task_list.size(); ++i){
            task = _task_list[i];

            if(!task->IsTaskSet()){ printf("%lu th task is not set!\n", i); exit(0); }
            task->getTaskJacobian(Jt);
            task->getTaskJacobianDotQdot(JtDotQdot);
            task->getCommand(xddot);

            JtPre = Jt * Npre;
            WB::_WeightedInverse(JtPre, WB::Ainv_, JtBar);

            qddot_pre = qddot_pre + JtBar * (xddot - JtDotQdot - Jt * qddot_pre);
            Npre = Npre * (_eye - JtBar * JtPre);
        }
    }
    _GetSolution(qddot_pre, cmd);

    _data->_opt_result = DVec<T>(_dim_opt);
    for(size_t i(0); i<_dim_opt; ++i){
        _data->_opt_result[i] = z[i];
    }

    if(_b_first_visit){ _b_first_visit = false; }
    //std::cout << "f: " << f << std::endl;
    //std::cout << "x: " << z << std::endl;
    //pretty_print(_xa_ddot, std::cout, "xa ddot");
    //std::cout << "cmd: "<<cmd<<std::endl;
    //pretty_print(_Sf, std::cout, "Sf");
    //pretty_print(qddot_pre, std::cout, "qddot_pre");
    //pretty_print(JcN, std::cout, "JcN");
    //pretty_print(Nci_, std::cout, "Nci");
    //DVec<T> eq_check = dyn_CE * data_->opt_result_;
    //pretty_print(dyn_ce0, std::cout, "dyn ce0");
    //pretty_print(eq_check, std::cout, "eq_check");

    //pretty_print(Jt, std::cout, "Jt");
    //pretty_print(JtDotQdot, std::cout, "Jtdotqdot");
    //pretty_print(xddot, std::cout, "xddot");


    //printf("G:\n");
    //std::cout<<G<<std::endl;
    //printf("g0:\n");
    //std::cout<<g0<<std::endl;

    //printf("CE:\n");
    //std::cout<<CE<<std::endl;
    //printf("ce0:\n");
    //std::cout<<ce0<<std::endl;

    //printf("CI:\n");
    //std::cout<<CI<<std::endl;
    //printf("ci0:\n");
    //std::cout<<ci0<<std::endl;
}

template<typename T> 
void WBDC_Full<T>::_SetInEqualityConstraint(){
    _dyn_CI.block(0, _dim_first_task + _dim_rf, _dim_Uf, _dim_rf) = _Uf;
    _dyn_ci0 = _Uf_ieq_vec;

    for(size_t i(0); i< _dim_Uf; ++i){
        for(size_t j(0); j<_dim_opt; ++j){
            CI[j][i] = _dyn_CI(i,j);
        }
        ci0[i] = -_dyn_ci0[i];
    }
    // pretty_print(dyn_CI, std::cout, "WBDC_Full: CI");
    // pretty_print(dyn_ci0, std::cout, "WBDC_Full: ci0");
}

template<typename T> 
void WBDC_Full<T>::_ContactBuilding(){
    DMat<T> Uf;
    DVec<T> Uf_ieq_vec;
    // Initial
    DMat<T> Jc;
    DVec<T> JcDotQdot;
    size_t dim_accumul_rf, dim_accumul_uf;
    _contact_list[0]->getContactJacobian(Jc);
    _contact_list[0]->getJcDotQdot(JcDotQdot);
    _contact_list[0]->getRFConstraintMtx(Uf);
    _contact_list[0]->getRFConstraintVec(Uf_ieq_vec);
    
    dim_accumul_rf = _contact_list[0]->getDim();
    dim_accumul_uf = _contact_list[0]->getDimRFConstraint();

    _Jc.block(0, 0, dim_accumul_rf, WB::num_qdot_) = Jc;
    _JcDotQdot.head(dim_accumul_rf) = JcDotQdot;
    _Uf.block(0, 0, dim_accumul_uf, dim_accumul_rf) = Uf;
    _Uf_ieq_vec.head(dim_accumul_uf) = Uf_ieq_vec;

    size_t dim_new_rf, dim_new_uf;

    for(size_t i(1); i < _contact_list.size(); ++i){
        _contact_list[i]->getContactJacobian(Jc);
        _contact_list[i]->getJcDotQdot(JcDotQdot);

        dim_new_rf = _contact_list[i]->getDim();
        dim_new_uf = _contact_list[i]->getDimRFConstraint();

        // Jc append
        _Jc.block(dim_accumul_rf, 0, dim_new_rf, WB::num_qdot_) = Jc;

        // JcDotQdot append
        _JcDotQdot.segment(dim_accumul_rf, dim_new_rf) = JcDotQdot;

        // Uf
        _contact_list[i]->getRFConstraintMtx(Uf);
        _Uf.block(dim_accumul_uf, dim_accumul_rf, dim_new_uf, dim_new_rf) = Uf;

        // Uf inequality vector
        _contact_list[i]->getRFConstraintVec(Uf_ieq_vec);
        _Uf_ieq_vec.segment(dim_accumul_uf, dim_new_uf) = Uf_ieq_vec;

        dim_accumul_rf += dim_new_rf;
        dim_accumul_uf += dim_new_uf;
    }
     //pretty_print(_Jc, std::cout, "[WBDC_Full] Jc");
     //pretty_print(_JcDotQdot, std::cout, "[WBDC_Full] JcDot Qdot");
     //pretty_print(_Uf, std::cout, "[WBDC_Full] Uf");
}

template<typename T> 
void WBDC_Full<T>::_GetSolution(const DVec<T> & qddot, DVec<T> & cmd){
    _data->_Fr = DVec<T>(_dim_rf);

    // get Reaction forces
    for(size_t i(0); i<_dim_rf; ++i) _data->_Fr[i] = z[i + _dim_rf + _dim_first_task];
    DVec<T> tot_tau = WB::A_ * qddot + WB::cori_ + WB::grav_ - _Jc.transpose() * _data->_Fr;

    _data->_qddot = qddot;
    cmd = tot_tau.tail(WB::num_act_joint_);

    //pretty_print(qddot, std::cout, "qddot");
    //pretty_print(_data->_Fr, std::cout, "Fr");
    //pretty_print(tot_tau, std::cout, "tot tau result");
    //pretty_print(cmd, std::cout, "final command");
}

template<typename T> 
void WBDC_Full<T>::_SetCost(){
    // Set Cost
    size_t idx_offset(0);
    for (size_t i(0); i< _dim_rf; ++i){
        G[i+idx_offset][i+idx_offset] = _data->_W_contact[i];
    }
    idx_offset += _dim_rf;
    for (size_t i(0); i< _dim_first_task; ++i){
        G[i+idx_offset][i+idx_offset] = _data->_W_task[i];
    }
    idx_offset += _dim_first_task;
    for (size_t i(0); i< _dim_rf; ++i){
        G[i+idx_offset][i+idx_offset] = _data->_W_rf[i];
    }

    //pretty_print(_data->_W_contact, std::cout, "W contact");
    //pretty_print(_data->_W_task, std::cout, "W task");
    //pretty_print(_data->_W_rf, std::cout, "W rf");
}


template<typename T> 
void WBDC_Full<T>::UpdateSetting(const DMat<T> & A,
        const DMat<T> & Ainv,
        const DVec<T> & cori,
        const DVec<T> & grav,
        void* extra_setting){
    WB::A_ = A;
    WB::Ainv_ = Ainv;
    WB::cori_ = cori;
    WB::grav_ = grav;
    WB::b_updatesetting_ = true;

    (void)extra_setting;
}

template<typename T> 
bool WBDC_Full<T>::_CheckNullSpace(const DMat<T> & Npre){
    DMat<T> M_check = _Sf * WB::A_ * Npre;
    Eigen::JacobiSVD<DMat<T>> svd(M_check, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //pretty_print(svd.singularValues(), std::cout, "svd singular value");

    for(int i(0); i<svd.singularValues().rows(); ++i){
        if(svd.singularValues()[i] > 0.00001) { 
            printf("non singular!!\n"); 
            pretty_print(svd.singularValues(), std::cout, "svd singular value");
            return false;
        }else{
            //printf("small enough singular value: %f\n", svd.singularValues()[i]);
        }
    }
    return true;
}


template class WBDC_Full<double>;
template class WBDC_Full<float>;
