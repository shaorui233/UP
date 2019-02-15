#include "Collision/ContactImpulse.h"
#include "Utilities/Utilities_print.h"

template <typename T>
void ContactImpulse<T>::UpdateQdot(FBModelState<T> & state){
    CC::_nContact = CC::_CheckContact();
    if(CC::_nContact > 0){
        DVec<T> qdot(_nDof);

        for(size_t i(0); i<6; ++i){ qdot[i] = state.bodyVelocity[i]; }
        for(size_t i(0); i<_nDof - 6; ++i){ qdot[i + 6] = state.qd[i]; }

        _UpdateVelocity(qdot);

        for(size_t i(0); i<6; ++i){ state.bodyVelocity[i] = qdot[i]; }
        for(size_t i(0); i<_nDof - 6; ++i){ state.qd[i] = qdot[i+6]; }

        // Update contact force w.r.t. global frame
        // This is not neccessary for dynamics, but 
        // the global contact forces are used for 
        // graphics
        for(size_t i(0); i<CC::_nContact; ++i){
            // Divide by dt to convert impulses to forces
            CC::_cp_force_list[CC::_idx_list[i]] = 
                CC::_cp_frame_list[i] * CC::_cp_local_force_list[i] / _dt;

            // Save the current local force for the next computation
            //CC::_cp_local_force_list_pre[CC::_idx_list[i]] = 
                //CC::_cp_local_force_list[i];
        }
    }
}

template<typename T>
void ContactImpulse<T>::_UpdateVelocity(DVec<T> & qdot){

    T* lambda_list = new T[CC::_nContact];
    T* des_vel_list = new T[CC::_nContact];
    T* min_list = new T[CC::_nContact];
    T* max_list = new T[CC::_nContact];
    vectorAligned< DVec<T> > AinvB_list(CC::_nContact);
    vectorAligned< D3Mat<T> > Jc_list(CC::_nContact);

    DVec<T> AinvB;
    Vec3<T> cp_local_vel;
    Vec3<T> direction;
    CC::_model->contactJacobians();


    // Prepare Matrix and Vector
    for(size_t i(0); i< CC::_nContact; ++i){
        direction = CC::_cp_frame_list[i].template block<3,1> (0, 2);

        lambda_list[i] = CC::_model->applyTestForce(CC::_idx_list[i], direction, AinvB_list[i]);
        lambda_list[i] = 1./lambda_list[i];
        cp_local_vel = CC::_cp_frame_list[i].transpose() * CC::_model->_vGC[CC::_idx_list[i]];

        if(cp_local_vel[2] < 0.){
            des_vel_list[i] = -CC::_cp_resti_list[i] * cp_local_vel[2] 
                -_penetration_recover_ratio * CC::_cp_penetration_list[i];
        }else {
            des_vel_list[i] = std::max(cp_local_vel[2], 
                    -_penetration_recover_ratio * CC::_cp_penetration_list[i]);
        }
        Jc_list[i] = CC::_cp_frame_list[i].transpose() * CC::_model->_Jc[CC::_idx_list[i]];

        min_list[i] = 0.;
        max_list[i] = 10000000.;
    }

    // Normal
    int idx(2);
    _UpdateQdotOneDirection(idx, Jc_list, lambda_list, AinvB_list, 
            des_vel_list, min_list, max_list, qdot);

    // X
    idx = 0;
    for(size_t i(0); i<CC::_nContact; ++i){
        max_list[i] = CC::_cp_mu_list[i] * CC::_cp_local_force_list[i][2];
        min_list[i] = -max_list[i];
        des_vel_list[i] = 0.;
        direction = CC::_cp_frame_list[i].template block<3,1> (0, idx);
        lambda_list[i] = CC::_model->applyTestForce(CC::_idx_list[i], direction, AinvB_list[i]);
        lambda_list[i] = 1./lambda_list[i];
    }
    _UpdateQdotOneDirection(idx, Jc_list, lambda_list, AinvB_list, 
            des_vel_list, min_list, max_list, qdot);

    // Y
    idx = 1;
    for(size_t i(0); i< CC::_nContact; ++i){
        direction = CC::_cp_frame_list[i].template block<3,1> (0, idx);
        lambda_list[i] = CC::_model->applyTestForce(CC::_idx_list[i], direction, AinvB_list[i]);
        lambda_list[i] = 1./lambda_list[i];
    }
    _UpdateQdotOneDirection(idx, Jc_list, lambda_list, AinvB_list, 
            des_vel_list, min_list, max_list, qdot);


    delete [] lambda_list;
    delete [] des_vel_list;

    delete [] min_list;
    delete [] max_list;
}

template <typename T>
void ContactImpulse<T>::_UpdateQdotOneDirection(
        size_t idx, const vectorAligned<D3Mat<T> > & Jc_list, 
        const T * lambda_list, 
        const vectorAligned<DVec<T> > AinvB_list, 
        T * des_vel_list,
        T * min_list, 
        T * max_list, 
        DVec<T> & qdot){

    T dforce, pre_force;
    for(size_t iter(0); iter< 5; ++iter){
        for(size_t i(0); i<CC::_nContact; ++i){
            dforce = (des_vel_list[i] 
                    - (Jc_list[i].block(idx,0, 1, _nDof) * qdot)(0,0))*lambda_list[i];
            pre_force = CC::_cp_local_force_list[i][idx];
            CC::_cp_local_force_list[i][idx] = 
                std::max(min_list[i], std::min(pre_force + dforce, max_list[i]));

            dforce = CC::_cp_local_force_list[i][idx] - pre_force;
            qdot += (AinvB_list[i] * dforce );
        }
    }
}

template class ContactImpulse<double>;
template class ContactImpulse<float>;


