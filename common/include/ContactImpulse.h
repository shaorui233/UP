#ifndef CONTACT_IMPULSE_H
#define CONTACT_IMPULSE_H

#include "ContactConstraint.h"
#include "FloatingBaseModel.h"

template <typename T>
class ContactImpulse: public ContactConstraint<T>{
    public:
        ContactImpulse(FloatingBaseModel<T>* model):ContactConstraint<T>(model){
            _nDof = CC::_model->_nDof;
        }
        virtual ~ContactImpulse(){}

        virtual void UpdateExternalForces(T K, T D, T dt){
            // Do nothing
            (void)K; (void)D; (void)dt;
        }
        virtual void UpdateQdot(FBModelState<T> & state);

    protected:
        size_t _nDof;
        void _UpdateVelocity(DVec<T> & qdot);
        void _UpdateQdotOneDirection(
                size_t idx, const vectorAligned<D3Mat<T> > & Jc_list, 
                const T * lambda_list, 
                const vectorAligned<DVec<T> > AinvB_list, 
                T * des_vel_list, 
                T * min_list, 
                T * max_list, 
                DVec<T> & qdot);
};

#endif
