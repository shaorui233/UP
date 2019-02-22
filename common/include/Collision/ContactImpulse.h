#ifndef CONTACT_IMPULSE_H
#define CONTACT_IMPULSE_H

#include "ContactConstraint.h"
#include "Dynamics/FloatingBaseModel.h"

template <typename T>
class ContactImpulse: public ContactConstraint<T>{
    public:
        ContactImpulse(FloatingBaseModel<T>* model):ContactConstraint<T>(model){
            _nDof = CC::_model->_nDof;
        }
        virtual ~ContactImpulse(){}

        virtual void UpdateExternalForces(T K, T D, T dt){
            (void)K; (void)D;
            // Set penetration recovery ration here
            _penetration_recover_ratio = 0.007 /dt;
            //_penetration_recover_ratio = 0.0 /dt;
            _dt = dt;
        }
        virtual void UpdateQdot(FBModelState<T> & state);

    protected:
        T _penetration_recover_ratio = 0.;
        size_t _nDof;
        void _UpdateVelocity(DVec<T> & qdot);
        void _UpdateQdotOneDirection(
                size_t idx, const vectorAligned<D3Mat<T> > & Jc_list, 
                const T * lambda_list, 
                const vectorAligned<DVec<T> > AinvB_list, 
                const T * des_vel_list, 
                const T * min_list, 
                const T * max_list, 
                DVec<T> & qdot);
    private:
        T _dt;
};

#endif
