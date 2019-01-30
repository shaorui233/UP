#ifndef CONTACT_SPRING_DAMPER_H
#define CONTACT_SPRING_DAMPER_H


#include "ContactConstraint.h"
#include "FloatingBaseModel.h"

template<typename T>
class ContactSpringDamper: public ContactConstraint<T>{
    public:
        ContactSpringDamper(FloatingBaseModel<T> * model):
            ContactConstraint<T>(model){
                _nGC = CC::_model->_nGroundContact;
                for(size_t i(0); i< _nGC; ++i){
                    _tangentialDeflections.push_back(Vec2<T>::Zero());
                }
                deflectionRate.resize(_nGC);
            }

        virtual ~ContactSpringDamper(){}

        virtual void UpdateExternalForces(T K, T D, T dt);
        virtual void UpdateQdot(FBModelState<T> & state){ (void)state; /* Do nothing */ }

    protected:
        size_t _nGC;
        void _groundContactWithOffset(T K, T D);

        vectorAligned<Vec2<T> > deflectionRate;
        vectorAligned<Vec2<T>> _tangentialDeflections;

};


#endif
