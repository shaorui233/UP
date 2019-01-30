#ifndef CONTACT_CONSTRAINT_H
#define CONTACT_CONSTRAINT_H

#include "cppTypes.h"
#include "Collision.h"
#include "Utilities_print.h"
#include <iostream>

template <typename T> class FloatingBaseModel;
template <typename T> class FBModelState;

template<typename T>
class ContactConstraint{
    public:
        ContactConstraint(FloatingBaseModel<T>* model):
            _nContact(0),
            _nCollision(0) {
                _model = model;

                for(size_t i(0); i<_model->_nGroundContact; ++i){
                    _tangentialDeflections.push_back(Vec2<T>::Zero());
                    _cp_force_list.push_back(Vec3<T>::Zero());
                }
                deflectionRate.resize(_model->_nGroundContact);
            }
        ~ContactConstraint(){}

        
        void UpdateExternalForces(T K, T D, T dt);

        void AddCollision(Collision<T>* collision){ 
            _collision_list.push_back(collision); 
            ++_nCollision;
        }

        const vectorAligned<Vec3<T> > & getContactPosList() { 
            return _cp_pos_list; 
        }

        const Vec3<T> & getGCForce(size_t idx){

            //for(size_t i(0); i<_cp_force_list.size(); ++i){
                //pretty_print(_cp_force_list[i], std::cout, "cp force");
            //}
            //printf("\n");
            return _cp_force_list[idx];
        }
    protected:
    vectorAligned<Vec2<T> > deflectionRate;
        void _groundContactWithOffset(T K, T D);

        size_t _CheckContact();

        vectorAligned<Vec2<T>> _tangentialDeflections;

        size_t _nContact;
        size_t _nCollision;
        FloatingBaseModel<T> * _model;
        std::vector<Collision<T>* >  _collision_list;
        std::vector<size_t> _idx_list;
        std::vector<T> _cp_resti_list;
        std::vector<T> _cp_mu_list;

        std::vector<T> _cp_penetration_list;
        vectorAligned<Vec3<T>> _cp_force_list; // For All contact point w.r.t Global
        vectorAligned<Vec3<T>> _cp_local_force_list;
        vectorAligned<Vec3<T>> _cp_pos_list;
        vectorAligned<Mat3<T>> _cp_frame_list;
};

#endif
