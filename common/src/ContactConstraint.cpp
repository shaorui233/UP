#include "ContactConstraint.h"

#include "FloatingBaseModel.h"

template<typename T>
void ContactConstraint<T>::UpdateExternalForces(T K, T D, T dt){
    _nContact = _CheckContact();
    for(size_t i(0); i<_model->_nGroundContact; ++i){
        deflectionRate[i] = (-K/D) * _tangentialDeflections[i];
    }
    if(_nContact > 0){
        _groundContactWithOffset(K, D);
        for(size_t i(0); i<_nContact; ++i ){
            _model->_externalForces.at(_model->_gcParent[_idx_list[i] ]) 
                += forceToSpatialForce(_cp_force_list[_idx_list[i]], _cp_pos_list[i]);
       }
        //for(size_t kk(0); kk<_cp_force_list.size(); ++kk){
            //pretty_print(_cp_force_list[kk], std::cout, "cp force");
            //pretty_print(_model->_externalForces.at(_model->_gcParent[kk]), 
                    //std::cout, "ext force");
        //}
        //printf("\n");

        static int count(0);
        ++count;
        if(count> 5){
            //exit(0);
        }
   }

   for(size_t i(0); i<_model->_nGroundContact; ++i){
       _tangentialDeflections[i] += dt * deflectionRate[i];
   }
}

/*!
 * Run the ground contact model for a single collision plane on a list of ground contact points
 * The ground is allowed to deform in the tangential direction, but not the normal direction.
 * The ground also "remembers" its deformation between separate contact events. (however it does spring back pretty quickly)
 * @param K Ground stiffness
 * @param D Ground damping
 * @param dt Timestep (used for deflection)
 */
template <typename T>
void ContactConstraint<T>::_groundContactWithOffset(T K, T D) {

    for(size_t i = 0; i < _nContact; i++) {

        Vec3<T> v = _cp_frame_list[i].transpose() * _model->_vGC[_idx_list[i]];              // velocity in plane coordinates
        T z = _cp_penetration_list[i];         // the penetration into the ground
        T zd = v[2];                          // the penetration velocity
        T zr = std::sqrt(std::max(T(0), -z)); // sqrt penetration into the ground, or zero if we aren't penetrating
        T normalForce = zr * (-K*z - D*zd);   // normal force is spring-damper * sqrt(penetration)

        // set output force to zero for now.
        _cp_local_force_list[i][0] = 0;
        _cp_local_force_list[i][1] = 0;
        _cp_local_force_list[i][2] = 0;


        if(normalForce >0){
            _cp_local_force_list[i][2] = normalForce;   // set the normal force. This is in the plane's coordinates for now

            // first, assume sticking
            // this means the tangential deformation happens at the speed of the foot.
            deflectionRate[_idx_list[i]] = v.template topLeftCorner<2,1>();
            Vec2<T> tangentialSpringForce = K * zr * _tangentialDeflections[_idx_list[i]]; // tangential force due to "spring"
            Vec2<T> tangentialForce = -tangentialSpringForce 
                - D * zr * deflectionRate[_idx_list[i]]; // add damping to get total tangential

            // check for slipping:
            T slipForce = _cp_mu_list[i] * normalForce;        // maximum force magnitude without slipping
            T tangentialForceMagnitude = tangentialForce.norm(); // actual force magnitude if we assume sticking
            T r = tangentialForceMagnitude / slipForce;          // ratio of force/max_force

            if(r > 1) {
                // we are slipping.
                tangentialForce = tangentialForce / r;    // adjust tangential force to avoid slipping
                deflectionRate[_idx_list[i]] = - (tangentialForce + tangentialSpringForce) / (D * zr);
            }
            // set forces
            _cp_local_force_list[i][0] = tangentialForce[0];
            _cp_local_force_list[i][1] = tangentialForce[1];
        }
        // move back into robot frame
        _cp_force_list[_idx_list[i]] = _cp_frame_list[i] * _cp_local_force_list[i];
    }
}

template<typename T>
size_t ContactConstraint<T>::_CheckContact(){

    _cp_pos_list.clear();
    _cp_local_force_list.clear();
    _cp_penetration_list.clear();

    _cp_frame_list.clear();
    _cp_resti_list.clear();
    _cp_mu_list.clear();
    _idx_list.clear();

    Vec3<T> tmp_vec;
    Mat3<T> cp_frame;



    T penetration;
    for(size_t i(0); i<_model->_nGroundContact; ++i){
        _cp_force_list[i].setZero();
        for(size_t j(0); j<_nCollision; ++j){
            if(_collision_list[j]->ContactDetection(
                        _model->_pGC[i], penetration, cp_frame)) {
                // Contact Happens
                _cp_pos_list.push_back(_model->_pGC[i]);
                _cp_local_force_list.push_back(Vec3<T>::Zero());
                _cp_frame_list.push_back(cp_frame);
                _cp_penetration_list.push_back(penetration);

                _idx_list.push_back(i);

                _cp_resti_list.push_back(_collision_list[j]->getRestitutionCoeff());
                _cp_mu_list.push_back(_collision_list[j]->getFrictionCoeff());
                //printf("idx: %lu, %lu, contact point: %f, %f, %f\n",
                //j, i, _model->_pGC[i][0],_model->_pGC[i][1],_model->_pGC[i][2]  );
            }
        }
    }
    return _cp_pos_list.size();
}

template class ContactConstraint<double>;
template class ContactConstraint<float>;

//template <typename T> class ContactConstraint;
