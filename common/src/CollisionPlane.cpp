#include "CollisionPlane.h"


#include "CollisionPlane.h"

template<typename T>
bool CollisionPlane<T>::ContactDetection(
        const Vec3<T> & cp_pos, T & penetration, 
        Mat3<T> & cp_frame){

    if(cp_pos[2] < _height){
        penetration = cp_pos[2]-_height;
        cp_frame.setIdentity();
        return true;
    }else {
        return false; 
    }
}

template class CollisionPlane<double>;
template class CollisionPlane<float>;

#ifdef PREVIOUS_CODE

/*!
 * Update body forces based on contact.
 * @param forces : output list of spatial forces on body (body indexed)
 * @param bodyMap : map from ground contact point to body in tree
 * @param p   : position of contact point (contact point indexed)
 * @param v   : velocity of contact point (contact point indexed)
 * @param dt  : simulator timestep (used for deformation model)
 */
template<typename T>
void CollisionPlane<T>::update(vectorAligned<SVec<T>>& external_forces, 
        std::vector<size_t>& bodyMap,
            vectorAligned<Vec3<T>>& p, vectorAligned<Vec3<T>>& v,vectorAligned<Vec3<T>>& fGC, T dt) {
  // first run the contact model on all points
  groundContactModelWithOffset(p, v, _tangentialDeflection, _forcesAtFoot, 
          _K, _D, _mu, dt, _location);

  // next update the spatial forces for the appropriate bodies:
  for(size_t i = 0; i < p.size(); i++) {
    exteranl_forces.at(bodyMap[i]) += forceToSpatialForce(_forcesAtFoot[i], p[i]);
    fGC[i] = _forcesAtFoot[i];
  }
}

#endif


