#include "CollisionPlane.h"


/*!
 * Update body forces based on contact.
 * @param forces : output list of spatial forces on body (body indexed)
 * @param bodyMap : map from ground contact point to body in tree
 * @param p   : position of contact point (contact point indexed)
 * @param v   : velocity of contact point (contact point indexed)
 * @param dt  : simulator timestep (used for deformation model)
 */
template<typename T>
void CollisionPlane<T>::update(vectorAligned<SVec<T>>& forces, std::vector<size_t>& bodyMap,
            vectorAligned<Vec3<T>>& p, vectorAligned<Vec3<T>>& v, T dt) {
  // first run the contact model on all points
  groundContactModelWithOffset(p, v, _tangentialDeflection, _forcesAtFoot, _K, _D, _mu, dt, _location);

  // next update the spatial forces for the appropriate bodies:
  for(size_t i = 0; i < p.size(); i++) {
    forces.at(bodyMap[i]) += forceToSpatialForce(_forcesAtFoot[i], p[i]);
  }
}

template class CollisionPlane<double>;
template class CollisionPlane<float>;
