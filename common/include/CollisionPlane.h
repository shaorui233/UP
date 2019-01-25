/*! @file CollisionPlane.h
 *  @brief Collision logic for an infinite plane
 *
 *  Simplest collision, used for floor and global bounding box
 */

#ifndef PROJECT_COLLISIONPLANE_H
#define PROJECT_COLLISIONPLANE_H

#include "cppTypes.h"
#include "spatial.h"
#include "collision_model.h"

#include <vector>

using namespace spatial;

/*!
 * Class to represent infinite collision planes (like a flat ground).
 */
template<typename T>
class CollisionPlane {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Construct a new collision plane
   * @param location : coordinate transformation to collision plane (collision surface is the xy-plane)
   * @param nContactPoints : number of contact points this collision plane will need to handle.
   * @param mu : coefficient of friction
   * @param K  : spring constant
   * @param D  : damping constant
   */
  CollisionPlane(SXform<T>& location, size_t nContactPoints, T mu, T K, T D) : _location(location),
     _mu(mu), _K(K), _D(D){

    // resize list of deflections
    _tangentialDeflection.resize(nContactPoints);
    _forcesAtFoot.resize(nContactPoints);

    // initialize all deflections to zero
    for(auto& u : _tangentialDeflection) {
      u = Vec2<T>::Zero();
    }
  }

  /*!
   * Get the location of the collision plane.
   */
  SXform<T>& getLocation() {
    return _location;
  }

  void update(vectorAligned<SVec<T>>& forces, std::vector<size_t>& bodyMap,
          vectorAligned<Vec3<T>>& p, vectorAligned<Vec3<T>>& v, 
          vectorAligned<Vec3<T>>& f, T dt);

private:
  SXform<T>& _location;
  T _mu, _K, _D;
  vectorAligned<Vec2<T>> _tangentialDeflection;
  vectorAligned<Vec3<T>> _forcesAtFoot;
};

#endif //PROJECT_COLLISIONPLANE_H

