/*! @file CollisionPlane.h
 *  @brief Collision logic for an infinite plane
 *
 *  Simplest collision, used for floor and global bounding box
 */

#ifndef PROJECT_COLLISIONPLANE_H
#define PROJECT_COLLISIONPLANE_H

#include <cppTypes.h> // linear algebra types
#include <vector>

/*!
 * Class to represent infinite collision planes (like a flat ground).
 */
template<typename T> // template on floating point type (float or double)
class CollisionPlane {
public:

  /*!
   * Construct a new collision plane
   * @param normal : vector normal to plane
   * @param offset : all points on the plane satisfy normal.dot(x) + offset = 0
   * @param nContactPoints : maximum contact point id
   * @param mu : coefficient of friction
   * @param K  : spring constant
   * @param D  : damping constant
   */
  CollisionPlane(Vec3<T>& normal, T offset, int nContactPoints, T mu, T K, T D) : _normal(normal), _offset(offset),
     _mu(mu), _K(K), _D(D){

    // resize list of deflections
    _tangentialDeflection.resize(nContactPoints);

    // initialize all deflections to zero
    for(auto& u : _tangentialDeflection) {
      u = Vec2<T>::Zero();
    }
  }

private:
  Vec3<T> _normal;
  double _offset, _mu, _K, _D;
  vectorAligned<Vec2<T>> _tangentialDeflection;
};

#endif //PROJECT_COLLISIONPLANE_H
