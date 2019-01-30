/*! @file CollisionPlane.h
 *  @brief Collision logic for an infinite plane
 *
 *  Simplest collision, used for floor and global bounding box
 */

#ifndef COLLISIONPLANE_H
#define COLLISIONPLANE_H




#include "Collision.h"
#include "cppTypes.h"
#include <vector>

/*!
 * Class to represent infinite collision planes (like a flat ground).
 */
template<typename T>
class CollisionPlane : public Collision<T>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

/*!
 * Construct a new collision plane
 * @param location : coordinate transformation to collision plane (collision surface is the xy-plane)
 * @param nContactPoints : number of contact points this collision plane will need to handle.
 * @param mu : coefficient of friction
 * @param restitution  : rebounding ratio (v+/v-)
 * @param height  : height of this plane
 */
        CollisionPlane(const T & mu, 
                const T & restitution, 
                const T & height) : 
            Collision<T>(mu, restitution),
            _height(height)    {}

        virtual ~CollisionPlane(){}

        virtual bool ContactDetection(
                const Vec3<T> & cp_pos, T& penetration, Mat3<T> & cp_frame);

    private:
        T _height;
};






#ifdef PREVIOUS_CODE

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

#endif

#endif //COLLISIONPLANE_H

