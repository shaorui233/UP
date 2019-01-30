/*! @file Collision.h
 *  @brief Collision logic for an infinite plane
 *
 *  Simplest collision, used for floor and global bounding box
 */

#ifndef COLLISION_H
#define COLLISION_H

#include "cppTypes.h"
#include <vector>

/*!
 * Abstract Collision Class
 */
template<typename T>
class Collision {
public:
  /*!
   * Construct a new collision 
   * @param mu : coefficient of friction
   */
  Collision(const T & mu, const T & resti):_mu(mu), _restitution_coeff(resti){}
  virtual ~Collision(){}

  virtual bool ContactDetection(const Vec3<T> & cp_pos, T& penetration, Mat3<T> & cp_frame) = 0;

  const T & getFrictionCoeff(){ return _mu; }
  const T & getRestitutionCoeff(){ return _restitution_coeff; }

protected:
  T _mu, _restitution_coeff;
};

#endif //COLLISION_H

