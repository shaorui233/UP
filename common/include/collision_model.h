/*! @file collision_model.h
 *  @brief Nonlinear friction and collision model used in the simulator for ground contact
 *
 *  The collision model is based on the gcontact.m file in spatial_v2 and modified by Pat for codegen
 *  This uses the model in "Modelling the Contact Between a Rolling Sphere and a Compliant Ground Plane" by
 *  Azad and Featherstone.  It assumes a point contact.
 *
 *  This assumes that the collision is occuring with a plane going through the origin with a normal of (0,0,1)
 */

#ifndef PROJECT_COLLISION_MODEL_H
#define PROJECT_COLLISION_MODEL_H

#include "cppTypes.h"
#include <cmath>

/*!
 * Run the ground contact model on a series of points.
 * This assumes that the ground is the xy-plane.
 * The ground is allowed the deform in the tangential direction, but not the normal direction.
 * The ground also "remembers" its deformation between separate contact events. (however it does spring back pretty quickly)
 * @param _pGC List of ground contact point locations.
 * @param _vGC List of ground contact point velocities
 * @param _deflections List of deflection state variables.  This is updated by this model
 * @param K Ground stiffness
 * @param D Ground damping
 * @param mu Ground friction
 * @param dt Timestep (used for deflection)
 */
template <typename T>
void groundContactModel(vectorAligned<Vec3<T>>& _pGC, vectorAligned<Vec3<T>>& _vGC,
        vectorAligned<Vec2<T>>& _deflections, vectorAligned<Vec3<T>>& _forces, T K, T D, T mu, T dt) {

  size_t nPt = _pGC.size();
  for(size_t i = 0; i < nPt; i++) {
    Vec2<T> deflection = _deflections[i];            // the deflection of the ground
    T z = _pGC[i][2];                                // the penetration into the ground
    T zd = _vGC[i][2];                               // the penetration velocity
    T zr = std::sqrt(std::max(T(0), -z));               // sqrt penetration into the ground
    T normalForce = zr * (-K*z - D*zd);              // normal force is spring-damper * sqrt(penetration)
    _forces[i][0] = 0;
    _forces[i][1] = 0;
    _forces[i][2] = 0;
    bool inContact = normalForce > 0;                // contact flag

    // first assume there's no contact, so the ground "springs back"
    Vec2<T> deflectionRate = (-K/D) * deflection.template topLeftCorner<2,1>();

    if(inContact) {
      _forces[i][2] = normalForce;
      // first, assume sticking
      // this means the tangential deformation happens at the speed of the foot.
      deflectionRate = _vGC[i].template topLeftCorner<2,1>();

      Vec2<T> zr2(zr, zr);
      Vec2<T> Kc = K * zr2; // effective tangential spring constant
      //Vec2<T> Dc = D * zr2; // effective tangential damping
      Vec2<T> tangentialSpringForce(Kc[0]*deflection[0], Kc[1]*deflection[1]); // spring force due to ground deformation

      Vec2<T> tangentialForce = -tangentialSpringForce - D * zr * deflectionRate; // add damping to get total tangential

      // check for slipping:
      T slipForce = mu * normalForce;
      T tangentialForceMagnitude = tangentialForce.norm();
      T r = tangentialForceMagnitude / slipForce;

      if(r > 1) {
        // we are slipping.
        tangentialForce = tangentialForce / r;
        deflectionRate = - (tangentialForce + tangentialSpringForce) / (D * zr); // not sure where this comes from, but okay.
      }

      _forces[i][0] = tangentialForce[0];
      _forces[i][1] = tangentialForce[1];
    }

    _deflections[i] += dt * deflectionRate;
  }
}



#endif //PROJECT_COLLISION_MODEL_H
