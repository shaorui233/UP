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


//template <typename T>
//void groundContactModel(vectorAligned<Vec3<T>>& _pGC, vectorAligned<Vec3<T>>& _vGC,
//        vectorAligned<Vec3<T>>& _deflections, T K, T D, T mu, T dt) {
//
//}



#endif //PROJECT_COLLISION_MODEL_H
