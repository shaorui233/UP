/*! @file CollisionBox.h
 *  @brief Collision logic for a box
 */

#ifndef COLLISION_BOX_H
#define COLLISION_BOX_H

#include "Collision.h"
#include "cppTypes.h"
#include "utilities.h"
#include <vector>

/*!
 * Class to represent box collision
 */
template<typename T>
class CollisionBox : public Collision<T>{
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
        CollisionBox(const T & mu, 
                const T & restitution, 
                const T & depth, const T & width, const T & height,
                const Vec3<T> & position, const Mat3<T> & ori ) :
            Collision<T>(mu, restitution),
            _position(position), _orientation(ori){

                _size[0] = depth;
                _size[1] = width;
                _size[2] = height;

            }

        virtual ~CollisionBox(){}
        virtual bool ContactDetection(
                const Vec3<T> & cp_pos, T& penetration, Mat3<T> & cp_frame);

    private:
        T _size[3];

        Vec3<T> _position;
        Mat3<T> _orientation;
};

#endif //PROJECT_COLLISIONPLANE_H

