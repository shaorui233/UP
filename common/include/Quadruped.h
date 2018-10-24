//
// Created by jared on 10/12/18.
//

#ifndef LIBBIOMIMETICS_QUADRUPED_H
#define LIBBIOMIMETICS_QUADRUPED_H

#include <FloatingBaseModel.h>
#include <SpatialInertia.h>
#include <eigen3/Eigen/StdVector>

#include <vector>

using std::vector;

/*!
 * Representation of a quadruped robot's physical properties.
 *
 * When viewed from the top, the quadruped's legs are:
 *
 * FRONT
 * 2 1   RIGHT
 * 4 3
 * BACK
 *
 */
template<typename T>
class Quadruped {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  T _bodyLength, _bodyWidth, _bodyHeight, _bodyMass;
  T _abadGearRatio, _hipGearRatio, _kneeGearRatio;
  T _abadLinkLength, _hipLinkLength, _kneeLinkLenght;
  SpatialInertia<T> _abadInertia, _hipInertia, _kneeInertia, _abadRotorInertia, _hipRotorInertia, _kneeRotorInertia;
  Vec3<T> _abadLocation, _abadRotorLocation;
  FloatingBaseModel<T> buildModel();
};




#endif //LIBBIOMIMETICS_QUADRUPED_H
