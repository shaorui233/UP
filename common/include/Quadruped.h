/*! @file Quadruped.h
 *  @brief Data structure containing parameters for quadruped robot
 *
 *  This file contains the Quadruped class.  This stores all the parameters for a quadruped robot.  There are utility
 *  functions to generate Quadruped objects for Cheetah 3 (and eventually mini-cheetah).
 *  There is a buildModel() method which can be used to create a floating-base dynamics model of the quadruped.
 */

#ifndef LIBBIOMIMETICS_QUADRUPED_H
#define LIBBIOMIMETICS_QUADRUPED_H

#include "FloatingBaseModel.h"
#include "SpatialInertia.h"

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
  SpatialInertia<T> _abadInertia, _hipInertia, _kneeInertia, _abadRotorInertia, _hipRotorInertia, _kneeRotorInertia, _bodyInertia;
  Vec3<T> _abadLocation, _abadRotorLocation, _hipLocation, _hipRotorLocation, _kneeLocation, _kneeRotorLocation;
  FloatingBaseModel<T> buildModel();
};

template<typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2>& v, int legID);




#endif //LIBBIOMIMETICS_QUADRUPED_H
