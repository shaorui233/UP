/*! @file FloatingBaseModel.h
 *  @brief Implementation of Rigid Body Floating Base model data structure
 *
 * This class stores the kinematic tree described in "Rigid Body Dynamics Algorithms" by Featherstone
 * (download from https://www.springer.com/us/book/9780387743141 on MIT internet)
 *
 * The tree includes an additional "rotor" body for each body.  This rotor is fixed to the parent body and
 * has a gearing constraint.  This is efficiently included using a technique similar to what is described in
 * Chapter 12 of "Robot and Multibody Dynamics" by Jain.  Note that this implementation is highly specific to the case
 * of a single rotating rotor per rigid body.  Rotors have the same joint type as their body, but with an additional
 * gear ratio multiplier applied to the motion subspace. The rotors associated with the floating base don't do anything.
 */

#include <spatial.h>
#include <orientation_tools.h>
#include <SpatialInertia.h>
#include <vector>
#include <string>

#ifndef LIBBIOMIMETICS_FLOATINGBASEMODEL_H
#define LIBBIOMIMETICS_FLOATINGBASEMODEL_H

#include <eigen3/Eigen/StdVector>

using std::vector;
using namespace ori;
using namespace spatial;

/*!
 * Class to represent a floating base rigid body model with rotors and ground contacts
 */
template<typename T>
class FloatingBaseModel {
public:
  FloatingBaseModel() : _gravity(0,0,-9.81) {}
  void addBase(const SpatialInertia<T>& inertia);
  void addBase(T mass, const Vec3<T>& com, const Mat3<T>& I);
  int addGroundContactPoint(int bodyID, const Vec3<T>& location);
  void addGroundContactBoxPoints(int bodyId, const Vec3<T>& dims);

  int addBody(const SpatialInertia<T>& inertia, const SpatialInertia<T>& rotorInertia, T gearRatio,
              int parent, JointType jointType, CoordinateAxis jointAxis, const Mat6<T>& Xtree, const Mat6<T>& Xrot);

  int addBody(const MassProperties<T>& inertia, const MassProperties <T>& rotorInertia, T gearRatio,
              int parent, JointType jointType, CoordinateAxis jointAxis, const Mat6<T>& Xtree, const Mat6<T>& Xrot);

  /*!
   * Set the gravity
   */
  void setGravity(Vec3<T>& g) {
    _gravity = g;
  }

private:
  int _nDof = 0;
  Vec3 <T> _gravity;
  vector<int> _parents;
  vector<int> _gearRatios;
  vector<JointType> _jointTypes;
  vector<CoordinateAxis> _jointAxes;
  vector<Mat6<T>, Eigen::aligned_allocator<Mat6<T>>> _Xtree, _Xrot;
  vector<SpatialInertia<T>, Eigen::aligned_allocator<Mat6<T>>> _Ibody, _Irot;
  vector<std::string> _bodyNames;

  int _nGroundContact = 0;
  vector<int> _gcParent;
  vector<Vec3<T>> _gcLocation;

};


#endif //LIBBIOMIMETICS_FLOATINGBASEMODEL_H
