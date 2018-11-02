/*! @file FloatingBaseModel.cpp
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

#include "FloatingBaseModel.h"
#include <orientation_tools.h>
#include <stdexcept>
#include <vector>
#include <string>

using namespace ori;
using namespace spatial;

// parents, gr, jtype, Xtree, I, Xrot, Irot,

/*!
 * Create the floating body
 * @param inertia Spatial inertia of the floating body
 */
template <typename T>
void FloatingBaseModel<T>::addBase(const SpatialInertia<T>& inertia) {
  if(_nDof) {
    throw std::runtime_error("Cannot add base multiple times!\n");
  }

  // the floating base has 6 DOFs
  _nDof = 6;

  Mat6<T> eye6 = Mat6<T>::Identity();
  Mat6<T> zero6 = Mat6<T>::Zero();
  SpatialInertia<T> zeroInertia(zero6);

  for(int i = 0; i < 6; i++) {
    _parents.push_back(0);
    _gearRatios.push_back(0);
    _jointTypes.push_back(JointType::Nothing); // doesn't actually matter
    _jointAxes.push_back(CoordinateAxis::X); // doesn't actually matter
    _Xtree.push_back(eye6);
    _Ibody.push_back(zeroInertia);
    _Xrot.push_back(eye6);
    _Irot.push_back(zeroInertia);
    _bodyNames.push_back("N/A");
  }

  _jointTypes[5] = JointType::FloatingBase;
  _Ibody[5] = inertia;
  _gearRatios[5] = 1;
  _bodyNames[5] = "Floating Base";
}

/*!
 * Create the floating body
 * @param mass Mass of the floating body
 * @param com  Center of mass of the floating body
 * @param I    Rotational inertia of the floating body
 */
template<typename T>
void FloatingBaseModel<T>::addBase(T mass, const Vec3 <T> &com, const Mat3 <T> &I) {
  SpatialInertia<T> IS(mass, com, I);
  addBase(IS);
}

/*!
 * Add a ground contact point to a model
 * @param bodyID The ID of the body containing the contact point
 * @param location The location (in body coordinate) of the contact point
 * @param isFoot True if foot or not.
 * @return The ID of the ground contact point
 */
template<typename T>
int FloatingBaseModel<T>::addGroundContactPoint(int bodyID, const Vec3 <T> &location, bool isFoot) {
  if((size_t)bodyID >= _nDof) {
    throw std::runtime_error("addGroundContactPoint got invalid bodyID: " + std::to_string(bodyID) + " nDofs: " + std::to_string(_nDof) + "\n");
  }

  //std::cout << "pt-add: " << location.transpose() << "\n";
  _gcParent.push_back(bodyID);
  _gcLocation.push_back(location);

  // add foot to foot list
  if(isFoot) {
    _footIndicesGC.push_back(_nGroundContact);
  }

  return _nGroundContact++;
}

/*!
 * Add the bounding points of a box to the contact model. Assumes the box is centered around the origin of the
 * body coordinate system and is axis aligned.
 */
template<typename T>
void FloatingBaseModel<T>::addGroundContactBoxPoints(int bodyId, const Vec3<T> &dims) {
  addGroundContactPoint(bodyId, Vec3<T>( dims(0),  dims(1),  dims(2))/2);
  addGroundContactPoint(bodyId, Vec3<T>(-dims(0),  dims(1),  dims(2))/2);
  addGroundContactPoint(bodyId, Vec3<T>( dims(0), -dims(1),  dims(2))/2);
  addGroundContactPoint(bodyId, Vec3<T>(-dims(0), -dims(1),  dims(2))/2);
  addGroundContactPoint(bodyId, Vec3<T>( dims(0),  dims(1), -dims(2))/2);
  addGroundContactPoint(bodyId, Vec3<T>(-dims(0),  dims(1), -dims(2))/2);
  addGroundContactPoint(bodyId, Vec3<T>( dims(0), -dims(1), -dims(2))/2);
  addGroundContactPoint(bodyId, Vec3<T>( dims(0), -dims(1), -dims(2))/2);
}

/*!
 * Add a body
 * @param inertia The inertia of the body
 * @param rotorInertia The inertia of the rotor the body is connected to
 * @param gearRatio The gear ratio between the body and the rotor
 * @param parent The parent body, which is also assumed to be the body the rotor is connected to
 * @param jointType The type of joint (prismatic or revolute)
 * @param jointAxis The joint axis (X,Y,Z), in the parent's frame
 * @param Xtree  The coordinate transformation from parent to this body
 * @param Xrot  The coordinate transformation from parent to this body's rotor
 * @return The body's ID (can be used as the parent)
 */
template<typename T>
int FloatingBaseModel<T>::addBody(const SpatialInertia<T> &inertia, const SpatialInertia<T> &rotorInertia, T gearRatio,
                                   int parent, JointType jointType, CoordinateAxis jointAxis, const Mat6<T> &Xtree,
                                   const Mat6<T> &Xrot) {
  if((size_t)parent >= _nDof) {
    throw std::runtime_error("addBody got invalid parent: " + std::to_string(parent) + " nDofs: " + std::to_string(_nDof) + "\n");
  }
  _parents.push_back(parent);
  _gearRatios.push_back(gearRatio);
  _jointTypes.push_back(jointType);
  _jointAxes.push_back(jointAxis);
  _Xtree.push_back(Xtree);
  _Xrot.push_back(Xrot);
  _Ibody.push_back(inertia);
  _Irot.push_back(rotorInertia);

  return _nDof++;
}

/*!
 * Add a body
 * @param inertia The inertia of the body
 * @param rotorInertia The inertia of the rotor the body is connected to
 * @param gearRatio The gear ratio between the body and the rotor
 * @param parent The parent body, which is also assumed to be the body the rotor is connected to
 * @param jointType The type of joint (prismatic or revolute)
 * @param jointAxis The joint axis (X,Y,Z), in the parent's frame
 * @param Xtree  The coordinate transformation from parent to this body
 * @param Xrot  The coordinate transformation from parent to this body's rotor
 * @return The body's ID (can be used as the parent)
 */
template<typename T>
int FloatingBaseModel<T>::addBody(const MassProperties<T> &inertia, const MassProperties<T> &rotorInertia, T gearRatio,
                                  int parent, JointType jointType, CoordinateAxis jointAxis, const Mat6<T> &Xtree,
                                  const Mat6<T> &Xrot) {
  return addBody(SpatialInertia<T>(inertia), SpatialInertia<T>(rotorInertia), gearRatio, parent, jointType, jointAxis,
          Xtree, Xrot);
}

template<typename T>
void FloatingBaseModel<T>::check() {
  if(_nDof != _parents.size())
    throw std::runtime_error("Invalid dof and parents length");
}

/*!
 * Compute the total mass of bodies which are not rotors.
 * @return
 */
template<typename T>
T FloatingBaseModel<T>::totalNonRotorMass() {
  T totalMass = 0;
  for(size_t i = 0; i < _nDof; i++) {
    totalMass += _Ibody[i].getMass();
  }
  return totalMass;
}

/*!
 * Compute the total mass of bodies which are not rotors
 * @return
 */
template<typename T>
T FloatingBaseModel<T>::totalRotorMass() {
  T totalMass = 0;
  for(size_t i = 0; i < _nDof; i++) {
    totalMass += _Irot[i].getMass();
  }
  return totalMass;
}

template class FloatingBaseModel<double>;
template class FloatingBaseModel<float>;