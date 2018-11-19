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
#include "orientation_tools.h"

#include <stdexcept>
#include <vector>
#include <string>
#include <stdio.h>

using namespace ori;
using namespace spatial;
using namespace std;


// parents, gr, jtype, Xtree, I, Xrot, Irot,

/*!
 * Populate member variables when bodies are added
 * @param count (6 for fb, 1 for joint)
 */
template<typename T>
void FloatingBaseModel<T>::addDynamicsVars(int count) {

  Mat6<T> eye6 = Mat6<T>::Identity();
  SVec<T> zero6 = SVec<T>::Zero();
  Mat6<T> zero66 = Mat6<T>::Zero();

  SpatialInertia<T> zeroInertia(zero66);
  for(int i = 0; i < count; i++) {

    _v.push_back(zero6);
    _vrot.push_back(zero6);
    _a.push_back(zero6);
    _arot.push_back(zero6);
    _avp.push_back(zero6);
    _avprot.push_back(zero6);
    _c.push_back(zero6);
    _crot.push_back(zero6);
    _S.push_back(zero6);
    _Srot.push_back(zero6);
    _f.push_back(zero6);
    _frot.push_back(zero6);
    _fvp.push_back(zero6);
    _fvprot.push_back(zero6);
    _ag.push_back(zero6);
    _agrot.push_back(zero6);
    _IC.push_back(zeroInertia);
    _Xup.push_back(eye6);
    _Xuprot.push_back(eye6);
    _Xa.push_back(eye6);
  }

  _J.push_back( D6Mat<T>::Zero(6,_nDof) );
  _Jdqd.push_back( SVec<T>::Zero() );
  
  resizeSystemMatricies();
}

/*!
 * Updates the size of H, C, Cqd, G, and Js when bodies are added
 */
template<typename T>
void FloatingBaseModel<T>::resizeSystemMatricies() {
  _H.setZero(_nDof, _nDof);
  _C.setZero(_nDof, _nDof);
  _Cqd.setZero(_nDof);
  _G.setZero(_nDof);
  for( size_t i = 0 ; i < _J.size()  ; i ++){
    _J[i].setZero(6, _nDof);
    _Jdqd[i].setZero();
  }

  for (size_t i = 0 ; i < _Jc.size() ; i++) {
    _Jc[i].setZero(3,_nDof);
    _Jcdqd[i].setZero();
  }
}

/*!
 * Create the floating body
 * @param inertia Spatial inertia of the floating body
 */
template <typename T>
void FloatingBaseModel<T>::addBase(const SpatialInertia<T>& inertia) {
  if(_nDof) {
    throw std::runtime_error("Cannot add base multiple times!\n");
  }

  Mat6<T> eye6 = Mat6<T>::Identity();
  Mat6<T> zero6 = Mat6<T>::Zero();
  SpatialInertia<T> zeroInertia(zero6);
  // the floating base has 6 DOFs

  _nDof = 6;
  for(size_t i = 0 ; i < 6 ; i++) {
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

  addDynamicsVars(6);
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

  Vec3<T> zero3 = Vec3<T>::Zero();

  _pGC.push_back(zero3);
  _vGC.push_back(zero3);

  D3Mat<T> J(3,_nDof);
  J.setZero();

  _Jc.push_back(J);
  _Jcdqd.push_back(zero3);
  _compute_contact_info.push_back(false);

  // add foot to foot list
  if(isFoot) {
    _footIndicesGC.push_back(_nGroundContact);
    _compute_contact_info[_nGroundContact] = true;
  }

  resizeSystemMatricies();
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
  _nDof++;

  addDynamicsVars(1);

  return _nDof;
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




/*!
 * Forward kinematics of all bodies.  Computes _Xup (from up the tree) and _Xa (from absolute)
 * Also computes _S (motion subspace), _v (spatial velocity in link coordinates), and 
 *_c (coriolis acceleration in link coordinates)
 */
template <typename T>
void FloatingBaseModel<T>::forwardKinematics() {
  if(_kin_flag)
    return;

  // calculate joint transformations
  _Xup[5] = createSXform(quaternionToRotationMatrix(_state.bodyOrientation), _state.bodyPosition);
  _v[5] = _state.bodyVelocity;
  for(size_t i = 6; i < _nDof ; i++) {
    // joint xform
    Mat6<T> XJ = jointXform(_jointTypes[i], _jointAxes[i], _state.q[i - 6]);
    _Xup[i] = XJ * _Xtree[i];
    _S[i] = jointMotionSubspace<T>(_jointTypes[i], _jointAxes[i]);
    SVec<T> vJ = _S[i] * _state.qd[i - 6];
    // total velocity of body i
    _v[i] = _Xup[i] * _v[_parents[i]] + vJ;
    

    // Same for rotors
    Mat6<T> XJrot = jointXform(_jointTypes[i], _jointAxes[i], _state.q[i - 6] * _gearRatios[i]);
    _Srot[i] = _S[i] * _gearRatios[i];
    SVec<T> vJrot = _Srot[i] * _state.qd[i - 6];
    _Xuprot[i] = XJrot * _Xrot[i];
    _vrot[i] = _Xuprot[i] * _v[_parents[i]] + vJrot;
    
    // Coriolis accelerations
    _c[i] = motionCrossProduct(_v[i], vJ);
    _crot[i] = motionCrossProduct(_vrot[i], vJrot);
  }

  // calculate from absolute transformations
  for (size_t i = 5; i < _nDof; i++) {
    if (_parents[i] == 0) {
      _Xa[i] = _Xup[i]; // float base
    } else {
      _Xa[i] = _Xup[i] * _Xa[_parents[i]];
    }
  }

  // ground contact points
  //  // TODO : we end up inverting the same Xa a few times (like for the 8 points on the body). this isn't super efficient.
  for(size_t j = 0; j < _nGroundContact; j++) {
    if(!_compute_contact_info[j])
      continue;
    size_t i = _gcParent.at(j);
    Mat6<T> Xai = invertSXform(_Xa[i]); // from link to absolute
    SVec<T> vSpatial = Xai * _v[i];

    // foot position in world
    _pGC.at(j) = sXFormPoint(Xai, _gcLocation.at(j));
    _vGC.at(j) = spatialToLinearVelocity(vSpatial, _pGC.at(j));
  }
  _kin_flag = true;
}


/*!
 * Compute the contact Jacobians (3xn matrices) for the velocity
 * of each contact point expressed in absolute coordinates
 */
template <typename T>
void FloatingBaseModel<T>::contactJacobians() {
  forwardKinematics();
  biasAccelerations();


  for (size_t k = 0 ; k < _nGroundContact ; k++ ) {
    _Jc[k].setZero();
    _Jcdqd[k].setZero();

    // Skip it if we don't care about it
    if(!_compute_contact_info[k])
      continue;

    size_t i = _gcParent.at(k);

    // Rotation to absolute coords
    Mat3<T> Rai = _Xa[i].template block<3,3>(0,0).transpose();
    Mat6<T> Xc = createSXform(Rai, _gcLocation.at(k));
    
    // Bias acceleration
    SVec<T> ac = Xc * _avp[i];
    SVec<T> vc = Xc * _v[i];
    // Correct to classical
    _Jcdqd[k] = sAccToClassicalAcc(ac, vc);

    // rows for linear velcoity in the world
    D3Mat<T> Xout = Xc.template bottomRows<3>();

    // from tips to base
    while (i > 5) {
      _Jc[k].col(i) = Xout * _S[i];
      Xout = Xout * _Xup[i];
      i = _parents[i];
    }
    _Jc[k].template leftCols<6>() = Xout;
  }
}

/*!
 * (Support Function) Computes velocity product accelerations of
 * each link and rotor _avp, and _avprot
 */
template <typename T>
void FloatingBaseModel<T>::biasAccelerations() {
  if(_acc_flag)
    return;
  forwardKinematics();
  // velocity product acceelration of base
  _avp[5] << 0,0,0,0,0,0;

  // from base to tips
  for(size_t i = 6; i < _nDof; i++) {

    // Outward kinamtic propagtion
    _avp[i] = _Xup[i] * _avp[_parents[i]] + _c[i];
    _avprot[i] = _Xuprot[i] * _avp[_parents[i]] + _crot[i];
  }
  _acc_flag = true;
}

/*!
 * Computes the generalized gravitational force (G) in the inverse dynamics
 * @return G (_nDof x 1 vector) 
 */
template <typename T>
DVec<T> FloatingBaseModel<T>::gravityForce() {
  compositeInertias();

  SVec<T> aGravity;
  aGravity << 0, 0, 0, _gravity[0], _gravity[1], _gravity[2];
  _ag[5] = _Xup[5] * aGravity;

  // Gravity comp force is the same as force required to accelerate 
  // oppostite gravity
  _G.template topRows<6>() = -_IC[5].getMatrix() * _ag[5];
  for(size_t i = 6; i < _nDof; i++) {
    _ag[i] = _Xup[i] * _ag[_parents[i]];
    _agrot[i] = _Xuprot[i] * _ag[_parents[i]];
    
    // body and rotor
    _G[i] = -_S[i].dot( _IC[i].getMatrix() * _ag[i]) - _Srot[i].dot(_Irot[i].getMatrix() * _agrot[i]); 
  }
  return _G;
}

/*!
 * Computes the generalized coriolis forces (Cqd) in the inverse dynamics
 * @return Cqd (_nDof x 1 vector) 
 */
template <typename T>
DVec<T> FloatingBaseModel<T>::coriolisForce() {
  biasAccelerations();

  // Floating base force
  Mat6<T> Ifb = _Ibody[5].getMatrix();
  SVec<T> hfb =  Ifb * _v[5];
  _fvp[5] = Ifb * _avp[5] + forceCrossProduct( _v[5] , hfb );

  for (size_t i = 6 ; i < _nDof ; i++ ) {
    // Force on body i
    Mat6<T> Ii = _Ibody[i].getMatrix();
    SVec<T> hi = Ii * _v[i];
    _fvp[i] = Ii * _avp[i] + forceCrossProduct( _v[i] , hi );

    // Force on rotor i
    Mat6<T> Ir = _Irot[i].getMatrix();
    SVec<T> hr = Ir * _vrot[i];
    _fvprot[i] = Ir * _avprot[i] + forceCrossProduct(_vrot[i] , hr );
  }

  for (size_t i = _nDof-1 ; i>5 ; i--) {
    // Extract force along the joints
    _Cqd[i] = _S[i].dot(_fvp[i]) + _Srot[i].dot(_fvprot[i]);

    // Propage force down the tree
    _fvp[_parents[i]] += _Xup[i].transpose() *  _fvp[i];
    _fvp[_parents[i]] += _Xuprot[i].transpose() *  _fvprot[i];
  }

  // Force on floating base
  _Cqd.template topRows<6>() = _fvp[5];
  return _Cqd;
}

/*!
 * (Support Function) Computes the composite rigid body inertia
 * of each subtree _IC[i] contains body i, and the body/rotor 
 * inertias of all successors of body i. 
 * (key note: _IC[i] does not contain rotor i)
 */
template <typename T>
void FloatingBaseModel<T>::compositeInertias() {
  if(_IC_flag)
    return;

  forwardKinematics();
  // initialize
  for (size_t i = 5 ; i < _nDof ; i++ ) {
    _IC[i].setMatrix( _Ibody[i].getMatrix() );
  }

  // backward loop
  for (size_t i = _nDof-1 ; i>5 ; i--) {
    // Propagate inertia down the tree
    _IC[_parents[i]].addMatrix( _Xup[i].transpose() * _IC[i].getMatrix() * _Xup[i] );
    _IC[_parents[i]].addMatrix( _Xuprot[i].transpose() * _Irot[i].getMatrix()  * _Xuprot[i] );
  }
  _IC_flag = true;
}

/*!
 * Computes the Mass Matrix (H) in the inverse dynamics formulation
 * @return H (_nDof x _nDof matrix) 
 */
template <typename T>
DMat<T> FloatingBaseModel<T>::massMatrix() {
  compositeInertias();
  _H.setZero();

  // Top left corner is the locked inertia of the whole system
  _H.template topLeftCorner<6,6>() = _IC[5].getMatrix();

  for (size_t j = 6 ; j < _nDof ; j++) {

    // f = spatial force required for a unit qdd_j
    SVec<T> f = _IC[j].getMatrix() * _S[j];
    SVec<T> frot = _Irot[j].getMatrix() * _Srot[j];
    
    _H(j,j) = _S[j].dot(f) + _Srot[j].dot(frot);
    
    // Propagate down the tree
    f = _Xup[j].transpose() * f + _Xuprot[j].transpose() * frot;
    size_t i = _parents[j];
    while (i > 5) {
      // in here f is expressed in frame {i}
      _H(i,j) = _S[i].dot(f);
      _H(j,i) = _H(i,j);

      // Propagate down the tree
      f = _Xup[i].transpose() * f;
      i = _parents[i];
    }

    // Force on floating base
    _H.template block<6,1>(0,j) = f;
    _H.template block<1,6>(j,0) = f.adjoint();
  }
  return _H;
}

/*!
 * Computes the inverse dynamics of the system
 * @return an _nDof x 1 vector. The first six entries
 * give the external wrengh on the base, with the remaining giving the 
 * joint torques
 */
template <typename T>
DVec<T> FloatingBaseModel<T>::inverseDynamics(FBModelStateDerivative<T> & dState) {
  forwardKinematics();
  biasAccelerations();
  
  // Initialize gravity with model info
  SVec<T> aGravity = SVec<T>::Zero();
  aGravity.template tail<3>() = _gravity;

  // Spatial force for floating base
  SVec<T> hb = _Ibody[5].getMatrix() * _v[5];
  _a[5] = -_Xup[5] * aGravity + dState.dBodyVelocity;
  _f[5] = _Ibody[5].getMatrix() * _a[5] + forceCrossProduct(_v[5], hb);

  // loop through joints
  for(size_t i = 6 ; i < _nDof ; i++) {
    // spatial acceleration
    _a[i] = _Xup[i]*_a[_parents[i]] + _S[i] * dState.qdd[i-6] + _c[i];
    _arot[i] = _Xuprot[i]*_a[_parents[i]] + _Srot[i] * dState.qdd[i-6] + _crot[i];
    
    // spatial momentum
    SVec<T> hi = _Ibody[i].getMatrix() * _v[i];
    SVec<T> hr = _Irot[i].getMatrix() * _vrot[i];
    
    // spatial force
    _f[i] = _Ibody[i].getMatrix() * _a[i] + forceCrossProduct(_v[i], hi);
    _frot[i] = _Irot[i].getMatrix() * _arot[i] + forceCrossProduct( _vrot[i], hr);
  }

  DVec<T> genForce(_nDof);
  for(size_t i = _nDof-1 ; i > 5 ; i--) {
    // Pull off compoents of force along the joint
    genForce[i] = _S[i].dot(_f[i]) + _Srot[i].dot(_frot[i]);

    // Propagate down the tree
    _f[_parents[i]] += _Xup[i].transpose() * _f[i];
    _f[_parents[i]] += _Xuprot[i].transpose() * _frot[i];
  }
  genForce.template head<6>() = _f[5];
  return genForce;
}

template class FloatingBaseModel<double>;
template class FloatingBaseModel<float>;
