/*! @file DynamicsSimulator.cpp
 *  @brief Rigid Body Dynamics Simulator with Collisions
 *
 *  Combines ABA, Collisions, integrator, and any other external forces to run a simulation.
 *  Doesn't do any graphics.
 */

#include "DynamicsSimulator.h"


/*!
 * Initialize the dynamics simulator by allocating memory for ABA matrices
 */
template <typename T>
DynamicsSimulator<T>::DynamicsSimulator(FloatingBaseModel<T> &model) :_model(model) {
  // initialize the ABA quantities:
  _nb = _model._nDof;
  _nGC = _model._nGroundContact;

  // allocate matrices
  _Xup.resize(_nb);
  _Xa.resize(_nb);
  _Xuprot.resize(_nb);
  _IA.resize(_nb);
  _v.resize(_nb);
  _vrot.resize(_nb);
  _a.resize(_nb);
  _c.resize(_nb);
  _crot.resize(_nb);
  _U.resize(_nb);
  _Urot.resize(_nb);
  _Utot.resize(_nb);
  _S.resize(_nb);
  _Srot.resize(_nb);
  _pA.resize(_nb);
  _pArot.resize(_nb);
  _d.resize(_nb);
  _u.resize(_nb);
  _pGC.resize(_nGC);
  _vGC.resize(_nGC);

  // set stuff for 0:5
  for(size_t i = 0; i < 6; i++) {
    _Xup[i] = Mat6<T>::Identity();
    _Xa[i] = Mat6<T>::Identity();
    _Xuprot[i] = Mat6<T>::Identity();
    _v[i] = SVec<T>::Zero();
    _vrot[i] = SVec<T>::Zero();
    _a[i] = SVec<T>::Zero();
    _c[i] = SVec<T>::Zero();
    _crot[i] = SVec<T>::Zero();
    _U[i] = SVec<T>::Zero();
    _Urot[i] = SVec<T>::Zero();
    _Utot[i] = SVec<T>::Zero();
    _S[i] = SVec<T>::Zero();
    _Srot[i] = SVec<T>::Zero();
    _d[i] = 0;
    _u[i] = 0;
    _pA[i] = SVec<T>::Zero();
    _pArot[i] = SVec<T>::Zero();
  }

  _state.bodyVelocity = SVec<T>::Zero();
  _state.bodyPosition = Vec3<T>::Zero();
  _state.bodyOrientation = Quat<T>::Zero();
  _state.q = DVec<T>::Zero(_nb - 6);
  _state.qd = DVec<T>::Zero(_nb - 6);

  _externalForces.resize(_nb);
  for(size_t i = 0; i < _nb; i++) {
    _externalForces[i] = SVec<T>::Zero();
  }
}

/*!
 * Take one simulation step
 * @param dt : timestep duration
 * @param tau : joint torques
 */
template <typename T>
void DynamicsSimulator<T>::step(T dt, const DVec<T> &tau) {
  // fwd-kin on gc points
  // compute ground contact forces
  // aba
  // integrate
  forwardKinematics(); // compute forward kinematics
  updateCollisions();  // process collisions
  updateGroundForces();
  runABA(tau);         // dynamics algorithm
  integrate(dt);       // step forward
}

/*!
 * Forward kinematics of all bodies.  Computes _Xup (from up the tree) and _Xa (from absolute)
 * Also computes _S (motion subspace), _v (spatial velocity in link coordinates), and _c
 */
template <typename T>
void DynamicsSimulator<T>::forwardKinematics() {

  // calculate joint transformations
  _Xup[5] = createSXform(quaternionToRotationMatrix(_state.bodyOrientation), _state.bodyPosition);
  _v[5] = _state.bodyVelocity;
  for(size_t i = 6; i < _nb; i++) {
    // joint xform
    Mat6<T> XJ = jointXform(_model._jointTypes[i], _model._jointAxes[i], _state.q[i - 6]);
    _Xup[i] = XJ * _model._Xtree[i];

    _S[i] = jointMotionSubspace<T>(_model._jointTypes[i], _model._jointAxes[i]);
    SVec<T> vJ = _S[i] * _state.qd[i - 6];

    // total velocity of body i
    _v[i] = _Xup[i] * _v[_model._parents[i]] + vJ;
    _c[i] = motionCrossProduct(_v[i], vJ); // this isn't
  }

  // calculate from absolute transformations
  for (size_t i = 5; i < _nb; i++) {
    if (_model._parents[i] == 0) {
      _Xa[i] = _Xup[i]; // float base
    } else {
      _Xa[i] = _Xup[i] * _Xa[_model._parents[i]];
    }
  }

  // ground contact points
//  // TODO : we end up inverting the same Xa a few times (like for the 8 points on the body). this isn't super efficient.
  for(size_t j = 0; j < _nGC; j++) {
    size_t i = _model._gcParent.at(j);
    Mat6<T> Xai = invertSXform(_Xa[i]); // from link to absolute
    SVec<T> vSpatial = Xai * _v[i];

    // foot position in world
    _pGC.at(j) = sXFormPoint(Xai, _model._gcLocation.at(j));
    _vGC.at(j) = spatialToLinearVelocity(vSpatial, _pGC.at(j));
  }
}

template <typename T>
void DynamicsSimulator<T>::updateCollisions() {
 // TODO
}

template <typename T>
void DynamicsSimulator<T>::updateGroundForces() {
 // TODO
}

template <typename T>
void DynamicsSimulator<T>::integrate(T dt) {

  Vec3<T> omegaBody = _dstate.dBaseVelocity.template block<3,1>(3,0);
  Mat6<T> X = createSXform(quaternionToRotationMatrix(_state.bodyOrientation), _state.bodyPosition);
  RotMat<T> R = rotationFromSXform(X);
  Vec3<T> omega0 = R.transpose() * omegaBody;
  Vec3<T> axis;
  T ang = omega0.norm();
  if(ang > 0) {
    axis = omega0 / ang;
  } else {
    axis = Vec3<T>(1,0,0);
  }

  ang *= dt;
  Vec3<T> ee = std::sin(ang/2) * axis;
  Quat<T> quatD(std::cos(ang), ee[0], ee[1], ee[2]);

  Quat<T> quatNew = quatProduct(quatD, _state.bodyOrientation);
  quatNew = quatNew / quatNew.norm();

  // actual integration
  _state.q += _state.qd * dt;
  _state.qd += _dstate.qdd * dt;
  _state.bodyVelocity += _dstate.dBaseVelocity * dt;
  _state.bodyPosition += _dstate.dBasePosition * dt;
  _state.bodyOrientation = quatNew;
}

/*!
 * Articulated Body Algorithm, modified by Pat to add rotors
 */
template <typename T>
void DynamicsSimulator<T>::runABA(const DVec<T> &tau) {
  (void)tau;
  // create spatial vector for gravity
  SVec<T> aGravity;
  aGravity << 0, 0, 0, _model._gravity[0], _model._gravity[1], _model._gravity[2];

  // float-base articulated inertia
  _IA[5] = _model._Ibody[5].getMatrix();
  SVec<T> ivProduct = _model._Ibody[5].getMatrix() * _v[5];
  _pA[5] = forceCrossProduct(_v[5], ivProduct);

  // loop 1, down the tree
  for(size_t i = 6; i < _nb; i++) {

    _IA[i] = _model._Ibody[i].getMatrix(); // initialize
    ivProduct = _model._Ibody[i].getMatrix() * _v[i];
    _pA[i] = forceCrossProduct(_v[i], ivProduct);

    // same for rotors
    Mat6<T> XJrot = jointXform(_model._jointTypes[i], _model._jointAxes[i], _state.q[i - 6] * _model._gearRatios[i]);
    _Srot[i] = _S[i] * _model._gearRatios[i];
    SVec<T> vJrot = _Srot[i] * _state.qd[i - 6];
    _Xuprot[i] = XJrot * _model._Xrot[i];
    _vrot[i] = _Xuprot[i] * _v[_model._parents[i]] + vJrot;
    _crot[i] = motionCrossProduct(_vrot[i], vJrot);
    ivProduct = _model._Irot[i].getMatrix() * _vrot[i];
    _pArot[i] = forceCrossProduct(_vrot[i], ivProduct);
  }

  // adjust pA for external forces
  for (size_t i = 5; i < _nb; i++) {
    // TODO add if statement (avoid these calculations if the force is zero)
    Mat3<T> R = rotationFromSXform(_Xa[i]);
    Vec3<T> p = translationFromSXform(_Xa[i]);
    Mat6<T> iX = createSXform(R.transpose(), -R * p);
    _pA[i] = _pA[i] - iX.transpose() * _externalForces.at(i);
  }

  // Pat's magic principle of least constraint
  for(size_t i = _nb - 1; i >= 6; i--) {
    _U[i] = _IA[i] * _S[i];
    _Urot[i] = _model._Irot[i].getMatrix() * _Srot[i];
    _Utot[i] = _Xup[i].transpose() * _U[i] + _Xuprot[i].transpose() * _Urot[i];

    _d[i] = _Srot[i].transpose() * _Urot[i];
    _d[i] += _S[i].transpose() * _U[i];
    _u[i] = tau[i - 6] - _S[i].transpose() * _pA[i] - _Srot[i].transpose() * _pArot[i] - _U[i].transpose()*_c[i] - _Urot[i].transpose() * _crot[i];

    // articulated inertia recursion
    Mat6<T> Ia = _Xup[i].transpose() * _IA[i] * _Xup[i] + _Xuprot[i].transpose() * _model._Irot[i].getMatrix() * _Xuprot[i] - _Utot[i] * _Utot[i].transpose() / _d[i];
    _IA[_model._parents[i]] += Ia;
    SVec<T> pa = _Xup[i].transpose() * (_pA[i] + _IA[i] * _c[i]) + _Xuprot[i].transpose() * (_pArot[i] + _model._Irot[i].getMatrix() * _crot[i]) + _Utot[i] * _u[i] / _d[i];
    _pA[_model._parents[i]] += pa;
  }

  // include gravity and compute acceleration of floating base
  SVec<T> a0 = -aGravity;
  SVec<T> ub = -_pA[5];
  _a[5] = _Xup[5] * a0;
  SVec<T> afb = _IA[5].colPivHouseholderQr().solve(ub - _IA[5].transpose() * _a[5]);
  _a[5] += afb;

  // joint accelerations
  _dstate.qdd = DVec<T>(_nb - 6);
  for(size_t i = 6; i < _nb; i++) {
    _dstate.qdd[i - 6] = (_u[i] - _Utot[i].transpose() * _a[_model._parents[i]]) / _d[i];
    _a[i] = _Xup[i] * _a[_model._parents[i]] + _S[i] * _dstate.qdd[i-6] + _c[i];
  }

  // output
  RotMat<T> Rup = rotationFromSXform(_Xup[5]);
  // TODO : I think this is wrong (and probably unused, as a result)
  _dstate.dQuat = quatDerivative(_state.bodyOrientation, _state.bodyVelocity.template block<3,1>(0,0));
  _dstate.dBasePosition = Rup.transpose() * _state.bodyVelocity.template block<3,1>(3,0);
  _dstate.dBaseVelocity = afb;
  // qdd is set in the for loop above

}

template class DynamicsSimulator<double>;
template class DynamicsSimulator<float>;