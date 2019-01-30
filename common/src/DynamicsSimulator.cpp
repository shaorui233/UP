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
    _contact_constr = new ContactConstraint<T>(&_model);

    // initialize the ABA quantities:
    _nb = _model._nDof;
    _nGC = _model._nGroundContact;

    // allocate matrices
    _Xup.resize(_nb);
    _Xa.resize(_nb);
    _ChiUp.resize(_nb);
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
    _fGC.resize(_nGC);

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
    _qdd_from_subqdd.resize(_nb-6,_nb-6);
    _qdd_from_base_accel.resize(_nb-6,6);

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
    _model.setState(_state);
    _model.forwardKinematics(); // compute forward kinematics
    updateCollisions(dt);  // process collisions
    // TEST
    DVec<T> test_command = tau;
    test_command.setZero();
    _model.runABA(test_command, _dstate);         // dynamics algorithm
    integrate(dt);       // step forward
    _model.resetExternalForces(); // clesetar external forces
    _model.resetCalculationFlags();
}

/*!
 * Forward kinematics of all bodies.  Computes _Xup (from up the tree) and _Xa (from absolute)
 * Also computes _S (motion subspace), _v (spatial velocity in link coordinates), and _c
 */
template <typename T>
void DynamicsSimulator<T>::forwardKinematics() {
    if (_kinematicsUpToDate)
        return;

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
    _kinematicsUpToDate = true;
}

template <typename T>
void DynamicsSimulator<T>::updateCollisions(T dt) {
    forwardKinematics(); 
    //for(auto& cp : _collisionPlanes) {
    //cp.update(_externalForces, _model._gcParent, _pGC, _vGC, _fGC, dt);
    //}
    _contact_constr->UpdateExternalForces(5e5, 5e3, dt);
}


/*!
 * Integrate the floating base state
 * @param dt timestep
 */
template <typename T>
void DynamicsSimulator<T>::integrate(T dt) {

    // this was incredibly wrong.
    //Vec3<T> omegaBody = _dstate.dBodyVelocity.template block<3,1>(0,0);
    Vec3<T> omegaBody = _state.bodyVelocity.template block<3,1>(0,0);
    Mat6<T> X = createSXform(quaternionToRotationMatrix(_state.bodyOrientation), _state.bodyPosition);
    RotMat<T> R = rotationFromSXform(X);
    Vec3<T> omega0 = R.transpose() * omegaBody;

    // actual integration
    _state.q += _state.qd * dt;
    _state.qd += _dstate.qdd * dt;
    _state.bodyVelocity += _dstate.dBodyVelocity * dt;
    _state.bodyPosition += _dstate.dBodyPosition * dt;
    _state.bodyOrientation = integrateQuat(_state.bodyOrientation, omega0, dt);
    resetCalculationFlags();
}

/*!
 * Support function for the ABA
 */
    template <typename T>
void DynamicsSimulator<T>::updateArticulatedBodies()
{
    if(_articulatedBodiesUpToDate)
        return;

    forwardKinematics();

    _IA[5] = _model._Ibody[5].getMatrix();

    // loop 1, down the tree
    for(size_t i = 6; i < _nb; i++) {
        _IA[i] = _model._Ibody[i].getMatrix(); // initialize
        Mat6<T> XJrot = jointXform(_model._jointTypes[i], _model._jointAxes[i], _state.q[i - 6] * _model._gearRatios[i]);
        _Xuprot[i] = XJrot * _model._Xrot[i];
        _Srot[i] = _S[i] * _model._gearRatios[i];

    }

    // Pat's magic principle of least constraint (Guass too!)
    for(size_t i = _nb - 1; i >= 6; i--) {
        _U[i] = _IA[i] * _S[i];
        _Urot[i] = _model._Irot[i].getMatrix() * _Srot[i];
        _Utot[i] = _Xup[i].transpose() * _U[i] + _Xuprot[i].transpose() * _Urot[i];

        _d[i] = _Srot[i].transpose() * _Urot[i];
        _d[i] += _S[i].transpose() * _U[i];

        // articulated inertia recursion
        Mat6<T> Ia = _Xup[i].transpose() * _IA[i] * _Xup[i] + _Xuprot[i].transpose() * _model._Irot[i].getMatrix() * _Xuprot[i] - _Utot[i] * _Utot[i].transpose() / _d[i];
        _IA[_model._parents[i]] += Ia;
    }

    _invIA5.compute(_IA[5]);
    _articulatedBodiesUpToDate = true;
}

/*!
 * Support function for contact inertia algorithms
 * Comptues force propagators across each joint
 */
    template <typename T>
void DynamicsSimulator<T>::updateForcePropagators()
{
    if(_forcePropagatorsUpToDate)
        return;
    updateArticulatedBodies();
    for (size_t i = 6 ; i < _nb ; i++) {
        _ChiUp[i] = _Xup[i] - _S[i]*_Utot[i].transpose()/_d[i];
    }
    _forcePropagatorsUpToDate = true;
}

/*!
 * Support function for contact inertia algorithms
 * Computes the qdd arising from "subqdd" components
 * If you are familiar with Featherstone's sparse Op sp
 * or jain's innovations factorization:
 * H = L * D * L^T
 * These subqdd components represnt the space in the middle
 * i.e. if H^{-1} = L^{-T} * D^{-1} * L^{1}
 * then what I am calling subqdd = L^{-1} * tau
 * This is an awful explanation. It needs latex.
 */
template <typename T>
void DynamicsSimulator<T>::udpateQddEffects() {
    if(_qddEffectsUpToDate)
        return;
    updateForcePropagators();
    _qdd_from_base_accel.setZero();
    _qdd_from_subqdd.setZero();

    // Pass for force props
    // This loop is semi-equivalent to a cholesky factorization on H
    // akin to Featherstone's sparse operational space algo
    // These computations are for treating the joint rates like a task space
    // To do so, F computes the dynamic effect of torues onto bodies down the tree
    // 
    for (size_t i = 6 ; i < _nb ; i++) {
        _qdd_from_subqdd(i-6,i-6) = 1;
        SVec<T> F = (_ChiUp[i].transpose() - _Xup[i].transpose() ) * _S[i];
        size_t j = _model._parents[i];
        while ( j > 5) {
            _qdd_from_subqdd(i-6,j-6) = _S[j].dot(F);
            F = _ChiUp[j].transpose()*F;
            j = _model._parents[j];
        }
        _qdd_from_base_accel.row(i-6) = F.transpose();
    }
    _qddEffectsUpToDate = true;
}


/*!
 * Apply a unit test force at a contact. Returns the inv contact inertia  in that direction
 * and computes the resultant qdd
 * @param gc_index index of the contact
 * @param force_ics_at_contact unit test forcoe
 * @params dstate - Output paramter of resulting accelerations
 * @return the 1x1 inverse contact inertia J H^{-1} J^T
 */
template <typename T>
    double
DynamicsSimulator<T>:: applyTestForce(const int gc_index, const Vec3<T>& force_ics_at_contact, FBModelStateDerivative<T> & dstate_out)
{
    forwardKinematics();
    updateArticulatedBodies();
    updateForcePropagators();
    udpateQddEffects();


    size_t i_opsp = _model._gcParent.at(gc_index);
    size_t i = i_opsp;

    dstate_out.qdd.setZero();


    // Rotation to absolute coords
    Mat3<T> Rai = _Xa[i].template block<3,3>(0,0).transpose();
    Mat6<T> Xc = createSXform(Rai, _model._gcLocation.at(gc_index));

    // D is one column of an extended force propagator matrix (See Wensing, 2012 ICRA)
    SVec<T> F = Xc.transpose().template rightCols<3>() * force_ics_at_contact;

    double LambdaInv = 0; 
    double tmp = 0;

    // from tips to base
    while (i > 5) {
        tmp = F.dot(_S[i]);
        LambdaInv += tmp*tmp/_d[i];
        dstate_out.qdd+= _qdd_from_subqdd.col(i-6) * tmp / _d[i];

        // Apply force propagator (see Pat's ICRA 2012 paper)
        // essentially, since the joint is articulated, only a portion of the force
        // is felt on the predecessor. So, while Xup^T sends a force backwards as if 
        // the joint was locked, ChiUp^T sends the force backward as if the joint
        // were free
        F = _ChiUp[i].transpose()*F;
        i = _model._parents[i];
    }

    // TODO: Only carry out the QR once within update Aritculated Bodies
    dstate_out.dBodyVelocity = _invIA5.solve(F);
    LambdaInv+= F.dot(dstate_out.dBodyVelocity);
    dstate_out.qdd+=_qdd_from_base_accel*dstate_out.dBodyVelocity;

    return LambdaInv;  
}

/*!
 * Compute the inverse of the contact inertia matrix (mxm) 
 * @param force_ics_at_contact (3x1) 
 *        e.g. if you want the cartesian inv. contact inertia in the z_ics
 *             force_ics_at_contact = [0 0 1]^T
 * @return the 1x1 inverse contact inertia J H^{-1} J^T
 */
template <typename T>
double
DynamicsSimulator<T>:: invContactInertia(const int gc_index, const Vec3<T>& force_ics_at_contact) {
    forwardKinematics();
    updateArticulatedBodies();
    updateForcePropagators();

    size_t i_opsp = _model._gcParent.at(gc_index);
    size_t i = i_opsp;

    // Rotation to absolute coords
    Mat3<T> Rai = _Xa[i].template block<3,3>(0,0).transpose();
    Mat6<T> Xc = createSXform(Rai, _model._gcLocation.at(gc_index));

    // D is one column of an extended force propagator matrix (See Wensing, 2012 ICRA)
    SVec<T> F = Xc.transpose().template rightCols<3>() * force_ics_at_contact;

    double LambdaInv = 0; 
    double tmp = 0;

    // from tips to base
    while (i > 5) {
        tmp = F.dot(_S[i]);
        LambdaInv += tmp*tmp/_d[i];

        // Apply force propagator (see Pat's ICRA 2012 paper)
        // essentially, since the joint is articulated, only a portion of the force
        // is felt on the predecessor. So, while Xup^T sends a force backwards as if 
        // the joint was locked, ChiUp^T sends the force backward as if the joint
        // were free
        F = _ChiUp[i].transpose()*F;
        i = _model._parents[i];
    }
    LambdaInv+= F.dot(_invIA5.solve(F));
    return LambdaInv;  
}


/*!
 * Compute the inverse of the contact inertia matrix (mxm) 
 * @param force_directions (6xm) each column denotes a direction of interest 
 *        col = [ moment in i.c.s., force in i.c.s.]
 *        e.g. if you want the cartesian inv. contact inertia 
 *             force_directions = [ 0_{3x3} I_{3x3}]^T
 *             if you only want the cartesian inv. contact inertia in one direction
 *             then use the overloaded version.
 * @return the mxm inverse contact inertia J H^{-1} J^T
 */
template <typename T>
DMat<T>
DynamicsSimulator<T>::invContactInertia(const int gc_index, const D6Mat<T>& force_directions) {
    forwardKinematics();
    updateArticulatedBodies();
    updateForcePropagators();

    size_t i_opsp = _model._gcParent.at(gc_index);
    size_t i = i_opsp;

    // Rotation to absolute coords
    Mat3<T> Rai = _Xa[i].template block<3,3>(0,0).transpose();
    Mat6<T> Xc = createSXform(Rai, _model._gcLocation.at(gc_index));

    // D is a subslice of an extended force propagator matrix (See Wensing, 2012 ICRA)
    D6Mat<T> D = Xc.transpose() * force_directions;

    size_t m = force_directions.cols();

    DMat<T> LambdaInv = DMat<T>::Zero(m,m); 
    DVec<T> tmp = DVec<T>::Zero(m);

    // from tips to base
    while (i > 5) {
        tmp = D.transpose()*_S[i];
        LambdaInv += tmp*tmp.transpose()/_d[i];

        // Apply force propagator (see Pat's ICRA 2012 paper)
        // essentially, since the joint is articulated, only a portion of the force
        // is felt on the predecessor. So, while Xup^T sends a force backwards as if 
        // the joint was locked, ChiUp^T sends the force backward as if the joint
        // were free
        D = _ChiUp[i].transpose()*D;
        i = _model._parents[i];
    }

    // TODO: Only carry out the QR once within update Aritculated Bodies
    LambdaInv+= D.transpose()*_invIA5.solve(D);

    return LambdaInv;
}


/*!
 * Articulated Body Algorithm, modified by Pat to add rotors
 */
template <typename T>
void DynamicsSimulator<T>::runABA(const DVec<T> &tau) {
    (void)tau;

    forwardKinematics();
    updateArticulatedBodies();

    // create spatial vector for gravity
    SVec<T> aGravity;
    aGravity << 0, 0, 0, _model._gravity[0], _model._gravity[1], _model._gravity[2];


    // float-base articulated inertia
    SVec<T> ivProduct = _model._Ibody[5].getMatrix() * _v[5];
    _pA[5] = forceCrossProduct(_v[5], ivProduct);

    // loop 1, down the tree
    for(size_t i = 6; i < _nb; i++) {
        ivProduct = _model._Ibody[i].getMatrix() * _v[i];
        _pA[i] = forceCrossProduct(_v[i], ivProduct);

        // same for rotors
        SVec<T> vJrot = _Srot[i] * _state.qd[i - 6];
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
        _u[i] = tau[i - 6] - _S[i].transpose() * _pA[i] - _Srot[i].transpose() * _pArot[i] - _U[i].transpose()*_c[i] - _Urot[i].transpose() * _crot[i];

        // articulated inertia recursion
        SVec<T> pa = _Xup[i].transpose() * (_pA[i] + _IA[i] * _c[i]) + _Xuprot[i].transpose() * (_pArot[i] + _model._Irot[i].getMatrix() * _crot[i]) + _Utot[i] * _u[i] / _d[i];
        _pA[_model._parents[i]] += pa;
    }

    // include gravity and compute acceleration of floating base
    SVec<T> a0 = -aGravity;
    SVec<T> ub = -_pA[5];
    _a[5] = _Xup[5] * a0;
    SVec<T> afb = _invIA5.solve(ub - _IA[5].transpose() * _a[5]);
    _a[5] += afb;

    // joint accelerations
    _dstate.qdd = DVec<T>(_nb - 6);
    for(size_t i = 6; i < _nb; i++) {
        _dstate.qdd[i - 6] = (_u[i] - _Utot[i].transpose() * _a[_model._parents[i]]) / _d[i];
        _a[i] = _Xup[i] * _a[_model._parents[i]] + _S[i] * _dstate.qdd[i-6] + _c[i];
    }

    // output
    RotMat<T> Rup = rotationFromSXform(_Xup[5]);
    _dstate.dBodyPosition = Rup.transpose() * _state.bodyVelocity.template block<3,1>(3,0);
    _dstate.dBodyVelocity = afb;
    // qdd is set in the for loop above
}

template class DynamicsSimulator<double>;
template class DynamicsSimulator<float>;
