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

    _state.bodyVelocity = SVec<T>::Zero();
    _state.bodyPosition = Vec3<T>::Zero();
    _state.bodyOrientation = Quat<T>::Zero();
    _state.q = DVec<T>::Zero(_model._nDof - 6);
    _state.qd = DVec<T>::Zero(_model._nDof - 6);
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
    updateCollisions(dt);  // process collisions
    runABA(tau);         // dynamics algorithm
    integrate(dt);       // step forward
    
    _model.setState(_state);
    _model.resetExternalForces(); // clesetar external forces
    _model.resetCalculationFlags();
}

template <typename T>
void DynamicsSimulator<T>::updateCollisions(T dt) {
    _model.forwardKinematics(); 
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
}

template class DynamicsSimulator<double>;
template class DynamicsSimulator<float>;
