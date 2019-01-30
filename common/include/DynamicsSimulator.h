/*! @file DynamicsSimulator.h
 *  @brief Rigid Body Dynamics Simulator with Collisions
 *
 *  Combines ABA, Collisions, integrator, and any other external forces to run a simulation.
 *  Doesn't do any graphics.
 */

#ifndef PROJECT_DYNAMICSSIMULATOR_H
#define PROJECT_DYNAMICSSIMULATOR_H

#include "orientation_tools.h"
#include "FloatingBaseModel.h"
#include "spatial.h"
#include "CollisionPlane.h"
#include "CollisionBox.h"
#include "ContactConstraint.h"

using namespace ori;
using namespace spatial;
#include <eigen3/Eigen/Dense>



/*!
 * Class (containing state) for dynamics simulation of a floating-base system
 */
template<typename T>
class DynamicsSimulator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DynamicsSimulator(FloatingBaseModel<T>& model); //! Initialize simulator with given model
  void step(T dt, const DVec<T>& tau); //! Simulate forward one step
  void runABA(const DVec<T>& tau); //! Find _dstate with the articulated body algorithm
  void forwardKinematics(); //! Do forward kinematics for feet
  void integrate(T dt); //! Integrate to find new _state


  DMat<T> invContactInertia(const int gc_index, const D6Mat<T>& force_directions);
  double invContactInertia(const int gc_index, const Vec3<T>& force_ics_at_contact); 
  double applyTestForce(const int gc_index, const Vec3<T>& force_ics_at_contact, FBModelStateDerivative<T> & dstate_out);

  /*!
   * Set the state of the robot being simulated
   */
  void setState(const FBModelState<T>& state) {
    _state = state;
    _articulatedBodiesUpToDate = false;
    _kinematicsUpToDate = false;
    _forcePropagatorsUpToDate = false;
    _qddEffectsUpToDate = false;
    // DH: Update Model state 
    _model._state = state;
  }

  void resetCalculationFlags() {
    _articulatedBodiesUpToDate = false;
    _kinematicsUpToDate = false;
    _forcePropagatorsUpToDate = false;
    _qddEffectsUpToDate = false;
  }

  /*!
   * Get the state of the robot
   * @return
   */
  const FBModelState<T>& getState() const {
    return _state;
  }

  /*!
   * Get the most recently calculated state derivative (updated by runABA)
   * @return
   */
  const FBModelStateDerivative<T>& getDState() const {
    return _dstate;
  }

  /*!
   * Add external forces. These are added on top of the ground reaction forces
   * The i-th force is the spatial force applied to body i.
   * This is cleared after each step of the simulator.
   */
  void setAllExternalForces(const vectorAligned<SVec<T>>& forces) {
    _externalForces = forces;
  }

  /*!
   * Reset any external forces
   */
  void resetExternalForces() {
    for(size_t i = 0; i < _nb; i++) {
      _externalForces[i] = SVec<T>::Zero();
    }
  }

  /*!
   * Add a collision plane. 
   */
  void addCollisionPlane(T mu, T rest, T height) {
    _contact_constr->AddCollision(new CollisionPlane<T>(mu, rest, height) );
  }

  void addCollisionBox(T mu, T rest, T depth, T width, T height,
          const Vec3<T> & pos, const Mat3<T> & ori) {
    _contact_constr->AddCollision(
            new CollisionBox<T>(mu, rest, depth, width, height, pos, ori) );
  }

  size_t getNumBodies() {
    return _nb;
  }

  vectorAligned<Vec3<T> > _pGC; // position of ground contacts in world coordinates
  vectorAligned<Vec3<T> > _vGC; // velocity of ground contacts in world coordinates
  vectorAligned<Vec3<T> > _fGC; // reation force of ground contacts in world coordinates
  vectorAligned<Mat6<T> > _Xup, _Xuprot, _IA, _Xa, _ChiUp;

  const size_t & getTotalNumGC(){ return _model._nGroundContact; }
  const Vec3<T> & getContactForce(size_t idx){ 
      return _contact_constr->getGCForce(idx); 
  }

private:

  void updateCollisions(T dt); //! Update ground collision list
  void updateArticulatedBodies();
  void updateForcePropagators();
  void udpateQddEffects();


  FBModelState<T> _state;
  FBModelStateDerivative<T> _dstate;

  // aba stuff
  vectorAligned<SVec<T> > _v, _vrot, _a, _c, _crot, _U, _Urot, _Utot, _S, _Srot, _pA, _pArot;


  vector<T> _d, _u;
  DMat<T> _qdd_from_base_accel;
  DMat<T> _qdd_from_subqdd;
  Eigen::ColPivHouseholderQR<Mat6<T> > _invIA5;

  FloatingBaseModel<T>& _model;
  size_t _nb, _nGC;

  bool  _articulatedBodiesUpToDate = false;
  bool  _kinematicsUpToDate = false;
  bool  _forcePropagatorsUpToDate = false;
  bool  _qddEffectsUpToDate = false;

  vectorAligned<SVec<T>> _externalForces;
  vector<CollisionPlane<T>> _collisionPlanes;
  ContactConstraint<T> * _contact_constr;
};


#endif //PROJECT_DYNAMICSSIMULATOR_H
