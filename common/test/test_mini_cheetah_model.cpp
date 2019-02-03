/*! @file test_dynamics.cpp
 *  @brief Test dynamics algorithms
 *
 * Test the model of Mini Cheetah
 */


#include "Dynamics/FloatingBaseModel.h"
#include "Dynamics/Quadruped.h"
#include "Utilities/utilities.h"
#include "Dynamics/MiniCheetah.h"
#include "Dynamics/DynamicsSimulator.h"

#include <iostream>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

using namespace spatial;

/*!
 * Test the total mass, tree, and spatial inertia sum
 */
TEST(MiniCheetah, miniCheetahModel1) {
  FloatingBaseModel<double> cheetah = buildMiniCheetah<double>().buildModel();

  // masses
  EXPECT_TRUE(fpEqual(8.2520, cheetah.totalNonRotorMass(), .0001));
  EXPECT_TRUE(fpEqual(0.66, cheetah.totalRotorMass(), .0001));

  // check tree structure
  std::vector<int> parentRef{0, 0, 0, 0, 0, 0, 5, 6, 7, 5, 9, 10, 5, 12, 13, 5, 15, 16};
  EXPECT_TRUE(vectorEqual(parentRef, cheetah.getParentVector()));

  // this is kind of stupid, but a reasonable sanity check for inertias
  Mat6<double> inertiaSumRef;
  inertiaSumRef << 0.0272, 0, 0.0001, 0, 0.0663, 0,
          0, 0.3758, 0.0000, -0.0663, 0, 0,
          0.0001, 0.0000, 0.0497, 0, 0, 0,
          0, -0.0663, 0, 8.4170, 0, 0,
          0.0663, 0, 0, 0, 8.4170, 0,
          0, 0, 0, 0, 0, 8.4170;

  Mat6<double> inertiaSum = Mat6<double>::Zero();

  for (size_t i = 0; i < 18; i++) {
    inertiaSum += cheetah.getBodyInertiaVector()[i].getMatrix() + cheetah.getRotorInertiaVector()[i].getMatrix() * 0.25;
  }

  EXPECT_TRUE(almostEqual(inertiaSum, inertiaSumRef, .0003));
}

/*!
 * Test the model transforms
 */
TEST(MiniCheetah, miniCheetahModel2) {
  FloatingBaseModel<double> cheetah = buildMiniCheetah<double>().buildModel();
  Mat6<double> XTotal = Mat6<double>::Identity();
  Mat6<double> XRotTotal = Mat6<double>::Identity();
  for (size_t i = 0; i < 18; i++) {
    XTotal = XTotal + cheetah._Xtree[i];
    XRotTotal = XRotTotal + cheetah._Xrot[i];
  }
  Mat6<double> Xtr, Xrtr;
  Xtr << 11.0000, 0.0000, 0, 0, 0, 0,
          -0.0000, 11.0000, 0, 0, 0, 0,
          0, 0, 19.0000, 0, 0, 0,
          0, -0.8360, 0, 11.0000, 0.0000, 0,
          0.8360, 0, 0.0000, -0.0000, 11.0000, 0,
          0, 0, 0, 0, 0, 19.0000;

  Xrtr << 11.0000, 0.0000, 0, 0, 0, 0,
          -0.0000, 11.0000, 0, 0, 0, 0,
          0, 0, 19.0000, 0, 0, 0,
          0, 0, 0, 11.0000, 0.0000, 0,
          0, 0, 0.0000, -0.0000, 11.0000, 0,
          0.0000, 0, 0, 0, 0, 19.0000;

  EXPECT_TRUE(almostEqual(Xtr, XTotal, .0005));
  EXPECT_TRUE(almostEqual(Xrtr, XRotTotal, .0005));
}

/*!
 * Creates a cheetah model and runs forward kinematics and ABA
 * Doesn't test anyting - this is just to make sure it doesn't crash
 */
TEST(MiniCheetah, simulatorDynamicsDoesntCrashMiniCheetah) {
  FloatingBaseModel<double> cheetah = buildMiniCheetah<double>().buildModel();
  DynamicsSimulator<double> sim(cheetah);
  DVec<double> tau(12);
  sim.forwardKinematics();
  sim.runABA(tau);
}

/*!
 * Run the articulated body algorithm (and forward kinematics) on Cheetah 3
 * Set a weird body orientation, velocity, q, dq, and tau
 * Checks that quatD, pd, vd, and qdd match MATLAB
 */
TEST(MiniCheetah, simulatorDynamicsABANoExternalForceMiniCheetah) {
  FloatingBaseModel<double> cheetahModel = buildMiniCheetah<double>().buildModel();
  DynamicsSimulator<double> sim(cheetahModel);

  RotMat<double> rBody = coordinateRotation(CoordinateAxis::X, .123) * coordinateRotation(CoordinateAxis::Z, .232) *
                         coordinateRotation(CoordinateAxis::Y, .111);
  SVec<double> bodyVel;
  bodyVel << 1, 2, 3, 4, 5, 6;
  FBModelState<double> x;
  DVec<double> q(12);
  DVec<double> dq(12);
  DVec<double> tau(12);
  for (size_t i = 0; i < 12; i++) {
    q[i] = i + 1;
    dq[i] = (i + 1) * 2;
    tau[i] = (i + 1) * -3.;
  }

  // set state
  x.bodyOrientation = rotationMatrixToQuaternion(rBody.transpose());
  x.bodyVelocity = bodyVel;
  x.bodyPosition = Vec3<double>(6, 7, 8);
  x.q = q;
  x.qd = dq;

  // do aba
  sim.setState(x);
  sim.forwardKinematics();
  sim.runABA(tau);

  // check:
  Vec3<double> pdRef(4.3717, 4.8598, 5.8541);
  SVec<double> vbdRef;
  vbdRef << 1.2926, -0.068, -0.1488, -.0024, 0.0331, -0.0587;
  vbdRef = vbdRef * 1e3;

  DVec<double> qddRef(12);
  qddRef << -2.1941,
          -0.9794,
          -1.4734,
          -3.2994,
          -1.3090,
          -2.6362,
          -6.5224,
          -4.0238,
          -4.2126,
          -7.3662,
          -3.5136,
          -5.1148;
  qddRef *= 1000;

  EXPECT_TRUE(almostEqual(pdRef, sim.getDState().dBodyPosition, .001));
  EXPECT_TRUE(almostEqual(vbdRef, sim.getDState().dBodyVelocity, 1));

  for (size_t i = 0; i < 12; i++) {
    // the qdd's are large - see qddRef, so we're only accurate to within ~1.
    EXPECT_TRUE(fpEqual(sim.getDState().qdd[i], qddRef[i], 3.));
  }
}

/*!
 * Run the articulated body algorithm (and forward kinematics) on Cheetah 3
 * Set a weird body orientation, velocity, q, dq, and tau
 * Sets external spatial forces on all bodies
 * Checks that quatD, pd, vd, and qdd match MATLAB
 */
TEST(MiniCheetah, simulatorDynamicsWithExternalForceMiniCheetah) {
  FloatingBaseModel<double> cheetahModel = buildMiniCheetah<double>().buildModel();
  DynamicsSimulator<double> sim(cheetahModel);

  RotMat<double> rBody = coordinateRotation(CoordinateAxis::X, .123) * coordinateRotation(CoordinateAxis::Z, .232) *
                         coordinateRotation(CoordinateAxis::Y, .111);
  SVec<double> bodyVel;
  bodyVel << 1, 2, 3, 4, 5, 6;
  FBModelState<double> x;
  DVec<double> q(12);
  DVec<double> dq(12);
  DVec<double> tau(12);
  for (size_t i = 0; i < 12; i++) {
    q[i] = i + 1;
    dq[i] = (i + 1) * 2;
    tau[i] = (i + 1) * -3.;
  }

  // set state
  x.bodyOrientation = rotationMatrixToQuaternion(rBody.transpose());
  x.bodyVelocity = bodyVel;
  x.bodyPosition = Vec3<double>(6, 7, 8);
  x.q = q;
  x.qd = dq;

  // generate external forces
  vectorAligned<SVec<double>> forces(18);
  for (size_t i = 0; i < 18; i++) {
    for (size_t j = 0; j < 6; j++) {
      forces[i][j] = .3 * (i + j + 1);
    }
  }

  // do aba
  sim.setState(x);
  sim.setAllExternalForces(forces);
  sim.step(0.0, tau, 5e5, 5e3);

  // check:
  Vec3<double> pdRef(4.3717, 4.8598, 5.8541);
  SVec<double> vbdRef;
  vbdRef << 3.2350,
          0.0153,
          0.2561,
          -0.0041,
          -0.0539,
          0.0691;
  vbdRef = vbdRef * 1e3;

  DVec<double> qddRef(12);
  qddRef << -1.2379,
          -1.1817,
          -2.0258,
          0.4152,
          -0.2857,
          -2.2896,
          -1.8900,
          -4.0431,
          -4.9216,
          0.0840,
          -2.4799,
          -4.9679;
  qddRef *= 1000;

  EXPECT_TRUE(almostEqual(pdRef, sim.getDState().dBodyPosition, .001));
  EXPECT_TRUE(almostEqual(vbdRef, sim.getDState().dBodyVelocity, 1));

  for (size_t i = 0; i < 12; i++) {
    // the qdd's are large - see qddRef, so we're only accurate to within ~1.
    EXPECT_TRUE(fpEqual(sim.getDState().qdd[i], qddRef[i], 3.));
  }
}

/*!
 * Run the articulated body algorithm (and forward kinematics) on Cheetah 3
 * Set a weird body orientation, velocity, q, dq, and tau
 * Checks that foot position and velocities match MATLAB
 */
TEST(MiniCheetah, simulatorFootPosVelMiniCheetah) {
  FloatingBaseModel<double> cheetahModel = buildMiniCheetah<double>().buildModel();
  DynamicsSimulator<double> sim(cheetahModel);

  RotMat<double> rBody = coordinateRotation(CoordinateAxis::X, .123) * coordinateRotation(CoordinateAxis::Z, .232) *
                         coordinateRotation(CoordinateAxis::Y, .111);
  SVec<double> bodyVel;
  bodyVel << 1, 2, 3, 4, 5, 6;
  FBModelState<double> x;
  DVec<double> q(12);
  DVec<double> dq(12);
  DVec<double> tau(12);
  for (size_t i = 0; i < 12; i++) {
    q[i] = i + 1;
    dq[i] = (i + 1) * 2;
    tau[i] = (i + 1) * -30.;
  }

  // set state
  x.bodyOrientation = rotationMatrixToQuaternion(rBody.transpose());
  x.bodyVelocity = bodyVel;
  x.bodyPosition = Vec3<double>(6, 7, 8);
  x.q = q;
  x.qd = dq;

  // generate external forces
  vectorAligned<SVec<double>> forces(18);
  for(size_t i = 0; i < 18; i++) {
    for(size_t j = 0; j < 6; j++) {
      forces[i][j] = i + j + 1;
    }
  }

  // fwd kin is included in this
  sim.setState(x);
  sim.setAllExternalForces(forces);
  sim.step(0.0, tau, 5e5, 5e3);

  Vec3<double> footpRefML(5.4937, 7.1459, 7.8096);
  Vec3<double> footvRefML(-2.5284, 2.0944, 16.3732);

  // I add the body points in a different order, so comparing them is kind of annoying.
  // this just one foot point.
  EXPECT_TRUE(almostEqual(footpRefML, sim.getModel()._pGC.at(15), .0005));
  EXPECT_TRUE(almostEqual(footvRefML, sim.getModel()._vGC.at(15), .0005));
}
