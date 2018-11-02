//
// Created by jared on 10/12/18.
//

#include <FloatingBaseModel.h>
#include <Quadruped.h>
#include <utilities.h>
#include "DynamicsSimulator.h"
#include <Cheetah3.h>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

using namespace spatial;


TEST(Dynamics, cheetah3model) {
  FloatingBaseModel<double> cheetah = buildCheetah3<double>();
  cheetah.check();

  // check total mass
  EXPECT_TRUE(fpEqual(41.0737, cheetah.totalNonRotorMass(), .0001));
  EXPECT_TRUE(fpEqual(6.3215, cheetah.totalRotorMass(), .0001));

  // check tree structure
  std::vector<int> parentRef{0, 0, 0, 0, 0, 0, 5, 6, 7, 5, 9, 10, 5, 12, 13, 5, 15, 16};
  EXPECT_TRUE(vectorEqual(parentRef, cheetah.getParentVector()));

  // this is kind of stupid, but a reasonable sanity check for inertias
  Mat6<double> inertiaSumRef;
  inertiaSumRef << 0.4352, -0.0000, -0.0044, 0, 0.6230, -0.0000,
          -0.0000, 1.0730, 0.0000, -0.6230, 0, -0.0822,
          -0.0044, 0.0000, 1.0071, 0.0000, 0.0822, 0,
          0, -0.6230, 0.0000, 42.6540, 0, 0,
          0.6230, 0, 0.0822, 0, 42.6540, 0,
          -0.0000, -0.0822, 0, 0, 0, 42.6540;

  Mat6<double> inertiaSum = Mat6<double>::Zero();

  for (size_t i = 0; i < 18; i++) {
    inertiaSum += cheetah.getBodyInertiaVector()[i].getMatrix() + cheetah.getRotorInertiaVector()[i].getMatrix() * 0.25;
  }
  EXPECT_TRUE(almostEqual(inertiaSum, inertiaSumRef, .0001));
}

// this test just checks that sim.runABA doesn't cause any matrix dimension errors.
TEST(Dynamics, simulatorDynamicsDoesntCrash) {
  FloatingBaseModel<double> cheetah = buildCheetah3<double>();
  DynamicsSimulator<double> sim(cheetah);
  DVec<double> tau(12);
  sim.forwardKinematics();
  sim.runABA(tau);
}

TEST(Dynamics, simulatorDynamicsABANoExternalForce) {
  FloatingBaseModel<double> cheetahModel = buildCheetah3<double>();
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

  // do aba
  sim.setState(x);
  sim.forwardKinematics();
  sim.runABA(tau);

  // check:
  Quat<double> quatDRef(0.2637, 0.5136, 1.0345, 1.4479);
  Vec3<double> pdRef(4.3717, 4.8598, 5.8541);
  SVec<double> vbdRef;
  vbdRef << 455.5224, 62.1684, -70.56, -11.4505, 10.2354, -15.2394;

  DVec<double> qddRef(12);
  qddRef << -0.7924, -0.2205, -0.9163, -1.6136, -0.4328, -1.6911, -2.9878, -0.9358, -2.6194, -3.3773, -1.3235, -3.1598;
  qddRef *= 1000;

  EXPECT_TRUE(almostEqual(quatDRef, sim.getDState().dQuat, .001));
  EXPECT_TRUE(almostEqual(pdRef, sim.getDState().dBasePosition, .001));
  EXPECT_TRUE(almostEqual(vbdRef, sim.getDState().dBaseVelocity, .001));

  for (size_t i = 0; i < 12; i++) {
    // the qdd's are large - see qddRef, so we're only accurate to within ~1.
    EXPECT_TRUE(fpEqual(sim.getDState().qdd[i], qddRef[i], 3.));
  }
}

TEST(Dynamics, simulatorDynamicsWithExternalForce) {
  FloatingBaseModel<double> cheetahModel = buildCheetah3<double>();
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
  ForceList<double> forces(18);
  for(size_t i = 0; i < 18; i++) {
    for(size_t j = 0; j < 6; j++) {
      forces[i][j] = i + j + 1;
    }
  }

  // do aba
  sim.setState(x);
  sim.setAllExternalForces(forces);
  sim.step(0.0, tau);

  // check:
  Quat<double> quatDRef(0.2637, 0.5136, 1.0345, 1.4479);
  Vec3<double> pdRef(4.3717, 4.8598, 5.8541);
  SVec<double> vbdRef;
  vbdRef << 806.6664,44.1266,33.8287,-10.1360,2.1066,12.1677;

  DVec<double> qddRef(12);
  qddRef <<  -0.5630, -0.1559,-1.0182,-0.9012,-0.3585,-1.6170,-2.0995,-0.8959,-2.7325,-2.0808,-1.3134,-3.1206;
  qddRef *= 1000;

  EXPECT_TRUE(almostEqual(quatDRef, sim.getDState().dQuat, .001));
  EXPECT_TRUE(almostEqual(pdRef, sim.getDState().dBasePosition, .001));
  EXPECT_TRUE(almostEqual(vbdRef, sim.getDState().dBaseVelocity, .001));

  for (size_t i = 0; i < 12; i++) {
    // the qdd's are large - see qddRef, so we're only accurate to within ~1.
    EXPECT_TRUE(fpEqual(sim.getDState().qdd[i], qddRef[i], 3.));
  }
}

TEST(Dynamics, simulatorFootPosVel) {
  FloatingBaseModel<double> cheetahModel = buildCheetah3<double>();
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
  ForceList<double> forces(18);
  for(size_t i = 0; i < 18; i++) {
    for(size_t j = 0; j < 6; j++) {
      forces[i][j] = i + j + 1;
    }
  }

  // fwd kin is included in this
  sim.setState(x);
  sim.setAllExternalForces(forces);
  sim.step(0.0, tau);

  Vec3<double> bodypRef1ML(6.25, 6.8271, 8.155);
  Vec3<double> footpRefML(5.1594, 7.3559, 7.674);
  Vec3<double> bodyvRef1ML(5.1989, 5.4008, 5.1234);
  Vec3<double> footvRefML(-9.3258, -0.1926, 26.3323);

  // I add the body points in a different order, so comparing them is kind of annoying.
  // this just tests one body point and one foot point.
  EXPECT_TRUE(almostEqual(bodypRef1ML, sim._pGC.at(2), .0005));
  EXPECT_TRUE(almostEqual(footpRefML, sim._pGC.at(15), .0005));
  EXPECT_TRUE(almostEqual(bodyvRef1ML, sim._vGC.at(2), .0005));
  EXPECT_TRUE(almostEqual(footvRefML, sim._vGC.at(15), .0005));
}

//TEST(Dynamics, simulatorTime) {
//  FloatingBaseModel<double> cheetahModel = buildCheetah3<double>();
//  DynamicsSimulator<double> sim(cheetahModel);
//
//  RotMat<double> rBody = coordinateRotation(CoordinateAxis::X, .123) * coordinateRotation(CoordinateAxis::Z, .232) *
//                         coordinateRotation(CoordinateAxis::Y, .111);
//  SVec<double> bodyVel;
//  bodyVel << 1, 2, 3, 4, 5, 6;
//  FBModelState<double> x;
//  DVec<double> q(12);
//  DVec<double> dq(12);
//  DVec<double> tau(12);
//  for (size_t i = 0; i < 12; i++) {
//    q[i] = i + 1;
//    dq[i] = (i + 1) * 2;
//    tau[i] = (i + 1) * -30.;
//  }
//
//  // set state
//  x.bodyOrientation = rotationMatrixToQuaternion(rBody);
//  x.bodyVelocity = bodyVel;
//  x.bodyPosition = Vec3<double>(6, 7, 8);
//  x.q = q;
//  x.qd = dq;
//
//  // generate external forces
//  ForceList<double> forces(18);
//  for(size_t i = 0; i < 18; i++) {
//    for(size_t j = 0; j < 6; j++) {
//      forces[i][j] = i + j + 1;
//    }
//  }
//
//  // do aba
//  sim.setState(x);
//  sim.setAllExternalForces(forces);
//
//  for(int i = 0; i < 10000; i++) {
//    sim.step(.0001, tau);
//  }
//
//  std::cout << sim.getState().bodyPosition << "\n";
//}