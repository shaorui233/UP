/*! @file test_dynamics.cpp
 *  @brief Test dynamics algorithms
 *
 * Test the model of Mini Cheetah
 */


#include "FloatingBaseModel.h"
#include "Quadruped.h"
#include "utilities.h"
#include "MiniCheetah.h"
#include "DynamicsSimulator.h"

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