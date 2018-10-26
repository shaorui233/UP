//
// Created by jared on 10/12/18.
//

#include <FloatingBaseModel.h>
#include <Quadruped.h>
#include <utilities.h>
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

  for(size_t i = 0; i < 18; i++) {
    inertiaSum += cheetah.getBodyInertiaVector()[i].getMatrix() + cheetah.getRotorInertiaVector()[i].getMatrix()  * 0.25;
  }
  EXPECT_TRUE(almostEqual(inertiaSum, inertiaSumRef, .0001));

}