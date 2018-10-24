//
// Created by jared on 10/12/18.
//

#include <orientation_tools.h>
#include <SpatialInertia.h>
#include <spatial.h>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

using namespace ori;
using namespace spatial;

TEST(Spatial, axisRotation) {
  SXform<double> X1, X2;
  X1 << 0.8384, 0.4580, -0.2955, 0, 0, 0,
          -0.4183, 0.8882, 0.1898, 0, 0, 0,
          0.3494, -0.0355, 0.9363, 0, 0, 0,
          0, 0, 0, 0.8384, 0.4580, -0.2955,
          0, 0, 0, -0.4183, 0.8882, 0.1898,
          0, 0, 0, 0.3494, -0.0355, 0.9363;
  X2 = spatialRotation(CoordinateAxis::X, .2) *
       spatialRotation(CoordinateAxis::Y, .3) *
       spatialRotation(CoordinateAxis::Z, .5);

  EXPECT_TRUE(almostEqual(X1, X2, .001));
}

TEST(Spatial, crm) {
  SVec<double> v1, v2, v3, v4, v5, v6;
  v1 << 1, 2, 3, 4, 5, 6;
  v2 << 6, 5, 4, 3, 2, 1;
  v3 = motionCrossMatrix(v1) * v2;
  v4 << -7, 14, -7, -14, 28, -14;
  v6 = motionCrossMatrix(v1) * v1;
  v5 << 0, 0, 0, 0, 0, 0;
  EXPECT_TRUE(almostEqual(v3, v4, .001));
  EXPECT_TRUE(almostEqual(v6, v5, .0001));
}

TEST(Spatial, crf) {
  SVec<double> v1, v2;
  v1 << 1, 2, 3, 4, 5, 6;
  v2 << 6, 5, 4, 3, 2, 1;
  Mat6<double> m1 = motionCrossMatrix(-v1);
  m1.transposeInPlace();
  Mat6<double> m2 = forceCrossMatrix(v1);
  EXPECT_TRUE(almostEqual(m1, m2, .0001));
}

TEST(Spatial, crm_prod) {
  SVec<double> v1, v2, v3, v4;
  v1 << 1, 2, 3, 4, 5, 6;
  v2 << 6, 5, 4, 3, 2, 1;
  v3 = motionCrossMatrix(v1) * v2;
  v4 = motionCrossProduct(v1, v2);
  EXPECT_TRUE(almostEqual(v3, v4, .0001));
}

TEST(Spatial, crf_prod) {
  SVec<double> v1, v2, v3, v4;
  v1 << 1, 2, 3, 4, 5, 6;
  v2 << 6, 5, 4, 3, 2, 1;
  v3 = forceCrossMatrix(v1) * v2;
  v4 = forceCrossProduct(v1, v2);
  EXPECT_TRUE(almostEqual(v3, v4, .0001));
}

TEST(Spatial, inertia) {
  Mat3<double> I;
  I << 1, 2, 3, 2, 4, 5, 3, 5, 6;
  Vec3<double> com(10, 11, 12);
  SpatialInertia<double> IS(42, com, I);
  Mat6<double> ref;
  Mat4<double> pref;
  pref << 4204.5, 4618, 5037, 420,
          4618, 5083.5, 5539, 462,
          5037, 5539, 6047.5, 504,
          420, 462, 504, 42;

  ref << 11131, -4618, -5037, 0, -504, 462,
          -4618, 10252, -5539, 504, 0, -420,
          -5037, -5539, 9288, -462, 420, 0,
          0, 504, -462, 42, 0, 0,
          -504, 0, 420, 0, 42, 0,
          462, -420, 0, 0, 0, 42;

  SpatialInertia<double> IS2(IS.getPseudoInertia());

  EXPECT_TRUE(almostEqual(ref, IS.getMatrix(), .001));
  EXPECT_EQ(42, IS.getMass());
  EXPECT_TRUE(almostEqual(I, IS.getInertiaTensor(), .00001));
  EXPECT_TRUE(almostEqual(com, IS.getCOM(), .00001));
  EXPECT_TRUE(almostEqual(IS.getPseudoInertia(), pref, .1));
  EXPECT_TRUE(almostEqual(IS.getMatrix(), IS2.getMatrix(), .00001));
}

TEST(Spatial, inertia_flips) {
  MassProperties<double> a, aref;
  a << 1,2,3,4,5,6,7,8,9,10;
  aref << 1,2,-3,4,5,6,7,-8,9,-10;
  EXPECT_TRUE(almostEqual(SpatialInertia<double>(aref).getMatrix(),
          SpatialInertia<double>(a).flipAlongAxis(CoordinateAxis::Y).getMatrix(), .0001));
}

TEST(Spatial, pluho_and_plux) {
  RotMat<double> R = coordinateRotation(CoordinateAxis::X, 1.0) * coordinateRotation(CoordinateAxis::Y, 2.0) *
          coordinateRotation(CoordinateAxis::Z, 3.0);
  Vec3<double> r(4,5,6);
  Mat6<double> X = createSXform(R, r);
  Mat6<double> Xref, X2;
  Mat4<double> H = sxformToHomogeneous(X);
  Mat4<double> Href;
  X2 = homogeneousToSXform(H);
  Xref << 0.4120, -0.0587, -0.9093, 0, 0, 0,
          -0.8337, -0.4269, -0.3502, 0, 0, 0,
          -0.3676, 0.9024, -0.2248, 0, 0, 0,
          -4.1941, 6.1091, -2.2948, 0.4120, -0.0587, -0.9093,
          0.8106, -3.6017, 2.4610, -0.8337, -0.4269, -0.3502,
          -6.5385, -1.3064, 5.4477, -0.3676, 0.9024, -0.2248;
  Href << 0.4120,   -0.0587,   -0.9093,    4.1015,
         -0.8337,   -0.4269,   -0.3502,    7.5706,
         -0.3676,    0.9024,   -0.2248 ,  -1.6923,
               0,         0,         0 ,   1.0000;
  EXPECT_TRUE(almostEqual(Xref, X, .001));
  EXPECT_TRUE(almostEqual(R, rotationFromSXform(X), .001));
  EXPECT_TRUE(almostEqual(r, translationFromSXform(X), .001));
  EXPECT_TRUE(almostEqual(H, Href, .001));
  EXPECT_TRUE(almostEqual(X, X2, .00001));
}

TEST(Spatial, jcalc) {
  Mat6<double> Xr, Xp, Xr_ref, Xp_ref;
  SVec<double> phi_r, phi_p, phi_r_ref, phi_p_ref;

  Xr = jointXform(JointType::Revolute, CoordinateAxis::Y, std::asin(.707));
  Xp = jointXform(JointType::Prismatic, CoordinateAxis::Z, .2);
  phi_r = jointMotionSubspace<double>(JointType::Revolute, CoordinateAxis::Y);
  phi_p = jointMotionSubspace<double>(JointType::Prismatic, CoordinateAxis::Z);

  Xr_ref << 0.7072 ,        0   ,-0.7070  ,       0  ,       0  ,       0,
               0  ,  1.0000  ,       0   ,      0  ,       0   ,      0,
          0.7070  ,       0 ,   0.7072  ,       0  ,       0  ,       0,
               0  ,       0  ,       0  ,  0.7072 ,        0 ,  -0.7070,
            0    ,     0     ,    0    ,     0   , 1.0000     ,    0,
              0   ,      0   ,      0  ,  0.7070  ,       0  ,  0.7072;

  phi_r_ref << 0,1,0,0,0,0;

  Xp_ref << 1.0000 ,        0  ,       0   ,      0   ,      0   ,      0,
               0  ,  1.0000  ,       0   ,      0 ,        0    ,     0,
               0 ,        0  ,  1.0000  ,       0   ,      0   ,      0,
               0  ,  0.2000   ,      0  ,  1.0000  ,       0   ,      0,
         -0.2000  ,       0  ,       0 ,        0  ,  1.0000  ,       0,
               0  ,       0   ,      0  ,       0  ,       0  ,  1.0000;

  phi_p_ref << 0,0,0,0,0,1;

  EXPECT_TRUE(almostEqual(Xr, Xr_ref, .001));
  EXPECT_TRUE(almostEqual(Xp, Xp_ref, .001));
  EXPECT_TRUE(almostEqual(phi_r, phi_r_ref, .000001));
  EXPECT_TRUE(almostEqual(phi_p, phi_p_ref, .000001));
}

TEST(Spatial, mass_properties) {
  MassProperties<double> a;
  a << 1,2,3,4,5,6,7,8,9,10;
  Mat6<double> I, I_ref;
  I_ref << 5,    10  ,   9  ,   0   , -4 ,    3,
          10 ,    6  ,   8  ,   4  ,   0 ,   -2,
           9 ,    8  ,   7  ,  -3  ,   2   ,  0,
           0 ,    4 ,   -3  ,   1   ,  0 ,    0,
          -4 ,    0   ,  2   ,  0  ,   1  ,   0,
           3 ,   -2   ,  0   ,  0   ,  0   ,  1;
  SpatialInertia<double> IS(a);
  EXPECT_TRUE(almostEqual(I_ref, IS.getMatrix(), .001));
  EXPECT_TRUE(almostEqual(a, IS.asMassPropertyVector(), .001));
}

TEST(Spatial, box_inertia) {
  Mat3<double> I_ref; I_ref << 2.0833, 0, 0, 0, 1.66667, 0, 0, 0, 1.083333;
  Mat3<double> I_calc = rotInertiaOfBox(1., Vec3<double>(2,3,4));
  EXPECT_TRUE(almostEqual(I_ref, I_calc, .001));
}