
#include <orientation_tools.h>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

using namespace ori;

TEST(Orientation, rad2deg) {
  // check radians to degrees calculation
  EXPECT_EQ(M_PI/4., deg2rad(45.));
  EXPECT_EQ(45, rad2deg(M_PI/4.));
}

TEST(Orientation, almostEqual) {
  // check matrix "almostEqual" function
  Mat3<double> a, b;
  a << 1,2,3,4,5,6,7,8,9.2;
  b << 1,2,3,4,5,6,7,8,9.1;
  EXPECT_EQ(true, almostEqual(a,b,.3));
  EXPECT_EQ(false, almostEqual(a,b,.01));
}

TEST(Orientation, coordinateRotation) {
  // check rotation matrices
  double s = std::sin(.2);
  double c = std::cos(.2);
  Mat3<double> R_ref_x, R_ref_y, R_ref_z;
  R_ref_x << 1, 0, 0,
             0, c, s,
             0, -s, c;
  R_ref_y  << c, 0, -s,
               0, 1, 0,
               s, 0, c;
  R_ref_z << c, s, 0,
             -s, c, 0,
             0, 0, 1;
  EXPECT_EQ(R_ref_x, coordinateRotation(CoordinateAxis::X, .2));
  EXPECT_EQ(R_ref_y, coordinateRotation(CoordinateAxis::Y, .2));
  EXPECT_EQ(R_ref_z, coordinateRotation(CoordinateAxis::Z, .2));
}


TEST(Orientation, skew) {
  // check skew vec->mat and mat->vec
  Mat3<double> A,B;
  A << 8,1,6,3,5,7,4,9,2;
  B << 0, -3, 2, 3, 0, -1, -2, 1, 0;
  Vec3<double> v(1,1,1);
  Vec3<double> w(1,2,3);

  EXPECT_EQ(matToSkewVec(A), v);
  EXPECT_EQ(B, vectorToSkewMat(w));
  EXPECT_EQ(w, matToSkewVec(vectorToSkewMat(w)));
  EXPECT_EQ(v, matToSkewVec(vectorToSkewMat(v)));
}

TEST(Orientation, quatToRotm) {
  // check R -> q and q -> R
  Quat<double> q(.9672, -0.0672, -0.1653, -0.1808);
  Mat3<double> R;
  R << .8799, .3720, -.2955, -.3276, .9256, .1898, .3441, -.0702, .9363;
  Quat<double> q2 = rotationMatrixToQuaternion(R);
  Mat3<double> R2 = quaternionToRotationMatrix(q);
  EXPECT_TRUE(almostEqual(q2, q, .0001));
  EXPECT_TRUE(almostEqual(R2, R, .0001));
}

TEST(Orientation, quatToRpy) {
  // check q -> rpy
  Quat<double> q(0.9672, -0.0672, -0.1653, -0.1808);
  Vec3<double> rpy1(-0.0748, -0.3513, -0.3564);
  EXPECT_TRUE(almostEqual(rpy1, quatToRPY(q), .1));
}

TEST(Orienation, quaternionDerivative) {
  Quat<double> ref(-10.8376, -0.6752, -2.5128, -1.3504);
  Quat<double> q = quatDerivative(Quat<double>(1,2,3,4), Vec3<double>(1,2,3));
  EXPECT_TRUE(almostEqual(q,ref,.0005));
}