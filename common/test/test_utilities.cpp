/*! @file test_utilities.cpp
 *  @brief Test Utilities functions
 *
 * Test the various utilities
 */

#include "cppTypes.h"
#include "Utilities/utilities.h"

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <string>
#include <unordered_map>

/*!
 * Test the coerce function
 */
TEST(Utilities, coerce) {
  EXPECT_EQ(0.1f, coerce<float>(0.5f, 0.f, 0.1f));
  EXPECT_EQ(-0.1f, coerce<float>(-0.5f, -0.1f, 0.1f));
  EXPECT_EQ(0.5f, coerce<float>(0.5f, -1.f, 1.f));
}


TEST(Utilities, sgn) {
  EXPECT_EQ(1, sgn<int>(10));
  EXPECT_EQ(-1, sgn<int>(-10));
  EXPECT_EQ(0, sgn<int>(0));

  EXPECT_EQ(1, sgn<double>(13.23));
  EXPECT_EQ(-1, sgn<double>(-.23));
  EXPECT_EQ(0, sgn<double>(0.));
}

TEST(Utilities, uMapContains) {
  std::unordered_map<std::string, int> s;
  EXPECT_FALSE(uMapContains(s, std::string("test")));
  s["test"] = 2;
  EXPECT_TRUE(uMapContains(s, std::string("test")));
}

TEST(Utilities, mapContains) {
  std::map<std::string, int> s;
  EXPECT_FALSE(mapContains(s, std::string("test")));
  s["test"] = 2;
  EXPECT_TRUE(mapContains(s, std::string("test")));
}

TEST(Utilities, mapToRange) {
  EXPECT_TRUE(fpEqual(2.0, mapToRange(0.4, 0.2, 1.2, 0.0, 10.0), .00001));
}