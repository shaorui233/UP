/*! @file test_utilities.cpp
 *  @brief Test Utilities functions
 *
 * Test the various utilities
 */

#include "cppTypes.h"
#include "utilities.h"

#include "gtest/gtest.h"
#include "gmock/gmock.h"

/*!
 * Test the coerce function
 */
TEST(Utilities, coerce) {
  EXPECT_EQ(0.1f, coerce<float>(0.5f, 0.f, 0.1f));
  EXPECT_EQ(-0.1f, coerce<float>(-0.5f, -0.1f, 0.1f));
  EXPECT_EQ(0.5f, coerce<float>(0.5f, -1.f, 1.f));
}
