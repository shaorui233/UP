//
// Created by jared on 10/12/18.
//

#include <FloatingBaseModel.h>
#include <Quadruped.h>
#include <Cheetah3.h>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

using namespace spatial;




TEST(Dynamics, dynamics) {
  FloatingBaseModel<double> cheetah = buildCheetah3<double>();
}