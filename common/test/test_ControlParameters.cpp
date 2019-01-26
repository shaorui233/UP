/*! @file test_ControlParameters.cpp
 *  @brief
 *
 */

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "ControlParameters.h"
#include "SimulatorParameters.h"

class TestControlParameters : public ControlParameters {
public:
  TestControlParameters() :
  ControlParameters("test-parameters"),
  test_double_param("test_double", test_double, collection),
  test_float_param("test_float", test_float, collection),
  test_integer_param("test_integer", test_integer, collection){ }

  double test_double;
  ControlParameter test_double_param;

  float test_float;
  ControlParameter test_float_param;

  s64 test_integer;
  ControlParameter test_integer_param;
};

TEST(ControlParams, testSimple) {
  TestControlParameters settings;

  EXPECT_FALSE(settings.isFullyInitialized());

  EXPECT_THROW(settings.initializeInteger("test_double", 1), std::runtime_error);
  EXPECT_THROW(settings.initializeDouble("not-a-real-thing", 1.2), std::runtime_error);

  settings.initializeDouble("test_double", 1.2);

  EXPECT_FALSE(settings.isFullyInitialized());
  EXPECT_TRUE(1.2 == *settings.collection.lookup("test_double")._value.d);
  EXPECT_TRUE(ControlParameterValueKind::DOUBLE == settings.collection.lookup("test_double")._kind);
  EXPECT_TRUE("test_double" == settings.collection.lookup("test_double")._name);
  EXPECT_TRUE(settings.collection.lookup("test_double")._set);

  settings.initializeFloat("test_float", 1);
  EXPECT_FALSE(settings.isFullyInitialized());
  EXPECT_TRUE(1 == *settings.collection.lookup("test_float")._value.f);
  EXPECT_TRUE(ControlParameterValueKind::FLOAT == settings.collection.lookup("test_float")._kind);
  EXPECT_TRUE("test_float" == settings.collection.lookup("test_float")._name);
  EXPECT_TRUE(settings.collection.lookup("test_float")._set);

  settings.initializeInteger("test_integer", 8);
  EXPECT_TRUE(settings.isFullyInitialized());
  EXPECT_TRUE(8 == *settings.collection.lookup("test_integer")._value.i);
  EXPECT_TRUE(ControlParameterValueKind::S64 == settings.collection.lookup("test_integer")._kind);
  EXPECT_TRUE("test_integer" == settings.collection.lookup("test_integer")._name);
  EXPECT_TRUE(settings.collection.lookup("test_integer")._set);
}

TEST(ControlParams, testIni) {

  // create and initialize some parameters
  TestControlParameters settings;
  EXPECT_FALSE(settings.isFullyInitialized());
  settings.initializeDouble("test_double", 2e-9);
  settings.initializeFloat("test_float", 1);
  settings.initializeInteger("test_integer", 8);
  EXPECT_TRUE(settings.isFullyInitialized());

  // write to ini:
  settings.writeToIniFile("control-params-test-file.ini");

  // read from ini
  TestControlParameters settingsFromIni;
  EXPECT_FALSE(settingsFromIni.isFullyInitialized());
  settingsFromIni.initializeFromIniFile("control-params-test-file.ini");
  printf("%s\n", settingsFromIni.generateUnitializedList().c_str());
  EXPECT_TRUE(settingsFromIni.isFullyInitialized());

  EXPECT_TRUE(fpEqual(settings.test_double, settingsFromIni.test_double, 1e-10));
  EXPECT_TRUE(fpEqual(settings.test_float,  settingsFromIni.test_float, 1e-5f));
  EXPECT_TRUE(settings.test_integer == settingsFromIni.test_integer);
}

// check to see that the simulator default settings file contains all the simulator settings.
TEST(ControlParams, CheckSimulatorDefaults) {
  SimulatorControlParameters simParams;
  simParams.initializeFromIniFile(getConfigDirectoryPath() + SIMULATOR_DEFAULT_PARAMETERS);
  if(!simParams.isFullyInitialized()) {
    printf("Missing parameters:\n%s\n", simParams.generateUnitializedList().c_str());
  }
  EXPECT_TRUE(simParams.isFullyInitialized());
}