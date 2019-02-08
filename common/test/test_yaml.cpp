#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "ParamHandler.hpp"
#include "Utilities/utilities.h"


TEST(YAML, yaml_category_read) {
  ParamHandler paramHandler(getConfigDirectoryPath() + "test-yaml.yaml");
  double c1p1, c2p1;
  std::vector<double> c1p2, c2p2;
  paramHandler.getValue("category-1", "parameter-1", c1p1);
  paramHandler.getValue("category-2", "parameter-1", c2p1);
  paramHandler.getVector("category-1", "parameter-2", c1p2);
  paramHandler.getVector("category-2", "parameter-2", c2p2);

  EXPECT_TRUE(c1p1 == 12.);
  EXPECT_TRUE(c1p2[0] == 1.);
  EXPECT_TRUE(c1p2[1] == 2.);
  EXPECT_TRUE(c1p2[2] == 3.);
  EXPECT_TRUE(c1p2.size() == 3);

  EXPECT_TRUE(c2p1 == 23.);
  EXPECT_TRUE(c2p2[0] == 2.);
  EXPECT_TRUE(c2p2[1] == 3.);
  EXPECT_TRUE(c2p2.size() == 2);
}

