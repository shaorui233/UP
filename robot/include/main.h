#ifndef ROBOT_MAIN_H
#define ROBOT_MAIN_H

#include <Types.h>

struct MasterConfig {
  RobotType _robot;
  bool simulated = false;
};

extern MasterConfig gMasterConfig;

#endif //ROBOT_MAIN_H
