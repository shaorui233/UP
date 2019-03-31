#ifndef PROJECT_ROBOTINTERFACE_H
#define PROJECT_ROBOTINTERFACE_H


#include <ControlParameters/RobotParameters.h>
#include <lcm-cpp.hpp>
#include <thread>
#include <Utilities/PeriodicTask.h>
#include "Graphics3D.h"

class RobotInterface : PeriodicTask {
public:
  RobotInterface(RobotType robotType, Graphics3D* gfx);
  RobotControlParameters& getParams() { return _controlParameters; }
  void startInterface();
  void stopInterface();
  void lcmHandler();

  void init() { }
  void run();
  void cleanup() { }

private:
  PeriodicTaskManager _taskManager;
  void updateGraphics();
  lcm::LCM _lcm;
  uint64_t _robotID;
  std::thread _lcmThread;
  VisualizationData _visualizationData;
  RobotControlParameters _controlParameters;
  Graphics3D* _gfx;
  RobotType _robotType;
  bool _running = false;
};


#endif //PROJECT_ROBOTINTERFACE_H
