#include "RobotInterface.h"
#include "ControlParameters/SimulatorParameters.h"


RobotInterface::RobotInterface(RobotType robotType, Graphics3D *gfx) : PeriodicTask(&_taskManager, 1.f/60.f, "robot-interface"),
_lcm(getLcmUrl(255))
{
  _gfx = gfx;
  _robotType = robotType;
  printf("[RobotInterface] Load parameters...\n");
  if(_robotType == RobotType::MINI_CHEETAH) {
    _controlParameters.initializeFromYamlFile(getConfigDirectoryPath() + MINI_CHEETAH_DEFAULT_PARAMETERS);
  } else if(_robotType == RobotType::CHEETAH_3) {
    _controlParameters.initializeFromYamlFile(getConfigDirectoryPath() + CHEETAH_3_DEFAULT_PARAMETERS);
  } else {
    assert(false);
  }

  if(!_controlParameters.isFullyInitialized()) {
    printf("Not all robot control parameters were initialized. Missing:\n%s\n", _controlParameters.generateUnitializedList().c_str());
    throw std::runtime_error("not all parameters initialized from ini file");
  }
  printf("[RobotInterface] Init LCM\n");
  printf("[RobotInterface] Init graphics\n");
  _robotID = _robotType == RobotType::MINI_CHEETAH ? gfx->setupMiniCheetah() : gfx->setupCheetah3();
  printf("draw list has %lu items\n", _gfx->_drawList._kinematicXform.size());
  _gfx->_drawList._visualizationData = &_visualizationData;
  Checkerboard checker(10, 10, 10, 10);
  uint64_t floorID = _gfx->_drawList.addCheckerboard(checker);
  _gfx->_drawList.updateCheckerboard(0, floorID);
  _gfx->_drawList.buildDrawList();
}


void RobotInterface::run() {
  if(_gfx) {
    _gfx->update();
  }
}
void RobotInterface::startInterface() {
  _running = true;
  this->start();
  _lcmThread = std::thread(&RobotInterface::lcmHandler, this);
}

void RobotInterface::stopInterface() {
  printf("stopInterface\n");
  _running = false;
  _taskManager.stopAll();
  printf("stopall done\n");
  _lcmThread.join();
  printf("lcmthread joined\n");
}

void RobotInterface::lcmHandler() {
  while(_running) {
    _lcm.handleTimeout(1000);
  }
}