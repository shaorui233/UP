#include "RobotController.h"
#include "Cheetah3.h"
#include "MiniCheetah.h"


void RobotController::initialize() {
  printf("[RobotController] initialize\n");
  if(robotType == RobotType::MINI_CHEETAH) {
    _quadruped = buildMiniCheetah<float>();
  } else {
    _quadruped = buildCheetah3<float>();
  }

  _legController = new LegController<float>(_quadruped);
}


void RobotController::step() {
  setupStep();

  // run the controller:
  Vec3<float> pDes(0,0,driverCommand->leftStickAnalog[1]);
  float kp = controlParameters->stand_kp_cartesian;
  float kd = controlParameters->stand_kd_cartesian;
  Mat3<float> kpMat; kpMat << kp, 0, 0, 0, kp, 0, 0, 0, kp;
  Mat3<float> kdMat; kdMat << kd, 0, 0, 0, kd, 0, 0, 0, kd;
  for(int leg = 0; leg < 4; leg++) {
    _legController->commands[leg].pDes = pDes;
    _legController->commands[leg].kpCartesian = kpMat;
    _legController->commands[leg].kdCartesian = kdMat;
  }


  finalizeStep();
}


void RobotController::setupStep() {
  if(robotType == RobotType::MINI_CHEETAH) {
    _legController->updateData(spiData);
    _legController->zeroCommand();
  } else {
    assert(false);
  }

  // todo safety checks, sanity checks, etc...
}


void RobotController::finalizeStep() {
  if(robotType == RobotType::MINI_CHEETAH) {
    _legController->updateCommand(spiCommand);
  } else {
    assert(false);
  }

}

RobotController::~RobotController() {

}