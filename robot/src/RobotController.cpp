#include "RobotController.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"


void RobotController::initialize() {
  printf("[RobotController] initialize\n");
  if(robotType == RobotType::MINI_CHEETAH) {
    _quadruped = buildMiniCheetah<float>();
  } else {
    _quadruped = buildCheetah3<float>();
  }

  _legController = new LegController<float>(_quadruped);

  // For WBC state
  _model = _quadruped.buildModel();
  _wbc_state = new Cheetah_interface<float>(&_model);
  _data = new Cheetah_Data<float>();

}


void RobotController::step() {
  setupStep();
 

  // ======= WBC state command computation  =============== //
  for(size_t i(0); i<4; ++i){
      _data->body_ori[i] = cheaterState->orientation[i];
  }
  for(int i(0);i<3; ++i){
      _data->ang_vel[i] = cheaterState->omega[i];
  }

  for(int leg(0); leg<4; ++leg){
      for(int jidx(0); jidx<3; ++jidx){
          _data->jpos[3*leg + jidx] = _legController->datas[leg].q[jidx];
          _data->jvel[3*leg + jidx] = _legController->datas[leg].qd[jidx];
      }
  }
  _wbc_state->GetCommand(_data, _legController->commands);
  // === End of WBC state command computation  =========== //

  // run the controller:
  //Vec3<float> pDes(0,0,driverCommand->leftStickAnalog[1]);
  float kp = controlParameters->stand_kp_cartesian;
  float kd = controlParameters->stand_kd_cartesian;
  Mat3<float> kpMat; kpMat << kp, 0, 0, 0, kp, 0, 0, 0, kp;
  Mat3<float> kdMat; kdMat << kd, 0, 0, 0, kd, 0, 0, 0, kd;
  for(int leg = 0; leg < 4; leg++) {
    //_legController->commands[leg].pDes = pDes;
    //_legController->commands[leg].kpCartesian = kpMat;
    //_legController->commands[leg].kdCartesian = kdMat;

    _legController->commands[leg].kpJoint = kpMat;
    _legController->commands[leg].kdJoint = kdMat;

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
