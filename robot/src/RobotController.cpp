#include "Controllers/OrientationEstimator.h"
#include "RobotController.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"
#include "Controllers/ContactEstimator.h"



void RobotController::initialize() {
  printf("[RobotController] initialize\n");
  if(robotType == RobotType::MINI_CHEETAH) {
    _quadruped = buildMiniCheetah<float>();
  } else {
    _quadruped = buildCheetah3<float>();
  }

  _legController = new LegController<float>(_quadruped);
  _stateEstimator = new StateEstimatorContainer<float>(cheaterState, kvhImuData, vectorNavData,
          _legController->datas, &_stateEstimate, controlParameters);
  initializeStateEstimator(false);


  // For WBC state
  _model = _quadruped.buildModel();
  _wbc_state = new Cheetah_interface<float>(&_model);
  _data = new Cheetah_Data<float>();

}


void RobotController::step() {
  setupStep();
  _stateEstimator->run();

  // for now, we will always enable the legs:
  _legController->setEnabled(true);
  _legController->setMaxTorqueCheetah3(208.5);
 

  // ======= WBC state command computation  =============== //
  for(size_t i(0); i<4; ++i){
      _data->body_ori[i] = cheaterState->orientation[i];
  }
  for(int i(0);i<3; ++i){
      _data->ang_vel[i] = cheaterState->omegaBody[i];
  }

  for(int leg(0); leg<4; ++leg){
      for(int jidx(0); jidx<3; ++jidx){
          _data->jpos[3*leg + jidx] = _legController->datas[leg].q[jidx];
          _data->jvel[3*leg + jidx] = _legController->datas[leg].qd[jidx];
      }
  }
  _data->dir_command[0] = driverCommand->leftStickAnalog[1];
  _data->dir_command[1] = driverCommand->leftStickAnalog[0];

  _wbc_state->GetCommand(_data, _legController->commands);
  // === End of WBC state command computation  =========== //

  // run the controller:
  //Vec3<float> pDes(0,0,driverCommand->leftStickAnalog[1]);
  Mat3<float> kpMat; kpMat << controlParameters->stand_kp_cartesian[0], 0, 0,
  0, controlParameters->stand_kp_cartesian[1], 0,
  0, 0, controlParameters->stand_kp_cartesian[2];

  Mat3<float> kdMat; kdMat << controlParameters->stand_kd_cartesian[0], 0, 0,
  0, controlParameters->stand_kd_cartesian[1], 0,
  0, 0, controlParameters->stand_kd_cartesian[2];

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
  // leg data
  if(robotType == RobotType::MINI_CHEETAH) {
    _legController->updateData(spiData);
  } else if(robotType == RobotType::CHEETAH_3) {
    _legController->updateData(tiBoardData);
  } else {
    assert(false);
  }
  _legController->zeroCommand();

  // state estimator
  // check transition to cheater mode:
  if(!_cheaterModeEnabled && controlParameters->cheater_mode) {
    printf("[RobotController] Transitioning to Cheater Mode...\n");
    initializeStateEstimator(true);
    // todo any configuration
    _cheaterModeEnabled = true;
  }

  // check transition from cheater mode:
  if(_cheaterModeEnabled && !controlParameters->cheater_mode) {
    printf("[RobotController] Transitioning from Cheater Mode...\n");
    initializeStateEstimator(false);
    // todo any configuration
    _cheaterModeEnabled = false;
  }


  // todo safety checks, sanity checks, etc...
}


void RobotController::finalizeStep() {
  if(robotType == RobotType::MINI_CHEETAH) {
    _legController->updateCommand(spiCommand);
  } else if(robotType == RobotType::CHEETAH_3) {
    _legController->updateCommand(tiBoardCommand);
  } else {
    assert(false);
  }
}

void RobotController::initializeStateEstimator(bool cheaterMode) {
  _stateEstimator->removeAllEstimators();
  if(cheaterMode) {
    _stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
  } else if(robotType == RobotType::MINI_CHEETAH) {
    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  } else if(robotType == RobotType::CHEETAH_3) {
    _stateEstimator->addEstimator<KvhOrientationEstimator<float>>();
  } else {
    assert(false);
  }
}

RobotController::~RobotController() {
  delete _legController;
  delete _stateEstimator;
}
