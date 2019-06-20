#include "RobotRunner.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/OrientationEstimator.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"

#include <Utilities/Utilities_print.h>
#include <ParamHandler.hpp>
#include <Utilities/Timer.h>
#include <unistd.h>

#include <Controllers/PositionVelocityEstimator.h>
#include "rt/rt_interface_lcm.h"

RobotRunner::RobotRunner(RobotController* robot_ctrl, 
                         PeriodicTaskManager* manager, 
                         float period, std::string name):
                         PeriodicTask(manager, period, name){
    _robot_ctrl = robot_ctrl;
}

/**
 * Initializes the robot model, state estimator, leg controller,
 * robot data, and any control logic specific data.
 */
void RobotRunner::init() {
  printf("[RobotRunner] initialize\n");

  // Build the appropriate Quadruped object
  if (robotType == RobotType::MINI_CHEETAH) {
    _quadruped = buildMiniCheetah<float>();
  } else {
    _quadruped = buildCheetah3<float>();
  }

  // Initialize the model and robot data
  _model = _quadruped.buildModel();
  _jpos_initializer = new JPosInitializer<float>(3.);
  
  // Always initialize the leg controller and state entimator
  _legController = new LegController<float>(_quadruped);
  _stateEstimator = new StateEstimatorContainer<float>(
      cheaterState, kvhImuData, vectorNavData, _legController->datas,
      &_stateEstimate, controlParameters);
  initializeStateEstimator(false);

  // Controller initializations
  _robot_ctrl->_model = _model;
  _robot_ctrl->_legController = _legController;
  _robot_ctrl->_stateEstimator = _stateEstimator;

  _robot_ctrl->initializeController();
}

/**
 * Runs the overall robot control system by calling each of the major components
 * to run each of their respective steps.
 */
void RobotRunner::run() {
  // Increment the iteration counter
  iter++;

  // Update the data from the robot
  setupStep();

  // Run Control 
  _robot_ctrl->runController();

  // Update Visualization
  _robot_ctrl->updateVisualization();


  // Visualization (will make this into a separate function later)
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      cheetahMainVisualization->q[leg * 3 + joint] =
          _legController->datas[leg].q[joint];
    }
  }
  cheetahMainVisualization->p.setZero();
  cheetahMainVisualization->p = _stateEstimate.position;

  // Sets the leg controller commands for the robot appropriate commands
  finalizeStep();
}

void RobotRunner::setupStep() {
  // Update the leg data
  if (robotType == RobotType::MINI_CHEETAH) {
    _legController->updateData(spiData);
  } else if (robotType == RobotType::CHEETAH_3) {
    _legController->updateData(tiBoardData);
  } else {
    assert(false);
  }

  // Setup the leg controller for a new iteration
  _legController->zeroCommand();
  _legController->setEnabled(true);
  _legController->setMaxTorqueCheetah3(208.5);

  // state estimator
  // check transition to cheater mode:
  if (!_cheaterModeEnabled && controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning to Cheater Mode...\n");
    initializeStateEstimator(true);
    // todo any configuration
    _cheaterModeEnabled = true;
  }

  // check transition from cheater mode:
  if (_cheaterModeEnabled && !controlParameters->cheater_mode) {
    printf("[RobotRunner] Transitioning from Cheater Mode...\n");
    initializeStateEstimator(false);
    // todo any configuration
    _cheaterModeEnabled = false;
  }

  // todo safety checks, sanity checks, etc...
}

void RobotRunner::finalizeStep() {
  if (robotType == RobotType::MINI_CHEETAH) {
    _legController->updateCommand(spiCommand);
  } else if (robotType == RobotType::CHEETAH_3) {
    _legController->updateCommand(tiBoardCommand);
  } else {
    assert(false);
  }
  _iterations++;
}

void RobotRunner::initializeStateEstimator(bool cheaterMode) {
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();
  if (cheaterMode) {
    _stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
  } else if (robotType == RobotType::MINI_CHEETAH) {
    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  } else if (robotType == RobotType::CHEETAH_3) {
    _stateEstimator->addEstimator<KvhOrientationEstimator<float>>();
  } else {
    assert(false);
  }
}

RobotRunner::~RobotRunner() {
  delete _legController;
  delete _stateEstimator;
  delete _jpos_initializer;
}

void RobotRunner::cleanup() {}
