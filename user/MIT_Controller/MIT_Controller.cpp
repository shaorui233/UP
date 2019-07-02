#include "MIT_Controller.hpp"

MIT_Controller::MIT_Controller():RobotController(){  }


/**
 * Initializes the Control FSM.
 */
void MIT_Controller::initializeController() {
  // Initialize a new GaitScheduler object
  _gaitScheduler = new GaitScheduler<float>();

  // Initialize the DesiredStateCommand object
  _desiredStateCommand =
      new DesiredStateCommand<float>(_driverCommand, _stateEstimate);

  // Initialize a new ContactEstimator object
  //_contactEstimator = new ContactEstimator<double>();
  ////_contactEstimator->initialize();

  // Initializes the Control FSM with all the required data
  _controlFSM = new ControlFSM<float>(_quadruped, _stateEstimator,
                                      _legController, _gaitScheduler,
                                      _desiredStateCommand, _controlParameters, _visualizationData);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void MIT_Controller::runController() {
  // Find the current gait schedule
  _gaitScheduler->step();

  // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands();

  // Run the Control FSM code
  _controlFSM->runFSM();
}


