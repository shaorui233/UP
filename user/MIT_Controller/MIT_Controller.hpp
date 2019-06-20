#ifndef MIT_CONTROLLER
#define MIT_CONTROLLER

/**
 * Enumerate all of the Control logic options. Each one will have
 * an initialize and a run function that pieces together the control
 * logic from the various robot controllers and sensors.
 */


class MIT_Controller: public RobotController{
  ControlFSM<float>* _controlFSM;

  // Gait Scheduler controls the nominal contact schedule for the feet
  GaitScheduler<float>* _gaitScheduler;

};



/**
 * Initializes the Control FSM.
 */
void RobotRunner::initializeControlOptionControlFSM() {
  // Initialize a new GaitScheduler object
  _gaitScheduler = new GaitScheduler<float>();

  // Initialize the DesiredStateCommand object
  _desiredStateCommand =
      new DesiredStateCommand<float>(driverCommand, &_stateEstimate);

  // Initialize a new ContactEstimator object
  //_contactEstimator = new ContactEstimator<double>();
  ////_contactEstimator->initialize();

  // Initializes the Control FSM with all the required data
  _controlFSM = new ControlFSM<float>(&_quadruped, _stateEstimator,
                                      _legController, _gaitScheduler,
                                      _desiredStateCommand, controlParameters);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void RobotRunner::runControlOptionControlFSM() {
  // Run the state estimator step
  _stateEstimator->run(cheetahMainVisualization);

  // Find the current gait schedule
  _gaitScheduler->step();

  // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands();

  // Run the Control FSM code
  _controlFSM->runFSM();
}

#endif
