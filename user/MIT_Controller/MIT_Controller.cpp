#include "MIT_Controller.hpp"
#include "rt/rt_interface_lcm.h"
MIT_Controller::MIT_Controller():RobotController(){  }

//#define RC_ESTOP
/**
 * Initializes the Control FSM.
 */
void MIT_Controller::initializeController() {
  memset(&main_control_settings, 0, sizeof(gui_main_control_settings_t));
  // Initialize a new GaitScheduler object
  _gaitScheduler = new GaitScheduler<float>(_controlParameters->controller_dt);

  // Initialize the DesiredStateCommand object
  _desiredStateCommand =
      new DesiredStateCommand<float>(_driverCommand,
          &main_control_settings,
          _controlParameters,
          _stateEstimate,
          _controlParameters->controller_dt);

  // Initialize a new ContactEstimator object
  //_contactEstimator = new ContactEstimator<double>();
  ////_contactEstimator->initialize();

  // Initializes the Control FSM with all the required data
  _controlFSM = new ControlFSM<float>(_quadruped, _stateEstimator,
                                      _legController, _gaitScheduler,
                                      _desiredStateCommand, _controlParameters, 
                                      _visualizationData, &userParameters);
}

/**
 * Calculate the commands for the leg controllers using the ControlFSM logic.
 */
void MIT_Controller::runController() {

  bool rcEstop = false;
  get_main_control_settings(&main_control_settings);

  static int iter(0);
  ++iter;
#ifdef RC_ESTOP

  if(main_control_settings.mode == 0) {
    rcEstop = true;
    if(iter%300 ==0)
    printf("ESTOP!\n");
  }
#endif

  if(!rcEstop) {
    // Find the current gait schedule
    _gaitScheduler->step();

    // Find the desired state trajectory
    _desiredStateCommand->convertToStateCommands();

    // Run the Control FSM code
    _controlFSM->runFSM();
  } else {
    if(iter%300 ==0)
    printf("ESTOP!\n");
  }

}


