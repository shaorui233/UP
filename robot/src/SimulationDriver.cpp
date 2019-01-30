/*! @file SimulatorDriver.cpp
 *  @brief  The SimulatorDriver runs a RobotController and connects it to a Simulator, using shared memory.
 *
 */

#include "SimulationDriver.h"
#include "LegController.h"


void SimulationDriver::run() {
  // init shared memory:
  _sharedMemory.attach(DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME);
  _sharedMemory().init();


  printf("[Simulation Driver] Starting main loop...\n");
  bool firstRun = true;
  for(;;) {
    // wait for our turn to access the shared memory
    // on the first loop, this gives the simulator a chance to put stuff in shared memory before we start
    _sharedMemory().waitForSimulator();

    if(firstRun) {
      firstRun = false;
      // check that the robot type is correct:
      if(_robot != _sharedMemory().simToRobot.robotType) {
        printf("simulator and simulatorDriver don't agree on which robot we are simulating (robot %d, sim %d)\n", (int)_robot, (int)_sharedMemory().simToRobot.robotType);
        throw std::runtime_error("robot mismatch!");
      }
    }

    _simMode = _sharedMemory().simToRobot.mode;
    switch(_simMode) {
      case SimulatorMode::RUN_CONTROL_PARAMETERS:
        handleControlParameters();
        break;
      case SimulatorMode::RUN_CONTROLLER:
        _iterations++;
        runRobotControl();
        break;
      case SimulatorMode::EXIT:
        printf("[Simulation Driver] Transitioned to exit mode\n");
        return;
        break;
      default:
        throw std::runtime_error("unknown simulator mode");
    }

    _sharedMemory().robotIsDone();
  }
}


void SimulationDriver::handleControlParameters() {
  ControlParameterRequest& request = _sharedMemory().simToRobot.controlParameterRequest;
  ControlParameterResponse& response = _sharedMemory().robotToSim.controlParameterResponse;
  if(request.requestNumber <= response.requestNumber) {
    // nothing to do!
    return;
  }

  // sanity check
  u64 nRequests = request.requestNumber - response.requestNumber;
  assert(nRequests == 1);

  response.nParameters = _robotParams.collection._map.size(); // todo don't do this every single time?

  switch(request.requestKind) {
    case ControlParameterRequestKind::SET_PARAM_BY_NAME:
    {
      std::string name(request.name);
      ControlParameter& param = _robotParams.collection.lookup(name);

      // type check
      if(param._kind != request.parameterKind) {
        throw std::runtime_error("type mismatch for parameter " + name + ", robot thinks it is "
        + controlParameterValueKindToString(param._kind) + " but received a command to set it to " +
        controlParameterValueKindToString(request.parameterKind));
      }

      // do the actual set
      param.set(request.value, request.parameterKind);

      // respond:
      response.requestNumber = request.requestNumber; // acknowledge that the set has happened
      response.parameterKind = request.parameterKind; // just for debugging print statements
      response.value = request.value;                 // just for debugging print statements
      strcpy(response.name, name.c_str());            // just for debugging print statements
      response.requestKind = request.requestKind;


      printf("%s\n", response.toString().c_str());

    }
      break;


    case ControlParameterRequestKind::GET_PARAM_BY_NAME:
    {
      std::string name(request.name);
      ControlParameter& param = _robotParams.collection.lookup(name);

      // type check
      if(param._kind != request.parameterKind) {
        throw std::runtime_error("type mismatch for parameter " + name + ", robot thinks it is "
                                 + controlParameterValueKindToString(param._kind) + " but received a command to set it to " +
                                 controlParameterValueKindToString(request.parameterKind));
      }

      // respond
      response.value = param.get(request.parameterKind);
      response.requestNumber = request.requestNumber;   // acknowledge
      response.parameterKind = request.parameterKind;   // just for debugging print statements
      strcpy(response.name, name.c_str());              // just for debugging print statements
      response.requestKind = request.requestKind;       // just for debugging print statements


      printf("%s\n", response.toString().c_str());
    }
      break;
  }
}

void SimulationDriver::runRobotControl() {
  if(_firstControllerRun) {
    printf("[Simulator Driver] First run of robot controller...\n");
    if(_robotParams.isFullyInitialized()) {
      printf("\tAll %ld control parameters are initialized\n", _robotParams.collection._map.size());
      _simMode = SimulatorMode::RUN_CONTROLLER;
    } else {
      printf("\tbut not all control parameters were initialized. Missing:\n%s\n", _robotParams.generateUnitializedList().c_str());
      throw std::runtime_error("not all parameters initialized when going into RUN_CONTROLLER");
    }
    _firstControllerRun = false;
  }



  // run robot control
//  if(!(_iterations % 100)) {
//    printf("%s\n", _sharedMemory().simToRobot.driverCommand.toString().c_str());
//    printf("%.3f\n", _sharedMemory().simToRobot.spiData.q_hip[2]);
//  }

  // hack

  for(int leg = 0; leg < 4; leg++) {
    for(int axis = 0; axis < 3; axis++) {
      _sharedMemory().robotToSim.spiCommand.tau_abad_ff[leg] = 200;
      _sharedMemory().robotToSim.spiCommand.tau_hip_ff[leg] = 200;
      _sharedMemory().robotToSim.spiCommand.tau_knee_ff[leg] = _sharedMemory().simToRobot.driverCommand.leftStickAnalog[1];
      _sharedMemory().robotToSim.spiCommand.flags[leg] = 1;
    }
  }
}