/*! @file SimulatorDriver.cpp
 *  @brief  The SimulatorDriver runs a RobotController and connects it to a Simulator, using shared memory.
 *
 */

#include "SimulationDriver.h"

void SimulationDriver::run() {
  // init shared memory:
  _sharedMemory.attach(DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME);
  _sharedMemory().init();

  printf("[Simulation Driver] Starting main loop...\n");
  bool firstRun = true;
  for(;;) {
    // wait for our turn to access the shared memory
    _sharedMemory().waitForSimulator();

    if(firstRun) {
      firstRun = false;
      // check that the robot type is correct:
      if(_robot != _sharedMemory().simToRobot.robotType) {
        throw std::runtime_error("simulator and simulatorDriver don't agree on which robot we are simulating");
      }
    }

    // run robot control
    if(!(_iterations % 100)) {
      printf("%s\n", _sharedMemory().simToRobot.driverCommand.toString().c_str());
    }


    _sharedMemory().robotIsDone();
    _iterations++;
  }
}