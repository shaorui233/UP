/*! @file SimulatorDriver.h
 *  @brief  The SimulatorDriver runs a RobotController and connects it to a Simulator, using shared memory.
 *
 */

#ifndef PROJECT_SIMULATIONDRIVER_H
#define PROJECT_SIMULATIONDRIVER_H

#include "Types.h"
#include "SharedMemory.h"
#include "SimulatorMessage.h"

class SimulationDriver {
public:
  explicit SimulationDriver(RobotType robot) : _robot(robot) { }
  void run();

private:
  RobotType _robot;
  SharedMemoryObject<SimulatorSyncronizedMessage> _sharedMemory;
  u64 _iterations = 0;
};


#endif //PROJECT_SIMULATIONDRIVER_H
