/*! @file SimulatorDriver.h
 *  @brief  The SimulatorDriver runs a RobotController and connects it to a Simulator, using shared memory.
 *
 */

#ifndef PROJECT_SIMULATIONDRIVER_H
#define PROJECT_SIMULATIONDRIVER_H

#include "Types.h"
#include "Utilities/SharedMemory.h"
#include "SimUtilities/SimulatorMessage.h"
#include "ControlParameters/RobotParameters.h"
#include "RobotController.h"

class SimulationBridge {
public:
  explicit SimulationBridge(RobotType robot) : _robot(robot) { }
  void run();
  void handleControlParameters();
  void runRobotControl();
  ~SimulationBridge() {
    delete _robotController;
  }
private:
  bool _firstControllerRun = true;
  RobotType _robot;
  RobotController* _robotController = nullptr;
  SimulatorMode _simMode;
  SharedMemoryObject<SimulatorSyncronizedMessage> _sharedMemory;
  RobotControlParameters _robotParams;
  u64 _iterations = 0;
};


#endif //PROJECT_SIMULATIONDRIVER_H
