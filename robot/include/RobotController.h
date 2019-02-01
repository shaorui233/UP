#ifndef PROJECT_ROBOTCONTROLLER_H
#define PROJECT_ROBOTCONTROLLER_H

#include <IMUTypes.h>
#include <ControlParameterInterface.h>
#include <RobotParameters.h>
#include "LegController.h"
#include "Quadruped.h"
#include "GamepadCommand.h"

// gamepadCommand
// robotType
// kvh
// vectorNav
// cheaterState
// spiData
// controlParameterRequest
// "mode" ?

// robotType
// spiCommand
//


class RobotController {
public:
  RobotController() = default;
  void initialize();
  void step();
  ~RobotController();


  GamepadCommand* driverCommand;
  RobotType  robotType;
  KvhImuData* kvhImuData;
  VectorNavData* vectorNavData;
  CheaterState<double>* cheaterState;
  SpiData* spiData;
  SpiCommand* spiCommand;
  RobotControlParameters* controlParameters;

private:
  void setupStep();
  void finalizeStep();
  Quadruped<float> _quadruped;
  LegController<float>* _legController;
};


#endif //PROJECT_ROBOTCONTROLLER_H
