#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Controllers/LegController.h"
#include "Dynamics/FloatingBaseModel.h"
#include <Controllers/StateEstimatorContainer.h>
#include <SimUtilities/VisualizationData.h>
#include "SimUtilities/GamepadCommand.h"

class RobotController{
  friend class RobotRunner;
public:
  RobotController(){}
  virtual ~RobotController(){}

  virtual void initializeController() = 0;
/**
 * Called one time every control loop 
 */
  virtual void runController() = 0;
  virtual void updateVisualization() = 0;
  virtual ControlParameters* getUserControlParameters() = 0;

protected:
  Quadruped<float>* _quadruped = nullptr;
  FloatingBaseModel<float>* _model = nullptr;
  LegController<float>* _legController = nullptr;
  StateEstimatorContainer<float>* _stateEstimator = nullptr;
  StateEstimate<float>* _stateEstimate = nullptr;
  GamepadCommand* _driverCommand = nullptr;
  RobotControlParameters* _controlParameters = nullptr;

  VisualizationData* _visualizationData = nullptr;
  RobotType _robotType;
};

#endif
