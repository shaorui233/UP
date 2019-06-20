#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Controllers/LegController.h"
#include "Dynamics/FloatingBaseModel.h"
#include <Controllers/StateEstimatorContainer.h>

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

protected:
  FloatingBaseModel<float> _model;
  LegController<float>* _legController;
  StateEstimatorContainer<float>* _stateEstimator;
};

#endif
