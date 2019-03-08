#ifndef PROJECT_ROBOTCONTROLLER_H
#define PROJECT_ROBOTCONTROLLER_H

#include <SimUtilities/IMUTypes.h>
#include <ControlParameters/ControlParameterInterface.h>
#include <ControlParameters/RobotParameters.h>
#include "Controllers/LegController.h"
#include "Dynamics/Quadruped.h"
#include "SimUtilities/GamepadCommand.h"
#include "SimUtilities/VisualizationData.h"

#include <WBC_state/Cheetah_interface.hpp>
#include <Controllers/StateEstimatorContainer.h>
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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotController() = default;
  void initialize();
  void step();
  void initializeStateEstimator(bool cheaterMode = false);
  ~RobotController();


  GamepadCommand* driverCommand;
  RobotType  robotType;
  KvhImuData* kvhImuData;
  VectorNavData* vectorNavData;
  CheaterState<double>* cheaterState;
  SpiData* spiData;
  SpiCommand* spiCommand;
  TiBoardCommand* tiBoardCommand;
  TiBoardData* tiBoardData;
  RobotControlParameters* controlParameters;
  VisualizationData* visualizationData;

private:
  void setupStep();
  void finalizeStep();
  void testDebugVisualization();
  void StepLocationVisualization();
  Quadruped<float> _quadruped;
  LegController<float>* _legController = nullptr;
  StateEstimate<float> _stateEstimate;
  StateEstimatorContainer<float>* _stateEstimator;
  bool _cheaterModeEnabled = false;

  // For WBC stat
  Cheetah_interface<float>* _wbc_state;
  Cheetah_Data<float>* _data;
  Cheetah_Extra_Data<float>* _extra_data;
  FloatingBaseModel<float> _model;
  u64 _iterations = 0;
};


#endif //PROJECT_ROBOTCONTROLLER_H
