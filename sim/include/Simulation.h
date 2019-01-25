#ifndef PROJECT_SIMULATION_H
#define PROJECT_SIMULATION_H

#include "Quadruped.h"
#include "Graphics3D.h"
#include "MiniCheetah.h"
#include "Cheetah3.h"
#include "Timer.h"
#include "SpineBoard.h"

#include <vector>



/*!
 * Top-level control of a simulation.
 * A simulation includes 1 robot and 1 controller
 * It does not include the graphics window: this must be set with the setWindow method
 */
class Simulation {
public:
  explicit Simulation(bool useMiniCheetah, Graphics3D* window);

  /*!
   * Explicitly set the state of the robot
   */
  void setRobotState(FBModelState<double>& state) {
    _simulator->setState(state);
  }


  void addCollisionPlane(SXform<double>& plane, double mu, double K, double D, bool addToWindow = true);
  void step(double dt, double dtLowLevelControl, double dtHighLevelControl);

  /*!
   * Updates the graphics from the connected window
   */
  void updateGraphics() {
    _window->_drawList.updateRobotFromModel(*_simulator, _robotID);
    _window->_drawList.updateAdditionalInfo(*_simulator);
    _window->update();
  }

  void freeRun(double dt, double dtLowLevelControl, double dtHighLevelControl, bool graphics = true);
  void runAtSpeed(double dt, double dtLowLevelControl, double dtHighLevelControl, double x, bool graphics = true);

  void resetSimTime() {
    _currentSimTime = 0.;
    _timeOfNextLowLevelControl = 0.;
    _timeOfNextHighLevelControl = 0.;
  }


  ~Simulation() {
      delete _simulator;
  }

  const FBModelState<double>& getRobotState() {
    return _simulator->getState();
  }


private:
  size_t _robotID;
  Graphics3D *_window = nullptr;
  Quadruped<double> _quadruped;
  FloatingBaseModel<double> _model;
  DVec<double> _tau;
  DynamicsSimulator<double>* _simulator = nullptr;
  std::vector<ActuatorModel<double>> _actuatorModels;
  SpiCommand _spiCommand;
  SpiData    _spiData;
  SpineBoard _spineBoards[4];
  bool _isMiniCheetah = false;
  bool _running = false;
  double _desiredSimSpeed = 1.;
  double _currentSimTime = 0.;
  double _timeOfNextLowLevelControl = 0.;
  double _timeOfNextHighLevelControl = 0.;
};

#endif //PROJECT_SIMULATION_H
