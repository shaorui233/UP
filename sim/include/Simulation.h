#ifndef PROJECT_SIMULATION_H
#define PROJECT_SIMULATION_H

#include "Quadruped.h"
#include "Graphics3D.h"
#include "MiniCheetah.h"
#include "Cheetah3.h"
#include "Timer.h"

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
  void step(double dt);

  /*!
   * Updates the graphics from the connected window
   */
  void updateGraphics() {
    _window->_drawList.updateRobotFromModel(*_simulator, _robotID);
    _window->update();
  }

  void freeRun(double dt, bool graphics = true);
  void runAtSpeed(double dt, double x, bool graphics = true);

  void resetSimTime() {
    _currentSimTime = 0.f;
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
  bool _running = false;
  double _desiredSimSpeed = 1.;
  double _currentSimTime = 0.;
};

#endif //PROJECT_SIMULATION_H
