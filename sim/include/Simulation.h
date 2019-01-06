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

  /*!
   * Add an infinite collision plane to the simulator
   * @param plane : location of the plane
   * @param mu    : friction of the plane
   * @param K     : spring constant of plane
   * @param D     : damping constant of plane
   * @param addToWindow : if true, also adds graphics for the plane
   */
  void addCollisionPlane(SXform<double>& plane, double mu, double K, double D, bool addToWindow = true);

  /*!
   * Take a single timestep of dt seconds
   */
  void step(double dt) {
    _simulator->step(dt, _tau);
  }

  /*!
   * Updates the graphics from the connected window
   */
  void updateGraphics() {
    _window->_drawList.updateRobotFromModel(*_simulator, _robotID);
  }


  /*!
   * Runs the simulator in the current thread until the _running variable is set to false.
   * Updates graphics at 60 fps if desired.
   * Runs simulation as fast as possible.
   * @param dt
   */
  void freeRun(double dt, bool graphics = true) {
    _running = true;
    Timer tim;
    Timer freeRunTimer;
    double lastSimTime = _currentSimTime;
    while(_running) {
      step(dt);
      _currentSimTime += dt;
      double realElapsedTime = tim.getSeconds();
      if(graphics && _window && realElapsedTime >= (1./60.)) {
        double simRate = (_currentSimTime - lastSimTime) / realElapsedTime;
        lastSimTime = _currentSimTime;
        tim.start();
        _window->infoString = "Mode: Freerun\nReal Time: " + std::to_string(freeRunTimer.getSeconds()) + "\n" +
                "Sim Time: " + std::to_string(_currentSimTime) + "\n" +
                "Rate: " + std::to_string(simRate);
        updateGraphics();
        _window->update();
      }
    }
  }

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
  double _currentSimTime = 0.f;
};

#endif //PROJECT_SIMULATION_H
