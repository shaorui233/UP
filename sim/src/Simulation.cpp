#include "Simulation.h"
#include "Quadruped.h"

#include <unistd.h>
#include <include/GameController.h>

// if DISABLE_HIGH_LEVEL_CONTROL is defined, the simulator will run freely, without trying to connect to a robot
#define DISABLE_HIGH_LEVEL_CONTROL

Simulation::Simulation(bool useMiniCheetah, Graphics3D *window) : _tau(12) {
  printf("[Simulation] Build quadruped...\n");
  _quadruped = useMiniCheetah ? buildMiniCheetah<double>() : buildCheetah3<double>();
  printf("[Simulation] Build actuator model...\n");
  _actuatorModels = _quadruped.buildActuatorModels();
  _window = window;
  _isMiniCheetah = useMiniCheetah;
  printf("[Simulation] Setup Cheetah graphics...\n");
  if(_window) {
    _robotID = useMiniCheetah ? window->setupMiniCheetah() : window->setupCheetah3();
  }
  printf("[Simulation] Build rigid body model...\n");
  _model = _quadruped.buildModel();
  _simulator = new DynamicsSimulator<double>(_model);

  DVec<double> zero12(12);
  for(u32 i = 0; i < 12; i++) {
    zero12[i] = 0.;
  }

  // set some sane defaults:
  _tau = zero12;
  FBModelState<double> x0;
  x0.bodyOrientation = rotationMatrixToQuaternion(ori::coordinateRotation(CoordinateAxis::Y, .4));
  x0.bodyPosition = Vec3<double>(0,0,2);
  SVec<double> v0 = SVec<double>::Zero();
  v0[3] = 10;
  x0.bodyVelocity = v0;
  x0.q = zero12;
  x0.qd = zero12;

  setRobotState(x0);

  printf("[Simulation] Setup low-level control...\n");
  // init spine:
  if(useMiniCheetah) {
    for(int leg = 0; leg < 4; leg++) {
      _spineBoards[leg].init(Quadruped<float>::getSideSign(leg), leg);
      _spineBoards[leg].data = &_spiData;
      _spineBoards[leg].cmd =  &_spiCommand;
      _spineBoards[leg].resetData();
      _spineBoards[leg].resetCommand();
    }
  } else {
    // init ti board
    assert(false); // todo
  }

  // init parameters
  printf("[Simulation] Load parameters...\n");
  _simParams.initializeFromIniFile(getConfigDirectoryPath() + SIMULATOR_DEFAULT_PARAMETERS);
  if(!_simParams.isFullyInitialized()) {
    printf("[ERROR] Simulator parameters are not fully initialized.  You forgot: \n%s\n", _simParams.generateUnitializedList().c_str());
    throw std::runtime_error("simulator not initialized");
  }

  // init shared memory
  printf("[Simulation] Setup shared memory...\n");
  _sharedMemory.createNew(DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME, true);
  _sharedMemory().init();
  _sharedMemory().simToRobot.robotType = useMiniCheetah ? RobotType::MINI_CHEETAH : RobotType::CHEETAH_3;

  printf("[Simulation] Ready!\n");
}

/*!
 * Take a single timestep of dt seconds
 */
void Simulation::step(double dt, double dtLowLevelControl, double dtHighLevelControl) {

  if(_currentSimTime >= _timeOfNextLowLevelControl) {
    lowLevelControl();
    _timeOfNextLowLevelControl = _timeOfNextLowLevelControl + dtLowLevelControl;
  }

  if(_currentSimTime >= _timeOfNextHighLevelControl) {
#ifndef DISABLE_HIGH_LEVEL_CONTROL
    highLevelControl();
#endif
    _timeOfNextHighLevelControl = _timeOfNextHighLevelControl + dtHighLevelControl;
  }

  _currentSimTime += dt;
  _simulator->step(dt, _tau);
}

void Simulation::lowLevelControl() {
  if(_isMiniCheetah) {
    // update spine board data:
    for(int leg = 0; leg < 4; leg++) {
      _spiData.q_abad[leg] = _simulator->getState().q[leg*3 + 0];
      _spiData.q_hip[leg]  = _simulator->getState().q[leg*3 + 1];
      _spiData.q_knee[leg] = _simulator->getState().q[leg*3 + 2];

      _spiData.qd_abad[leg] = _simulator->getState().qd[leg*3 + 0];
      _spiData.qd_hip[leg]  = _simulator->getState().qd[leg*3 + 1];
      _spiData.qd_knee[leg] = _simulator->getState().qd[leg*3 + 2];
    }

    // run spine board control:
    for(auto &spineBoard : _spineBoards) {
      spineBoard.run();
    }

    // run actuator model and move torque into simulator
    for(int leg = 0; leg < 4; leg++) {
      for(int joint = 0; joint < 3; joint++) {
        _tau[leg*3 + joint] = _actuatorModels[joint].getTorque(_spineBoards[leg].torque_out[joint],
                                                               _simulator->getState().qd[leg*3 + joint]);
      }
    }
  } else {
    // todo Cheetah 3
    assert(false);
  }
}

void Simulation::highLevelControl() {
  // send joystick data to robot:
  _sharedMemory().simToRobot.driverCommand = _window->getDriverCommand();
  _sharedMemory().simToRobot.driverCommand.applyDeadband(_simParams.game_controller_deadband);

  // signal to the robot that it can start running
  _sharedMemory().simulatorIsDone();

  // wait for robot code to finish
  _sharedMemory().waitForRobot();
}

/*!
 * Add an infinite collision plane to the simulator
 * @param plane : location of the plane
 * @param mu    : friction of the plane
 * @param K     : spring constant of plane
 * @param D     : damping constant of plane
 * @param addToWindow : if true, also adds graphics for the plane
 */
void Simulation::addCollisionPlane(SXform<double>& plane, double mu, double K, double D, bool addToWindow) {
  size_t simulatorID = _simulator->addCollisionPlane(plane, mu, K, D);
  if(addToWindow && _window) {
    Checkerboard checker(20,20,40,40);
    _window->lockGfxMutex();
    size_t graphicsID = _window->_drawList.addCheckerboard(checker);
    _window->_drawList.buildDrawList();
    _window->_drawList.updateCheckerboardFromCollisionPlane(_simulator->getCollisionPlane(simulatorID), graphicsID);
    _window->unlockGfxMutex();
  }
}

/*!
 * Runs the simulator in the current thread until the _running variable is set to false.
 * Updates graphics at 60 fps if desired.
 * Runs simulation as fast as possible.
 * @param dt
 */
void Simulation::freeRun(double dt, double dtLowLevelControl, double dtHighLevelControl, bool graphics) {
  assert(!_running);
  _running = true;
  Timer tim;
  Timer freeRunTimer;
  double lastSimTime = _currentSimTime;
  while(_running) {
    step(dt, dtLowLevelControl, dtHighLevelControl);
    double realElapsedTime = tim.getSeconds();
    if(graphics && _window && realElapsedTime >= (1./60.)) {
      double simRate = (_currentSimTime - lastSimTime) / realElapsedTime;
      lastSimTime = _currentSimTime;
      tim.start();
      sprintf(_window->infoString, "[Simulation Freerun]\n"
                                   "real-time:%8.3f\n"
                                   "sim-time: %8.3f\n"
                                   "rate:     %8.3f\n",freeRunTimer.getSeconds(), _currentSimTime, simRate);
      updateGraphics();
    }
  }
}

/*!
 * Runs the simulator in the current thread until the _running variable is set to false.
 * Updates graphics at 60 fps if desired.
 * Runs simulation at the desired speed
 * @param dt
 */
void Simulation::runAtSpeed(double dt, double dtLowLevelControl, 
        double dtHighLevelControl, double x, bool graphics) {
  assert(!_running);
  _running = true;
  _desiredSimSpeed = x;
  Timer tim;
  Timer freeRunTimer;
  Timer frameTimer;
  double lastFrameTime = 0;

  double lastSimTime = _currentSimTime; // simulation time at last graphics update

  printf("[Simulator] Starting run loop (dt %f, dt-low-level %f, dt-high-level %f speed %f graphics %d)...\n",
          dt, dtLowLevelControl, dtHighLevelControl, x, graphics);
  while(_running) {
    frameTimer.start();
    int nStepsPerFrame = (int)(((1. / 60.) / dt) * _desiredSimSpeed);

    for(int i = 0; i < nStepsPerFrame; i++) {
      step(dt, dtLowLevelControl, dtHighLevelControl);
    }

    double realElapsedTime = tim.getSeconds();
    if(graphics && _window) {
      double simRate = (_currentSimTime - lastSimTime) / realElapsedTime;
      lastSimTime = _currentSimTime;
      tim.start();
      sprintf(_window->infoString, "[Simulation Run %5.2fx]\n"
                                   "real-time:  %8.3f\n"
                                   "sim-time:   %8.3f\n"
                                   "rate:       %8.3f\n"
                                   "frame-time: %8.3f\n"
                                   "cpu-pct:    %8.3f\n", _desiredSimSpeed, freeRunTimer.getSeconds(),
                                   _currentSimTime, simRate, lastFrameTime * 1000., 100 * (lastFrameTime) / (1. / 60.));
      updateGraphics();
      double frameTime = frameTimer.getSeconds();
      lastFrameTime = frameTime;
      frameTimer.start();
      double extraTime = (1. / 60.) - frameTime;
      if(extraTime > 0) {
        usleep((u32)(extraTime * 1000000));
      }
    }
  }
}
