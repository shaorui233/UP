#include "Simulation.h"
#include "Quadruped.h"

#include <unistd.h>
#include <include/GameController.h>

// if DISABLE_HIGH_LEVEL_CONTROL is defined, the simulator will run freely, without trying to connect to a robot
#define DISABLE_HIGH_LEVEL_CONTROL

Simulation::Simulation(RobotType robot, Graphics3D *window) : _tau(12) {
  printf("[Simulation] Build quadruped...\n");
  _robot = robot;
  _quadruped = _robot == RobotType::MINI_CHEETAH ? buildMiniCheetah<double>() : buildCheetah3<double>();
  printf("[Simulation] Build actuator model...\n");
  _actuatorModels = _quadruped.buildActuatorModels();
  _window = window;

  if(_window) {
    printf("[Simulation] Setup Cheetah graphics...\n");
    _robotID = _robot == RobotType::MINI_CHEETAH ? window->setupMiniCheetah() : window->setupCheetah3();
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
  //v0[3] = 10;
  x0.bodyVelocity = v0;
  x0.q = zero12;
  x0.qd = zero12;

  setRobotState(x0);

  printf("[Simulation] Setup low-level control...\n");
  // init spine:
  if(_robot == RobotType::MINI_CHEETAH) {
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

  // shared memory fields:
  _sharedMemory().simToRobot.robotType = _robot;


  // load control parameters
  printf("[Simulation] Load control parameters...\n");
  if(_robot == RobotType::MINI_CHEETAH) {
    _robotParams.initializeFromIniFile(getConfigDirectoryPath() + MINI_CHEETAH_DEFAULT_PARAMETERS);
  } else {
    assert(false);
  }

  if(!_robotParams.isFullyInitialized()) {
    printf("Not all robot control parameters were initialized. Missing:\n%s\n", _robotParams.generateUnitializedList().c_str());
    throw std::runtime_error("not all parameters initialized from ini file");
  }

  // send all control parameters
  printf("[Simulation] Send control parameters to robot...\n");
  for(auto& kv : _robotParams.collection._map) {
    printf("send %s\n", kv.first.c_str());
    sendControlParameter(kv.first, kv.second->get(kv.second->_kind), kv.second->_kind);
  }

  // init IMU simulator
  printf("[Simulation] Setup IMU simulator...\n");
  _imuSimulator = new ImuSimulator<double>(_simParams);

  printf("[Simulation] Ready!\n");
}

void Simulation::sendControlParameter(const std::string &name, ControlParameterValue value, ControlParameterValueKind kind) {
  (void)name;
  (void)value;
  (void)kind;
#ifndef DISABLE_HIGH_LEVEL_CONTROL
  ControlParameterRequest& request = _sharedMemory().simToRobot.controlParameterRequest;
  ControlParameterResponse& response = _sharedMemory().robotToSim.controlParameterResponse;

  // first check no pending message
  assert(request.requestNumber == response.requestNumber);

  // new message
  request.requestNumber++;

  // message data
  request.requestKind = ControlParameterRequestKind::SET_PARAM_BY_NAME;
  strcpy(request.name, name.c_str());
  request.value = value;
  request.parameterKind = kind;
  printf("%s\n", request.toString().c_str());

  // run robot:
  _robotMutex.lock();
  _sharedMemory().simToRobot.mode = SimulatorMode::RUN_CONTROL_PARAMETERS;
  _sharedMemory().simulatorIsDone();

  // wait for robot code to finish
  _sharedMemory().waitForRobot();
  _robotMutex.unlock();

  // verify response is good
  assert(response.requestNumber == request.requestNumber);
  assert(response.parameterKind == request.parameterKind);
  assert(std::string(response.name) == request.name);
#endif
}

/*!
 * Take a single timestep of dt seconds
 */
void Simulation::step(double dt, double dtLowLevelControl, double dtHighLevelControl) {

  // Low level control (if needed)
  if(_currentSimTime >= _timeOfNextLowLevelControl) {
    lowLevelControl();
    _timeOfNextLowLevelControl = _timeOfNextLowLevelControl + dtLowLevelControl;
  }

  // High level control
  if(_currentSimTime >= _timeOfNextHighLevelControl) {
#ifndef DISABLE_HIGH_LEVEL_CONTROL
    highLevelControl();
#endif
    _timeOfNextHighLevelControl = _timeOfNextHighLevelControl + dtHighLevelControl;
  }

  // actuator model:
  for(int leg = 0; leg < 4; leg++) {
    for(int joint = 0; joint < 3; joint++) {
      _tau[leg*3 + joint] = _actuatorModels[joint].getTorque(_spineBoards[leg].torque_out[joint],
                                                             _simulator->getState().qd[leg*3 + joint]);
    }
  }

  // dynamics
  _currentSimTime += dt;
  _simulator->step(dt, _tau);
}

void Simulation::lowLevelControl() {
  if(_robot == RobotType::MINI_CHEETAH) {
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


  } else {
    // todo Cheetah 3
    assert(false);
  }
}

void Simulation::highLevelControl() {
  // send joystick data to robot:
  _sharedMemory().simToRobot.driverCommand = _window->getDriverCommand();
  _sharedMemory().simToRobot.driverCommand.applyDeadband(_simParams.game_controller_deadband);

  // send IMU data to robot:
  _imuSimulator->updateCheaterState(_simulator->getState(), _simulator->getDState(), _sharedMemory().simToRobot.cheaterState);
  if(_robot == RobotType::MINI_CHEETAH) {
    _imuSimulator->updateVectornav(_simulator->getState(), _simulator->getDState(),
            &_sharedMemory().simToRobot.vectorNav);
  } else {
    _imuSimulator->updateKVH(_simulator->getState(), _simulator->getDState(),
                                   &_sharedMemory().simToRobot.kvh);
  }

  // send leg data to robot
  if(_robot == RobotType::MINI_CHEETAH) {
    _sharedMemory().simToRobot.spiData = _spiData;
  } else {
    assert(false); // todo cheetah 3
  }

  // signal to the robot that it can start running
  // the _robotMutex is used to prevent qt (which runs in its own thread) from sending a control parameter
  // while the robot code is already running.
  _robotMutex.lock();
  _sharedMemory().simToRobot.mode = SimulatorMode::RUN_CONTROLLER;
  _sharedMemory().simulatorIsDone();

  // wait for robot code to finish
  _sharedMemory().waitForRobot();
  _robotMutex.unlock();

  // update
  if(_robot == RobotType::MINI_CHEETAH) {
    _spiCommand = _sharedMemory().robotToSim.spiCommand;
  } else {
    assert(false);
  }

}

/*!
 * Add an infinite collision plane to the simulator
 * @param mu          : friction of the plane
 * @param resti       : restitution coefficient
 * @param height      : height of plane
 * @param addToWindow : if true, also adds graphics for the plane
 */
void Simulation::addCollisionPlane(
        double mu, double resti, double height, bool addToWindow){

    _simulator->addCollisionPlane(mu, resti, height);
    if(addToWindow && _window) {
        _window->lockGfxMutex();
        Checkerboard checker(20,20,40,40);

        size_t graphicsID = _window->_drawList.addCheckerboard(checker);
        _window->_drawList.buildDrawList();
        _window->_drawList.updateCheckerboard(height, graphicsID);
        _window->unlockGfxMutex();
     }
}


/*!
 * Add an box collision to the simulator
 * @param mu          : location of the box
 * @param resti       : restitution coefficient
 * @param depth       : depth (x) of box
 * @param width       : width (y) of box
 * @param height      : height (z) of box 
 * @param pos         : position of box
 * @param ori         : orientation of box 
 * @param addToWindow : if true, also adds graphics for the plane
 */
void Simulation::addCollisionBox(
        double mu, double resti, 
        double depth, double width, double height, 
        const Vec3<double> & pos, const Mat3<double> & ori,
        bool addToWindow){

    _simulator->addCollisionBox(mu, resti, depth, width, height, pos, ori);
    if(addToWindow && _window) {
        _window->lockGfxMutex();
        _window->_drawList.addBox(depth, width, height, pos, ori);
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

    if(!_window->IsPaused()){
        for(int i = 0; i < nStepsPerFrame; i++) {
            step(dt, dtLowLevelControl, dtHighLevelControl);
        }
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
