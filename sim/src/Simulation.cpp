#include "Simulation.h"
#include "Dynamics/Quadruped.h"
#include "ParamHandler.hpp"

#include <unistd.h>
#include <include/GameController.h>

// if DISABLE_HIGH_LEVEL_CONTROL is defined, the simulator will run freely, without trying to connect to a robot
//#define DISABLE_HIGH_LEVEL_CONTROL

/*!
 * Initialize the simulator here.  It is _not_ okay to block here waiting for the robot to connect.
 * Use firstRun() instead!
 */
Simulation::Simulation(RobotType robot, Graphics3D *window, SimulatorControlParameters& params) :
_simParams(params),
_tau(12) {


  // init parameters
  printf("[Simulation] Load parameters...\n");
  _simParams.lockMutex(); // we want exclusive access to the simparams at this point
  if(!_simParams.isFullyInitialized()) {
    printf("[ERROR] Simulator parameters are not fully initialized.  You forgot: \n%s\n", _simParams.generateUnitializedList().c_str());
    throw std::runtime_error("simulator not initialized");
  }

  // init LCM
  if(_simParams.sim_state_lcm) {
    printf("[Simulation] Setup LCM...\n");
    _lcm = new lcm::LCM(getLcmUrl(_simParams.sim_lcm_ttl));
    if(!_lcm->good()) {
      printf("[ERROR] Failed to set up LCM\n");
      throw std::runtime_error("lcm bad");
    }
  }

  // init quadruped info
  printf("[Simulation] Build quadruped...\n");
  _robot = robot;
  _quadruped = _robot == RobotType::MINI_CHEETAH ? buildMiniCheetah<double>() : buildCheetah3<double>();
  printf("[Simulation] Build actuator model...\n");
  _actuatorModels = _quadruped.buildActuatorModels();
  _window = window;

  // init graphics
  if(_window) {
    printf("[Simulation] Setup Cheetah graphics...\n");
    _robotID = _robot == RobotType::MINI_CHEETAH ? window->setupMiniCheetah() : window->setupCheetah3();
  }

  // init rigid body dynamics
  printf("[Simulation] Build rigid body model...\n");
  _model = _quadruped.buildModel();
  _simulator = new DynamicsSimulator<double>(_model, (bool)_simParams.use_spring_damper);

  DVec<double> zero12(12);
  for(u32 i = 0; i < 12; i++) {
    zero12[i] = 0.;
  }

  // set some sane defaults:
  _tau = zero12;
  FBModelState<double> x0;
  //x0.bodyOrientation = rotationMatrixToQuaternion(ori::coordinateRotation(CoordinateAxis::Y, .4));
  x0.bodyOrientation = rotationMatrixToQuaternion(ori::coordinateRotation(CoordinateAxis::Y, .0));
  //x0.bodyPosition = Vec3<double>(0,0,-0.0);
  x0.bodyPosition = Vec3<double>(0,0,-0.449);
  SVec<double> v0 = SVec<double>::Zero();
  //v0[3] = 10;
  x0.bodyVelocity = v0;
  x0.q = zero12;
  x0.qd = zero12;

  // Initial Posture
  x0.q[0] = -0.807;
  x0.q[1] = -1.2;
  x0.q[2] = 2.4;

  x0.q[3] = 0.807;
  x0.q[4] = -1.2;
  x0.q[5] = 2.4;

  x0.q[6] = 0.807;
  x0.q[7] = -1.2;
  x0.q[8] = 2.4;

  x0.q[9] = 0.807;
  x0.q[10] = -1.2;
  x0.q[11] = 2.4;

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
  } else if(_robot == RobotType::CHEETAH_3) {
    // init ti board
    for(int leg = 0; leg < 4; leg++) {
      _tiBoards[leg].init(Quadruped<float>::getSideSign(leg));
      _tiBoards[leg].set_link_lengths(_quadruped._abadLinkLength, _quadruped._hipLinkLength, _quadruped._kneeLinkLenght);
      _tiBoards[leg].reset_ti_board_command();
      _tiBoards[leg].reset_ti_board_data();
      _tiBoards[leg].run_ti_board_iteration();
    }
  } else {
    assert(false);
  }


  // init shared memory
  printf("[Simulation] Setup shared memory...\n");
  _sharedMemory.createNew(DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME, true);
  _sharedMemory().init();

  // shared memory fields:
  _sharedMemory().simToRobot.robotType = _robot;


  // load robot control parameters
  printf("[Simulation] Load control parameters...\n");
  if(_robot == RobotType::MINI_CHEETAH) {
    _robotParams.initializeFromYamlFile(getConfigDirectoryPath() + MINI_CHEETAH_DEFAULT_PARAMETERS);
  } else if(_robot == RobotType::CHEETAH_3) {
    _robotParams.initializeFromYamlFile(getConfigDirectoryPath() + CHEETAH_3_DEFAULT_PARAMETERS);
  } else {
    assert(false);
  }

  if(!_robotParams.isFullyInitialized()) {
    printf("Not all robot control parameters were initialized. Missing:\n%s\n", _robotParams.generateUnitializedList().c_str());
    throw std::runtime_error("not all parameters initialized from ini file");
  }

  // init IMU simulator
  printf("[Simulation] Setup IMU simulator...\n");
  _imuSimulator = new ImuSimulator<double>(_simParams);

  _simParams.unlockMutex();
  printf("[Simulation] Ready!\n");
}

void Simulation::sendControlParameter(const std::string &name, ControlParameterValue value, ControlParameterValueKind kind) {
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
  if(_sharedMemory().waitForRobotWithTimeout()) {

  } else {
    _wantStop = true;
    _running = false;
    _connected = false;
    printf("[ERROR] Timed out waiting for message from robot!  Did it crash?\n");
    return;
  }

  //_sharedMemory().waitForRobot();
  _robotMutex.unlock();

  // verify response is good
  assert(response.requestNumber == request.requestNumber);
  assert(response.parameterKind == request.parameterKind);
  assert(std::string(response.name) == request.name);
#endif
}

/*!
 * Called before the simulator is run the first time.  It's okay to put stuff in here that blocks on having
 * the robot connected.
 */
void Simulation::firstRun() {
  // connect to robot
  _robotMutex.lock();
  _sharedMemory().simToRobot.mode = SimulatorMode::DO_NOTHING;
  _sharedMemory().simulatorIsDone();

  printf("[Simulation] Waiting for robot...\n");

  // this loop will check to see if the robot is connected at 10 Hz
  // doing this in a loop allows us to click the "stop" button in the GUI
  // and escape from here before the robot code connects, if needed
  while(!_sharedMemory().tryWaitForRobot()) {
    if(_wantStop) {
      return;
    }
    usleep(100000);
  }
  printf("Success! the robot is alive\n");
  _connected = true;
  _robotMutex.unlock();


  // send all control parameters
  printf("[Simulation] Send control parameters to robot...\n");
  for(auto& kv : _robotParams.collection._map) {
    sendControlParameter(kv.first, kv.second->get(kv.second->_kind), kv.second->_kind);
  }
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
  if(_robot == RobotType::MINI_CHEETAH) {
    for(int leg = 0; leg < 4; leg++) {
      for(int joint = 0; joint < 3; joint++) {
        _tau[leg*3 + joint] = _actuatorModels[joint].getTorque(_spineBoards[leg].torque_out[joint],
                                                               _simulator->getState().qd[leg*3 + joint]);
      }
    }
  } else if(_robot == RobotType::CHEETAH_3) {
    for(int leg = 0; leg < 4; leg++) {
      for(int joint = 0; joint < 3; joint++) {
        _tau[leg*3 + joint] = _actuatorModels[joint].getTorque(_tiBoards[leg].data->tau_des[joint],
                                                               _simulator->getState().qd[leg*3 + joint]);
      }
    }
  } else {
    assert(false);
  }


  // dynamics
  _currentSimTime += dt;
  _simulator->step(dt, _tau, _simParams.floor_kp, _simParams.floor_kd);
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


  } else if(_robot == RobotType::CHEETAH_3) {

    // update data
    for(int leg = 0; leg < 4; leg++) {
      for(int joint = 0; joint < 3; joint++) {
        _tiBoards[leg].data->q[joint] = _simulator->getState().q[leg*3 + joint];
        _tiBoards[leg].data->dq[joint] = _simulator->getState().qd[leg*3 + joint];
      }
    }

    // run control
    for(auto &tiBoard : _tiBoards) {
      tiBoard.run_ti_board_iteration();
    }
  } else {
    assert(false);
  }
}

void Simulation::highLevelControl() {
  // send joystick data to robot:
  _sharedMemory().simToRobot.gamepadCommand = _window->getDriverCommand();
  _sharedMemory().simToRobot.gamepadCommand.applyDeadband(_simParams.game_controller_deadband);

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
  } else if(_robot == RobotType::CHEETAH_3) {
    for(int i = 0; i < 4; i++) {
      _sharedMemory().simToRobot.tiBoardData[i] = *_tiBoards[i].data;
    }
  } else {
    assert(false);
  }

  // signal to the robot that it can start running
  // the _robotMutex is used to prevent qt (which runs in its own thread) from sending a control parameter
  // while the robot code is already running.
  _robotMutex.lock();
  _sharedMemory().simToRobot.mode = SimulatorMode::RUN_CONTROLLER;
  _sharedMemory().simulatorIsDone();

  // wait for robot code to finish (and send LCM while waiting)
  if(_lcm) {
    buildLcmMessage();
    _lcm->publish(SIM_LCM_NAME, &_simLCM);
  }

  // first make sure we haven't killed the robot code
  if(_wantStop) return;

  // next try waiting at most 1 second:
  if(_sharedMemory().waitForRobotWithTimeout()) {

  } else {
    _wantStop = true;
    _running = false;
    _connected = false;
    printf("[ERROR] Timed out waiting for message from robot!  Did it crash?\n");
    return;
  }
  _robotMutex.unlock();

  // update

  _visualizationData = _sharedMemory().robotToSim.visualizationData;
  if(_robot == RobotType::MINI_CHEETAH) {
    _spiCommand = _sharedMemory().robotToSim.spiCommand;
  } else if(_robot == RobotType::CHEETAH_3) {
    for(int i = 0; i < 4; i++) {
      _tiBoards[i].command = _sharedMemory().robotToSim.tiBoardCommand[i];
    }
  } else {
    assert(false);
  }

  _highLevelIterations++;

}

void Simulation::buildLcmMessage() {
  _simLCM.time = _currentSimTime;
  _simLCM.timesteps = _highLevelIterations;
  auto& state = _simulator->getState();
  auto& dstate = _simulator->getDState();

  Vec3<double> rpy = ori::quatToRPY(state.bodyOrientation);
  RotMat<double> Rbody = ori::quaternionToRotationMatrix(state.bodyOrientation);
  Vec3<double> omega = Rbody.transpose() * state.bodyVelocity.head<3>();
  Vec3<double> v = Rbody.transpose() * state.bodyVelocity.tail<3>();

  for(size_t i = 0; i < 4; i++) {
    _simLCM.quat[i] = state.bodyOrientation[i];
  }

  for(size_t i = 0; i < 3; i++) {
    _simLCM.vb[i] = state.bodyVelocity[i + 3]; // linear velocity in body frame
    _simLCM.rpy[i] = rpy[i];
    for(size_t j = 0; j < 3; j++) {
      _simLCM.R[i][j] = Rbody(i,j);
    }
    _simLCM.omegab[i] = state.bodyVelocity[i];
    _simLCM.omega[i] = omega[i];
    _simLCM.p[i] = state.bodyPosition[i];
    _simLCM.v[i] = v[i];
    _simLCM.vbd[i] = dstate.dBodyVelocity[i + 3];
  }

  for(size_t leg = 0; leg < 4; leg++) {
    for(size_t joint = 0; joint < 3; joint++) {
      _simLCM.q[leg][joint] = state.q[leg*3 + joint];
      _simLCM.qd[leg][joint] = state.qd[leg*3 + joint];
      _simLCM.qdd[leg][joint] = dstate.qdd[leg*3 + joint];
      _simLCM.tau[leg][joint] = _tau[leg*3 + joint];
      size_t gcID = _simulator->getModel()._footIndicesGC.at(leg);
      _simLCM.p_foot[leg][joint] = _simulator->getModel()._pGC.at(gcID)[joint];
      _simLCM.f_foot[leg][joint] = _simulator->getContactForce(gcID)[joint];
    }
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
        double mu, double resti, double height, double sizeX, double sizeY, double checkerX,
        double checkerY, bool addToWindow){

    _simulator->addCollisionPlane(mu, resti, height);
    if(addToWindow && _window) {
        _window->lockGfxMutex();
        Checkerboard checker(sizeX, sizeY, checkerX, checkerY);

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
        bool addToWindow, bool transparent){

    _simulator->addCollisionBox(mu, resti, depth, width, height, pos, ori);
    if(addToWindow && _window) {
        _window->lockGfxMutex();
        _window->_drawList.addBox(depth, width, height, pos, ori, transparent);
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
void Simulation::runAtSpeed(bool graphics) {
  firstRun(); // load the control parameters

  // if we requested to stop, stop.
  if(_wantStop) return;
  assert(!_running);
  _running = true;
  Timer frameTimer;
  Timer freeRunTimer;
  u64 desiredSteps = 0;
  u64 steps = 0;


  double frameTime = 1./60.;
  double lastSimTime = 0;

  printf("[Simulator] Starting run loop (dt %f, dt-low-level %f, dt-high-level %f speed %f graphics %d)...\n",
         _simParams.dynamics_dt, _simParams.low_level_dt, _simParams.high_level_dt, _simParams.simulation_speed, graphics);


  while(_running) {
    double dt = _simParams.dynamics_dt;
    double dtLowLevelControl = _simParams.low_level_dt;
    double dtHighLevelControl = _simParams.high_level_dt;
    _desiredSimSpeed = _simParams.simulation_speed;
    u64 nStepsPerFrame = (u64)(((1. / 60.) / dt) * _desiredSimSpeed);
    if(!_window->IsPaused() && steps < desiredSteps) {
      _simParams.lockMutex();
      step(dt, dtLowLevelControl, dtHighLevelControl);
      _simParams.unlockMutex();
      steps++;
    } else {
      double timeRemaining = frameTime - frameTimer.getSeconds();
      if(timeRemaining >  0) {
        usleep((u32)(timeRemaining * 1e6));
      }
    }
    if(frameTimer.getSeconds() > frameTime) {
      double realElapsedTime = frameTimer.getSeconds();
      frameTimer.start();
      if(graphics && _window) {
        double simRate = (_currentSimTime - lastSimTime) / realElapsedTime;
        lastSimTime = _currentSimTime;
              sprintf(_window->infoString, "[Simulation Run %5.2fx]\n"
                                   "real-time:  %8.3f\n"
                                   "sim-time:   %8.3f\n"
                                   "rate:       %8.3f\n", _desiredSimSpeed, freeRunTimer.getSeconds(),
                                   _currentSimTime, simRate);
        updateGraphics();
      }
      if(!_window->IsPaused() && (desiredSteps - steps) < nStepsPerFrame)
        desiredSteps += nStepsPerFrame;
    }
  }
}

void Simulation::loadTerrainFile(const std::string &terrainFileName, bool addGraphics) {
  printf("load terrain %s\n", terrainFileName.c_str());
  ParamHandler paramHandler(terrainFileName);

  if(!paramHandler.fileOpenedSuccessfully()) {
    printf("[ERROR] could not open yaml file for terrain\n");
    throw std::runtime_error("yaml bad");
  }

  std::vector<std::string> keys = paramHandler.getKeys();

  for(auto& key: keys) {

    auto load = [&](double& val, const std::string& name) {
      if(!paramHandler.getValue<double>(key, name, val))
        throw std::runtime_error("terrain read bad: " + key + " " + name);};

    auto loadVec = [&](double& val, const std::string& name, size_t idx) {
      std::vector<double> v;
      if(!paramHandler.getVector<double>(key, name, v))
        throw std::runtime_error("terrain read bad: " + key + " " + name);
      val = v.at(idx);
    };

    auto loadArray = [&](double* val, const std::string& name, size_t idx) {
      std::vector<double> v;
      if(!paramHandler.getVector<double>(key, name, v))
        throw std::runtime_error("terrain read bad: " + key + " " + name);
      assert(v.size() == idx);
      for(size_t i = 0; i < idx; i++)
        val[i] = v[i];
    };

    printf("terrain element %s\n", key.c_str());
    std::string typeName;
    paramHandler.getString(key, "type", typeName);
    if(typeName == "infinite-plane") {
      double mu, resti, height, gfxX, gfxY, checkerX, checkerY;
      load(mu, "mu");
      load(resti, "restitution");
      load(height, "height");
      loadVec(gfxX, "graphicsSize", 0);
      loadVec(gfxY, "graphicsSize", 1);
      loadVec(checkerX, "checkers", 0);
      loadVec(checkerY, "checkers", 1);
      addCollisionPlane(mu, resti, height, gfxX, gfxY, checkerX, checkerY, addGraphics);
    } else if(typeName == "box") {
      double mu, resti, depth, width, height, transparent;
      double pos[3];
      double ori[3];
      load(mu, "mu");
      load(resti, "restitution");
      load(depth, "depth");
      load(width, "width");
      load(height, "height");
      loadArray(pos, "position", 3);
      loadArray(ori, "orientation", 3);
      load(transparent, "transparent");

      Mat3<double> R_box = ori::rpyToRotMat(Vec3<double>(ori));
      R_box.transposeInPlace(); // collisionBox uses "rotation" matrix instead of "transformation"
      addCollisionBox(mu, resti, depth, width, height, Vec3<double>(pos), R_box, addGraphics, transparent != 0.);
    } else if(typeName == "stairs") {
      double mu, resti, rise, run, stepsDouble, width, transparent;
      double pos[3];
      double ori[3];
      load(mu, "mu");
      load(resti, "restitution");
      load(rise, "rise");
      load(width, "width");
      load(run, "run");
      load(stepsDouble, "steps");
      loadArray(pos, "position", 3);
      loadArray(ori, "orientation", 3);
      load(transparent, "transparent");

      Mat3<double> R = ori::rpyToRotMat(Vec3<double>(ori));
      Vec3<double> pOff(pos);
      R.transposeInPlace(); // "graphics" rotation matrix

      size_t steps = (size_t)stepsDouble;

      double heightOffset = rise/2;
      double runOffset = run/2;
      for(size_t step = 0; step < steps; step++) {
        Vec3<double> p(runOffset, 0, heightOffset);
        p = R*p + pOff;

        addCollisionBox(mu, resti, run, width, heightOffset * 2, p, R, addGraphics, transparent != 0.);

        heightOffset += rise/2;
        runOffset += run;
      }

    } else {
      throw std::runtime_error("unknown terrain " + typeName);
    }
  }
}
