#include <DynSimulation.hpp>
#include <Utilities/Utilities_print.h>
#include <unistd.h>
#include <mutex>
#include <ControlParameters/SimulatorParameters.h>

void DynSimulation::LCM_Loop(){
    // Data
    DVec<double> full_config(cheetah::dim_config);
    DVec<double> full_vel(cheetah::dim_config);

    printf("LCM Loop Start\n");
    // Loop
    while(true){
        lcm_subscribe.handle();
        // Reset
        if(handler->_reset_call) reset(handler->_state);

        // Step
        step(handler->_jpos_cmd, handler->_jvel_cmd, full_config, full_vel);
        _updateOutput(full_config, full_vel, _output);
        lcm_publish.publish("simulation_to_python", &_output);
    }
}

DynSimulation::DynSimulation(SimGraphics3D *window):
    _tau(cheetah::num_act_joint),
    _torque_cmd(cheetah::num_act_joint)
{
    // Simulation Setting
    _quadruped = buildMiniCheetah<double>() ;
    _actuatorModels = _quadruped.buildActuatorModels();
    _model = _quadruped.buildModel();
    //_quadruped.buildModel(_model);
    _simulator = new DynamicsSimulator<double>(_model);

    _window = window;
    _robotID = _window->setupMiniCheetah();

    _window->lockGfxMutex();
    SimCheckerBoard checker(20, 20, 40, 40);
    size_t graphicsID = _window->_drawList.addCheckerboard(checker);
    _window->_drawList.buildSimDrawList();
    _window->_drawList.updateCheckerboard(0., graphicsID);
    _window->unlockGfxMutex();
    _simulator->addCollisionPlane(_mu, _resti, 0.);

    DVec<double> zero12(12);
    for(u32 i = 0; i < 12; i++) {
        zero12[i] = 0.;
    }

    // set some sane defaults:
    _tau = zero12;
    FBModelState<double> x0;
    x0.bodyOrientation = rotationMatrixToQuaternion(ori::coordinateRotation(CoordinateAxis::Z, 0.));
    // Mini Cheetah
    x0.bodyPosition.setZero(); 
    SVec<double> v0 = SVec<double>::Zero();
    //v0[3] = 10;
    x0.bodyVelocity = v0;
    x0.q = zero12;
    x0.qd = zero12;

    // Mini Cheetah Initial Posture
    x0.bodyPosition[2] = 0.3;
    x0.q[0] = -0.03;
    x0.q[1] = -0.79;
    x0.q[2] = 1.715;

    x0.q[3] = 0.03;
    x0.q[4] = -0.79;
    x0.q[5] = 1.715;

    x0.q[6] = -0.03;
    x0.q[7] = -0.72; 
    x0.q[8] = 1.715;

    x0.q[9] = 0.03;
    x0.q[10] = -0.72;
    x0.q[11] = 1.715;

    setRobotState(x0);

    printf("[Simulation] Setup low-level control...\n");
    // init spine:
    //if(_robot == RobotType::MINI_CHEETAH) {
    for(int leg = 0; leg < 4; leg++) {
    _spineBoards[leg].init(Quadruped<float>::getSideSign(leg), leg);
    //_spineBoards[leg].data = &_spiData;
    //_spineBoards[leg].cmd =  &_spiCommand;
    _spineBoards[leg].resetData();
    _spineBoards[leg].resetCommand();
    }
    //} else if(_robot == RobotType::CHEETAH_3) {
    // init ti board
    //for(int leg = 0; leg < 4; leg++) {
    //_tiBoards[leg].init(Quadruped<float>::getSideSign(leg));
    //_tiBoards[leg].set_link_lengths(_quadruped._abadLinkLength, _quadruped._hipLinkLength, _quadruped._kneeLinkLenght);
    //_tiBoards[leg].reset_ti_board_command();
    //_tiBoards[leg].reset_ti_board_data();
    //_tiBoards[leg].run_ti_board_iteration();
    //}
    //} else {
    //assert(false);
    //}


    // load robot control parameters
    printf("[Simulation] Load control parameters...\n");
    //if(_robot == RobotType::MINI_CHEETAH) {
    _robotParams.initializeFromYamlFile(getConfigDirectoryPath() + MINI_CHEETAH_DEFAULT_PARAMETERS);
    //} else if(_robot == RobotType::CHEETAH_3) {
    //_robotParams.initializeFromYamlFile(getConfigDirectoryPath() + CHEETAH_3_DEFAULT_PARAMETERS);
    //} else {
    //assert(false);
    //}

    if(!_robotParams.isFullyInitialized()) {
        printf("Not all robot control parameters were initialized. Missing:\n%s\n", _robotParams.generateUnitializedList().c_str());
        throw std::runtime_error("not all parameters initialized from ini file");
    }

    // LCM Setting
    if(!lcm_subscribe.good()) { printf("LCM subscriber Error\n"); }
    if(!lcm_publish.good()) { printf("LCM publisher Error\n"); }

    handler = new Handler();
    lcm_subscribe.subscribe("python_to_simulation", &Handler::handleMessage, handler);

    //std::thread lcm_loop_thread([this](){LCM_Loop();});
    lcm_loop_thread = new std::thread([this](){LCM_Loop();});
    //lcm_loop_thread.join();

    printf("[DynSimulation] Contructed\n");
}

DynSimulation::~DynSimulation(){
    delete handler;
    delete _window;
}

void DynSimulation::reset(DVec<double> & state){
    printf("resect call\n");
    pretty_print(state, std::cout, "state");

}
void DynSimulation::step(
        const DVec<double> & jpos_cmd, const DVec<double> & jvel_cmd, 
        DVec<double> & nx_full_config, DVec<double> & nx_full_vel){

    //printf("doing some simulation...\n");
    pretty_print(jpos_cmd, std::cout, "jpos_cmd");
    pretty_print(jvel_cmd, std::cout, "jvel_cmd");
    printf("\n");
    _tau.setZero();
    for(size_t i(0); i<cheetah::num_act_joint; ++i){
        _torque_cmd[i] = 100.*(jpos_cmd[i] -_simulator->getState().q[i] ) + 
            3.*(jvel_cmd[i] - _simulator->getState().qd[i]);
    }
    _tau_update();
    _simulator->step(_dt, _tau, 500., 10.);
    _updateOutputConfig(nx_full_config, nx_full_vel);
    _updateGraphics();
}
void DynSimulation::_updateOutputConfig(DVec<double> & config, DVec<double> & config_vel){
    Vec3<double> body_ori_so3;
    body_ori_so3 = ori::quatToso3(_simulator->getState().bodyOrientation);
    for(size_t i(0); i<3; ++i){
        config_vel[i] = _simulator->getState().bodyVelocity[i];
        config_vel[i + 3] = _simulator->getState().bodyVelocity[i+3];

        config[i] = body_ori_so3[i];
        config[i+3] = _simulator->getState().bodyPosition[i];
    }

    for(size_t i(0);i<cheetah::num_act_joint; ++i){
        config[i+6] = _simulator->getState().q[i];
        config_vel[i+6] = _simulator->getState().qd[i];
    }
}
void DynSimulation::_updateGraphics() {
    _window->_drawList.updateRobotFromModel(*_simulator, _robotID, true);
    _window->_drawList.updateAdditionalInfo(*_simulator);
}


void DynSimulation::_SetupParam(){
}

void DynSimulation::_tau_update() {

    // Low level control (if needed)
    //if(_currentSimTime >= _timeOfNextLowLevelControl) {
    //lowLevelControl();
    //_timeOfNextLowLevelControl = _timeOfNextLowLevelControl + dtLowLevelControl;
    //}

    // High level control
    //if(_currentSimTime >= _timeOfNextHighLevelControl) {
    //#ifndef DISABLE_HIGH_LEVEL_CONTROL
    //highLevelControl();
    //#endif
    //_timeOfNextHighLevelControl = _timeOfNextHighLevelControl + dtHighLevelControl;
    //}

    // actuator model:
    for(int leg = 0; leg < 4; leg++) {
        for(int joint = 0; joint < 3; joint++) {
            _tau[leg*3 + joint] = _actuatorModels[joint].getTorque(
                    _torque_cmd[3*leg + joint], 
                    _simulator->getState().qd[leg*3 + joint]);
        }
    }

    // dynamics
    //_currentSimTime += dt;
    //_simulator->step(dt, _tau, _simParams.floor_kp, _simParams.floor_kd);
}


void DynSimulation::run() {

    while(true) {
        //double dtLowLevelControl = _simParams.low_level_dt;
        //double dtHighLevelControl = _simParams.high_level_dt;
        //_desiredSimSpeed = _simParams.simulation_speed;
        //u64 nStepsPerFrame = (u64)(((1. / 60.) / dt) * _desiredSimSpeed);
        //if(!_window->IsPaused()) {
        //step_simulator();
        //} else {
        //usleep((u32)(1e6));
        //}
        //if(frameTimer.getSeconds() > frameTime) {
        //double realElapsedTime = frameTimer.getSeconds();
        //frameTimer.start();
        //double simRate = (_currentSimTime - lastSimTime) / realElapsedTime;
        //lastSimTime = _currentSimTime;
        //sprintf(_window->infoString, "[Simulation Run %5.2fx]\n"
        //"real-time:  %8.3f\n"
        //"sim-time:   %8.3f\n"
        //"rate:       %8.3f\n", _desiredSimSpeed, freeRunTimer.getSeconds(),
        //_currentSimTime, simRate);
        _window->update();
        usleep(1000);
        //if(!_window->IsPaused() && (desiredSteps - steps) < nStepsPerFrame)
        //desiredSteps += nStepsPerFrame;
        //}
    }
}


void DynSimulation::_updateOutput(
        const DVec<double> & full_config, const DVec<double> & full_vel, 
        simulation_to_python & output){
    for(size_t i(0); i<cheetah::dim_config; ++i){
        output.config[i] = full_config[i];
        output.config_vel[i] = full_vel[i];
    }
}

