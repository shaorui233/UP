#include "Controllers/OrientationEstimator.h"
#include "RobotController.h"
#include "Dynamics/Cheetah3.h"
#include "Dynamics/MiniCheetah.h"
#include "Controllers/ContactEstimator.h"

#include <Utilities/Utilities_print.h>
#include <WBC_States/BodyCtrl/BodyCtrlTest.hpp>
#include <WBC_States/JPosCtrl/JPosCtrlTest.hpp>
#include <WBC_States/OptPlay/OptPlayTest.hpp>
#include <WBC_States/PlannedTrot/PlannedTrotTest.hpp>


void RobotController::initialize() {
  printf("[RobotController] initialize\n");
  if(robotType == RobotType::MINI_CHEETAH) {
    _quadruped = buildMiniCheetah<float>();
  } else {
    _quadruped = buildCheetah3<float>();
  }

  _legController = new LegController<float>(_quadruped);
  _stateEstimator = new StateEstimatorContainer<float>(
          cheaterState, kvhImuData, vectorNavData,
          _legController->datas, &_stateEstimate, controlParameters);
  initializeStateEstimator(false);

  // Initialize a new GaitScheduler object
  _gaitScheduler = new GaitScheduler<double>();
  _gaitScheduler->initialize();

  // For WBC state
  _model = _quadruped.buildModel();
  //_wbc_state = new BodyCtrlTest<float>(&_model, robotType);
  //_wbc_state = new JPosCtrlTest<float>(&_model, robotType);
  //_wbc_state = new OptPlayTest<float>(&_model, robotType);
  _wbc_state = new PlannedTrotTest<float>(&_model, robotType);

  _data = new Cheetah_Data<float>();
  _extra_data = new Cheetah_Extra_Data<float>();
}


void RobotController::step() {
  setupStep();
  _stateEstimator->run();
  //testDebugVisualization();
  StepLocationVisualization();
  BodyPathVisualization();
  BodyPathArrowVisualization();

  // for now, we will always enable the legs:
  _legController->setEnabled(true);
  _legController->setMaxTorqueCheetah3(208.5);
 
  // Find the current gait schedule
  //_gaitScheduler->step();

  // ======= WBC state command computation  =============== //
  for(size_t i(0); i<4; ++i){
      _data->body_ori[i] = cheaterState->orientation[i];
  }
  for(int i(0);i<3; ++i){
      _data->ang_vel[i] = cheaterState->omegaBody[i];
      _data->global_body_pos[i] = cheaterState->position[i];
  }
  _data->global_body_pos[2] += 0.5;// because ground is -0.5

  for(int leg(0); leg<4; ++leg){
      for(int jidx(0); jidx<3; ++jidx){
          _data->jpos[3*leg + jidx] = _legController->datas[leg].q[jidx];
          _data->jvel[3*leg + jidx] = _legController->datas[leg].qd[jidx];
      }
  }
  _data->dir_command[0] = driverCommand->leftStickAnalog[1];
  _data->dir_command[1] = driverCommand->leftStickAnalog[0];
  
  // Orientation
  _data->ori_command[0] = driverCommand->rightTriggerAnalog;
  _data->ori_command[0] -= driverCommand->leftTriggerAnalog;

  _data->ori_command[1] = driverCommand->rightStickAnalog[1];
  _data->ori_command[2] = driverCommand->rightStickAnalog[0];

  //pretty_print(_data->ori_command, "ori command", 3);

  _wbc_state->GetCommand(_data, _legController->commands, _extra_data);
  // === End of WBC state command computation  =========== //

  // run the controller:
  Mat3<float> kpMat; kpMat << controlParameters->stand_kp_cartesian[0], 0, 0,
  0, controlParameters->stand_kp_cartesian[1], 0,
  0, 0, controlParameters->stand_kp_cartesian[2];

  Mat3<float> kdMat; kdMat << controlParameters->stand_kd_cartesian[0], 0, 0,
  0, controlParameters->stand_kd_cartesian[1], 0,
  0, 0, controlParameters->stand_kd_cartesian[2];

  for(int leg = 0; leg < 4; leg++) {
    //_legController->commands[leg].pDes = pDes;
    //_legController->commands[leg].kpCartesian = kpMat;
    //_legController->commands[leg].kdCartesian = kdMat;

    _legController->commands[leg].kpJoint = kpMat;
    _legController->commands[leg].kdJoint = kdMat;

  }

  finalizeStep();
}

void RobotController::StepLocationVisualization(){
  // Cones
  //visualizationData->num_cones = 20*2;
  int num_step = _extra_data->num_step;
  visualizationData->num_cones = num_step;
  for (int j(0) ; j < num_step  ; j++)
  {
    ConeVisualization cone;
    cone.radius = 0.03;
    cone.direction << 0, 0 , 0.05;
    //cone.direction += .2f * j *Vec3<float>(sinf(.4f * j * t + j * .2f), cosf(.4f * j * t + j * .2f), 0);
    cone.point_position <<  
        _extra_data->loc_x[j], 
        _extra_data->loc_y[j], 
        (_extra_data->loc_z[j] - 0.5);  // Ground is -0.5
    cone.color << .6 , .2 ,  .4, .6;
    visualizationData->cones[j] = cone;
  }
}
void RobotController::BodyPathVisualization(){
  // Test Path visualization
  PathVisualization path;
  path.num_points = _extra_data->num_path_pt;
  for (size_t j = 0 ; j < path.num_points ; j++)
  {
    path.position[j] << 
        _extra_data->path_x[j],
        _extra_data->path_y[j],
        _extra_data->path_z[j]-0.5; //Ground is -0.5
  }
  //pretty_print(_extra_data->path_x, "path x", path.num_points);
  //pretty_print(_extra_data->path_y, "path y", path.num_points);
  path.color << 0.6 ,  0.2, 0.05 ,  1;

  visualizationData->num_paths = 1;
  visualizationData->paths[0] = path;
}

void RobotController::BodyPathArrowVisualization(){
    visualizationData->num_arrows = _extra_data->num_middle_pt;

    double ar_len(0.07);
    double yaw;
    for(int i(0); i<_extra_data->num_middle_pt; ++i){
        visualizationData->arrows[i].base_position << 
            _extra_data->mid_x[i], _extra_data->mid_y[i], _extra_data->mid_z[i] -0.5 ;
        //visualizationData->arrows[i].direction << 
            //_extra_data->mid_ori_roll[i], 
            //_extra_data->mid_ori_pitch[i], 
            //_extra_data->mid_ori_yaw[i];

        yaw = _extra_data->mid_ori_yaw[i];

        visualizationData->arrows[i].direction << ar_len*cos(yaw), ar_len*(sin(yaw)), 0.;

        visualizationData->arrows[i].head_width = 0.02;
        visualizationData->arrows[i].head_length = 0.03;
        visualizationData->arrows[i].shaft_width = 0.01;

        visualizationData->arrows[i].color << 
            0.8, 0.3, 0.1, 1;
    }
}

void RobotController::testDebugVisualization() {
  // Todo, move some of this into the robot controller.

  float t = (float)_iterations / 1000;
  // Test sphere visualization
  SphereVisualization sphere;
  sphere.position[0] = 0; sphere.position[1] = 5 * std::sin(t); sphere.position[2] = 5 * std::cos(t);
  sphere.radius = 1;
  sphere.color[0] = 1; sphere.color[1] = 1; sphere.color[2] = 0; sphere.color[3] = 1;
  visualizationData->num_spheres = 1;
  visualizationData->spheres[0] = sphere;


  // Cones
  visualizationData->num_cones = 5;
  for (size_t j = 0 ; j < 5  ; j++)
  {
    ConeVisualization cone;
    cone.radius = (j+1)/10.;
    cone.direction << 0, 0 , .3*j+.2;
    cone.direction += .2f * j *Vec3<float>(sinf(.4f * j * t + j * .2f), cosf(.4f * j * t + j * .2f), 0);
    cone.point_position <<  3+j, 3,  0;
    cone.color << .4 , .1 ,  .2, .6;
    visualizationData->cones[j] = cone;
  }

  // boxes 
  visualizationData->num_blocks = 1;
  BlockVisualization block;
  block.corner_position << -5, -5,  0;
  block.dimension <<  1, .2, .3;
  block.color<<  1,  0,  0, std::fmod((float)_iterations / 1000, 1.f) ;
  visualizationData->blocks[0] = block;


  // Test Arrow Visualization
  visualizationData->num_arrows = 1;

  visualizationData->arrows[0].base_position << 1,1, 1;

  visualizationData->arrows[0].direction << 1,1,1;

  visualizationData->arrows[0].head_width = 0.1;
  visualizationData->arrows[0].head_length = 0.2;
  visualizationData->arrows[0].shaft_width = 0.05;

  visualizationData->arrows[0].color << std::fmod((float)_iterations / 1000 + 0.5f, 1.f), std::fmod((float)_iterations / 1000, 1.f) , 1, .6;

  // Test Path visualization
  PathVisualization path;
  path.num_points = 150;
  for (size_t j = 0 ; j < path.num_points ; j++)
  {
    path.position[j] << j*2.0/ path.num_points, sin(j*10. / path.num_points + ((float) _iterations) / 1000) ,  .5;
  }
  path.color << 0 ,  0, 1 ,  1;

  visualizationData->num_paths = 1;
  visualizationData->paths[0] = path;
}


void RobotController::setupStep() {
  // leg data
  if(robotType == RobotType::MINI_CHEETAH) {
    _legController->updateData(spiData);
  } else if(robotType == RobotType::CHEETAH_3) {
    _legController->updateData(tiBoardData);
  } else {
    assert(false);
  }
  _legController->zeroCommand();

  // state estimator
  // check transition to cheater mode:
  if(!_cheaterModeEnabled && controlParameters->cheater_mode) {
    printf("[RobotController] Transitioning to Cheater Mode...\n");
    initializeStateEstimator(true);
    // todo any configuration
    _cheaterModeEnabled = true;
  }

  // check transition from cheater mode:
  if(_cheaterModeEnabled && !controlParameters->cheater_mode) {
    printf("[RobotController] Transitioning from Cheater Mode...\n");
    initializeStateEstimator(false);
    // todo any configuration
    _cheaterModeEnabled = false;
  }


  // todo safety checks, sanity checks, etc...
}


void RobotController::finalizeStep() {
  if(robotType == RobotType::MINI_CHEETAH) {
    _legController->updateCommand(spiCommand);
  } else if(robotType == RobotType::CHEETAH_3) {
    _legController->updateCommand(tiBoardCommand);
  } else {
    assert(false);
  }
  _iterations++;
}

void RobotController::initializeStateEstimator(bool cheaterMode) {
  _stateEstimator->removeAllEstimators();
  if(cheaterMode) {
    _stateEstimator->addEstimator<CheaterOrientationEstimator<float>>();
  } else if(robotType == RobotType::MINI_CHEETAH) {
    _stateEstimator->addEstimator<VectorNavOrientationEstimator<float>>();
  } else if(robotType == RobotType::CHEETAH_3) {
    _stateEstimator->addEstimator<KvhOrientationEstimator<float>>();
  } else {
    assert(false);
  }
}

RobotController::~RobotController() {
  delete _legController;
  delete _stateEstimator;
}
