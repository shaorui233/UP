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
#include <WBC_States/WBDCTrot/WBDCTrotTest.hpp>
#include <WBC_States/WBLCTrot/WBLCTrotTest.hpp>


void RobotController::initialize() {
    printf("[RobotController] initialize\n");
    if (robotType == RobotType::MINI_CHEETAH) {
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
    _gaitScheduler = new GaitScheduler<float>(_quadruped);
    _gaitScheduler->initialize();

    _desiredStateCommand = new DesiredStateCommand<float>(driverCommand, &_stateEstimate);

    // Initialize a new ContactEstimator object
    //_contactEstimator = new ContactEstimator<double>();
    //_contactEstimator->initialize();

    // For WBC state
    _model = _quadruped.buildModel();
    _wbc_state = new BodyCtrlTest<float>(&_model, robotType);
    //_wbc_state = new JPosCtrlTest<float>(&_model, robotType);
    //_wbc_state = new OptPlayTest<float>(&_model, robotType);
    //_wbc_state = new PlannedTrotTest<float>(&_model, robotType);
    //_wbc_state = new WBDCTrotTest<float>(&_model, robotType);
    //_wbc_state = new WBLCTrotTest<float>(&_model, robotType);

    _data = new Cheetah_Data<float>();
    _extra_data = new Cheetah_Extra_Data<float>();
}


void RobotController::step() {
    setupStep();
    _stateEstimator->run();
    testDebugVisualization();
    StepLocationVisualization();
    BodyPathVisualization();
    BodyPathArrowVisualization();

    // for now, we will always enable the legs:
    _legController->setEnabled(true);
    _legController->setMaxTorqueCheetah3(208.5);

    // for debugging the visualizations from robot code


    // DH: Test
    // Find the current gait schedule
    //_gaitScheduler->step();

    // ======= WBC state command computation  =============== //
    // Commenting out WBC for now to test Locomotion control
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

    // Find the current gait schedule
    _gaitScheduler->step();

    // Temporary fix for testing 
    //_stateEstimate.position(0) = cheaterState->position(0);
    //_stateEstimate.position(1) = cheaterState->position(1);
    //_stateEstimate.position(2) = cheaterState->position(2);

    // Find the desired state trajectory
    _desiredStateCommand->convertToStateCommands();
    //Vec10<float> dtVec;
    //dtVec << 0.1,0.1,0.1,0.1,0.1,0,0,0,0,0;
    //_desiredStateCommand->desiredStateTrajectory(5, dtVec);

    // This function should eventually be moved to whatever the Locomotion FSM state ends up being
    //LocomotionControlStep();

    // Sets the leg controller commands for the robot appropriate commands
    finalizeStep();


    // DH: TEST
    //_gaitScheduler->printGaitInfo();
    //_gamepadControl->printRawInfo();
    //_desiredStateCommand->printStateCommandInfo();
}



//---------------------------------------------------------------------------------------------
// This section is just testing for now (should be in FSM_State_Locomotion or something similar)

/*
 * Calculate the commands for the leg controllers for each of the feet.
 */
void RobotController::LocomotionControlStep() {

    // Contact state logic
    //

    // Run the balancing controllers to get GRF and next step locations
    runControls();

    //std::cout << groundReactionForces << std::endl;

    // Calculate appropriate control actions for each leg to be sent out
    for (int leg = 0; leg < 4; leg++) {

        if (_gaitScheduler->gaitData.contactStateScheduled(leg)) {
            // Leg is in contact
            //std::cout << "[CONTROL] Leg " << leg << " is in stance" << std::endl;

            // Impedance control for the stance leg
            //stanceLegImpedanceControl(leg);

            // Stance leg Ground Reaction Force command
            _legController->commands[leg].forceFeedForward = groundReactionForces.col(leg);

        } else if (!_gaitScheduler->gaitData.contactStateScheduled(leg)) {
            // Leg is not in contact
            //std::cout << "[CONTROL] Leg " << leg << " is in swing" << std::endl;

            // Swing leg trajectory
            // TODO

            // Feedforward torques for swing leg tracking
            // TODO

        } else {
            std::cout << "[CONTROL ERROR] Undefined scheduled contact state\n" << std::endl;
        }

        // Singularity barrier calculation (maybe an overall safety checks function?)
        // TODO
    }

}


/*
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 */
void RobotController::runControls() {
    // This option should be set from the user interface or autonomously eventually
    int CONTROLLER_OPTION = 0;

    // Reset the forces and steps to 0
    groundReactionForces = Mat34<float>::Zero();
    footstepLocations = Mat34<float>::Zero();

    // Choose the controller to run for picking step locations and balance forces
    if (CONTROLLER_OPTION == 0) {
        // Test to make sure we can control the robot
        for (int leg = 0; leg < 4; leg++) {
            groundReactionForces.col(leg) << 0.0, 0.0, -110.36;
            groundReactionForces.col(leg) = _stateEstimate.rBody * groundReactionForces.col(leg);

            footstepLocations.col(leg) << 0.3, 0.1, 0.45;
        }
    } else if (CONTROLLER_OPTION == 1) {
        // QP Balance Controller
        runBalanceController();

        // Swing foot landing positions are calculated with heuristics
        footstepHeuristicPlacement();

    } else if (CONTROLLER_OPTION == 2) {
        // WBC
        runWholeBodyController();

    } else if (CONTROLLER_OPTION == 3) {
        // cMPC
        runConvexModelPredictiveController();

        // Swing foot landing positions are calculated with heuristics
        footstepHeuristicPlacement();

    } else if (CONTROLLER_OPTION == 4) {
        // RPC
        runRegularizedPredictiveController();

    } else {
        groundReactionForces = Mat34<float>::Zero();
        footstepLocations = Mat34<float>::Zero();
    }

}


/*
 * Calls the interface for the controller
 */
void RobotController::runBalanceController() {
    // TODO
}


/*
 * Calls the interface for the controller
 */
void RobotController::runWholeBodyController() {
    // TODO
}


/*
 * Calls the interface for the controller
 */
void RobotController::runConvexModelPredictiveController() {
    // TODO
}


/*
 * Calls the interface for the controller
 */
void RobotController::runRegularizedPredictiveController() {
    // TODO
}


/*
 *
 */
void RobotController::footstepHeuristicPlacement() {

    // Create the projection matrix for the 2D foot placement components
    Mat23<float> projectionMatrix;
    projectionMatrix << 1, 0, 0,
                     0, 1, 0;

    Vec3<float> velDes = _desiredStateCommand->data.stateDes.block<3,1>(6,0);
    Vec3<float> angVelDes = _desiredStateCommand->data.stateDes.block<3,1>(9,0);
    Mat3<float> rBody = _stateEstimate.rBody;

    // Find each of the footstep locations for the swing feet
    for (int leg = 0; leg < 4; leg++) {
        if (_gaitScheduler->gaitData.contactStateScheduled(leg)) {
            // The leg is in contact so nothing to do here

        } else {
            // Pull out the approximate yaw rate component of the robot in the world.
            Vec3<float> yaw_rate;
            yaw_rate << 0, 0, _stateEstimate.omegaWorld(3);

            Vec3<float> posHip = _quadruped.getHipLocation(leg);

            float timeStance = _gaitScheduler->gaitData.timeStance(leg);

            // Footstep heuristic composed of several parts in the world frame
            footstepLocations.col(leg) << projectionMatrix.transpose()*projectionMatrix*      // Ground projection
                (_stateEstimate.position +                             //
                 rBody * posHip +                                      // Foot under hips
                 timeStance / 2 * velDes +                             // Raibert Heuristic
                 timeStance / 2 * (angVelDes.cross(rBody * posHip)) +  // Turning Raibert Heuristic
                 (_stateEstimate.vBody - velDes));   
        }
    }

}


/*
 * Find desired foot placement and velocity in the hip frame for the stance legs
 * and their respective cartesian gains.
 */
void RobotController::stanceLegImpedanceControl(int leg) {
    // Hold the leg position at touchdown
    //_legController->commands[leg].pDes = _stateEstimate.rBody.transpose()*(_gaitScheduler->gaitData.posFootTouchdownWorld.col(leg) - _stateEstimate.position) - _quadruped.getHipLocation(int leg);
    _legController->commands[leg].pDes = _stateEstimate.rBody.transpose() * ( - _stateEstimate.position) - _quadruped.getHipLocation(leg);

    // Match the foot velocity in the body frame
    _legController->commands[leg].vDes = -(_stateEstimate.vBody + _stateEstimate.omegaBody.cross(_quadruped.getHipLocation(leg)));

    // Create the cartesian P gain matrix
    Mat3<float> kpMat;
    kpMat << controlParameters->stand_kp_cartesian[0], 0, 0,
          0, controlParameters->stand_kp_cartesian[1], 0,
          0, 0, controlParameters->stand_kp_cartesian[2];
    _legController->commands[leg].kpCartesian = kpMat;

    // Create the cartesian D gain matrix
    Mat3<float> kdMat;
    kdMat << controlParameters->stand_kd_cartesian[0], 0, 0,
          0, controlParameters->stand_kd_cartesian[1], 0,
          0, 0, controlParameters->stand_kd_cartesian[2];
    _legController->commands[leg].kdCartesian = kdMat;
}


// ----------------------------------------------------------------------------------------------




void RobotController::StepLocationVisualization() {
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

  cheetahMainVisualization->q.setZero();
  cheetahMainVisualization->p.setZero();
  cheetahMainVisualization->quat.setZero();
  cheetahMainVisualization->color << 1,1,1,0.5f;
  cheetahMainVisualization->p[2] = sin(t * 10);

  return;


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
