/*============================ Vision =============================*/
/**
 * FSM State for robot locomotion. Manages the contact specific logic
 * and handles calling the interfaces to the controllers. This state
 * should be independent of controller, gait, and desired trajectory.
 */

#include "FSM_State_Vision.h"
#include <Utilities/Timer.h>
#include <Controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_Vision<T>::FSM_State_Vision(
    ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::LOCOMOTION, "LOCOMOTION"),
        cMPCOld(_controlFSMData->controlParameters->controller_dt,
                //30 / (1000. * _controlFSMData->controlParameters->controller_dt),
                25 / (1000. * _controlFSMData->controlParameters->controller_dt),
                _controlFSMData->userParameters)
      //_visionLCM(getLcmUrl(255)),
      //_pointsLCM(getLcmUrl(255))
{
  //_visionLCM.subscribe("local_heightmap", &FSM_State_Vision<T>::handleVisionLCM, this);
  //_visionLCMThread = std::thread(&FSM_State_Vision<T>::visionLCMThread, this);

  //_pointsLCM.subscribe("cf_pointcloud", &FSM_State_Vision<T>::handlePointsLCM, this);
  //_pointsLCMThread = std::thread(&FSM_State_Vision<T>::pointsLCMThread, this);

  //_map = DMat<T>::Zero(x_size, y_size);
  // Set the safety checks
  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;

  // Initialize GRF and footstep locations to 0s
  this->footFeedForwardForces = Mat34<T>::Zero();
  this->footstepLocations = Mat34<T>::Zero();
  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->_quadruped->buildModel());
  _wbc_data = new LocomotionCtrlData<T>();
}

//template <typename T>
//void FSM_State_Vision<T>::handleVisionLCM(const lcm::ReceiveBuffer *rbuf,
                                      //const std::string &chan,
                                      //const heightmap_t *msg) {
  //(void)rbuf;
  //(void)chan;

  //_b_writing = true;
  //for(size_t i(0); i<x_size; ++i){
    //for(size_t j(0); j<y_size; ++j){
         //_map(i,j) = msg->map[i][j];
    //}
  //}
  //_b_writing = false;
//}

//template <typename T>
//void FSM_State_Vision<T>::visionLCMThread() {
  //while (true) {
    //_visionLCM.handle();
    //_b_vision_update = true;
  //}
//}


//template <typename T>
//void FSM_State_Vision<T>::handlePointsLCM(const lcm::ReceiveBuffer *rbuf,
                                      //const std::string &chan,
                                      //const rs_pointcloud_t*msg) {
  //(void)rbuf;
  //(void)chan;

  //for(size_t i(0); i<num_points; ++i){
      //for(size_t j(0); j<3; ++j){
         //_points[i][j] = msg->pointlist[i][j];
      //}
  //}
//}

//template <typename T>
//void FSM_State_Vision<T>::pointsLCMThread() {
  //while (true) {
    //_pointsLCM.handle();
  //}
//}


template <typename T>
void FSM_State_Vision<T>::onEnter() {
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();
  cMPCOld.initialize();
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_Vision<T>::run() {
  //_AddMeshDrawing();
  //_AddPointsDrawing();
  // Call the locomotion control logic for this iteration
  LocomotionControlStep();
}


//template<typename T>
//void FSM_State_Vision<T>::_AddMeshDrawing(){
  //auto* mesh = this->_data->visualizationData->addMesh();
  //if(mesh){
    //Vec3<T> pos = this->_data->_stateEstimator->getResult().position;

    //mesh->left_corner.setZero();
    //mesh->left_corner[0] = -0.75 + pos[0];
    //mesh->left_corner[1] = -0.75 + pos[1];
    //mesh->rows = x_size;
    //mesh->cols = y_size;
    //mesh->grid_size = 0.015;
    //mesh->height_max = 0.7;
    //mesh->height_min = 0.;

      //for(int i(0); i<mesh->rows; ++i){
        //for(int j(0); j<mesh->cols; ++j){
          //mesh->height_map(i,j) = _map(i,j);
        //}
      //}
  //}
//}


//template<typename T>
//void FSM_State_Vision<T>::_AddPointsDrawing(){
  //int num_skip = 5;
  //for(size_t i(0); i<num_points/num_skip; ++i){
    //auto* point = this->_data->visualizationData->addSphere();
    //if(point){
      //point->position = _points[i*num_skip];
      //point->color = {1.0, 0.2, 0.2, 1.0};
      //point->radius = 0.007;
    //} 
  //}
//}


/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template <typename T>
FSM_StateName FSM_State_Vision<T>::checkTransition() {
  // Get the next state
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->controlParameters->control_mode) {
    case K_VISION:
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      this->transitionDuration = 0.0;
      break;

    case K_BALANCE_STAND:
      // Requested change to BALANCE_STAND
      this->nextStateName = FSM_StateName::BALANCE_STAND;

      // Transition time is immediate
      this->transitionDuration = 0.0;

      break;

    case K_PASSIVE:
      // Requested change to BALANCE_STAND
      this->nextStateName = FSM_StateName::PASSIVE;

      // Transition time is immediate
      this->transitionDuration = 0.0;

      break;

    case K_STAND_UP:
      this->nextStateName = FSM_StateName::STAND_UP;
      this->transitionDuration = 0.;
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      this->transitionDuration = 0.;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_VISION << " to "
                << this->_data->controlParameters->control_mode << std::endl;
  }

  // Return the next state name to the FSM
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template <typename T>
TransitionData<T> FSM_State_Vision<T>::transition() {
  // Switch FSM control mode
  switch (this->nextStateName) {
    case FSM_StateName::BALANCE_STAND:
      LocomotionControlStep();

      iter++;
      if (iter >= this->transitionDuration * 1000) {
        this->transitionData.done = true;
      } else {
        this->transitionData.done = false;
      }

      break;

    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();

      this->transitionData.done = true;

      break;

    case FSM_StateName::STAND_UP:
      this->transitionData.done = true;
      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;


    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition"
                << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_Vision<T>::onExit() {
  // Nothing to clean up when exiting
  iter = 0;
}

/**
 * Calculate the commands for the leg controllers for each of the feet by
 * calling the appropriate balance controller and parsing the results for
 * each stance or swing leg.
 */
template <typename T>
void FSM_State_Vision<T>::LocomotionControlStep() {
  // StateEstimate<T> stateEstimate = this->_data->_stateEstimator->getResult();

  // Contact state logic
  cMPCOld.run<T>(*this->_data);

  if(this->_data->userParameters->use_wbc > 0.9){
    _wbc_data->pBody_des = cMPCOld.pBody_des;
    _wbc_data->vBody_des = cMPCOld.vBody_des;
    _wbc_data->aBody_des = cMPCOld.aBody_des;

    _wbc_data->pBody_RPY_des = cMPCOld.pBody_RPY_des;
    _wbc_data->vBody_Ori_des = cMPCOld.vBody_Ori_des;
    
    for(size_t i(0); i<4; ++i){
      _wbc_data->pFoot_des[i] = cMPCOld.pFoot_des[i];
      _wbc_data->vFoot_des[i] = cMPCOld.vFoot_des[i];
      _wbc_data->aFoot_des[i] = cMPCOld.aFoot_des[i];
      _wbc_data->Fr_des[i] = cMPCOld.Fr_des[i]; 
    }
    _wbc_data->contact_state = cMPCOld.contact_state;
    _wbc_ctrl->run(_wbc_data, *this->_data);

  }
}

template class FSM_State_Vision<float>;
