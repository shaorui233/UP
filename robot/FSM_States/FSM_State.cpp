/*============================= FSM State =============================*/
/**
 * FSM State base class
 */

#include "FSM_State.h"

/**
 * Constructor for the FSM State class.
 *
 * @param _controlFSMData holds all of the relevant control data
 * @param stateNameIn the enumerated state name
 * @param stateStringIn the string name of the current FSM state
 */
template <typename T>
FSM_State<T>::FSM_State(ControlFSMData<T>* _controlFSMData,
                        FSM_StateName stateNameIn,
                        std::string stateStringIn):
  _data(_controlFSMData),
  stateName(stateNameIn),
  stateString(stateStringIn) {
  std::cout << "[FSM_State] Initialized FSM state: " <<  stateStringIn << std::endl;
}


/**
 * Cartesian impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param qDes desired joint position
 * @param dqDes desired joint velocity
 */
template <typename T>
void FSM_State<T>::jointPDControl(int leg, Vec3<T> qDes, Vec3<T> qdDes) {

  _data->_legController->commands[leg].qDes = qDes;
  // Create the cartesian P gain matrix
  //controlParameters->stand_kp_cartesian[0]
  kpMat << 500, 0, 0,
        0, 500, 0,
        0, 0, 500;
  _data->_legController->commands[leg].kpJoint = kpMat;

  _data->_legController->commands[leg].qdDes = qdDes;
  // Create the cartesian D gain matrix
  //controlParameters->stand_kd_cartesian[0]
  kdMat << 10, 0, 0,
        0, 10, 0,
        0, 0, 10;
  _data->_legController->commands[leg].kdJoint = kdMat;

}


/**
 * Cartesian impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param pDes desired foot position
 * @param vDes desired foot velocity
 */
template <typename T>
void FSM_State<T>::cartesianImpedanceControl(int leg, Vec3<T> pDes, Vec3<T> vDes) {

  _data->_legController->commands[leg].pDes = pDes;
  // Create the cartesian P gain matrix
  kpMat << _data->controlParameters->stand_kp_cartesian[0], 0, 0,
        0, _data->controlParameters->stand_kp_cartesian[1], 0,
        0, 0, _data->controlParameters->stand_kp_cartesian[2];
  _data->_legController->commands[leg].kpCartesian = kpMat;

  _data->_legController->commands[leg].vDes = vDes;
  // Create the cartesian D gain matrix
  kdMat << _data->controlParameters->stand_kd_cartesian[0], 0, 0,
        0, _data->controlParameters->stand_kd_cartesian[1], 0,
        0, 0, _data->controlParameters->stand_kd_cartesian[2];
  _data->_legController->commands[leg].kdCartesian = kdMat;

}


/*
 *
 */
/*
template <typename T>
void FSM_State<T>::footstepHeuristicPlacement(int leg) {

  // Create the projection matrix for the 2D foot placement components
  Mat23<float> projectionMatrix;
  projectionMatrix << 1, 0, 0,
                   0, 1, 0;

  Vec3<float> velDes = _desiredStateCommand->data.stateDes.block<3, 1>(6, 0);
  Vec3<float> angVelDes = _desiredStateCommand->data.stateDes.block<3, 1>(9, 0);
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
*/


/*
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 */
template <typename T>
void FSM_State<T>::runControls() {
  // This option should be set from the user interface or autonomously eventually
  //int CONTROLLER_OPTION = 0;

  // Reset the forces and steps to 0
  groundReactionForces = Mat34<float>::Zero();
  footstepLocations = Mat34<float>::Zero();
  /*
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
  */

}


//template class FSM_State<double>;
template class FSM_State<float>;