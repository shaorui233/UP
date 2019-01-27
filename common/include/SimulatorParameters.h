/*! @file SimulatorParameters.cpp
 *  @brief Declaration of various simulator parameters
 *
 *  This class contains all the ControlParameters for the simulator.
 *  In most cases, the simulator just loads the control parameters in simulator-defaults.ini and this is okay
 */


#ifndef PROJECT_SIMULATORPARAMETERS_H
#define PROJECT_SIMULATORPARAMETERS_H

#include "ControlParameters.h"

#define SIMULATOR_DEFAULT_PARAMETERS "/simulator-defaults.ini"
#define MINI_CHEETAH_DEFAULT_PARAMETERS "/mini-cheetah-defaults.ini"

class SimulatorControlParameters : public ControlParameters {
public:

  SimulatorControlParameters() :
          ControlParameters("simulator-parameters"),
          kvh_imu_accelerometer_noise_param("kvh_imu_accelerometer_noise", kvh_imu_accelerometer_noise, collection),
          kvh_imu_gyro_noise_param("kvh_imu_gyro_noise", kvh_imu_gyro_noise, collection),
          vectornav_imu_accelerometer_noise_param("vectornav_imu_accelerometer_noise", vectornav_imu_accelerometer_noise, collection),
          vectornav_imu_gyro_noise_param("vectornav_imu_gyro_noise", vectornav_imu_gyro_noise, collection),
          vectornav_imu_quat_noise_param("vectornav_imu_quat_noise", vectornav_imu_quat_noise, collection),
          game_controller_deadband_param("game_controller_deadband", game_controller_deadband, collection)

  {

  }





  float kvh_imu_accelerometer_noise;
  ControlParameter kvh_imu_accelerometer_noise_param;

  float kvh_imu_gyro_noise;
  ControlParameter kvh_imu_gyro_noise_param;

  float vectornav_imu_accelerometer_noise;
  ControlParameter vectornav_imu_accelerometer_noise_param;

  float vectornav_imu_gyro_noise;
  ControlParameter vectornav_imu_gyro_noise_param;

  float vectornav_imu_quat_noise;
  ControlParameter vectornav_imu_quat_noise_param;

  float game_controller_deadband;
  ControlParameter game_controller_deadband_param;

};

#endif //PROJECT_SIMULATORPARAMETERS_H
