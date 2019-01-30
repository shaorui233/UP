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
          INIT_PARAMETER(kvh_imu_accelerometer_noise),
          INIT_PARAMETER(kvh_imu_gyro_noise),
          INIT_PARAMETER(vectornav_imu_accelerometer_noise),
          INIT_PARAMETER(vectornav_imu_gyro_noise),
          INIT_PARAMETER(vectornav_imu_quat_noise),
          INIT_PARAMETER(game_controller_deadband),
          INIT_PARAMETER(simulation_speed),
          INIT_PARAMETER(simulation_paused),
          INIT_PARAMETER(high_level_dt),
          INIT_PARAMETER(low_level_dt),
          INIT_PARAMETER(dynamics_dt)
  { }


  DECLARE_PARAMETER(float, kvh_imu_accelerometer_noise)
  DECLARE_PARAMETER(float, kvh_imu_gyro_noise)
  DECLARE_PARAMETER(float, vectornav_imu_accelerometer_noise)
  DECLARE_PARAMETER(float, vectornav_imu_gyro_noise)
  DECLARE_PARAMETER(float, vectornav_imu_quat_noise)

  DECLARE_PARAMETER(float, game_controller_deadband)

  DECLARE_PARAMETER(double, simulation_speed)
  DECLARE_PARAMETER(s64, simulation_paused)
  DECLARE_PARAMETER(double, high_level_dt)
  DECLARE_PARAMETER(double, low_level_dt)
  DECLARE_PARAMETER(double, dynamics_dt)
};

#endif //PROJECT_SIMULATORPARAMETERS_H
