/*! @file RobotParameters.cpp
 *  @brief Declaration of various robot parameters
 *
 *  This class contains all the ControlParameters for the robot.
 */


#ifndef PROJECT_ROBOTPARAMETERS_H
#define PROJECT_ROBOTPARAMETERS_H

#include "ControlParameters/ControlParameters.h"

class RobotControlParameters : public ControlParameters {
public:

  RobotControlParameters() :
          ControlParameters("robot-parameters"),
          INIT_PARAMETER(myValue),
          INIT_PARAMETER(testValue),
          INIT_PARAMETER(controller_dt),
          INIT_PARAMETER(stand_kp_cartesian),
          INIT_PARAMETER(stand_kd_cartesian)
  {

  }

  DECLARE_PARAMETER(double, myValue)
  DECLARE_PARAMETER(double, testValue)
  DECLARE_PARAMETER(double, controller_dt)
  DECLARE_PARAMETER(Vec3<double>, stand_kp_cartesian)
  DECLARE_PARAMETER(Vec3<double>, stand_kd_cartesian)
};

#endif //PROJECT_ROBOTPARAMETERS_H
