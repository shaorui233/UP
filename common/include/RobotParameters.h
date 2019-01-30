/*! @file RobotParameters.cpp
 *  @brief Declaration of various robot parameters
 *
 *  This class contains all the ControlParameters for the robot.
 */


#ifndef PROJECT_ROBOTPARAMETERS_H
#define PROJECT_ROBOTPARAMETERS_H

#include "ControlParameters.h"

class RobotControlParameters : public ControlParameters {
public:

  RobotControlParameters() :
          ControlParameters("robot-parameters"),
          INIT_PARAMETER(myValue),
          INIT_PARAMETER(testValue),
          INIT_PARAMETER(controller_dt)
  {

  }



  DECLARE_PARAMETER(double, myValue)
  DECLARE_PARAMETER(double, testValue)
  DECLARE_PARAMETER(double, controller_dt)
};

#endif //PROJECT_ROBOTPARAMETERS_H
