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
          myValueParam("myValue", myValue, collection),
          testValueParam("testValue", testValue, collection)
  {

  }



  double myValue;
  ControlParameter myValueParam;

  double testValue;
  ControlParameter testValueParam;
};

#endif //PROJECT_ROBOTPARAMETERS_H
