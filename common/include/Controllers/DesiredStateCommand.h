/*========================= Gamepad Control ==========================*/
/*
 *
 */
#ifndef DESIRED_STATE_COMMAND_H
#define DESIRED_STATE_COMMAND_H

#include "SimUtilities/GamepadCommand.h"
#include <iostream>
#include <cppTypes.h>

/*
 *
 */
template <typename T>
struct DesiredStateData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DesiredStateData() {
    zero();
  }

  // Zero out all of the data
  void zero();

  Vec12<T> stateDes;
};


/*
 *
 */
template <typename T>
class DesiredStateCommand
{
public:
  // Initialize with the GamepadCommand struct
  DesiredStateCommand(GamepadCommand *command) : gamepadCommand(command)  { }

  void convertToStateCommands();
  void printRawInfo();
  void printStateCommandInfo();
  float deadband(float command, T minVal, T maxVal);

  T maxRoll = 0.4;
  T minRoll = -0.4;
  T maxPitch = 0.4;
  T minPitch = -0.4;
  T maxVelX = 3.0;
  T minVelX = -3.0;
  T maxVelY = 2.0;
  T minVelY = -2.0;
  T maxTurnRate = 4.0;
  T minTurnRate = -4.0;

  T posXDes;
  T posYDes;
  T posZDes;
  T rollDes;
  T pitchDes;
  T yawDes;
  T velXDes;
  T velYDes;
  T velZDes;
  T angVelXDes;
  T angVelYDes;
  T angVelZDes;

  DesiredStateData<T> data;

private:
  GamepadCommand *gamepadCommand;

  // Control loop timestep change
  T dt = 0.001;

  // value cutoff for the analog stick deadband
  T deadbandRegion = 0.075;

  // Choose how often to print info, every N iterations
  int printNum = 4; // N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 0;

};

#endif