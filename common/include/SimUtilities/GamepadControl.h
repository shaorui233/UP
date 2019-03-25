/*========================= Gamepad Control ==========================*/
/*
 *
 */
#ifndef GAMEPAD_CONTROL_H
#define GAMEPAD_CONTROL_H

#include "SimUtilities/GamepadCommand.h"
#include <iostream>

template <typename T>
class GamepadControl
{
public:
  // Initialize with the GamepadCommand struct
  GamepadControl(GamepadCommand *command) : gamepadCommand(command)  { }

  void convertToStateCommands();
  void printRawInfo();
  void printStateCommandInfo();

private:
  GamepadCommand *gamepadCommand;

  // Choose how often to print info, every N iterations
  int printNum = 4;	// N*(0.001s) in simulation time

  // Track the number of iterations since last info print
  int printIter = 0;

};

#endif