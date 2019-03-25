/*========================= Gamepad Control ==========================*/
/*
 *
 */

#include "SimUtilities/GamepadControl.h"




template <typename T>
void GamepadControl<T>::convertToStateCommands() {


}






template <typename T>
void GamepadControl<T>::printRawInfo() {

  // Increment printing iteration
  printIter++;

  // Print at requested frequency
  if (printIter == printNum) {

    std::cout << "[GAMEPAD COMMAND] Printing Raw Info...\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Left Analog X: " << gamepadCommand->leftStickAnalog[0] << " | Y: " << gamepadCommand->leftStickAnalog[1] << "\n";
    std::cout << "Right Analog X: " << gamepadCommand->rightStickAnalog[0] << " | Y: " << gamepadCommand->rightStickAnalog[1] << "\n";
    std::cout << "Button A: " << gamepadCommand->a << " | B: " << gamepadCommand->b << " | X: " << gamepadCommand->x << " | Y: " << gamepadCommand->y << "\n";
    std::cout << std::endl;
      
    /*"Result:\nleftBumper: " + boolToString(leftBumper) + "\n" +
    "rightBumper: " + boolToString(rightBumper) + "\n" +
    "leftTriggerButton: " + boolToString(leftTriggerButton) + "\n" +
    "rightTriggerButton: " + boolToString(rightTriggerButton) + "\n" +
    "back: " + boolToString(back) + "\n" +
    "start: " + boolToString(start) + "\n" +
    "a: " + boolToString(a) + "\n" +
    "b: " + boolToString(b) + "\n" +
    "x: " + boolToString(x) + "\n" +
    "y: " + boolToString(y) + "\n" +
    "leftStickButton: " + boolToString(leftStickButton) + "\n" +
    "rightStickButton: " + boolToString(rightStickButton) + "\n" +
    "leftTriggerAnalog: " + std::to_string(leftTriggerAnalog) + "\n" +
    "rightTriggerAnalog: " + std::to_string(rightTriggerAnalog) + "\n" +
    "leftStickAnalog: " + eigenToString(leftStickAnalog) + "\n" +
    "rightStickAnalog: " + eigenToString(rightStickAnalog) + "\n";*/
    std::cout << std::endl;

    // Reset iteration counter
    printIter = 0;
  }
}

template <typename T>
void GamepadControl<T>::printStateCommandInfo() {

  // Increment printing iteration
  printIter++;

  // Print at requested frequency
  if (printIter == printNum) {

    std::cout << "[GAMEPAD COMMAND] Printing Raw Info...\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Left Analog X: " << gamepadCommand->leftStickAnalog[0] << " | Y: " << gamepadCommand->leftStickAnalog[1] << "\n";
    std::cout << "Right Analog X: " << gamepadCommand->rightStickAnalog[0] << " | Y: " << gamepadCommand->rightStickAnalog[1] << "\n";std::cout << std::endl;
    std::cout << std::endl;

    // Reset iteration counter
    printIter = 0;
  }
}


template class GamepadControl<double>;
template class GamepadControl<float>;