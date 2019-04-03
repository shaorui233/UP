/*========================= Gamepad Control ==========================*/
/*
 *
 */

#include "Controllers/DesiredStateCommand.h"

/*=========================== Gait Data ===============================*/
/*
 *
 */
template <typename T>
void DesiredStateData<T>::zero() {

  // Overall desired state
  stateDes = Vec12<T>::Zero();
  stateTrajDes = Eigen::Matrix<T, 12, 10>::Zero();
}

template struct DesiredStateData<double>;
template struct DesiredStateData<float>;






template <typename T>
void DesiredStateCommand<T>::convertToStateCommands() {

  data.zero();

  velXDes = deadband(gamepadCommand->leftStickAnalog[1], minVelX, maxVelX);
  velYDes = deadband(gamepadCommand->leftStickAnalog[0], minVelY, maxVelY);
  angVelZDes = deadband(gamepadCommand->rightStickAnalog[0], minTurnRate, maxTurnRate);
  pitchDes = deadband(gamepadCommand->rightStickAnalog[1], minPitch, maxPitch);

  posXDes = stateEstimate->position(0) + dt * velXDes;
  posYDes = stateEstimate->position(1) + dt * velYDes;
  posZDes = 0.45;

  rollDes = 0.0;
  yawDes = stateEstimate->rpy(2) + dt * angVelZDes;

  velZDes = 0.0;

  angVelXDes = 0.0;
  angVelYDes = 0.0;


  // Add the values to the desired state data
  data.stateDes(0) = posXDes;
  data.stateDes(1) = posYDes;
  data.stateDes(2) = posZDes;
  data.stateDes(3) = rollDes;
  data.stateDes(4) = pitchDes;
  data.stateDes(5) = yawDes;
  data.stateDes(6) = velXDes;
  data.stateDes(7) = velYDes;
  data.stateDes(8) = velZDes;
  data.stateDes(9) = angVelXDes;
  data.stateDes(10) = angVelYDes;
  data.stateDes(11) = angVelZDes;
}



template <typename T>
void DesiredStateCommand<T>::desiredStateTrajectory(int N, Vec10<T> dtVec) {
  A = Mat12<T>::Zero();
  A(0, 0) = 1;
  A(1, 1) = 1;
  A(2, 2) = 1;
  A(3, 3) = 1;
  A(4, 4) = 1;
  A(5, 5) = 1;
  A(6, 6) = 1;
  A(7, 7) = 1;
  A(8, 8) = 1;
  A(9, 9) = 1;
  A(10, 10) = 1;
  A(11, 11) = 1;
  data.stateTrajDes.col(0) = data.stateDes;

  for (int k = 1; k < N; k++) {
    A(0, 6) = dtVec(k - 1);
    A(1, 7) = dtVec(k - 1);
    A(2, 8) = dtVec(k - 1);
    A(3, 9) = dtVec(k - 1);
    A(4, 10) = dtVec(k - 1);
    A(5, 11) = dtVec(k - 1);
    data.stateTrajDes.col(k) = A * data.stateTrajDes.col(k - 1);
    for (int i = 0; i < 12; i++) {
      std::cout << data.stateTrajDes(i, k) << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;

}



template <typename T>
float DesiredStateCommand<T>::deadband(float command, T minVal, T maxVal) {
  if (command < deadbandRegion && command > -deadbandRegion) {
    return 0.0;
  } else {
    return (command / (2)) * (maxVal - minVal);
  }
}



template <typename T>
void DesiredStateCommand<T>::printRawInfo() {

  // Increment printing iteration
  printIter++;

  // Print at requested frequency
  if (printIter == printNum) {

    std::cout << "[DESIRED STATE COMMAND] Printing Raw Gamepad Info...\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Button Start: " << gamepadCommand->start << " | Back: " << gamepadCommand->back << "\n";
    std::cout << "Button A: " << gamepadCommand->a << " | B: " << gamepadCommand->b << " | X: " << gamepadCommand->x << " | Y: " << gamepadCommand->y << "\n";
    std::cout << "Left Stick Button: " << gamepadCommand->leftStickButton << " | X: " << gamepadCommand->leftStickAnalog[0] << " | Y: " << gamepadCommand->leftStickAnalog[1] << "\n";
    std::cout << "Right Analog Button: " << gamepadCommand->rightStickButton << " | X: " << gamepadCommand->rightStickAnalog[0] << " | Y: " << gamepadCommand->rightStickAnalog[1] << "\n";
    std::cout << "Left Bumper: " << gamepadCommand->leftBumper << " | Trigger Switch: " << gamepadCommand->leftTriggerButton << " | Trigger Value: " << gamepadCommand->leftTriggerAnalog << "\n";
    std::cout << "Right Bumper: " << gamepadCommand->rightBumper << " | Trigger Switch: " << gamepadCommand->rightTriggerButton << " | Trigger Value: " << gamepadCommand->rightTriggerAnalog << "\n\n";
    std::cout << std::endl;

    // Reset iteration counter
    printIter = 0;
  }
}

template <typename T>
void DesiredStateCommand<T>::printStateCommandInfo() {

  // Increment printing iteration
  printIter++;

  // Print at requested frequency
  if (printIter == printNum) {

    std::cout << "[DESIRED STATE COMMAND] Printing State Command Info...\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Position X: " << data.stateDes(0) << " | Y: " << data.stateDes(1) << " | Z: " << data.stateDes(2) << "\n";
    std::cout << "Orientation Roll: " << data.stateDes(3) << " | Pitch: " << data.stateDes(4) << " | Yaw: " << data.stateDes(5) << "\n";
    std::cout << "Velocity X: " << data.stateDes(6) << " | Y: " << data.stateDes(7) << " | Z: " << data.stateDes(8) << "\n";
    std::cout << "Angular Velocity X: " << data.stateDes(9) << " | Y: " << data.stateDes(10) << " | Z: " << data.stateDes(11) << "\n"; std::cout << std::endl;
    std::cout << std::endl;

    // Reset iteration counter
    printIter = 0;
  }
}


template class DesiredStateCommand<double>;
template class DesiredStateCommand<float>;