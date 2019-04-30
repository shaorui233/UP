
// Checks the robot state and commands for safety
#include "SafetyChecker.h"


/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 *
 * @return the appropriate operating mode
 */
template <typename T>
bool SafetyChecker<T>::safetyPostCheck() {

  // Check for safe desired foot positions
  if (true){//currentState->checkPDesFoot) {
    T maxAngle = 1.0472;  // 60 degrees (should be changed)
    T maxPDes = data->_quadruped->_maxLegLength * sin(maxAngle);

    // Check all of the legs
    for (int leg = 0; leg < 4; leg++) {
      // Keep the foot from going too far from the body in +x
      if (data->_legController->commands[leg].pDes(0) > maxPDes) {
        std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg << " | coordinate: " << 0 << "\n";
        std::cout << "   commanded: " << data->_legController->commands[leg].pDes(0) << " | modified: " << maxPDes << std::endl;
        data->_legController->commands[leg].pDes(0) = maxPDes;
      }

      // Keep the foot from going too far from the body in -x
      if (data->_legController->commands[leg].pDes(0) < -maxPDes) {
        std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg << " | coordinate: " << 0 << "\n";
        std::cout << "   commanded: " << data->_legController->commands[leg].pDes(0) << " | modified: " << -maxPDes << std::endl;
        data->_legController->commands[leg].pDes(0) = -maxPDes;
      }

      // Keep the foot from going too far from the body in +y
      if (data->_legController->commands[leg].pDes(1) > maxPDes) {
        std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg << " | coordinate: " << 1 << "\n";
        std::cout << "   commanded: " << data->_legController->commands[leg].pDes(1) << " | modified: " << maxPDes << std::endl;
        data->_legController->commands[leg].pDes(1) = maxPDes;
      }

      // Keep the foot from going too far from the body in -y
      if (data->_legController->commands[leg].pDes(1) < -maxPDes) {
        std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg << " | coordinate: " << 1 << "\n";
        std::cout << "   commanded: " << data->_legController->commands[leg].pDes(1) << " | modified: " << -maxPDes << std::endl;
        data->_legController->commands[leg].pDes(1) = -maxPDes;
      }

      // Keep the leg under the motor module (don't raise above body or crash into module)
      if (data->_legController->commands[leg].pDes(2) > -data->_quadruped->_maxLegLength / 4) {
        std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg << " | coordinate: " << 2 << "\n";
        std::cout << "   commanded: " << data->_legController->commands[leg].pDes(2) << " | modified: " << -data->_quadruped->_maxLegLength / 4 << std::endl;
        data->_legController->commands[leg].pDes(2) = -data->_quadruped->_maxLegLength / 4;
      }

      // Keep the foot within the kinematic limits
      if (data->_legController->commands[leg].pDes(2) < -data->_quadruped->_maxLegLength) {
        std::cout << "[CONTROL FSM] Safety: PDes leg: " << leg << " | coordinate: " << 2 << "\n";
        std::cout << "   commanded: " << data->_legController->commands[leg].pDes(2) << " | modified: " << -data->_quadruped->_maxLegLength << std::endl;
        data->_legController->commands[leg].pDes(2) = -data->_quadruped->_maxLegLength;
      }
    }

  }

  // Check for safe desired feedforward forces
  if (true){//currentState->checkForceFeedForward) {
    // Initialize maximum vertical and lateral forces
    T maxLateralForce = 0;
    T maxVerticalForce = 0;

    // Maximum force limits for each robot
    if (data->_quadruped->_robotType == RobotType::CHEETAH_3) {
      maxLateralForce = 1800;
      maxVerticalForce = 1800;

    } else if (data->_quadruped->_robotType == RobotType::MINI_CHEETAH) {
      maxLateralForce = 350;
      maxVerticalForce = 350;
    }

    // Check all of the legs
    for (int leg = 0; leg < 4; leg++) {
      // Limit the lateral forces in +x body frame
      if (data->_legController->commands[leg].forceFeedForward(0) > maxLateralForce) {
        std::cout << "[CONTROL FSM] Safety: Force leg: " << leg << " | coordinate: " << 0 << "\n";
        std::cout << "   commanded: " << data->_legController->commands[leg].forceFeedForward(0) << " | modified: " << maxLateralForce << std::endl;
        data->_legController->commands[leg].forceFeedForward(0) = maxLateralForce;
      }

      // Limit the lateral forces in -x body frame
      if (data->_legController->commands[leg].forceFeedForward(0) < -maxLateralForce) {
        std::cout << "[CONTROL FSM] Safety: Force leg: " << leg << " | coordinate: " << 0 << "\n";
        std::cout << "   commanded: " << data->_legController->commands[leg].forceFeedForward(0) << " | modified: " << -maxLateralForce << std::endl;
        data->_legController->commands[leg].forceFeedForward(0) = -maxLateralForce;
      }

      // Limit the lateral forces in +y body frame
      if (data->_legController->commands[leg].forceFeedForward(1) > maxLateralForce) {
        std::cout << "[CONTROL FSM] Safety: Force leg: " << leg << " | coordinate: " << 1 << "\n";
        std::cout << "   commanded: " << data->_legController->commands[leg].forceFeedForward(1) << " | modified: " << maxLateralForce << std::endl;
        data->_legController->commands[leg].forceFeedForward(1) = maxLateralForce;
      }

      // Limit the lateral forces in -y body frame
      if (data->_legController->commands[leg].forceFeedForward(1) < -maxLateralForce) {
        std::cout << "[CONTROL FSM] Safety: Force leg: " << leg << " | coordinate: " << 1 << "\n";
        std::cout << "   commanded: " << data->_legController->commands[leg].forceFeedForward(1) << " | modified: " << -maxLateralForce << std::endl;
        data->_legController->commands[leg].forceFeedForward(1) = -maxLateralForce;
      }

      // Limit the vertical forces in +z body frame
      if (data->_legController->commands[leg].forceFeedForward(2) > maxVerticalForce) {
        std::cout << "[CONTROL FSM] Safety: Force leg: " << leg << " | coordinate: " << 2 << "\n";
        std::cout << "   commanded: " << data->_legController->commands[leg].forceFeedForward(2) << " | modified: " << -maxVerticalForce << std::endl;
        data->_legController->commands[leg].forceFeedForward(2) = maxVerticalForce;
      }

      // Limit the vertical forces in -z body frame
      if (data->_legController->commands[leg].forceFeedForward(2) < -maxVerticalForce) {
        std::cout << "[CONTROL FSM] Safety: Force leg: " << leg << " | coordinate: " << 2 << "\n";
        std::cout << "   commanded: " << data->_legController->commands[leg].forceFeedForward(2) << " | modified: " << maxVerticalForce << std::endl;
        data->_legController->commands[leg].forceFeedForward(2) = -maxVerticalForce;
      }
    }
  }

  // Default is to return the current operating mode
  return true;

}



//template class SafetyChecker<double>; This should be fixed... need to make RobotController a template
template class SafetyChecker<float>;