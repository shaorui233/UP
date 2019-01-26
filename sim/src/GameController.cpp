/*! @file GameController.cpp
 *  @brief Code to read the Logitech F310 Game Controller
 *  Creates a DriverCommand object to be sent to the robot controller
 *  Used in the development simulator and in the robot control mode
 *
 *  NOTE: Because QT is weird, the updateDriverCommand has to be called from a QT event.
 *  Running it in another thread will cause it to not work.
 *  As a result, this only works if called in the update method of a QTObject
 */

#include "GameController.h"

#include <QtCore/QObject>
#include <QtGamepad/QGamepad>

/*!
 * By default, the game controller selects the "first" joystick, printing a warning if there are multiple joysticks
 * On Linux, this is /dev/input/js0
 * If no joystick is found, it will print an error message, and will return zero.
 * It is possible to change/add a joystick later with findNewController
 */
GameController::GameController(QObject *parent) : QObject(parent) {
  findNewController();
}

/*!
 * Re-run the joystick finding code to select the "first" joystick. This can be used to set up the joystick if the
 * simulator is started without a joystick plugged in
 */
void GameController::findNewController() {
  delete _qGamepad;
  _qGamepad = nullptr; // in case this doesn't work!

  auto gamepadList = QGamepadManager::instance()->connectedGamepads();
  if(gamepadList.empty()) {
    printf("[ERROR: GameController] No controller was connected! All joystick commands will be zero!\n");
  } else {
    if(gamepadList.size() > 1) {
      printf("[ERROR: GameController] There are %d joysticks connected.  Using the first one.\n", gamepadList.size());
    } else {
      printf("[GameController] Found 1 joystick\n");
    }

    _qGamepad = new QGamepad(*gamepadList.begin());
  }
}

/*!
 * Overwrite a driverCommand with the current joystick state.  If there's no joystick, sends zeros
 * TODO: what happens if the joystick is unplugged?
 */
void GameController::updateDriverCommand(DriverCommand &driverCommand) {
  if(_qGamepad) {
    driverCommand.leftBumper = _qGamepad->buttonL1();
    driverCommand.rightBumper = _qGamepad->buttonR1();
    driverCommand.leftTriggerButton = _qGamepad->buttonL2() != 0.;
    driverCommand.rightTriggerButton = _qGamepad->buttonR2() != 0.;
    driverCommand.back = _qGamepad->buttonSelect();
    driverCommand.start = _qGamepad->buttonStart();
    driverCommand.a = _qGamepad->buttonA();
    driverCommand.b = _qGamepad->buttonB();
    driverCommand.x = _qGamepad->buttonX();
    driverCommand.y = _qGamepad->buttonY();
    driverCommand.leftStickButton = _qGamepad->buttonL3();
    driverCommand.rightStickButton = _qGamepad->buttonR3();
    driverCommand.leftTriggerAnalog = (float)_qGamepad->buttonL2();
    driverCommand.rightTriggerAnalog = (float)_qGamepad->buttonR2();
    driverCommand.leftStickAnalog = Vec2<float>(_qGamepad->axisLeftX(), -_qGamepad->axisLeftY());
    driverCommand.rightStickAnalog = Vec2<float>(_qGamepad->axisRightX(), -_qGamepad->axisRightY());
  } else {
    driverCommand.zero(); // no joystick, return all zeros
  }

  // printf("%s\n", driverCommand.toString().c_str());

}

GameController::~GameController() {
  delete _qGamepad;
}