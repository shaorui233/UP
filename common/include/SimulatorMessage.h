/*! @file SimulatorMessage.h
 *  @brief Messages sent to/from the development simulator
 *
 *  These messsages contain all data that is exchanged between the robot program and the simulator
 *  using shared memory.   This is basically everything except for debugging logs, which are handled by LCM instead
 */

#ifndef PROJECT_SIMULATORTOROBOTMESSAGE_H
#define PROJECT_SIMULATORTOROBOTMESSAGE_H

#include "SharedMemory.h"
#include "DriverCommand.h"

/*!
 * A plain message from the simulator to the robot
 */
struct SimulatorToRobotMessage {
  char burp[20];
  DriverCommand driverCommand;
  RobotType robotType;

  // todo add these:
  // RobotControlParameters
  // leg data to robot
  // imu data to robot
  // cheater data to robot
};

/*!
 * A plain message from the robot to the simulator
 */
struct RobotToSimulatorMessage {
  RobotType robotType;

  // todo add these:
  // leg command from robot
  // visualization data
  // RobotControlParameters
};

/*!
 * All the data shared between the robot and the simulator
 */
struct SimulatorMessage {
  RobotToSimulatorMessage robotToSim;
  SimulatorToRobotMessage simToRobot;
};

/*!
 * A SimulatorSyncronizedMessage is stored in shared memory and is accessed by both the simulator and the robot
 * The simulator and robot take turns have exclusive access to the entire message.
 * The intended sequence is:
 *  - robot: waitForSimulator()
 *  - simulator: *simulates robot* (simulator can read/write, robot cannot do anything)
 *  - simulator: simDone()
 *  - simulator: waitForRobot()
 *  - robot: *runs controller*    (robot can read/write, simulator cannot do anything)
 *  - robot: robotDone();
 *  - robot: waitForSimulator()
 *  ...
 */
struct SimulatorSyncronizedMessage : public SimulatorMessage {

  /*!
   * The init() method should only be called *after* shared memory is connected!
   */
  void init() {
    robotToSimSemaphore.init(0);
    simToRobotSemaphore.init(0);
  }

  void waitForSimulator() {
    simToRobotSemaphore.decrement();
  }

  void simulatorIsDone() {
    simToRobotSemaphore.increment();
  }

  void waitForRobot() {
    robotToSimSemaphore.decrement();
  }

  void robotIsDone() {
    robotToSimSemaphore.increment();
  }

private:
  SharedMemorySemaphore robotToSimSemaphore, simToRobotSemaphore;
};

#endif //PROJECT_SIMULATORTOROBOTMESSAGE_H
