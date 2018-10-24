#include <iostream>
#include <main.h>

MasterConfig gMasterConfig;

/*!
 * Print a message describing the command line flags for the robot program
 */
void printUsage() {
  printf("Usage: robot [robot-id] [sim-or-robot]\n\twhere robot-id:     3 for cheetah 3, m for mini-cheetah\n\t      sim-or-robot: s for sim, r for robot\n");
}

int main(int argc, char** argv) {

  if(argc != 3) {
    printUsage();
    return EXIT_FAILURE;
  }

  if(argv[1][0] == '3') {
    gMasterConfig._robot = RobotType::CHEETAH_3;
  } else if(argv[1][0] == 'm') {
    gMasterConfig._robot = RobotType::MINI_CHEETAH;
  } else {
    printUsage();
    return EXIT_FAILURE;
  }

  if(argv[2][0] == 's') {
    gMasterConfig.simulated = true;
  } else if(argv[2][0] == 'r') {
    gMasterConfig.simulated = false;
  } else {
    printUsage();
    return EXIT_FAILURE;
  }

  printf("[Startup] Cheetah Software\n");

  return 0;
}