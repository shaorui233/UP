#include <cstring>
#include <thread>
#include <sys/mman.h>
#include <unistd.h>
#include "HardwareBridge.h"
#include "rt/rt_vectornav.h"

/*!
 * If an error occurs during initialization, before motors are enabled, print error and exit.
 * @param reason Error message string
 * @param printErrno If true, also print C errno
 */
void HardwareBridge::initError(const char *reason, bool printErrno) {
  printf("FAILED TO INITIALIZE HARDWARE: %s\n", reason);

  if(printErrno) {
    printf("Error: %s\n", strerror(errno));
  }

  exit(-1);
}

/*!
 * All initialization code that is common between Cheetah 3 and Mini Cheetah
 */
void HardwareBridge::initCommon() {
  prefaultStack();
  setupScheduler();
}

/*!
 * Writes to a 16 KB buffer on the stack. If we are using 4K pages for our stack, this will make
 * sure that we won't have a page fault when the stack grows.  Also mlock's all pages associated with the current
 * process, which prevents the cheetah software from being swapped out.  If we do run out of memory, the robot
 * program will be killed by the OOM process killer (and leaves a log) instead of just becoming unresponsive.
 */
void HardwareBridge::prefaultStack() {
  printf("[Init] Prefault stack...\n");
  volatile  char stack[MAX_STACK_SIZE];
  memset(const_cast<char*>(stack), 0, MAX_STACK_SIZE);
  if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    initError("mlockall failed.  This is likely because you didn't run robot as root.\n", true);
  }
}

/*!
 * Configures the
 */
void HardwareBridge::setupScheduler() {
  printf("[Init] Setup RT Scheduler...\n");
  struct sched_param params;
  params.sched_priority = TASK_PRIORITY;
  if(sched_setscheduler(0, SCHED_FIFO, &params) == -1) {
    initError("sched_setscheduler failed.\n", true);
  }
}

MiniCheetahHardwareBridge::MiniCheetahHardwareBridge()
{

}

void MiniCheetahHardwareBridge::run() {
  initCommon();
  initHardware();

  _robotController = new RobotController;

//  _robotController->driverCommand = &_sharedMemory().simToRobot.gamepadCommand;
//  _robotController->spiData       = &_sharedMemory().simToRobot.spiData;
//  _robotController->tiBoardData   = _sharedMemory().simToRobot.tiBoardData;
//  _robotController->robotType     = _robot;
//  _robotController->kvhImuData    = &_sharedMemory().simToRobot.kvh;
//  _robotController->vectorNavData = &_sharedMemory().simToRobot.vectorNav;
//  _robotController->cheaterState  = &_sharedMemory().simToRobot.cheaterState;
//  _robotController->spiCommand    = &_sharedMemory().robotToSim.spiCommand;
//  _robotController->tiBoardCommand = _sharedMemory().robotToSim.tiBoardCommand;
//  _robotController->controlParameters = &_robotParams;
//  _robotController->visualizationData = &_sharedMemory().robotToSim.visualizationData;

  _robotController->initialize();
  _firstRun = false;

  statusTask.start();

  for(;;) {
    usleep(1000000);
  }
}

void MiniCheetahHardwareBridge::initHardware() {
  init_vectornav();
  // init spi
  // init sbus
  // init lidarlite
  // init LCM
  // init LCM read thread
  // init LCM logging thread
  // init control thread

  //
}

