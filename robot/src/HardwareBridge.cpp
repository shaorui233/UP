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
  printf("[HardwareBridge] Init stack\n");
  prefaultStack();
  printf("[HardwareBridge] Init scheduler\n");
  setupScheduler();
  if(!_interfaceLCM.good()) {
    initError("_interfaceLCM failed to initialize\n", false);
  }

  printf("[HardwareBridge] Subscribe LCM\n");
  _interfaceLCM.subscribe("interface", &HardwareBridge::handleGamepadLCM, this);

  printf("[HardwareBridge] Start interface LCM handler\n");
  _interfaceLcmThread = std::thread(&HardwareBridge::handleInterfaceLCM, this);
}

void HardwareBridge::handleInterfaceLCM() {
  while(!_interfaceLcmQuit)
    _interfaceLCM.handle();
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

void HardwareBridge::handleGamepadLCM(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                                      const gamepad_lcmt *msg) {
  _gamepadCommand.set(msg);
}

MiniCheetahHardwareBridge::MiniCheetahHardwareBridge()
{

}

void MiniCheetahHardwareBridge::run() {
  initCommon();
  initHardware();

  _robotController = new RobotController;

  _robotController->driverCommand = &_gamepadCommand;
//  _robotController->spiData       = &_sharedMemory().simToRobot.spiData;

  _robotController->robotType     = RobotType::MINI_CHEETAH;
  _robotController->vectorNavData = &_vectorNavData;

//  _robotController->spiCommand    = &_sharedMemory().robotToSim.spiCommand;

//  _robotController->controlParameters = &_robotParams;
//  _robotController->visualizationData = &_sharedMemory().robotToSim.visualizationData;

  _robotController->initialize();
  _firstRun = false;

  // init control thread

  statusTask.start();

  for(;;) {
    usleep(1000000);
  }
}

void MiniCheetahHardwareBridge::initHardware() {
  printf("[MiniCheetahHardware] Init vectornav\n");
  init_vectornav(&_vectorNavData);
  // init spi
  // init sbus
  // init lidarlite

  // init LCM hardware logging thread


  //
}

