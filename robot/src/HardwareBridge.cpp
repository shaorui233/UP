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


void MiniCheetahHardwareBridge::run() {
  initCommon();
  initHardware();

  statusTask.start();
  for(;;) std::this_thread::yield();
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


void MiniCheetahHardwareBridge::abort(const char *reason) {
  printf("!!!!!!!!\n");
  printf("ABORT: %s\n", reason);
  printf("!!!!!!!!!\n");
}

void MiniCheetahHardwareBridge::abort(const std::string &reason) {
  abort(reason.c_str());
}