#ifndef PROJECT_HARDWAREBRIDGE_H
#define PROJECT_HARDWAREBRIDGE_H


#define MAX_STACK_SIZE 16384 // 16KB  of stack
#define TASK_PRIORITY 49

#include <string>
#include "Utilities/PeriodicTask.h"


class HardwareBridge {
public:
  HardwareBridge() : statusTask(&taskManager, 0.5f) { }
  void prefaultStack();
  void setupScheduler();
  void addPeriodicTask(void* func, uint64_t periodNs);
  void initError(const char* reason, bool printErrno = false);
  void initCommon();

protected:
  PeriodicTaskManager taskManager;
  PrintTaskStatus     statusTask;

};

class MiniCheetahHardwareBridge : public HardwareBridge {
public:
  void initHardware();
  void run();
  void abort(const std::string& reason);
  void abort(const char* reason);

private:
  // periodic task functions



};

#endif //PROJECT_HARDWAREBRIDGE_H
