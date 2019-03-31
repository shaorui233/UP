#ifndef PROJECT_HARDWAREBRIDGE_H
#define PROJECT_HARDWAREBRIDGE_H


#define MAX_STACK_SIZE 16384 // 16KB  of stack
#define TASK_PRIORITY 49

#include <string>
#include "Utilities/PeriodicTask.h"
#include "RobotController.h"


class HardwareBridge {
public:
  HardwareBridge() : statusTask(&taskManager, 0.5f) { }
  void prefaultStack();
  void setupScheduler();
  void addPeriodicTask(void* func, uint64_t periodNs);
  void initError(const char* reason, bool printErrno = false);
  void initCommon();
  ~HardwareBridge() {
    delete _robotController;
  }

protected:
  PeriodicTaskManager taskManager;
  PrintTaskStatus     statusTask;
  bool _firstRun = true;
  RobotController* _robotController = nullptr;
  RobotControlParameters _robotParams;
  u64 _iterations = 0;

};

class MiniCheetahHardwareBridge : public HardwareBridge {
public:
  MiniCheetahHardwareBridge();
  void initHardware();
  void run();
  void abort(const std::string& reason);
  void abort(const char* reason);







};

#endif //PROJECT_HARDWAREBRIDGE_H
