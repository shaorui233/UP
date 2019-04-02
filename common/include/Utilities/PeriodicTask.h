#ifndef PROJECT_PERIODICTASK_H
#define PROJECT_PERIODICTASK_H

#include <vector>
#include <string>
#include <thread>


class PeriodicTaskManager;



class PeriodicTask {
public:
  PeriodicTask(PeriodicTaskManager* taskManager, float period, std::string name);
  void start();
  void stop();
  void printStatus();
  void clearMax();
  bool isSlow();
  virtual void init() = 0;
  virtual void run() = 0;
  virtual void cleanup() = 0;
  virtual ~PeriodicTask() {
    stop();
  }

  float getPeriod() {
    return _period;
  }

  float getRuntime() {
    return _lastRuntime;
  }

  float getMaxPeriod() {
    return _maxPeriod;
  }

  float getMaxRuntime() {
    return _maxRuntime;
  }

private:
  void loopFunction();

  float _period;
  volatile bool _running = false;
  float _lastRuntime = 0;
  float _lastPeriodTime = 0;
  float _maxPeriod = 0;
  float _maxRuntime = 0;
  std::string _name;
  std::thread _thread;
};

class PeriodicTaskManager {
public:
  PeriodicTaskManager() = default;
  ~PeriodicTaskManager();
  void addTask(PeriodicTask* task);
  void printStatus();
  void printStatusOfSlowTasks();
  void stopAll();

private:
  std::vector<PeriodicTask*> _tasks;
};


class PeriodicFunction : public PeriodicTask {
public:
  PeriodicFunction(PeriodicTaskManager* taskManager, float period,
                   std::string name, void (*function)()) : PeriodicTask(taskManager, period, name), _function(function) {
  }
  void cleanup() { }
  void init() { }
  void run() {
    _function();
  }

  ~PeriodicFunction() {
    stop();
  }
private:
  void (*_function)() = nullptr;
};


class PrintTaskStatus : public PeriodicTask {
public:
  PrintTaskStatus(PeriodicTaskManager* tm, float period) : PeriodicTask(tm, period, "print-tasks"), _tm(tm) { }
  void run() override {
    _tm->printStatus();
  }

  void init() override {

  }

  void cleanup() override {

  }
private:
  PeriodicTaskManager* _tm;
};


#endif //PROJECT_PERIODICTASK_H
