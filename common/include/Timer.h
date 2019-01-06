//
// Created by jared on 1/6/19.
//

#ifndef PROJECT_TIMER_H
#define PROJECT_TIMER_H

#include <time.h>
#include <stdint.h>
#include <assert.h>

class Timer {
public:
  explicit Timer() {
    start();
  }

  void start() {
    clock_gettime(CLOCK_MONOTONIC, &_startTime);
  }

  double getMs() {
    return (double)getNs() / 1.e6;
  }

  uint64_t getNs() {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if ((now.tv_nsec - _startTime.tv_nsec) < 0) {
      return (uint64_t)(now.tv_nsec - _startTime.tv_nsec + 1000000000);
    } else {
      return (uint64_t)(now.tv_nsec - _startTime.tv_nsec);
    }
  }

  double getSeconds() {
    return (double)getNs() / 1.e9;
  }

  struct timespec _startTime;
};

#endif //PROJECT_TIMER_H
