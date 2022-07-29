#include "Counter.h"

namespace timers {
  void Counter::start() {
      isStarted = 1;
    }
    void Counter::count(double dt) {
      if (elapsed < duration) {
        if (isStarted) elapsed += dt;
      }
      else isComplete = true;
    }
    void Counter::reset() {
      elapsed = 0;
      isComplete = 0;
      isStarted = 0;
    }
    bool Counter::getIsComplete() {
      return isComplete;
    }
    bool Counter::getIsStarted() {
      return isStarted;
    }
    double Counter::getElapsed() {
      return elapsed;
    }
}