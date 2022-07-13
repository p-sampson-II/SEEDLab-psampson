// Counter.h
// Uses time steps to wait for (approximately) the correct time while we let the system do other things.
// Author: Paul Sampson

#ifndef COUNTER_H
#define COUNTER_H

class Counter {
  public:
    Counter(double d): duration(d), elapsed(0), isComplete(0) {}
    void start() {
      isStarted = 1;
    }
    void count(double dt) {
      if (elapsed < duration) {
        if (isStarted) elapsed += dt;
      }
      else isComplete = true;
    }
    void reset() {
      elapsed = 0;
      isComplete = 0;
      isStarted = 0;
    }
    bool getIsComplete() {
      return isComplete;
    }
    bool getIsStarted() {
      return isStarted;
    }
    double getElapsed() {
      return elapsed;
    }
  protected:
    double duration;
    double elapsed;
    bool isComplete;
    bool isStarted;
};

# endif
