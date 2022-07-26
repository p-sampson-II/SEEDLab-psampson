// Counter.h
// Uses time steps to wait for (approximately) the correct time while we let the system do other things.
// Author: Paul Sampson

#ifndef COUNTER_H
#define COUNTER_H

namespace timers {
  class Counter {
  public:
    Counter(): duration(0), elapsed(0), isComplete(0) {}
    Counter(double d): duration(d), elapsed(0), isComplete(0) {}
    Counter(Counter &other): duration(other.duration), elapsed(other.elapsed), isComplete(other.isComplete) {}
    void start();
    void count(double dt);
    void reset();
    bool getIsComplete();
    bool getIsStarted();
    double getElapsed();
    friend class Osc;
  protected:
    double duration;
    double elapsed;
    bool isComplete;
    bool isStarted;
  };
}



# endif
