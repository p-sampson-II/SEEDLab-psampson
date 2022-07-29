#include "Counter.h"

#ifndef OSC_H
#define OSC_H

namespace timers {
  class Osc {
    public:
      Osc(double on, double off);
      void force(bool state);
      void tick(double dt);
      void reset();
      bool getIsOn();
      bool isRisingEdge();
    protected:
      Counter counters[2];
      bool isOn[2];
  };
}

#endif // OSC_H