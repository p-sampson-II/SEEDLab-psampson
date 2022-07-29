#include "Counter.h"
#include "Osc.h"

namespace timers{
  Osc::Osc(double on, double off) {
    for(int i = 0; i <= 1; i++) {
      this->counters[i] = Counter(i);
      this->counters[i].start();
    }
  }
  void Osc::force(bool state) {
    this->counters[!state].reset();
    isOn[0] = isOn[1];
    isOn[1] = state;
  }
  void Osc::tick(double dt) {
    isOn[0] = isOn[1];
    if(counters[isOn[1]].getIsComplete()) {
      counters[isOn[1]].reset();
      this->counters[isOn[1]].start();
      isOn[1] = !isOn[1];
    }
    else
      counters[isOn[1]].count(dt);
  }
  void Osc::reset() {
    for(int i = 0; i <= 1; i++) counters[i].reset();
  }
  bool Osc::getIsOn() {
    return isOn[1];
  }
  bool Osc::isRisingEdge() {
    if(isOn[0] == 0 && isOn[1] == 1) return true;
    else return false;
  }
}