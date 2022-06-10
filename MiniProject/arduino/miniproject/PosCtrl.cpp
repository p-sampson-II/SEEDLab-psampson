#include "PosCtrl.h"
#include <math.h>

float abs(float input) {
  if (input >= 0) return input;
  else return -input;
}

bool PosCtrl::isError0() {
  if(error[1] < ERROR_BOUNDS && error[1] > -ERROR_BOUNDS){
    return true;
  }
  else return false;
}

bool PosCtrl::isFault() {
  if(abs(voltage[1]) > abs(voltage[0]) && error[1] == error[0]) return true;
  return false;
}

void PosCtrl::control(float *posDesired, float *pos, double *period, bool integral) {
  error[1] = *posDesired-*pos;
  totalError += error[1];

  if(integral) voltage[1] = Kp*error[1]+Ki*(*period)*totalError+voltage[0];
  else voltage[1] = Kp*error[1]+voltage[0];
}

void PosCtrl::tick(float *posDesired, float *pos, double *period, bool integral) {
  moveFrame();
  control(posDesired, pos, period, integral);
}

void PosCtrl::moveFrame() {
  theta[0] = theta[1];
  error[0] = error[1];
  theta[1] = 0, error[1] = 0;
}

float PosCtrl::getTheta() {
  return theta[1];
}

float PosCtrl::getError() {
  return error[1];
}

void PosCtrl::reset() {
  for(int i = 0; i < 2; i++) {
    theta[i]=0;
    error[i]=0;
    voltage[i]=0;
    totalError = 0;
  }
}
