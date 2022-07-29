#include "VelCtrl.h"
#include "constants.h"

void VelCtrl::control(float & velocity_desired, double & dt, float & velL, float & velR) {
  rhodot[1] = (velL+velR)/2;
  error[1] = velocity_desired - rhodot[1];
  //if(error[1] < 0) error[1] = 0;

  statevar[1] = statevar[0] + dt*error[0];

  voltage[1] = Ki * statevar[1] + Kp * error[1];
}

void VelCtrl::tick(float & velocity_desired, double & dt, float & velL, float & velR) {
  moveFrame();
  control(velocity_desired, dt, velL, velR);
}

void VelCtrl::moveFrame() {
  rhodot[0] = rhodot[1], statevar[0] = statevar[1], error[0] = error[1],
    rhodot[1] = 0, statevar[1] = 0, error[1] = 0;
}

float VelCtrl::getRhodot() {
  return rhodot[1];
}

float VelCtrl::getError() {
  return error[1];
}

float VelCtrl::getVoltage() {
  return voltage[1];
}

