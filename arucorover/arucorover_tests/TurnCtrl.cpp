#include "TurnCtrl.h"
#include "constants.h"

void TurnCtrl::control(float & angleDesired, double & dt, float & velL, float & velR) {
  phi[1] = phi[0] + (dt)*(4*wheel_radius/wheel_span)*(velL-velR)/2;
  error[1] = camera_Kp*(angleDesired) - phi[1];
  totalError += error[1]; 
  statevar[1] = statevar[0] + dt * error[0];
  if (voltage[1] <= v_max)
    voltage[1] = Kp * error[1] + Ki * totalError;
  else
    voltage[1] = voltage[0];
}

void TurnCtrl::tick(float & angleDesired, double & dt, float & velL, float & velR) {
  moveFrame();
  float stopGapVelL = -velL;
  control(angleDesired, dt, stopGapVelL, velR);
}

void TurnCtrl::moveFrame() {
  phi[0] = phi[1];
  statevar[0] = statevar[1];
  statevarD[0] = statevarD[1];
  error[0] = error[1];
  phi[1], statevar[1], statevarD[1], error[1] = 0;
}

bool TurnCtrl::isError0() {
  if (error[1] < error_bounds && error[1] > -error_bounds){
    error[1] = 0;
    totalError = 0;
    return true;
  }
  else return false;
}

float TurnCtrl::getPhi() {
  return phi[1];
}

float TurnCtrl::getError() {
  return error[1];
}

float TurnCtrl::getVoltage() {
  return voltage[1];
}

void TurnCtrl::reset() {
  phi[0] = 0;
  statevar[0] = 0;
  statevarD[0] = 0;
  error[0] = 0;
  phi[1] = 0;
  statevar[1] = 0;
  statevarD[1] = 0;
  error[1] = 0;
}
