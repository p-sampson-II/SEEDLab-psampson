// PosCtrl.cpp
// The control system for the motor, which controls position.
// Author: Paul Sampson

#include "PosCtrl.h"

// Removes the necessity to include cmath
float abs(float input) {
  if (input >= 0) return input;
  else return -input;
}

// Checks if there is approximately zero error in position
bool PosCtrl::isError0() {
  if(error[1] < ERROR_BOUNDS && error[1] > -ERROR_BOUNDS){
    return true;
  }
  else return false;
}

// Checks if there is an issue with the connection to the encoder, allowing the
// user to stop the motor if necessary.
bool PosCtrl::isFault() {
  // If abs(voltage) goes up but error stays the same, there may be a problem.
  if(abs(voltage[1]) > abs(voltage[0]) && error[1] == error[0]) return true;
  return false;
}

// Calculate the new voltage to send to the motor
void PosCtrl::control(float *posDesired, float *pos, double *period, bool integral) {
  error[1] = *posDesired-*pos;
  totalError += error[1];

  if(integral) voltage[1] = Kp*error[1]+Ki*(*period)*totalError+voltage[0];
  else voltage[1] = Kp*error[1]+voltage[0];
}

// Bring the controller to the present by updating it on what has happened to
// time and position
void PosCtrl::tick(float *posDesired, float *pos, double *period, bool integral) {
  moveFrame();
  control(posDesired, pos, period, integral);
}

// copy the new variables ([1]) to old ([0]), clear the new variables ([1]), 
void PosCtrl::moveFrame() {
  theta[0] = theta[1];
  error[0] = error[1];
  theta[1] = 0, error[1] = 0;
}

// Give the user read access to the present value of these variables:

float PosCtrl::getTheta() {
  return theta[1];
}

float PosCtrl::getError() {
  return error[1];
}

float PosCtrl::getVoltage() {
  return voltage[1];
}

// Completely clear all states of the controller.
void PosCtrl::reset() {
  for(int i = 0; i < 2; i++) {
    theta[i]=0;
    error[i]=0;
    voltage[i]=0;
    totalError = 0;
  }
}
