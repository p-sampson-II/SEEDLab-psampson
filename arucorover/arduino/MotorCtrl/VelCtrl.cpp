#include "VelCtrl.h"
#include "constants.h"

void Vel_Ctrl::control(float* velocity_desired, double* delta_t, float* vel_l, float* vel_r) {
  rhodot[1] = (*vel_l+*vel_r)/2;
  rhodot_error[1] = *velocity_desired - rhodot[1];
  //if(rhodot_error[1] < 0) rhodot_error[1] = 0;
  
  rhodot_statevar[1] = rhodot_statevar[0] + (*delta_t)*rhodot_error[0];
  
  mag_v[1] = Ki * rhodot_statevar[1] + Kp * rhodot_error[1];
}

void Vel_Ctrl::tick(float* velocity_desired, double* delta_t, float* vel_l, float* vel_r) {
  move_frame();
  control(velocity_desired, delta_t, vel_l, vel_r);
}

void Vel_Ctrl::move_frame() {
  rhodot[0] = rhodot[1];
  rhodot_statevar[0] = rhodot_statevar[1];
  rhodot_error[0] = rhodot_error[1];
  rhodot[1], rhodot_statevar[1], rhodot_error[1] = 0;
}

float Vel_Ctrl::get_rhodot() {
  return rhodot[1];
}

float Vel_Ctrl::get_rhodot_error() {
  return rhodot_error[1];
}
