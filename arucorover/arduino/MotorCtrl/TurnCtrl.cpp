#include "TurnCtrl.h"
#include "constants.h"

void Turn_Ctrl::control(float* angle_desired, double* delta_t, float* vel_l, float* vel_r) {
  phi[1] = phi[0] + (*delta_t)*(4*wheel_radius/wheel_span)*(*vel_l-*vel_r)/2;
  phi_error[1] = camera_Kp*(*angle_desired) - phi[1];
  phi_statevar[1] = phi_statevar[0] + *delta_t * phi_error[0];
  if (delta_v[1] <= v_max)
    delta_v[1] = Kp * phi_error[1];
  else
    delta_v[1] = delta_v[0]; 
}

void Turn_Ctrl::tick(float* angle_desired, double* delta_t, float* vel_l, float* vel_r) {
  move_frame();
  control(angle_desired, delta_t, vel_l, vel_r);
}

void Turn_Ctrl::move_frame() {
  phi[0] = phi[1];
  phi_statevar[0] = phi_statevar[1];
  phi_statevar_d[0] = phi_statevar_d[1];
  phi_error[0] = phi_error[1];
  phi[1], phi_statevar[1], phi_statevar_d[1], phi_error[1] = 0;
}

bool Turn_Ctrl::is_error_0() {
  if (phi_error[1] < error_bounds && phi_error[1] > -error_bounds){
    phi_error[1] = 0;
    return true;
  }
  else return false;
}

float Turn_Ctrl::get_phi() {
  return phi[1];
}

float Turn_Ctrl::get_phi_error() {
  return phi_error[1];
}

void Turn_Ctrl::reset_phi() {
  phi[0] = 0;
  phi_statevar[0] = 0;
  phi_statevar_d[0] = 0;
  phi_error[0] = 0;
  phi[1] = 0;
  phi_statevar[1] = 0;
  phi_statevar_d[1] = 0;
  phi_error[1] = 0;
}
