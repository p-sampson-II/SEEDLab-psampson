
#ifndef POSCTRL_H
#define POSCTRL_H

class Pos_Ctrl {
    const float Kp = 1;
    const float Ki = 0;
    const float error_bounds = 0.05;

    float theta [2] = {0,0};
    float theta_statevar [2] = {0,0};
    float theta_statevar_d [2] = {0,0};
    float theta_error [2] = {0,0};

    void move_frame();
    void control(float* pos_desired, double* delta_t, float* vel_l, float* vel_r);

  public:
    float delta_v[2];
    bool is_error_0();
    void tick(float* pos_desired, double* delta_t, float* vel_l, float* vel_r);
    void reset_theta();
    
    float get_theta();
    float get_theta_error();
};

#endif
