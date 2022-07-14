
#ifndef TURNCTRL_H
#define TURNCTRL_H

// Fundamental control system steps
class Turn_Ctrl {
    const float Kp = 0.8; //0.6842;
    const float camera_Kp = 2.5;//3.409;
    const float error_bounds = 0.2;

    float phi [2] = {0,0};
    float phi_statevar [2] = {0,0};
    float phi_statevar_d [2] = {0,0};
    float phi_error [2] = {0,0};

    void move_frame();
    void control(float* angle_desired, double* delta_t, float* vel_l, float* vel_r);

  public:
    float delta_v[2];
    bool is_error_0();
    void tick(float* angle_desired, double* delta_t, float* vel_l, float* vel_r);
    void reset_phi();
    
    float get_phi();
    float get_phi_error();
};

#endif
