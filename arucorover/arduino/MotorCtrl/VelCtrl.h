
#ifndef VELCTRL_H
#define VELCTRL_H

// Fundamental control system steps
class Vel_Ctrl {
    const float Ki = 0.2;
    const float Kp = 1;

    float rhodot [2] = {0,0};
    float rhodot_statevar [2] = {0,0};
    float rhodot_error [2] = {0,0};

    void move_frame();
    void control(float* velocity_desired, double* delta_t, float* vel_l, float* vel_r);

  public:
    float mag_v[2];
    void tick(float* velocity_desired, double* delta_t, float* vel_l, float* vel_r);

    float get_rhodot();
    float get_rhodot_error();
};

#endif
