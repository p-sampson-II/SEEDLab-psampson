
#ifndef TURNCTRL_H
#define TURNCTRL_H

// Fundamental control system steps
class TurnCtrl {
    const float Kp = 0.8; //0.6842;
    const float camera_Kp = 2.5;//3.409;
    const float error_bounds = 0.2;

    float phi [2] = {0,0};
    float statevar [2] = {0,0};
    float statevarD [2] = {0,0};
    float error [2] = {0,0};
    float voltage[2] = {0,0};

    void moveFrame();
    void control(float & angle_desired, double & delta_t, float & vel_l, float & vel_r);

  public:
    bool isError0();
    void tick(float & angle_desired, double & delta_t, float & vel_l, float & vel_r);
    void reset();

    float getPhi();
    float getError();
    float getVoltage();
};

#endif
