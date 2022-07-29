#ifndef VELCTRL_H
#define VELCTRL_H

// Fundamental control system steps
class VelCtrl {
    const float Ki = 0.2;
    const float Kp = 1;

    float rhodot [2] = {0,0};
    float statevar [2] = {0,0};
    float error [2] = {0,0};
    float voltage[2] = {0,0};

    void moveFrame();
    void control(float & velocityDesired, double & dt, float & velL, float & velR);

  public:
    void tick(float & velocityDesired, double & dt, float & velL, float & velR);

    float getRhodot();
    float getError();
    float getVoltage();
};

#endif
