// PosCtrl.h
// The control system for the motor, which controls position.
// Author: Paul Sampson

#ifndef POSCTRL_H
#define POSCTRL_H

class PosCtrl {
    const float Kp = 1.349*5;
    const float Ki = 0.2598*5;
    const float ERROR_BOUNDS = 0.008;
    const float fudge = 1; //3.125;

    float theta [2] = {0,0};
    float error [2] = {0,0};
    float voltage [2] = {0,0};
    double totalError = 0;

    void moveFrame();
    void control(float &posDesired, float &pos, double &period, bool integral);

  public:
    float delta_v[2];
    bool isError0();
    bool isFault();
    void tick(float &posDesired, float &pos, double &period, bool integral);
    void reset();
    
    float getTheta();
    float getError();
    float getVoltage();
};

#endif
