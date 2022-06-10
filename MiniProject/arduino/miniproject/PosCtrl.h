
#ifndef POSCTRL_H
#define POSCTRL_H

class PosCtrl {
    const float Kp = 8.0236;
    const float Ki = 4;//0.7659;
    const float ERROR_BOUNDS = 0.005;

    float theta [2] = {0,0};
    float error [2] = {0,0};
    float totalError = 0;

    void moveFrame();
    void control(float *posDesired, float *pos, double *period, bool integral);

  public:
    float voltage[2];
    bool isError0();
    bool isFault();
    void tick(float *posDesired, float *pos, double *period, bool integral);
    void reset();
    
    float getTheta();
    float getError();
};

#endif
