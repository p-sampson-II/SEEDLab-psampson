#include <Encoder.h>
#include <math.h>

#define MOTORDIR 7
#define MOTORPWM 9

#define MFB A1
#define EN 4

#define T 8

const float WHEEL_RADIUS = 0.062;

float linearStep = 2*PI*WHEEL_RADIUS/3600;
int motorPWM = 0;

double deltat = 0;
long timeStamp[2] = {0, 0};
long stepCount[2] = {0, 0};
float pos[2] = {0, 0};
float vel[2] = {0, 0};
 
uint8_t state = 0;

Encoder enc(2,11);

class Counter {
public:
  Counter(double d): duration(d), elapsed(0), isComplete(0) {}
  void count(double dt) {
    if (elapsed < duration) elapsed += dt;
    else isComplete = true;
    }
  void reset() { 
    elapsed = 0;
    isComplete = 0;
  }
  bool getIsComplete() { return isComplete; }
  double getElapsed() { return elapsed; }
protected:
  double duration;
  double elapsed;
  bool isComplete;
};

void calcDeltas() {
  stepCount[0] = stepCount[1];
  pos[0] = pos[1];
  vel[0] = vel[1];
  timeStamp[0] = timeStamp[1];
  
  stepCount[1] = -enc.read();
  pos[1] = (float)linearStep*stepCount[1];
  timeStamp[1] = micros();
  deltat = double(timeStamp[1] - timeStamp[0]) / 1000000;
  vel[1] = (pos[1]-pos[0])/deltat;
}

Counter startTmr(1);
Counter stopTmr(5);

void reportData() {
  Serial.print((startTmr.getElapsed()+stopTmr.getElapsed()), 8);
  Serial.print("\t");
  Serial.print(vel[1]);
  Serial.print("\t");
  Serial.print(state);
  Serial.println("");
}

void setup() {
  // put your setup code here, to run once:
  digitalWrite(EN, HIGH); // Enable the motor driver
  analogWrite(MOTORPWM, 0);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(T); //Wait the approximate duration of a period.
  calcDeltas();
  if(startTmr.getIsComplete() && state == 0) {
    analogWrite(MOTORPWM, 255);
    state = 1;
  }
  if(!startTmr.getIsComplete())
    startTmr.count(deltat);
  
  if(stopTmr.getIsComplete() && state == 1) {
    analogWrite(MOTORPWM, 0);
    state = 2;
  }
  if(!stopTmr.getIsComplete() && state >= 1) stopTmr.count(deltat);
  if(state != 2) reportData();
}
