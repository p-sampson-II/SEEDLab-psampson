#include <Wire.h>
#include <Encoder.h>
#include <math.h>

#define MOTORDIR 7
#define MOTORPWM 9

#define MFB A1
#define EN 4

#define T 8

// Important constants for I2C:
#define PERIPH_ADDRESS 0x04
#define RECEIVED_MX 32

Encoder mtrEnc(2,11);

int motorPWM = 0;

int motorPWM = 0;

double deltat = 0;
long timeStamp[2] = {0, 0};
int pos[2] = {0, 0};

uint8_t state = 0;

Encoder enc(2,11);

uint8_t received[RECEIVED_MX];
uint8_t receivedAmt = 0;

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
  pos[0] = pos[1];
  timeStamp[0] = timeStamp[1];
  
  pos[1] = enc.read();
  timeStamp[1] = micros();
  deltat = double(timeStamp[1] - timeStamp[0]) / 1000000;
}

Counter oneSecond(1);
Counter tenSecond(10);

void reportData() {
  Serial.print((oneSecond.getElapsed()+tenSecond.getElapsed()), 8);
  Serial.print("\t");
  Serial.print(motorPWM);
  Serial.print("\t");
  Serial.print(pos[1]);
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
  if(oneSecond.getIsComplete() && state == 0) {
    analogWrite(MOTORPWM, 255);
    state = 1;
  }
  if(!oneSecond.getIsComplete())
    oneSecond.count(deltat);
  
  if(tenSecond.getIsComplete() && state == 1) {
    analogWrite(MOTORPWM, 0);
    state = 2;
  }
  if(!tenSecond.getIsComplete() && state >= 1) tenSecond.count(deltat);
  if(state != 2) reportData();
}
