#include "Counter.h"
#include "TurnCtrl.h"
#include "VelCtrl.h"
#include "PosCtrl.h"
#include "constants.h"
#include "Osc.h"
#include <Encoder.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

#define MOTORDIR 7
#define MOTORPWM 9

#define MFB A1
#define EN 4
#define VMAX 6

// Important constants for I2C:
#define PERIPH_ADDRESS 0x04
#define RECEIVED_MX 32

using namespace timers;

const bool debugComm = true;
const bool debugActuation = false;
const bool info = true;

// int motorPWM = 0;

double dt = 0;
long timeStamp[2] = {0, 0};
float velDesired = 3;
float angleDesired = 0;
float posDesired = 0;
float offsetL = 0.2;

float velL[2] = {0, 0};
float velR[2] = {0, 0};
float posL[2] = {0, 0};
float posR[2] = {0, 0};

enum states { initial, linpos, vel, turn, stationary } state, stateOld, stateOrig;

Encoder encL(6,5);
Encoder encR(2, 3);

VelCtrl velCtrl;
TurnCtrl turnCtrl;
PosCtrl posCtrl;

Osc commOsc(0.01,0.02);

uint8_t received[RECEIVED_MX];
uint8_t receivedAmt = 0;

bool isArucoDetected = 0;
bool isArucoExitBottom = 0;
bool begin = 0;
bool isStateChange = 0;
bool doParse = 1;
bool complete = 0;

void calcDeltas();
void reportData();
void requestEvent();
void receiveEvent(int count);
void parseReceived();
float fromBytes(uint8_t *input_array);
void infoPrint(String notification);
void infoPrint(String notification, String status);
void completionCheck();
void resetCtrl();

void stateInitial();
void stateLinPos();
void stateVel();
void stateTurn();
void stateStationary();

void setup() {
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH); // Enable the motor driver
  digitalWrite(A2, HIGH); // Give left encoder power
  Wire.begin(PERIPH_ADDRESS);
  Wire.onReceive(receiveEvent); // Define interupt method for receiving data
  Wire.onRequest(requestEvent); // Define interupt method for sending data
  Serial.begin(115200);
  infoPrint("Initialized.");  
}

void loop() {
  delay(T);     // Wait the approximate duration of a period.
  calcDeltas(); // Calculate what has changed while we waited
  switch (state) {
  case states::initial:
    stateInitial();
    break;
  case states::linpos:
    stateLinPos();
    break;
  case states::vel:
    stateVel();
    break;
  case states::turn:
    stateTurn();
    break;
  case states::stationary:
    stateStationary();
    break;
  }

  if(commOsc.isRisingEdge())
    reportData();
  commOsc.tick(dt);
  //infoPrint("Timer is on", String(commOsc.getIsOn()));
  
  if(received[0] && doParse) parseReceived();
}

void calcDeltas() {
  posL[0] = posL[1];
  posR[0] = posR[1];
  velL[0] = velL[1];
  velR[0] = velR[1];
  timeStamp[0] = timeStamp[1];
  if(stateOld == state) isStateChange = false;
  else {
    isStateChange = true;
    posCtrl.reset();
    turnCtrl.reset();
    stateOrig = stateOld;
  } 
  stateOld = state;

  posL[1] = encR.read() * rad_enc_step;
  posR[1] = encR.read() * rad_enc_step;
  timeStamp[1] = micros();
  dt = double(timeStamp[1] - timeStamp[0]) / 1000000;

  //velL[1] = (posL[1] - posL[0]) * rad_enc_step / dt;
  velR[1] = (posR[1] - posR[0]) / (dt);
  velL[1] = (posL[1] - posL[0]) / (dt);
}

void updateMotors(uint8_t mode, float voltage) {
  int pwm = float(voltage / VMAX) * 150;
  const uint8_t pwm_torque = 35;
  const uint8_t pwm_thresh = 15;

  if (abs(voltage) > VMAX)
    pwm = 150 * voltage / abs(voltage);
  else if (abs(pwm) < pwm_torque) {
    if(abs(pwm) > pwm_thresh) pwm = 35
    else pwm = 0
  }

  if (mode == TURNMODE) {
    if (pwm >= 0) {
      digitalWrite(MRDIR, LOW);
      digitalWrite(MLDIR, LOW);
    } else {
      digitalWrite(MRDIR, HIGH);
      digitalWrite(MLDIR, HIGH);
    }
  }

  if (mode == LINEMODE) {
    if (pwm < 0) {
      digitalWrite(MRDIR, HIGH);
      digitalWrite(MLDIR, LOW);
    } else {
      digitalWrite(MRDIR, LOW);
      digitalWrite(MLDIR, HIGH);
    }
  }

  int pwmL = pwm*(1+offsetL);
  if(pwmL > 255) pwmL = 255;

  analogWrite(MLPWM, abs(pwmL));
  analogWrite(MRPWM, abs(pwm));
}

void reportData() {
  Serial.print(dt, 8);
  Serial.print("\t");
  Serial.print(posDesired);
  Serial.print("\t");
  Serial.print(posCtrl.getError(), 5);
  Serial.print("\t");
  Serial.print(posCtrl.getVoltage());
  Serial.print("\t");
  Serial.print(angleDesired);
  Serial.print("\t");
  Serial.print(turnCtrl.getError(), 5);
  Serial.print("\t");
  Serial.print(turnCtrl.getVoltage());
  Serial.print("\t");
  Serial.print(str_states[state]);
  Serial.print("\t");
  Serial.print(complete);
  Serial.println("");
}

void parseReceived() {
  infoPrint("Parsing...");
  uint8_t value[4];
  for (int i = 0; i < 4; i++) {
    value[i] = received[i + 2];
    // Serial.print(received[i+2], HEX);
  }
  if (received[0] >= 0x01 && received[0] <= 0x04) 
    state = static_cast<states>(received[0]);  
  if (received[0] == 0x01)
    posDesired = fromBytes(value);
  if (received[0] == 0x02)
    velDesired = fromBytes(value);
  if (received[0] == 0x03)
    angleDesired = fromBytes(value); 
  if (received[0] == 0xFE)
    offsetL = fromBytes(value);   
  if (received[0] == 0xFF) {
    state = static_cast<states>(received[0]);
    infoPrint("Machine state forced to", str_states[state]);      
  }
  received[0] = 0;
}

void infoPrint(String notification) {
  if (info)  {
    Serial.print("[INFO] ");
    Serial.print(notification);
    Serial.println("");
  }
}

void infoPrint(String notification, String status) {
  if(info)  {
    Serial.print("[INFO] ");
    Serial.print(notification);
    Serial.print(":\t");
    Serial.print(status);
    Serial.println("");}
}

void requestEvent() {
  infoPrint("Request received");
  Wire.write((char)static_cast<int>(state));
  received[0] = 0;
}

void receiveEvent(int count) {
  if (debugComm)
    Serial.print("Data received:");
  uint8_t i = 0; // Index for the number of bytes received
  while (Wire.available()) {
    received[i] = Wire.read(); // take the next byte
    if (debugComm) {
      Serial.print("0x");
      Serial.print(received[i], HEX); // print that byte as a hexadecimal
      Serial.print(" "); 
    }
    i++; // increment the index
  }
  receivedAmt = i; // The index is now the length of the message
  if (debugComm)
    Serial.println(""); // end the line on the serial console
}

float fromBytes(uint8_t *input_array) {
  uint32_t input = 0; // Buffer for storing four bytes as one variable
  // This for loop "stacks" the bytes on top of each other
  // by shifting each byte by some multiple of eight and ORing
  // with the buffer:
  for (int i = 0; i < 4; i++) {
    input |= (uint32_t)input_array[i] << (8 * i);
  }
  // Copy the data at the input buffer location to a floating point
  // as if that the data at that location was a floating point.
  float output = *(float *)&input;
  return output;
}

void stateInitial() {
  delay(100);
}

void stateTurn() {
  if(doParse == true) doParse = false;
  turnCtrl.tick(angleDesired, dt, velL[1], velR[1]);
  updateMotors(TURNMODE, turnCtrl.getVoltage());
  completionCheck();
}

void stateVel() {
  velCtrl.tick(velDesired, dt, velL[1], velR[1]);
  updateMotors(LINEMODE, velCtrl.getVoltage());
  completionCheck();
}

void stateLinPos() {
  if(doParse == true) doParse = false;
  float meterPos = posL[1]*wheel_radius;
  posCtrl.tick(posDesired, meterPos, dt, true);
  updateMotors(LINEMODE, posCtrl.getVoltage());
  completionCheck();
}

void completionCheck() {
  if (state == vel) {
    resetCtrl(); 
  }
  else if ((state == turn && turnCtrl.isError0())||(state == linpos && posCtrl.isError0())) {
    doParse = true;
    state = states::stationary;
    resetCtrl(); 
  }
}

void resetCtrl() {
  posDesired = 0;
  angleDesired = 0;
  posCtrl.reset();
  turnCtrl.reset();
  encL.write(0);
  encR.write(0);
  for(int i = 0; i < 2; i++) {
    posL[i] = 0;
    posR[i] = 0;
  }
}

void stateStationary() {
  updateMotors(LINEMODE, 0);
  completionCheck();
}
