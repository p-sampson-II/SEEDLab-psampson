#include "Counter.h"
#include "TurnCtrl.h"
#include "VelCtrl.h"
#include "constants.h"
#include <Encoder.h>
#include <Wire.h>
#include <math.h>

#define MOTORDIR 7
#define MOTORPWM 9

#define MFB A1
#define EN 4
#define VMAX 6

#define T 8

#define RADIUS 1

// Important constants for I2C:
#define PERIPH_ADDRESS 0x04
#define RECEIVED_MX 32

const bool debugComm = false;
const bool debugActuation = true;

// int motorPWM = 0;

double dt = 0;
long timeStamp[2] = {0, 0};
float velDesired = 1;
float angleDesired = 0;

float velL[2] = {0, 0};
float velR[2] = {0, 0};
float posL[2] = {0, 0};
float posR[2] = {0, 0};

enum states { initial, seek, corr, fwd, stop } state, stateOld;

Encoder encL(2, 11);
Encoder encR(3, 5);
VelCtrl velCtrl;
TurnCtrl turnCtrl;

uint8_t received[RECEIVED_MX];
uint8_t receivedAmt = 0;

bool isArucoDetected = 0;
bool isArucoExitBottom = 0;
bool begin = 0;
bool isStateChange = 0;
bool doParse = 1;

void calcDeltas();
void reportData();
void requestEvent();
void receiveEvent(int count);
void parseReceived();
float fromBytes(uint8_t *input_array);

void state_initial();
void state_seek();
void state_corr();
void state_fwd();
void state_stop();

void setup() {
  digitalWrite(EN, HIGH); // Enable the motor driver
  analogWrite(MOTORPWM, 0);
  Wire.begin(PERIPH_ADDRESS);
  Wire.onReceive(receiveEvent); // Define interupt method for receiving data
  Wire.onRequest(requestEvent); // Define interupt method for sending data
  Serial.begin(115200);
}

void loop() {
  delay(T);     // Wait the approximate duration of a period.
  calcDeltas(); // Calculate what has changed while we waited
  switch (state) {
  case states::initial:
    state_initial();
    break;
  case states::seek:
    state_seek();
    break;
  case states::corr:
    state_corr();
    break;
  case states::fwd:
    state_fwd();
    break;
  case states::stop:
    state_stop();
    break;
  }
  reportData();
  if(received[0] && doParse) parseReceived();
}

void calcDeltas() {
  posL[0] = posL[1];
  posR[0] = posR[1];
  velL[0] = velL[1];
  velR[0] = velR[1];
  timeStamp[0] = timeStamp[1];
  if(stateOld == state) isStateChange = false;
  else isStateChange = true;
  stateOld = state;

  posL[1] = encL.read();
  posR[1] = encR.read();
  timeStamp[1] = micros();
  dt = double(timeStamp[1] - timeStamp[0]) / 1000000;

  velL[1] = (posL[1] - posL[0]) * rad_enc_step / dt;
  velR[1] = (posR[1] - posR[0]) * rad_enc_step / dt;
}

void updateMotors(uint8_t mode, float voltage) {
  int pwm = float(voltage / VMAX) * 255;
  const uint8_t pwm_minimum = 28;

  if (abs(voltage) > VMAX)
    pwm = 255 * voltage / abs(voltage);
  else if (abs(voltage) < 0.1)
    pwm = 0;

  if (mode == TURNMODE) {
    if (pwm >= 0) {
      digitalWrite(MRDIR, LOW);
      digitalWrite(MLDIR, HIGH);
    } else {
      digitalWrite(MRDIR, HIGH);
      digitalWrite(MLDIR, LOW);
    }
  }

  if (mode == LINEMODE) {
    if (pwm < 0) {
      digitalWrite(MRDIR, HIGH);
      digitalWrite(MLDIR, HIGH);
    } else {
      digitalWrite(MRDIR, LOW);
      digitalWrite(MLDIR, LOW);
    }
  }

  analogWrite(MLPWM, abs(pwm * (1 + l_offset_coeff)));
  analogWrite(MRPWM, abs(pwm));
}

void reportData() {
  Serial.print(dt, 8);
  Serial.print("\t");
  Serial.print(angleDesired);
  Serial.print("\t");
  Serial.print(velL[1], 5);
  Serial.print("\t");
  Serial.print(velR[1], 5);
  Serial.print("\t");
  Serial.print(velCtrl.getError(), 5);
  Serial.print("\t");
  Serial.print(velCtrl.getVoltage());
  Serial.print("\t");
  Serial.print(turnCtrl.getError(), 5);
  Serial.print("\t");
  Serial.print(turnCtrl.getVoltage());
  Serial.print("\t");
  Serial.print(str_states[state]);
  Serial.println("");
}

void parseReceived() {
  uint8_t value[4];
  for (int i = 0; i < 4; i++) {
    value[i] = received[i + 2];
    // Serial.print(received[i+2], HEX);
  }
  if (received[0] == 0x01)
    velDesired = fromBytes(value);
  if (received[0] == 0x02)
    angleDesired = fromBytes(value);
  if (received[0] == 0x03)
    isArucoDetected = received[1];
  if (received[0] == 0x04)
    isArucoExitBottom = received[1];
  if (received[0] == 0x05)
    begin = received[1];
  received[0] = 0;
}

void requestEvent() {
  if (debugComm)
    Serial.println("Request received");
  uint8_t message[2] = {0x04, 0x01};
  for (uint8_t i = 0; i < 2; i++)
    Wire.write((char)message[i]);
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

void state_initial() {
  delay(100);
  if (begin)
    state = states::seek;
}

void state_seek() {
  if(isStateChange) doParse = true;
  turnCtrl.tick(angleDesired, dt, velL[1], velR[1]);
  updateMotors(TURNMODE, turnCtrl.getVoltage());
  if (isArucoDetected)
    state = states::corr;
  else if(turnCtrl.isError0()) turnCtrl.reset();
}
void state_corr() {
  if(isStateChange) doParse = false;
  turnCtrl.tick(angleDesired, dt, velL[1], velR[1]);
  updateMotors(TURNMODE, turnCtrl.getVoltage());
  if (!isArucoDetected)
    state = states::seek;
  if (turnCtrl.isError0())
    state = states::fwd;
}
void state_fwd() {
  if(isStateChange) doParse = true;
  velCtrl.tick(velDesired, dt, velL[1], velR[1]);
  turnCtrl.tick(angleDesired, dt, velL[1], velR[1]);
  updateMotors(LINEMODE, velCtrl.getVoltage());
  if (!turnCtrl.isError0())
    state = states::fwd;
}
void state_stop() {
  updateMotors(LINEMODE, 0);
  delay(1000);
  state = states::seek;
}
