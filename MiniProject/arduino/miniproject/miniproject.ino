#include <Wire.h>
#include <Encoder.h>
#include <math.h>
#include "PosCtrl.h"
#include "Counter.h"

#define MOTORDIR 7
#define MOTORPWM 9

#define MFB A1
#define EN 4

#define VMAX 6

#define T 8
#define WHEEL_RADIUS 0.50

// Important constants for I2C:
#define PERIPH_ADDRESS 0x04
#define RECEIVED_MX 32

Encoder mtrEnc(2, 11);

namespace globalVars {
  const double linearStep = 2 * PI * WHEEL_RADIUS / 3200;
  double period = 0;
  long timeStamp[2] = {0, 0};
  float pos = 0;
  float posDesired = 0;
  int duty = 0;
  uint8_t state = 0;
  uint8_t received[RECEIVED_MX];
  uint8_t receivedAmt = 0;
}

using namespace globalVars;


Encoder enc(2, 11);
PosCtrl posCtrl;
Counter overShootDetect(5);

void runStateMachine();
void reportData();
void updateMotor();
void updateSensors();
void receiveEvent(int);
void requestEvent();
void parse();
float fromBytes(uint8_t *input_array);

void setup() {
  digitalWrite(EN, HIGH); // Enable the motor driver
  analogWrite(MOTORPWM, 0);
  Serial.begin(115200);
  Wire.begin(PERIPH_ADDRESS);
  Wire.onReceive(receiveEvent); // Define interupt method for receiving data
  Wire.onRequest(requestEvent); // Define interupt method for sending data
  Serial.println("Initialization complete."); // Let the user know we're ready.
  //state = 1;
  //posDesired = PI / 8;
}

void loop() {
  delay(T); //Wait the approximate duration of a period.
  runStateMachine();
  updateSensors();
  updateMotor();
  reportData();
}

void runStateMachine() {
  if (state == 0) {
    posCtrl.tick(&posDesired, &pos, &period, false);
    if (receivedAmt) parse();
    if (!posCtrl.isError0()) {
      state = 1;
      overShootDetect.reset();
    }
    delay(100);
  }
  if (state == 1) {
    posCtrl.tick(&posDesired, &pos, &period, true);
    //if(posCtrl.isFault()) state = 2;
    if (posCtrl.isError0()) {
      if (overShootDetect.getIsComplete()) {
        state = 0;
      }
      else if (!overShootDetect.getIsStarted()) overShootDetect.start();
    }
    if (overShootDetect.getIsStarted()) overShootDetect.count(period);
  }
  /*if (state == 2) {
    posCtrl.tick(0,0,0, false);
    posCtrl.reset();
    if(!posCtrl.isFault()) state = 0;
  }*/
}

// Formats data and prints it on the serial port for a human to read.
void reportData() {
  Serial.print(period, 8);
  Serial.print("\t");
  Serial.print(posCtrl.voltage[1]);
  Serial.print("\t");
  Serial.print(pos);
  Serial.print("\t");
  Serial.print(posDesired);
  Serial.print("\t");
  Serial.print(posCtrl.getError(),5);
  Serial.print("\t");
  Serial.print(duty);
  Serial.print("\t");
  Serial.print(state);
  Serial.print("\t");
  Serial.print(overShootDetect.getElapsed());
  Serial.print("\t");
  Serial.println("");
}

void updateMotor() {
  if (abs(posCtrl.voltage[1]) > 0.2) {
    if (abs(posCtrl.voltage[1]) < VMAX) duty = abs(posCtrl.voltage[1] * 255 / VMAX);
    else duty = 255;
  }
  else duty = 0;

  if (posCtrl.voltage[1] >= 0) digitalWrite(MOTORDIR, HIGH);
  else digitalWrite(MOTORDIR, LOW);

  analogWrite(MOTORPWM, duty);
}

// Updates sensor data and the period of time since the last reading.
void updateSensors() {
  timeStamp[0] = timeStamp[1];

  pos = -enc.read() * linearStep;
  timeStamp[1] = micros();
  period = double(timeStamp[1] - timeStamp[0]) / 1000000;
}

void receiveEvent(int count) {
  //Serial.print("Data received:");
  uint8_t i = 0; // Index for the number of bytes received
  while (Wire.available()) {
    received[i] = Wire.read(); // take the next byte
    //Serial.print("0x");
    //Serial.print(received[i], HEX); //print that byte as a hexadecimal
    //Serial.print(" ");
    i++; // increment the index
  }
  receivedAmt = i; // The index is now the length of the message
  //Serial.println(""); // end the line on the serial console
}

void requestEvent() {
  //Serial.println("Request received");
  uint8_t message[2] = {0x04, 0x01};
  for (uint8_t i = 0; i < 2; i++) Wire.write((char)message[i]);
}

/*

*/
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

void parse() {
  // We expect bytes 3-6 to be a floating point value (4 bytes):
  uint8_t value[4];
  for (uint8_t i = 2; i <= 5; i++) value[i - 2] = received[i];
  // For each register, set the associated parameter:
  if (received[0] == 0x01) posDesired = fromBytes(value);
  receivedAmt = 0;
}
