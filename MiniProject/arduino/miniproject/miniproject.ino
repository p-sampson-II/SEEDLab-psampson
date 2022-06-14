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

#define RADIUS 1

// Important constants for I2C:
#define PERIPH_ADDRESS 0x04
#define RECEIVED_MX 32

const bool debugComm = false;
const bool debugActuation = true;

int motorPWM = 0;

double dt = 0;
long timeStamp[2] = {0, 0};
float pos[2] = {0, 0};
float posDesired = 0;

uint8_t state = 0;

Encoder enc(2,11);
Counter overShootDetect(5);
PosCtrl controller;

uint8_t received[RECEIVED_MX];
uint8_t receivedAmt = 0;

void calcDeltas();
void reportData();
void requestEvent();
void receiveEvent(int count);
void parseReceived() ;
float fromBytes(uint8_t *input_array);


void setup() {
  digitalWrite(EN, HIGH); // Enable the motor driver
  analogWrite(MOTORPWM, 0);
  Wire.begin(PERIPH_ADDRESS);
  Wire.onReceive(receiveEvent); // Define interupt method for receiving data
  Wire.onRequest(requestEvent); // Define interupt method for sending data
  Serial.begin(115200);
}

void loop() {
  delay(T); // Wait the approximate duration of a period.
  calcDeltas(); // Calculate what has changed while we waited
  // In state 0, the position of the motor, and the desired position to move the motor to, are monitored.
  if(state == 0) {
    parseReceived(); // Interpret the most recent instructions from the I2C controller
    if(!controller.isError0()) {
      state = 1; // If the current and desired positions are not the same, go to state 1.
      return;
     }
     controller.tick(&posDesired, &pos[1], &dt, false); // Use a P controller while we wait for something to happen
     updateMotors(controller.getVoltage()); // Most likely, voltage will be 0 or close enough to zero that it is ignored
  }

  // State 1 gets the position of the motor from where it is, to (nearly) where it needs to be.
  if(state == 1) {
    controller.tick(&posDesired, &pos[1], &dt , true); // Use the PI controller to determine what voltage to send the motor
    updateMotors(controller.getVoltage()); // Send the voltage to the motor
    if(controller.isError0()) {
      overShootDetect.start(); // The first time 0 error is reached, we anticipate an overshoot. Start a timer.
      state = 2; // Go to state 2
    }
  }
  // In state 2, the motor waits 5 seconds to ensure an overshoot does not cause a steady-state error when I is turned off
  if(state == 2) {
    overShootDetect.count(dt); // Count to 5 seconds
    if(overShootDetect.getIsComplete()) {
      if(controller.isError0()){
        state = 0; // If at the end of that 5 seconds the controller reports 0 error, go to state 0.
        overShootDetect.reset(); // Reset the timer for next time there is an overshoot.
        return;
      }
      else {
        state = 1; // If there is a significant error, correct it.
        return;
      }
    }
    else {
      controller.tick(&posDesired, &pos[1], &dt , true); // Let the controller do its thing while we wait
      updateMotors(controller.getVoltage()); // Update the motor while we wait
    }
  }
  if(debugActuation) reportData(); // Gives the user a table of variables to test the system with
}

void calcDeltas() {
  pos[0] = pos[1];
  timeStamp[0] = timeStamp[1];
  
  pos[1] = enc.read()*2*PI*RADIUS/3200;
  timeStamp[1] = micros();
  dt = double(timeStamp[1] - timeStamp[0]) / 1000000;
}

void updateMotors(float voltage) {
  int pwm = float(voltage/VMAX)*255;
  if(abs(voltage) > VMAX) pwm = 255*voltage/abs(voltage);
  else if(abs(voltage) < 0.1) pwm = 0;

  if (pwm >= 0) {
    digitalWrite(MOTORDIR, LOW);
  }
  else {
    digitalWrite(MOTORDIR, HIGH); 
  }

  analogWrite(MOTORPWM, abs(pwm));
  
}

void reportData() {
  Serial.print(dt, 8);
  Serial.print("\t");
  Serial.print(posDesired);
  Serial.print("\t");
  Serial.print(motorPWM);
  Serial.print("\t");
  Serial.print(pos[1],5);
  Serial.print("\t");
  Serial.print(controller.getError(),5);
  Serial.print("\t");
  Serial.print(controller.getVoltage());
  Serial.print("\t");
  Serial.print(overShootDetect.getElapsed());
  Serial.print("\t");
  Serial.print(state);
  Serial.println("");
}

void parseReceived() {
  uint8_t value[4];
  for(int i = 0; i < 4; i++) {
    value[i] = received[i+2];
    //Serial.print(received[i+2], HEX);
  }
  if(received[0] == 0x01){
    posDesired = fromBytes(value);
  }
}

void requestEvent() {
  if(debugComm) Serial.println("Request received");
  uint8_t message[2] = {0x04, 0x01};
  for(uint8_t i = 0; i < 2; i++) Wire.write((char)message[i]);
}

void receiveEvent(int count) {
  if(debugComm) Serial.print("Data received:");
  uint8_t i = 0; // Index for the number of bytes received
  while (Wire.available()) {
    received[i] = Wire.read(); // take the next byte
    if(debugComm) {
      Serial.print("0x");
      Serial.print(received[i], HEX); //print that byte as a hexadecimal
      Serial.print(" ");
    }
    i++; // increment the index
  }
  receivedAmt = i; // The index is now the length of the message
  if(debugComm) Serial.println(""); // end the line on the serial console
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
