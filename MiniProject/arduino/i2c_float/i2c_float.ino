#include <Wire.h>

// Pin definitions
#define M1PWM 9
#define M2PWM 10
#define M1DIR 7
#define M2DIR 8

#define M1FB A0
#define M2FB A1

#define EN 4

//Important constants for I2C:
#define PERIPH_ADDRESS 0x04
#define RECEIVED_MX 32
#define COMM_DEBUG 1

float pos = 0;
float posOld = 0;

uint8_t received[RECEIVED_MX];
uint8_t receivedAmt = 0;

void setup() {
  Serial.begin(115200); // Initialize Serial interface
  Wire.begin(PERIPH_ADDRESS); // Initialize I2C
  Wire.onReceive(receiveEvent); // Define interupt method for receiving data
  Wire.onRequest(requestEvent); // Define interupt method for sending data
  Serial.println("Initialization complete."); // Let the user know we're ready.
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(8);
  //Check if we got data:
  if (receivedAmt) {
    // We expect bytes 3-6 to be a floating point value (4 bytes):
    uint8_t value[4] = {received[2], received[3], received[4], received[5]};
    // For each register, set the associated parameter:
    if (received[0] == 0x00) Serial.println("Halt!");
    if (received[0] == 0x01) pos = fromBytes(value);
    receivedAmt = 0;
  }
  // If a parameter changed, let the user know:
  if (posOld != pos) {
    Serial.print("Angle: ");
    Serial.println(pos);
    posOld = pos;
  }
}

void requestEvent() {
  Serial.println("Request received");
  uint8_t message[2] = {0x04, 0x01};
  for(uint8_t i = 0; i < 2; i++) Wire.write((char)message[i]);
}

void receiveEvent(int count) {
  Serial.print("Data received:");
  uint8_t i = 0; // Index for the number of bytes received
  while (Wire.available()) {
    received[i] = Wire.read(); // take the next byte
    Serial.print("0x");
    Serial.print(received[i], HEX); //print that byte as a hexadecimal
    Serial.print(" ");
    i++; // increment the index
  }
  receivedAmt = i; // The index is now the length of the message
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
