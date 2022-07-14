#include <Wire.h>
#include <Encoder.h>
#include <StateMachine.h>

#include "TurnCtrl.h"
#include "VelCtrl.h"
#include "constants.h"

Encoder myEncR(3, 5);
Encoder myEncL(2, 11);

float l_offset_coeff = 0.2;

float magnitude_voltage = 0;  //Voltage magnitude, intended for moving the plant forward
uint8_t motor_PWM = 0; //The unsigned integer to be sent to the PWM ports connected to the motors.

uint8_t received_old[RECEIVED_MX];
uint8_t received[RECEIVED_MX];
uint8_t received_amt = 0;

//uint8_t cnt_right_angles = 0;
uint8_t cross_fwd_cnt = 0;

uint8_t report_counter = 0; // When it reaches 13, data is reported to the serial console.

bool is_tape_detected = 0;
bool is_cross_detected = 0;
bool is_turn_sharp = 0;
//bool is_tape_broken = 0;
bool is_tape_ended = 0;
bool success = 0;

bool debug_actuation = 1;
bool debug_comm = 0;

float angle_bounds = 0.1;
float angle_desired = 0;

float velocity_desired = 2;
float vel_l = 0;
float vel_r = 0;
double delta_t = 0;
long time_stamp[2] = {0, 0};

Turn_Ctrl turn_ctrl;
Vel_Ctrl vel_ctrl;

int pos_l[2] = {0, 0};
int pos_r[2] = {0, 0};

Encoder enc_r(3,5);
Encoder enc_l(11,2);

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

//StateMachine comm = StateMachine();

bool is_tape_within_bounds() {
  if(abs(angle_desired) < angle_bounds) return true;
  else return false;
}

// States for the communications machine
void parse() {
  // We expect bytes 3-6 to be a floating point value (4 bytes):
  uint8_t value[4] = {received[2], received[3], received[4], received[5]};
  // For each register, set the associated parameter:
  if (received[0] == 0x01) velocity_desired = fromBytes(value);
  if (received[0] == 0x02) angle_desired = -fromBytes(value);
  if (received[0] == 0x03) is_tape_detected = received[1];
  if (received[0] == 0x04) is_cross_detected = received[1];
  if (received[0] == 0x05) is_turn_sharp = received[1];
  //if (received[0] == 0x06) is_tape_broken = received[1];
  if (received[0] == 0x07 && received[1] == 0x01 && received[2] == 0x0 ) is_tape_ended = received[1];
  for(int i = 0; i < RECEIVED_MX; i++) received_old[i] = received[i];
}

void report() {
  if(debug_actuation) report_data();
 }

// States for the actuation machine
void state0() {
  delay(100);
}

void state1() {
  turn_ctrl.tick(&angle_atom, &delta_t, &vel_l, &vel_r);
  update_motors(TURNMODE,turn_ctrl.delta_v[1]);
}
void state2() {
  turn_ctrl.tick(&angle_desired, &delta_t, &vel_l, &vel_r);
  update_motors(TURNMODE,turn_ctrl.delta_v[1]);
  }
void state3() {
  vel_ctrl.tick(&velocity_desired, &delta_t, &vel_l, &vel_r);
  update_motors(LINEMODE,vel_ctrl.mag_v[1]);
  }

void state5() {
  cross_fwd_cnt++;
  if(cross_fwd_cnt >= int((1500)/(velocity_desired*T))){
    velocity_desired = 0;
  }
  vel_ctrl.tick(&velocity_desired, &delta_t, &vel_l, &vel_r);
  update_motors(LINEMODE, velocity_desired);
  }

bool transitionS0() {
  if(!angle_desired)
    return true;  // Continue to wait for data...
  else return false;
}
bool transitionS1() {
  if(turn_ctrl.is_error_0() && !is_tape_detected) turn_ctrl.reset_phi();
  return !is_tape_detected;  // Continue to turn until tape appears...
}
bool transitionS2() {
  if(!turn_ctrl.is_error_0()) {
    turn_ctrl.reset_phi();
    return true;
  }
  else return false;  // Continue to turn until the tape is centered...
}
bool transitionS3() {
  if(is_tape_detected && is_tape_within_bounds()) return true;
  else {
    turn_ctrl.reset_phi();
    return false;
  }
}

// S0->Si
bool transitionS0S1() {
  if(angle_desired && !is_tape_detected) return true;  // Data has been recieved, and no tape is detected.  Turn.
  else return false;
}
bool transitionS0S2() {
  if(angle_desired && is_tape_detected && !is_tape_within_bounds()) return true;
  else return false;  // Data has been recieved, and the tape is off center. Turn.
}
bool transitionS0S3() {
  if(angle_desired && is_tape_detected && is_tape_within_bounds()) return true;
  else return false;  // Data has been recieved, and the tape is centered. Forward.
}

// S1->Si
bool transitionS1S2() {
  if(is_tape_detected && !is_tape_within_bounds()) return true;
  else return false;  // The tape is now visible. Turn to center it.
}
bool transitionS1S3() {
  if(is_tape_detected && is_tape_within_bounds()) return true;
  else return false;  // The tape is now centered. Move forward.
}

// S2->Si
bool transitionS2S3() {
  if(is_tape_detected && is_tape_within_bounds()) return true;
  else return false;  // The tape is now centered. Move forward.
}
bool transitionS2S1() {
  return !is_tape_detected; // The tape has moved out of view. Search for the tape.
}

// S3->Si
bool transitionS3S2() {
  return !is_tape_within_bounds();  // The tape is off-center.  Center it.
}
bool transitionS3S5() {
  return is_tape_ended;
}

/*
bool transitionS3S4() {
  if(is_turn_sharp) {
    turn_ctrl.reset_phi();
    return true;
  }
  else return false;
}


// S4->Si
bool transitionS4S3() {
  if(turn_ctrl.is_error_0()) {
//    cnt_right_angles++;
    return true;
  }
  return false;
}
*/


bool is_actuate_per_elapsed = 1;

StateMachine actuate = StateMachine();

State* S0 = actuate.addState(&state0);
State* S1 = actuate.addState(&state1);
State* S2 = actuate.addState(&state2);
State* S3 = actuate.addState(&state3);
//State* S4 = actuate.addState(&state4);
State* S5 = actuate.addState(&state5);
//State* S6 = actuate.addState(&state6);

void calc_deltas() {
  pos_l[0] = pos_l[1];
  pos_r[0] = pos_r[1];
  time_stamp[0] = time_stamp[1];
  
  pos_l[1] = enc_l.read();
  pos_r[1] = enc_r.read();
  time_stamp[1] = micros();
  delta_t = double(time_stamp[1] - time_stamp[0]) / 1000000;
  
  vel_l = (pos_l[1]-pos_l[0])*rad_enc_step/delta_t;
  vel_r = (pos_r[1]-pos_r[0])*rad_enc_step/delta_t;
}

void receiveEvent(int count) {
  //Serial.print("Data received:");
  uint8_t i = 0; // Index for the number of bytes received
  while (Wire.available()) {
    received[i] = Wire.read(); // take the next byte
    if(debug_comm){
      Serial.print("0x");
      Serial.print(received[i], HEX); //print that byte as a hexadecimal
      Serial.print(" ");
    }
    i++; // increment the index
  }
  received_amt = i; // The index is now the length of the message
  if(debug_comm) Serial.println(""); // end the line on the serial console
  //if(received[0] == 0x07) Serial.println("Yep.");
}

void transitions_init() {
  //S3->addTransition(&transitionS3S4, S4);
  //S2->addTransition(&transitionS2S4, S4);
  S2->addTransition(&transitionS2S1, S1); // The tape has moved out of view. Search for the tape
  S3->addTransition(&transitionS3S5, S5); // The tape has disappeared from view. We have probably reached the cross
  S3->addTransition(&transitionS3S2, S2); // The tape is off-center.  Center it.
  S2->addTransition(&transitionS2S3, S3); // The tape is now centered. Move forward.
  
  // Self-transitions
  S0->addTransition(&transitionS0, S0); // Continue to wait for data...
  S1->addTransition(&transitionS1, S1); // Continue to turn until tape appears...
  S2->addTransition(&transitionS2, S2); // Continue to turn until the tape is centered...
  S3->addTransition(&transitionS3, S3); // Continue to move forward...

  // S0->Si
  S0->addTransition(&transitionS0S1, S1); // Data has been recieved, and no tape is detected.  Turn.
  S0->addTransition(&transitionS0S2, S2); // Data has been recieved, and the tape is off center. Turn.
  S0->addTransition(&transitionS0S3, S3); // Data has been recieved, and the tape is centered. Forward.

  // S1->Si
  S1->addTransition(&transitionS1S2, S2); // The tape is now visible. Turn to center it.
  S1->addTransition(&transitionS1S3, S3); // The tape is now centered. Move forward.
  

  // S3->Si




  // S4->Si
  //S4->addTransition(&transitionS4S3, S3); // We have detected the cross. Move forward until we don't see it anymore.

}

void update_motors(uint8_t mode, float voltage) {
  int pwm = float(voltage/v_max)*255;
  uint8_t pwm_minimum = 28;
  if(abs(pwm) < pwm_minimum && abs(pwm) > 0) pwm = (abs(pwm)/pwm)*pwm_minimum;
  //if(voltage > v_max) pwm = 255;
  //if(voltage < -(v_max)) pwm = -255;
  //Serial.println(pwm);
  if(mode == TURNMODE) {
    if (pwm >= 0) {
      digitalWrite(MRDIR, LOW);
      digitalWrite(MLDIR, HIGH);
    }
    else {
      digitalWrite(MRDIR, HIGH);
      digitalWrite(MLDIR, LOW); 
    }
  }

  if(mode == LINEMODE) {
    if (pwm < 0) {
      digitalWrite(MRDIR, HIGH);
      digitalWrite(MLDIR, HIGH);
    }
    else {
      digitalWrite(MRDIR, LOW);
      digitalWrite(MLDIR, LOW); 
    }
  }
  analogWrite(MLPWM, abs(pwm*(1+l_offset_coeff)));
  analogWrite(MRPWM, abs(pwm));
  
}

void report_data() {
  Serial.print(actuate.currentState);
  Serial.print("\t");
  Serial.print(is_tape_ended);
  Serial.print("\t");
  Serial.print(is_tape_within_bounds());
  Serial.print("\t");
  Serial.print(is_tape_detected);
  Serial.print("\t");
  Serial.print(is_turn_sharp);
  Serial.print("\t");
  Serial.print(is_tape_ended);
  Serial.print("\t");
  Serial.print(vel_l);
  Serial.print("\t");
  Serial.print(vel_r);
  Serial.print("\t");
  Serial.print(angle_desired);
  Serial.print("\t");
  Serial.print(turn_ctrl.get_phi_error());
  Serial.print("\t");
  Serial.print(turn_ctrl.get_phi());
  Serial.println("");
}

void setup() {
  Serial.begin(57600);

  transitions_init();
  
  digitalWrite(EN, HIGH);
  pinMode(MRPWM, OUTPUT);
  pinMode(MLPWM, OUTPUT);
  analogWrite(MRPWM, 0);
  analogWrite(MLPWM, 0);

  Wire.begin(PERIPH_ADDRESS); // Initialize I2C
  Wire.onReceive(receiveEvent); // Define interupt method for receiving data
  Serial.println("Initialization complete."); // Let the user know we're ready.
}

void loop() {
  delay(T);
  report_counter++;
  if(report_counter >= 13){
    parse();
    report_data();
    report_counter = 0;
  }
  calc_deltas();
  actuate.run();
  /*report_data();
  float right_angle = angle_atom*2;
  turn_ctrl.tick(&right_angle, &delta_t, &vel_l, &vel_r);
  if(!turn_ctrl.is_error_0()) update_motors(TURNMODE,turn_ctrl.delta_v[1]);*/
}
