#ifndef CONST_H
#define CONST_H
// Pin definitions
#define MLPWM 9
#define MRPWM 10
#define MRDIR 7
#define MLDIR 8

#define MRFB A0
#define MLFB A1

#define EN 4

// Important constants for I2C:
#define PERIPH_ADDRESS 0x04
#define RECEIVED_MX 32

// Period for actuation state machine (ms):
#define T 8

#define TURNMODE 0
#define LINEMODE 1

// States
const char str_states [5][11] = { "initial", "linpos", "vel", "turn", "stationary" };

// Period for communication state machine (ms):
#define U 100

const double rad_enc_step = 2 * 3.14159265359 / 3200; // The encoder has 3200 steps/rotation.

const float wheel_radius = 0.072; //Radius of the driving wheels
const float wheel_span = 0.27; // The distance between the wheels
const float v_max = 6; // An approximation of the maximum achievable voltage to drive the motors with.
const float angle_atom = (3.14159265359/(8));
#endif
