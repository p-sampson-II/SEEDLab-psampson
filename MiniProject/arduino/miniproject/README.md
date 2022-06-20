# ./miniproject/

The Arduino acts as an I2C peripheral, taking in a "register" byte and a corresponding integer or floating point value to change one of the variables in the program it is running.  The particular code only has one register, for one variable: the angle at which to move the wheel, as a 4-byte floating point value.

## miniproject.ino

The main code contains a rudamentary state machine implementation using if-else statements that waits for i2c data while maintaining the wheel's present position using a proportional controller.  If the wheel moves or if the desired position of the wheel changes, the program enters state 1, in which the motor is moved so that the encoder reports a value similar enough to the desired position that the controller's zero-error condition is met. When this happens, a 5-second timer is started.  If, at the end of 5 seconds, the zero-error condition is met, the system will enter state 0.  If not, it will continue to run the PI controller until the condition is met, then enter state 0.

## PosCtrl.h, PosCtrl.cpp

Contains code for managing the state of the Proportional-Integral controller that takes in encoder data and outputs a voltage with which to move the motor.  The controller can be either run as a Proportional-Integral controller, or as a Proportional controller, based on user input.

## Counter.h

Manages the state of a counter, which waits for an approximate duration of time before an "elapsed" condition is met, but does not hold up the entire system like using the delay function would.

