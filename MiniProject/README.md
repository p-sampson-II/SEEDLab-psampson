# Mini Project

The system is configured to move a wheel to specific angles based on which quadrant of the Raspberry Pi's camera contains a colored marker.  The project demonstrates basic knowledge of i2c, OpenCV, MATLAB and Simulink, and use of Arduino sketches.

## ./arduino

Arduino sketches are kept here. The Arduino acts as an I2C peripheral, taking in a "register" byte and a corresponding integer or floating point value to change one of the variables in the program it is running.  The particular code only has one register, for one variable: the angle at which to move the wheel, as a 4-byte floating point value.

## ./rpi

The python scripts for the Raspberry Pi are kept here.  These scripts process image data from the Pi Camera with thresholds, make inferences about the data by averaging the resulting mask, and send commands based on those inferences to the i2c peripheral as a controller device.

## ./matlab

MATLAB code and Simulink block diagrams were used to model the motor used for moving the wheel, and generate a Product-Integral controller to be used in the Arduino sketch.
