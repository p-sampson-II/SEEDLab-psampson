# ./rpi/

## cvdemos/

Contains three demonstrations written to familiarize myself with OpenCV.

## cv.py

Initializes the Raspberry Pi camera, captures images from the camera, and takes in only a range of light blue HSV values matching a marker.  This mask is then run through a binary threshold to set values above a certain threshold to white and all other values to black in a binary matrix.  The average x and y values of the white pixels in the binary matrix are then taken, and this is used to determine which quadrant of the camera's field of view the marker is located.

## i2c.py

Sends four bytes typecasted from a 32-bit floating point value through i2c to the peripheral device to be interpreted as the angle that the motor should turn to in relation to its initial position.
