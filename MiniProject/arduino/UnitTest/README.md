# ./UnitTest/UnitTest.ino

Runs a unit test on the Pololu 37D geared motor. For every time step, reports the length of the time step in seconds, the angular velocity of the motor in radians per second, and the state that the state machine is in. After a one-second count, the voltage sent to the motor is set to HIGH.  Data continues to be sent through the serial console. After ten more seconds, the motor stops and serial communication ceases.
