# RobotMicrocontroller
Software for Arduino Mega 2560 with a RAMPS 1.6 Hat for controlling my selfmade robotarm
The Arduino receives joint position and speed over Serial from the PC this data is send in a string so the arduino first unpack the data.
Then the joint positions (in degrees) need to be converted to steps for the motors.
The Arduino also homes every joint with the connected endstops and ensures that the joints will not be moved out of their mechanical bounds.

Every joint has a normal joint class where necessary attributes like steps per degree or which endstop switch and motor is connected are given.
This class has also some basic methods like converting degrees to steps or checking if the joint is homed before moving.
These classes can then be passed on to other classes which can use this information to drive the steppermotors together, like the differantial joint where two motors
need to work together to achieve tilt and rotation in one small package

The function for receiving the serial data is a modified version from this tutorial
https://forum.arduino.cc/t/serial-input-basics-updated/382007

This project is only for demonstration purposes and not in its final form So this code is not optimized for readability or usage by others (but you certainly can use it)
