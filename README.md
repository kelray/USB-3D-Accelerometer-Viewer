# USB-3D-Accelerometer-Viewer
USB 3D Accelerometer viewer using MMA8452Q

This is the source code and schematic for a USB-connected 3D accelerometer using MMA8452Q 3-axis accelerometer from InvenSense and MCP2221A USB-to-I2C bridge from Microchip. The project uses SparkFun MMA8452Q breakout module. 

Parts of the code related to interfacing the MMA8452Q 3-axis accelerometer over I2C bus are derived from SparkFun MMA8452 Arduino library: https://github.com/sparkfun/SparkFun_MMA8452Q_Arduino_Library
and "Arduino and MMA8452 sensor example" blog post listed here: http://arduinolearning.com/code/arduino-and-mma8452-sensor-example.php

Schematic:

Code dependencies:
- MCP2221A C++ library and DLL version 2.1.1 from Microchip Technologies
- Qt version 5.11 or higher

Disclaimer: Cannot be used for any life-critical application, use it at your own risk.
