# USB-3D-Accelerometer-Viewer
**USB 3D Accelerometer viewer using MMA8452Q**

This is the source code and schematic for a USB-connected 3D accelerometer viewer, it real-time graphs the motion in X, Y and Z directions in terms of gravitational acceleration "g". The GUI provides control on the MCP2221A 4 GPIOs pins, where you can turn them on/off.

Project hardware:
The hardware is built using MMA8452Q 3-axis accelerometer module from SparkFun (MMA8452Q IC is from InvenSense) and MCP2221A USB-to-I2C bridge from Microchip. 

**Schematic:**

<img width="553" alt="MMA8452-MCP2221_circuit" src="https://user-images.githubusercontent.com/8460504/62651860-c9eddb00-b90e-11e9-8e04-38fdca868ec6.png">

**Source code:**
Parts of the code related to interfacing the MMA8452Q 3-axis accelerometer over I2C bus are derived from SparkFun MMA8452 Arduino library: https://github.com/sparkfun/SparkFun_MMA8452Q_Arduino_Library
and "Arduino and MMA8452 sensor example" blog post listed here: http://arduinolearning.com/code/arduino-and-mma8452-sensor-example.php


**Screenshot:**

<img width="326" alt="MMA8452 3-Axis viewer_v2_2" src="https://user-images.githubusercontent.com/8460504/62651842-be9aaf80-b90e-11e9-966e-6bb1a2da8496.png">

**Other dependencies:**
- MCP2221A C++ library and DLL version 2.1.1 from Microchip Technologies
- Qt version 5.11 or higher

Please let me know if you notice any errors or problems with the source code.
**Disclaimer: Cannot be used for any life-critical application, use it at your own risk.**
