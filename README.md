# ICM42688-ArduinoLibrary
***Author:*** Sam Tan (chijie1998@hotmail.com)
Just a casual Arduino IDE library for 6 DOF IMU, ICM42688.
This device is relatively new and I could not find any existing arduino library from the internet, hence decided to write it myself as a part of my project needed this IMU to work with Teensy 4.1 using Arduino IDE.

### Important Notes
This library only provides basic functions such as initialize IMU, get temperature, gyro and accel data, check and set offset which is sufficient for normal uses. 
This library provides fixed initialization during device start up ( Accel config 2G 100Hz, Gyro 31.25dps 100Hz ). You may need to change the code in the function of defaultinit_imu() and get_MGDPSdata() in ICM42688.cpp for the scale that you desired. 

### Wiring with Teensy 4.1 
