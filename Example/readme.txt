IMU


This library only consists of hard-coded default settings for the IMU, if want to configure your own settings, please change in the library source code. 
Only SPI interface available.


How to use :
1. Declare ICM42688 class (set SPI freq and chip select pin) :
    ICM42688(int freq, int cs); //Although datasheet mentions that the IMU works up to 20Mhz, for me the SPI communications only works up to 6Mhz.


2. Initialize IMU and set offset :
   void defaultinit_imu();
   void offset(int g_xoff,int g_yoff,int g_zoff,int a_xoff,int a_yoff,int a_zoff); //argument in RAW, actually offsets are computed in dps and g. 
   void checkoffset();
   
3. Available function to call :
    void read_SPI(uint8_t reg, uint8_t *buff, uint32_t len);
    void write_SPI(uint8_t reg, uint8_t *data, uint32_t len);
    int check_data_ready(uint8_t flag);
    void software_reset();
    bool err_chk(int error);
    void get_accel_axis(ICM42688_axis_t *axis);
    void get_gyro_axis(ICM42688_axis_t *axis);
    void get_temperature(float *temperature);
    void get_data(ICM42688_axis_t *acc_axis,ICM42688_axis_t *gyro_axis );
    void get_MGDPSdata(ICM42688_axis_f *acc_axis,ICM42688_axis_f *gyro_axis );


4. Declare ICM42688_axis_t(16bits) or ICM42688_axis_f(converted to g or dps) structure to store accel or gyro data.

5. Call the functions by passing the arguments.

6. Refer librarytest.ino for examples on how to use the library.


Kalman Filter

1. Declare Kalman Filter class, KalmanFilter
2. Define and modify Kalman FIlter variables.
3. Declare Teensy hardware timer interrupt class, IntervalTimer myTimer;
4. Pass raw imu 3 axis gyro and accel data into Angletest() along with Kalman variables.
5. Print Angle and angle in Kalman class. Angle is expected tilted angle based on accel while angle is tilted angle estimated by Kalman filter. 
6. Refer to IMUandKalmanfilter.ino for examples.
