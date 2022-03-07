#include <KalmanFilter.h>
#include <ICM42688.h>
#include <SPI.h>
#define HW_TIMER_INTERVAL_US    5000L
ICM42688 icm(6000000,10);
uint8_t buf = 0x00;
int x=0;
float temperature;
ICM42688_axis_t accel;
ICM42688_axis_t gyro;
ICM42688_axis_f accelf;
ICM42688_axis_f gyrof;
KalmanFilter kalmanfilter;
double angleoutput;
double kp = 40, ki = 0.0, kd = 0.6;//You need to modify the parameters, angle PD control.
double kp_speed =5.20, ki_speed = 0.25, kd_speed = 0.0;// You need to modify the parameters, Speed PD control.
double kp_turn = 23, ki_turn = 0, kd_turn = 0.3;//Rotating PD settings
/********************angle data*********************/
float Q;
float Angle_ax; //Tilt angle calculated from acceleration
float Angle_ay;
float K1 = 0.05; // Weight of accelerometer values
float angle0 = 0.00; //Mechanical balance angle
int slong;
/***************Kalman_Filter*********************/
float Q_angle = 0.001, Q_gyro = 0.005; //Confidence of angle data and confidence of angular velocity data
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5; //Filtering time interval millisecond
float dt = timeChange * 0.001; //Note: the value of DT is filter sampling time.
/***************Kalman_Filter*********************/
/***************Timer/Interrupt*********************/
IntervalTimer myTimer;
elapsedMillis sincePrint;
/***************Timer/Interrupt*********************/
void TimerHandler()
{ sei();
  icm.get_data(&accel,&gyro);
  kalmanfilter.Angletest(accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  cli();
}

void setup() {
  // put your setup code here, to run once:
  //pinMode(LED_BUILTIN, OUTPUT);
  SPI.begin(); 
  Serial.begin(115200);
  while (!Serial && millis() < 15000);        
  Serial.println(F("\nSPI Test\n"));
  icm.defaultinit_imu();
  delay(100);
//icm.offset(-280,-320,168,-330,-44,-16500); // input is raw data
//icm.offset(-280,-320,168,0,0,0);
  icm.offset(0,0,0,0,0,0);// originally 0 offset, could comment out this line.
  delay(100);
  
  icm.checkoffset();// check offset register value, could comment out. this is for debugging.
  delay(1000);
  myTimer.begin(TimerHandler, 5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  //if (x < 2){
  //icm.get_data(&accel,&gyro);
//  icm.get_temperature(&temperature);
//  icm.get_MGDPSdata(&accelf,&gyrof);
//  kalmanfilter.Angletest(accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
//  angleoutput=kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;//PD angle control
//  Serial.print(" Accel X:");
//  Serial.print(accel.x, DEC);
//  Serial.print(" Accel Y:");
//  Serial.print(accel.y, DEC);
//  Serial.print(" Accel Z:");
//  Serial.println(accel.z, DEC);
//  Serial.print(" Gyro X:");
//  Serial.print(gyro.x, DEC);
//  Serial.print(" Gyro Y:");
//  Serial.print(gyro.y, DEC);
//  Serial.print(" Gyro Z:");
//  Serial.println(gyro.z, DEC);
//  Serial.print(" Temperature:");
//  Serial.println(temperature, DEC);
//  Serial.print(" Accel X:");
//  Serial.print(accelf.x, DEC);
//  Serial.print(" Accel Y:");
//  Serial.print(accelf.y, DEC);
//  Serial.print(" Accel Z:");
//  Serial.println(accelf.z, DEC);
//  Serial.print(" Gyro X:");
//  Serial.print(gyrof.x, DEC);
//  Serial.print(" Gyro Y:");
//  Serial.print(gyrof.y, DEC);
//  Serial.print(" Gyro Z:");
//  Serial.println(gyrof.z, DEC);
//  Serial.print(" Angle by accel:");
//  Serial.println(kalmanfilter.Angle, DEC);
//  Serial.print(" Angle:");
//  Serial.println(kalmanfilter.angle, DEC);
//  Serial.print(" Angle output for PID control:");
//  Serial.println(angleoutput, DEC);
//  Serial.print("----------------------------------\r\n");
//  delay(1000);
//  x++;
//  }
//simpleTimer.run();
//  static unsigned long lastTimer0   = 0;
//  //static bool timer0Stopped         = false;
//  static unsigned long currTime     = 0;;
//
//  currTime = millis();
//   
//  if (currTime - lastTimer0 > 500)
//  {
//    lastTimer0 = currTime;
// if (sincePrint > 10){
noInterrupts();
if (sincePrint > 250){
  Serial.print(" Accel X:");
  Serial.print(accel.x, DEC);
  Serial.print(" Accel Y:");
  Serial.print(accel.y, DEC);
  Serial.print(" Accel Z:");
  Serial.println(accel.z, DEC);
  Serial.print(" Gyro X:");
  Serial.print(gyro.x, DEC);
  Serial.print(" Gyro Y:");
  Serial.print(gyro.y, DEC);
  Serial.print(" Gyro Z:");
  Serial.println(gyro.z, DEC);
  Serial.print(" Angle by accel:");
  Serial.println(kalmanfilter.Angle, DEC);
  Serial.print(" Angle:");
  Serial.println(kalmanfilter.angle, DEC);
  Serial.print("----------------------------------\r\n");
  sincePrint=0;
  }
interrupts();
}
