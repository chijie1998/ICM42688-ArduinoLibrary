/****************************************************************************8
   亚博智能科技有限公司
   产品名称：Arduino 智能平衡小车
   产品型号：BST-ABC ver 1.2

*/
#include <PinChangeInt.h>

#include <MsTimer2.h>
//利用测速码盘计数实现速度PID控制

#include <BalanceCar.h>

#include <KalmanFilter.h>

//I2Cdev、MPU6050和PID_v1类库需要事先安装在Arduino 类库文件夹下
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"



MPU6050 mpu; //实例化一个 MPU6050 对象，对象名称为 mpu
BalanceCar balancecar;
KalmanFilter kalmanfilter;
int16_t ax, ay, az, gx, gy, gz;
//TB6612FNG驱动模块控制信号
#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8

#define tx A0
#define rx A1



#define PinA_left 2  //中断0
#define PinA_right 4 //中断1



//声明自定义变量
int time;
byte inByte; //串口接收字节
int num;
double Setpoint;                               //角度DIP设定点，输入，输出
double kp = 38, ki = 0.0, kd = 0.58;                   //需要你修改的参数
double Setpoints, Outputs = 0;                         //速度DIP设定点，输入，输出
double kp_speed = 3.1, ki_speed = 0.05, kd_speed = 0.0;            // 需要你修改的参数
double kp_turn = 28, ki_turn = 0, kd_turn = 0.29;                 //旋转PID设定
//转向PID参数

double setp0 = 0, dpwm = 0, dl = 0; //角度平衡点，PWM差，死区，PWM1，PWM2
float value;


//********************angle data*********************//
float Q;
float Angle_ax; //由加速度计算的倾斜角度
float Angle_ay;
float K1 = 0.05; // 对加速度计取值的权重
float angle0 = 0.00; //机械平衡角
int slong;

//********************angle data*********************//

//***************Kalman_Filter*********************//


float Q_angle = 0.001, Q_gyro = 0.005; //角度数据置信度,角速度数据置信度
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5; //滤波法采样时间间隔毫秒
float dt = timeChange * 0.001; //注意：dt的取值为滤波器采样时间
//***************Kalman_Filter*********************//

//声明 MPU6050 控制和状态变量



//*********************************************
//******************** speed count ************
//*********************************************

volatile long count_right = 0;//使用volatile lon类型是为了外部中断脉冲计数值在其他函数中使用时，确保数值有效
volatile long count_left = 0;//使用volatile lon类型是为了外部中断脉冲计数值在其他函数中使用时，确保数值有效
int speedcc = 0;



//////////////////////脉冲计算/////////////////////////
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;

/////////////////////脉冲计算////////////////////////////

//////////////转向、旋转参数///////////////////////////////
int turncount = 0; //转向介入时间计算



float turnoutput = 0;




//////////////转向、旋转参数///////////////////////////////

//////////////蓝牙控制量///////////////////
int front = 0;//前进变量
int back = 0;//后退变量
int turnl = 0;//左转标志
int turnr = 0;//右转标志
int spinl = 0;//左旋转标志
int spinr = 0;//右旋转标志
//////////////蓝牙控制量///////////////////

//////////////////超声波速度//////////////////

int chaoshengbo = 0;
int tingzhi = 0;
int jishi = 0;

//////////////////超声波速度//////////////////


//////////////////////脉冲计算///////////////////////
void countpluse()
{

  lz = count_left;
  rz = count_right;
  count_left = 0;
  count_right = 0;

  lpluse = lz;
  rpluse = rz;



  if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0))                     //小车运动方向判断 后退时（PWM即电机电压为负） 脉冲数为负数
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0))                 //小车运动方向判断 前进时（PWM即电机电压为正） 脉冲数为负数
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0))                 //小车运动方向判断 前进时（PWM即电机电压为正） 脉冲数为负数
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0))               //小车运动方向判断 左旋转 右脉冲数为负数 左脉冲数为正数
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }
  //  else if ((pwm1 == 0) && (pwm2 == 0))             //小车运动方向判断 右旋转 左脉冲数为负数 右脉冲数为正数
  //  {
  //    rpluse = 0;
  //    lpluse = 0;
  //  }

  //提起判断
  balancecar.stopr += rpluse;
  balancecar.stopl += lpluse;

  //每5ms进入中断时，脉冲数叠加
  balancecar.pulseright += rpluse;
  balancecar.pulseleft += lpluse;

}
////////////////////脉冲计算///////////////////////



//////////////////角度PD////////////////////
void angleout()
{
  balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;//PD 角度环控制
}
//////////////////角度PD////////////////////

//////////////////////////////////////////////
//////////////////中断定时 5ms定时中断////////////////////
//////////////////////////////////////////////
void inter()
{
  sei();                                            //开中断 由于AVR芯片的局限，无论进入任何中断，在对应中断函数中，芯片会将总中断关闭，这样会影响MPU获取角度数据。所以在这里必须进行开全局中断操作。但是在定时中断中，执行的代码不能超过5ms，不然会破坏了整体的中断。
  countpluse();                                     //脉冲叠加子函数
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC获取MPU6050六轴数据 ax ay az gx gy gz
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro,R_angle,C_0,K1);                                      //获取angle 角度和卡曼滤波
  angleout();                                       //角度环 PD控制
  turncount++;
  if (turncount > 1)                                //10ms进入旋转控制
  {
    turnoutput = balancecar.turnspin(turnl,turnr,spinl,spinr,kp_turn,kd_turn,kalmanfilter.Gyro_z);                                    //旋转子函数
    turncount = 0;
  }
  speedcc++;
  if (speedcc >= 10)                                //50ms进入速度环控制
  {
    Outputs = balancecar.speedpiout(kp_speed,ki_speed,kd_speed,front,back,setp0);
    speedcc = 0;
  }
  balancecar.posture++;
  balancecar.pwma(Outputs,turnoutput,kalmanfilter.angle,turnl,turnr,spinl,spinr,front,back,kalmanfilter.accelz,IN1M,IN2M,IN3M,IN4M,PWMA,PWMB);                                            //小车总PWM输出
}
//////////////////////////////////////////////
//////////////////中断定时////////////////////
//////////////////////////////////////////////



// ===    初始设置     ===
void setup() {
  // TB6612FNGN驱动模块控制信号初始化
  pinMode(IN1M, OUTPUT);                         //控制电机1的方向，01为正转，10为反转
  pinMode(IN2M, OUTPUT);
  pinMode(IN3M, OUTPUT);                        //控制电机2的方向，01为正转，10为反转
  pinMode(IN4M, OUTPUT);
  pinMode(PWMA, OUTPUT);                        //左电机PWM
  pinMode(PWMB, OUTPUT);                        //右电机PWM
  pinMode(STBY, OUTPUT);                        //TB6612FNG使能


  //初始化电机驱动模块
  digitalWrite(IN1M, 0);
  digitalWrite(IN2M, 1);
  digitalWrite(IN3M, 1);
  digitalWrite(IN4M, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  pinMode(PinA_left, INPUT);  //测速码盘输入
  pinMode(PinA_right, INPUT);

  // 加入I2C总线
  Wire.begin();                            //加入 I2C 总线序列

  Serial.begin(9600);                       //开启串口，设置波特率为 115200
  delay(1500);
  mpu.initialize();                       //初始化MPU6050
  delay(2);




  //5ms定时中断设置  使用timer2    注意：使用timer2会对pin3 pin11的PWM输出有影响，因为PWM使用的是定时器控制占空比，所以在使用timer的时候要注意查看对应timer的pin口。
  MsTimer2::set(5, inter);
  MsTimer2::start();

}

////////////////////////bluetooth//////////////////////
void kongzhi()
{
  while (Serial.available())                                    //等待蓝牙数据
    switch (Serial.read())                                      //读取蓝牙数据
    {
      case 0x01: front = 700;   break;                         //前进
      case 0x02: back = -700;   break;                        //后退
      case 0x03: turnl = 1;   break;                          //左转
      case 0x04: turnr = 1;   break;                          //右转
      case 0x05: spinl = 1;   break;                       //左旋转
      case 0x06: spinr = 1;   break;                       //右旋转
      case 0x07: turnl = 0; turnr = 0;  front = 0; back = 0; spinl = 0; spinr = 0;  break;                    //确保按键松开后为停车操作
      case 0x08: spinl = 0; spinr = 0;  front = 0; back = 0;  turnl = 0; turnr = 0;  break;                  //确保按键松开后为停车操作
      case 0x09: front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0; break;       // 确保按键松开后为停车操作
      default: front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0; break;
    }
}


////////////////////////////////////////turn//////////////////////////////////



// ===       主循环程序体       ===
void loop() {

  //主函数中循环检测及叠加脉冲 测定小车车速  使用电平改变既进入脉冲叠加 增加电机的脉冲数，保证小车的精确度。
  attachInterrupt(0, Code_left, CHANGE);
  attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);

  //蓝牙控制
  kongzhi();


}

////////////////////////////////////////pwm///////////////////////////////////



//////////////////////////脉冲中断计算/////////////////////////////////////

void Code_left() {

  count_left ++;

} //左测速码盘计数



void Code_right() {

  count_right ++;

} //右测速码盘计数

//////////////////////////脉冲中断计算/////////////////////////////////////



