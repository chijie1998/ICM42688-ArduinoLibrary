/*
  ICM42688.cpp - Library for ICM42688.
  Created by Sam Tan, January 31, 2022.
  Released into the public domain.
*/

#include "Arduino.h"
#include "SPI.h"
#include "ICM42688.h"
#include <cstdlib>

#define TIMEOUT     10000
#define DUMMY       0x00

ICM42688::ICM42688(int freq, int cs)
{
  pinMode(cs, OUTPUT);
  _cs = cs;
  _freq= freq;
}

int ICM42688::check_data_ready(uint8_t flag)
{
    uint8_t tmp_data;
    uint16_t timeout_cnt = 0;
    for ( ; ; )
    {
        ICM42688::read_SPI(C6DOFIMU14_REG0_INT_STATUS_1, &tmp_data, 1 );
        delay(1);
        if ( tmp_data & flag )
        {
            return 2;
        }
        if ( timeout_cnt++ > TIMEOUT )
        {
            return -2;
        }
    }
}


void ICM42688::read_SPI(uint8_t reg, uint8_t *buff, uint32_t len)
{
  SPISettings settingsA(_freq, MSBFIRST, SPI_MODE0);
  SPI.beginTransaction(settingsA);
  SPI.transfer(0x00);
  SPI.endTransaction();
  //unsigned int result = 0;
  uint8_t cmd=((reg & 0x7F) | 0x80);
  // Serial.print("Read at address :");
  // Serial.println(cmd,BIN);
  SPI.beginTransaction(settingsA);
  digitalWrite(_cs, LOW);
  delayMicroseconds(5);
  SPI.transfer(cmd);
  for (uint32_t indi = 0; indi < len; indi++)
  {
    *(buff + indi) = SPI.transfer(0x00);
  }
  delayMicroseconds(5);
  //*buff = SPI.transfer(0x00);
  //result = SPI.transfer(0x00);
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();
  //Serial.print("Read result :");
  //Serial.println(result,HEX);

  
}

void ICM42688::write_SPI(uint8_t reg, uint8_t *data, uint32_t len) 
{
  SPISettings settingsA(_freq, MSBFIRST, SPI_MODE0);
  SPI.beginTransaction(SPISettings(settingsA));
  SPI.transfer(0x00);
  SPI.endTransaction();
  uint8_t cmd=((reg & 0x7F) | 0x00);
  // Serial.print("Write at address :");
  // Serial.println(cmd,BIN);
  // Serial.print("Write ");
  // Serial.println(*data,BIN);
  SPI.beginTransaction(settingsA);
  digitalWrite(_cs, LOW);
  delayMicroseconds(5);
  SPI.transfer(cmd);
  for (uint32_t indi = 0; indi < len; indi++)
  {
    SPI.transfer(*(data +indi));
  }
  delayMicroseconds(5);
  // take the chip select high to de-select:
  digitalWrite(_cs, HIGH);
  // release control of the SPI port
  SPI.endTransaction();
}

void ICM42688::defaultinit_imu()
{
  uint8_t tmp_data;
  int error_check;
  bool err;
  ICM42688::software_reset();
  tmp_data = C6DOFIMU14_DRIVECONFIG_MIN_SLEW_RATE;
  ICM42688::write_SPI(C6DOFIMU14_REG0_DRIVE_CONFIG, &tmp_data,1); //config drive
  ICM42688::read_SPI(C6DOFIMU14_REG0_WHO_AM_I, &tmp_data,1);//get device id
  if ( tmp_data != C6DOFIMU14_DEF_WHO_AM_I )//compare default device ID
  {
    error_check= -1;
      err=ICM42688::err_chk(error_check);
    if (err== false){
      return;
    }
  }
  tmp_data =  C6DOFIMU14_PWR_TEMP_ENABLE | 
              C6DOFIMU14_PWR_IDLE_0 |
              C6DOFIMU14_PWR_GYRO_MODE_LOW_NOISE |
              C6DOFIMU14_PWR_ACCEL_MODE_LOW_NOISE;
  ICM42688::write_SPI(C6DOFIMU14_REG0_PWR_MGMT_0, &tmp_data,1);//config power moanagement mode, set RC oscillator to off when accel and gyro are off, set gyro and accel to Low Noise mode
  delay(1);
  tmp_data = C6DOFIMU14_ACFG0_FS_SEL_2G  |
            C6DOFIMU14_ACFG0_ODR_100HZ ;
  ICM42688::write_SPI(C6DOFIMU14_REG0_ACCEL_CONFIG_0, &tmp_data,1); //Accel config 2G 100Hz
  tmp_data =  C6DOFIMU14_GCFG0_FS_SEL_31p25DPS |
              C6DOFIMU14_GCFG0_ODR_100HZ;
  ICM42688::write_SPI(C6DOFIMU14_REG0_GYRO_CONFIG_0, &tmp_data,1); //Gyro 31.25dps 100Hz
  tmp_data = C6DOFIMU14_FIFOCONFIG_STREAM_TO_FIFO_MODE;
  ICM42688::write_SPI(C6DOFIMU14_REG0_FIFO_CONFIG, &tmp_data,1);
  tmp_data =  C6DOFIMU14_FIFOCONFIG1_TEMP_EN |
              C6DOFIMU14_FIFOCONFIG1_GYRO_EN |
              C6DOFIMU14_FIFOCONFIG1_ACCEL_EN;
  ICM42688::write_SPI(C6DOFIMU14_REG0_FIFO_CONFIG_1, &tmp_data,1);
  delay(100);
  error_check= 0;
  err=ICM42688::err_chk(error_check);
  if (err== false){
    return;
  }
}

void ICM42688::software_reset()
{
  uint8_t tmp_data = C6DOFIMU14_DEVCONFIG_SOFT_RESET_ENABLE;
  ICM42688::write_SPI(C6DOFIMU14_REG0_DEVICE_CONFIG, &tmp_data,1);
}

bool ICM42688::err_chk(int error)
{
  if (error ==  -1){
    //Serial.println("Error in initialising!");  
    return false;  
  }
  else if (error ==  -2){
    //Serial.println("Data not ready!");    
    return false; 
  }
  else if (error ==  2){
    //Serial.println("Data is ready!");  
    return true;    
  }
  else if (error ==  1){
    //Serial.println("Error getting values!");   
    return false;   
  }  
  else if (error ==  0){
    //Serial.println("Initialisation success!");    
    return true;  
  }
  else {
    //Serial.println("Initialisation success!"); 
    return true;  }
}

void ICM42688::get_accel_axis(ICM42688_axis_t *axis)
{
  uint8_t tmp_data;
  bool err;
  int error_check = ICM42688::check_data_ready(C6DOFIMU14_INTSTATUS_DATA_RDY);
  err = ICM42688::err_chk(error_check);
  if (err== false){
    return;
  }
  ICM42688::read_SPI(C6DOFIMU14_REG0_ACCEL_X_MSB, &tmp_data, 1);
  axis->x = tmp_data;
  axis->x <<= 8;
  ICM42688::read_SPI(C6DOFIMU14_REG0_ACCEL_X_LSB, &tmp_data, 1);
  axis->x |= tmp_data;

  ICM42688::read_SPI(C6DOFIMU14_REG0_ACCEL_Y_MSB, &tmp_data, 1);
  axis->y = tmp_data;
  axis->y <<= 8;
  ICM42688::read_SPI(C6DOFIMU14_REG0_ACCEL_Y_LSB, &tmp_data, 1);
  axis->y |= tmp_data;

  ICM42688::read_SPI(C6DOFIMU14_REG0_ACCEL_Z_MSB, &tmp_data, 1);
  axis->z = tmp_data;
  axis->z <<= 8;
  ICM42688::read_SPI(C6DOFIMU14_REG0_ACCEL_Z_LSB, &tmp_data, 1);
  axis->z |= tmp_data;
}

void ICM42688::get_gyro_axis(ICM42688_axis_t *axis)
{
  uint8_t tmp_data;
  bool err;
  int error_check = ICM42688::check_data_ready(C6DOFIMU14_INTSTATUS_DATA_RDY);
  err = ICM42688::err_chk(error_check);
  if (err== false){
    return;
  }
  ICM42688::read_SPI(C6DOFIMU14_REG0_GYRO_X_MSB, &tmp_data, 1);
  axis->x = tmp_data;
  axis->x <<= 8;
  ICM42688::read_SPI(C6DOFIMU14_REG0_GYRO_X_LSB, &tmp_data, 1);
  axis->x |= tmp_data;

  ICM42688::read_SPI(C6DOFIMU14_REG0_GYRO_Y_MSB, &tmp_data, 1);
  axis->y = tmp_data;
  axis->y <<= 8;
  ICM42688::read_SPI(C6DOFIMU14_REG0_GYRO_Y_LSB, &tmp_data, 1);
  axis->y |= tmp_data;

  ICM42688::read_SPI(C6DOFIMU14_REG0_GYRO_Z_MSB, &tmp_data, 1);
  axis->z = tmp_data;
  axis->z <<= 8;
  ICM42688::read_SPI(C6DOFIMU14_REG0_GYRO_Z_LSB, &tmp_data, 1);
  axis->z |= tmp_data;
}

void ICM42688::get_temperature(float *temperature){
  int16_t raw_data;
  uint8_t tmp_data;
  bool err;
  int error_check = ICM42688::check_data_ready(C6DOFIMU14_INTSTATUS_DATA_RDY);
  err = ICM42688::err_chk(error_check);
  if (err== false){
    return;
  }
  ICM42688::read_SPI(C6DOFIMU14_REG0_TEMP_DATA_MSB, &tmp_data, 1);
  raw_data = tmp_data;
  raw_data <<= 8;
  ICM42688::read_SPI(C6DOFIMU14_REG0_TEMP_DATA_LSB, &tmp_data, 1);
  raw_data |= tmp_data;
  *temperature = ( ( float ) raw_data / 132.48 ) + 25.00;
}

void ICM42688::get_data(ICM42688_axis_t *acc_axis,ICM42688_axis_t *gyro_axis ){
  uint8_t tmp_data[ 16 ];
  bool err;
  int error_check = ICM42688::check_data_ready(C6DOFIMU14_INTSTATUS_FIFO_FULL);
  err = ICM42688::err_chk(error_check);
  if (err== false){
    return;
  }
  ICM42688::read_SPI(C6DOFIMU14_REG0_FIFO_DATA, tmp_data, 16);
  if ( ( tmp_data[ 0 ] & 0xFC ) == ( C6DOFIMU14_FIFOHEADER_ACCEL | 
                                       C6DOFIMU14_FIFOHEADER_GYRO | 
                                       C6DOFIMU14_FIFOHEADER_TIMESTAMP_ODR ) )
    {
      acc_axis->x = tmp_data[ 1 ];
        acc_axis->x <<= 8;
        acc_axis->x |= tmp_data[ 2 ];

        acc_axis->y = tmp_data[ 3 ];
        acc_axis->y <<= 8;
        acc_axis->y |= tmp_data[ 4 ];
        
        acc_axis->z = tmp_data[ 5 ];
        acc_axis->z <<= 8;
        acc_axis->z |= tmp_data[ 6 ];
        
        gyro_axis->x = tmp_data[ 7 ];
        gyro_axis->x <<= 8;
        gyro_axis->x |= tmp_data[ 8 ];
        
        gyro_axis->y = tmp_data[ 9 ];
        gyro_axis->y <<= 8;
        gyro_axis->y |= tmp_data[ 10 ];
        
        gyro_axis->z = tmp_data[ 11 ];
        gyro_axis->z <<= 8;
        gyro_axis->z |= tmp_data[ 12 ];
    }
}

void ICM42688::get_MGDPSdata(ICM42688_axis_f *acc_axisf,ICM42688_axis_f *gyro_axisf ){
  ICM42688_axis_t acc_axis;
  ICM42688_axis_t gyro_axis;
  uint8_t tmp_data[ 16 ];
  bool err;
  int error_check = ICM42688::check_data_ready(C6DOFIMU14_INTSTATUS_FIFO_FULL);
  err = ICM42688::err_chk(error_check);
  if (err== false){
    return;
  }
  ICM42688::read_SPI(C6DOFIMU14_REG0_FIFO_DATA, tmp_data, 16);
  if ( ( tmp_data[ 0 ] & 0xFC ) == ( C6DOFIMU14_FIFOHEADER_ACCEL | 
                                       C6DOFIMU14_FIFOHEADER_GYRO | 
                                       C6DOFIMU14_FIFOHEADER_TIMESTAMP_ODR ) )
    {
      acc_axis.x = tmp_data[ 1 ];
        acc_axis.x <<= 8;
        acc_axis.x |= tmp_data[ 2 ];

        acc_axis.y = tmp_data[ 3 ];
        acc_axis.y <<= 8;
        acc_axis.y |= tmp_data[ 4 ];
        
        acc_axis.z = tmp_data[ 5 ];
        acc_axis.z <<= 8;
        acc_axis.z |= tmp_data[ 6 ];
        
        gyro_axis.x = tmp_data[ 7 ];
        gyro_axis.x <<= 8;
        gyro_axis.x |= tmp_data[ 8 ];
        
        gyro_axis.y = tmp_data[ 9 ];
        gyro_axis.y <<= 8;
        gyro_axis.y |= tmp_data[ 10 ];
        
        gyro_axis.z = tmp_data[ 11 ];
        gyro_axis.z <<= 8;
        gyro_axis.z |= tmp_data[ 12 ];

      acc_axisf->x = ((float)acc_axis.x)/16384;
      acc_axisf->y = ((float)acc_axis.y)/16384;
      acc_axisf->z = ((float)acc_axis.z)/16384;

      gyro_axisf->x = ((float)gyro_axis.x)/1048.6;
      gyro_axisf->y = ((float)gyro_axis.y)/1048.6;
      gyro_axisf->z = ((float)gyro_axis.z)/1048.6;
    }
}

void ICM42688::offset(int g_xoff,int g_yoff,int g_zoff,int a_xoff,int a_yoff,int a_zoff)
{
  float gyro_xoff;float gyro_yoff;float gyro_zoff;float accel_xoff;float accel_yoff;float accel_zoff;
  uint8_t tmp_data;
  uint8_t buffer[2];
  uint8_t buffer1[2];
  uint8_t tmp;
  int absval;
  int16_t twobytes;

  tmp_data = C6DOFIMU14_BANK_4;
  ICM42688::write_SPI(C6DOFIMU14_REG0_BANK_SEL, &tmp_data,1); //change bank to bank 4

  ICM42688::read_SPI(C6DOFIMU14_REG0_BANK_SEL, &tmp_data,1);//check bank selection
  if ( tmp_data != C6DOFIMU14_BANK_4 )
  {
    Serial.println("Cant access bank four to set offset");
    return;
  }
  
  absval = abs(g_xoff);
  Serial.println(absval,DEC);
  if (absval>67110){
    Serial.println("Gyro x Offset out of range, auto set to max offset");
    if (g_xoff>=0){
      g_xoff = 67110;
    }
    else{g_xoff = -67110;}
  }
  absval = abs(g_yoff);
  if (absval>67110){
    Serial.println("Gyro y Offset out of range, auto set to max offset");
    if (g_yoff>=0){
      g_yoff = 67110;
    }
    else{g_yoff = -67110;}
  }  
  absval = abs(g_zoff);
  if (absval>67110){
    Serial.println("Gyro z Offset out of range, auto set to max offset");
    if (g_zoff>=0){
      g_zoff = 67110;
    }
    else{g_zoff = -67110;}
  }
  absval = abs(a_xoff);
  if (absval>16384){
    Serial.println("Accel x Offset out of range, auto set to max offset");
    if (a_xoff>=0){
      a_xoff = 16384;
    }
    else{a_xoff = -16384;}
  }
  absval = abs(a_yoff);
  if (absval>16384){
    Serial.println("Accel y Offset out of range, auto set to max offset");
    if (a_yoff>=0){
      a_yoff = 16384;
    }
    else{a_yoff = -16384;}
  }
  absval = abs(a_zoff);
  if (absval>16384){
    Serial.println("Accel z Offset out of range, auto set to max offset");
    if (a_zoff>=0){
      a_zoff = 16384;
    }
    else{a_zoff = -16384;}
  }
  gyro_xoff = ((float)g_xoff/1048.6);
  gyro_yoff = ((float)g_yoff/1048.6);
  gyro_zoff = ((float)g_zoff/1048.6);
  accel_xoff = ((float)a_xoff/16384);
  accel_yoff = ((float)a_yoff/16384);
  accel_zoff = ((float)a_zoff/16384);

  twobytes=(gyro_xoff/0.03125);
  twobytes=((twobytes ^ 0xFFFF) + 0x0001); //2's complements
  buffer[0] = twobytes & 0xFF; // split lower 8 bit
  buffer[1] = (twobytes>>8) & 0x0F; //split bit 9 to 12
  
  ICM42688::write_SPI(C6DOFIMU14_REG4_OFFSET_USER_0, &buffer[0],1);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_0, &tmp_data,1);
  if ( tmp_data != buffer[0] )
  {
    Serial.println("Failed to set gyro x offset");
    return;
  }

  twobytes=gyro_yoff/0.03125;
  twobytes=((twobytes ^ 0xFFFF) + 0x0001); //2's complements
  buffer1[0] = twobytes & 0xFF;
  buffer1[1] = ((twobytes>>8) & 0x0F)<<4;
  tmp= buffer1[1] | buffer[1];
  ICM42688::write_SPI(C6DOFIMU14_REG4_OFFSET_USER_1, &tmp,1);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_1, &tmp_data,1);
  if ( tmp_data != tmp  )
  {
    Serial.println("Failed to set gyro x and y offset");
    return;
  }
  ICM42688::write_SPI(C6DOFIMU14_REG4_OFFSET_USER_2, &buffer1[0],1);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_2, &tmp_data,1);
  if ( tmp_data != buffer1[0]  )
  {
    Serial.println("Failed to set gyro y offset");
    return;
  }

  twobytes=gyro_zoff/0.03125;
  twobytes=((twobytes ^ 0xFFFF) + 0x0001); //2's complements
  buffer[0] = twobytes & 0xFF; // split lower 8 bit
  buffer[1] = (twobytes>>8) & 0x0F; //split bit 9 to 12
  
  ICM42688::write_SPI(C6DOFIMU14_REG4_OFFSET_USER_3, &buffer[0],1);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_3, &tmp_data,1);
  if ( tmp_data != buffer[0] )
  {
    Serial.println("Failed to set gyro z offset");
    return;
  }

  twobytes=accel_xoff/0.0005;
  twobytes=((twobytes ^ 0xFFFF) + 0x0001); //2's complements
  buffer1[0] = twobytes & 0xFF;
  buffer1[1] = ((twobytes>>8) & 0x0F)<<4;
  tmp= buffer1[1] | buffer[1];
  ICM42688::write_SPI(C6DOFIMU14_REG4_OFFSET_USER_4, &tmp,1);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_4, &tmp_data,1);
  if ( tmp_data != tmp  )
  {
    Serial.println("Failed to set accel x and gyro z offset");
    return;
  }
  ICM42688::write_SPI(C6DOFIMU14_REG4_OFFSET_USER_5, &buffer1[0],1);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_5, &tmp_data,1);
  if ( tmp_data != buffer1[0]  )
  {
    Serial.println("Failed to set accel x offset");
    return;
  }


  twobytes=accel_yoff/0.0005;
  twobytes=((twobytes ^ 0xFFFF) + 0x0001); //2's complements
  buffer[0] = twobytes & 0xFF; // split lower 8 bit
  buffer[1] = (twobytes>>8) & 0x0F; //split bit 9 to 12
  ICM42688::write_SPI(C6DOFIMU14_REG4_OFFSET_USER_6, &buffer[0],1);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_6, &tmp_data,1);
  if ( tmp_data != buffer[0] )
  {
    Serial.println("Failed to set accel y offset");
    return;
  }
  twobytes=accel_zoff/0.0005;
  twobytes=((twobytes ^ 0xFFFF) + 0x0001); //2's complements
  buffer1[0] = twobytes & 0xFF;
  buffer1[1] = ((twobytes>>8) & 0x0F)<<4;
  tmp= buffer1[1] | buffer[1];
  ICM42688::write_SPI(C6DOFIMU14_REG4_OFFSET_USER_7, &tmp,1);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_7, &tmp_data,1);
  if ( tmp_data != tmp  )
  {
    Serial.println("Failed to set accel y and accel z offset");
    return;
  }
  ICM42688::write_SPI(C6DOFIMU14_REG4_OFFSET_USER_8, &buffer1[0],1);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_8, &tmp_data,1);
  if ( tmp_data != buffer1[0]  )
  {
    Serial.println("Failed to set accel z offset");
    return;
  }

  tmp_data = C6DOFIMU14_BANK_0;
  ICM42688::write_SPI(C6DOFIMU14_REG0_BANK_SEL, &tmp_data,1); //change bank to bank 0

  ICM42688::read_SPI(C6DOFIMU14_REG0_BANK_SEL, &tmp_data,1);//check bank selection
  if ( tmp_data != C6DOFIMU14_BANK_0 )
  {
    Serial.println("Cant access bank zero after setting offset");
    return;
  }

  Serial.println("Successfully configure offset!");
  
}

void ICM42688::checkoffset(){
  uint8_t tmp_data;

  tmp_data = C6DOFIMU14_BANK_4;
  ICM42688::write_SPI(C6DOFIMU14_REG0_BANK_SEL, &tmp_data,1); //change bank to bank 4

  ICM42688::read_SPI(C6DOFIMU14_REG0_BANK_SEL, &tmp_data,1);//check bank selection
  if ( tmp_data != C6DOFIMU14_BANK_4 )
  {
    Serial.println("Cant access bank four to set offset");
    return;
  }
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_0, &tmp_data,1);
  Serial.print("0 :");
  Serial.println(tmp_data,BIN);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_1, &tmp_data,1);
  Serial.print("1 :");
  Serial.println(tmp_data,BIN);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_2, &tmp_data,1);
  Serial.print("2 :");
  Serial.println(tmp_data,BIN);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_3, &tmp_data,1);
  Serial.print("3 :");
  Serial.println(tmp_data,BIN);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_4, &tmp_data,1);
  Serial.print("4 :");
  Serial.println(tmp_data,BIN);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_5, &tmp_data,1);
  Serial.print("5 :");
  Serial.println(tmp_data,BIN);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_6, &tmp_data,1);
  Serial.print("6 :");
  Serial.println(tmp_data,BIN);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_7, &tmp_data,1);
  Serial.print("7 :");
  Serial.println(tmp_data,BIN);
  ICM42688::read_SPI(C6DOFIMU14_REG4_OFFSET_USER_8, &tmp_data,1);
  Serial.print("8 :");
  Serial.println(tmp_data,BIN);

  tmp_data = C6DOFIMU14_BANK_0;
  ICM42688::write_SPI(C6DOFIMU14_REG0_BANK_SEL, &tmp_data,1); //change bank to bank 0

  ICM42688::read_SPI(C6DOFIMU14_REG0_BANK_SEL, &tmp_data,1);//check bank selection
  if ( tmp_data != C6DOFIMU14_BANK_0 )
  {
    Serial.println("Cant access bank zero after setting offset");
    return;
  }
}