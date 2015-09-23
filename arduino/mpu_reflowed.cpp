/*
22 Sep 2015 Removed MPU6050.h completely, Joined calibration procs, Provide EEPROM save
_________________________________________________________________________________________

  -=-=-=-=-=-=-=-=-=-=-=-=  IMPORTANT  -=-=-=-=-=-=-=-=-=-=-=-=
+ Baud rate 57600
+ Connect only 3.3V to MPU Vcc pin NOT 5V
+ Connect AD0 pin to ground
+ All  I2C reads/writes (for 1byte) take ~960us.
+ Additional reads/writes (in blocks) take 100us (per byte) more!!
+ CAL_DEBUG can be used only on Arduino IDE Serial Monitor. All .py files and quad functions
  fail in this mode

FEATURES
+ Cleaner, unified method of comms. with MPU via I2CDevLib commands.
+ cal_mpu_off(mode) writes offset values into eeprom for later use. Makes bootup faster and
  these offsets determine the "home" position. If cal_mpu_off() is used at every bootup, a tilted
  starting orientation would be interpreted as flat, causing a catastrophe!
  ___________________________________________________
  cal_mpu_off(mode) MUST BE USED ONLY WHEN DEVICE IS LEVEL!
  ---------------------------------------------------
+ cfgr_mpu_off() MUST BE USED FOR USUAL TAKEOFFS

*  I2C operations are extremely costly. Minimise them in the loop() or else... 
     -  1  byte   480us
     -  2  bytes  580us
     - 12 bytes  1580us

*_* This can be reduced by a factor of ~4 if twi.h CONSTANTS are changed so that the
*_* I2C bus communicates @ 400kHz rather than standard 100kHz
*_* OR we use FastWire library for I2C rather than the default Wire library. I2Cdev works
*_* with both!!
  -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#include "I2Cdev.h"
#include "Wire.h"
#include <avr/eeprom.h>
// Device default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
volatile bool mint=false;
uint8_t int_status, temp, script_ready=0;
int16_t acc_t_gyro[7]; //intended global variable <---> MPU

// function declarations here:
void cal_mpu_off(uint8_t);
void cfgr_mpu_off();
void mpu_interrupt();

#define OFFSETS_EEPROM 12 // accel_gyro offsets from 12 thru 26
//#define CAL_DEBUG

void setup() {
  Wire.begin();
  Serial.begin(57600);
  I2Cdev::writeByte(0x68, 0x6B, 0x01); //PWR_MGMT1 for clock source as X-gyro
  uint8_t config_info[4] = {4, 3, 8, 0}; //, ACCEL range:2g | Refer the Datasheet if you want to change these.
  //0x19[SMPRT_DIV]: 0x04 set SamplingRate as (clock_source / 4) = 2kHz [Accelerometer has max sample rate of 1kHz]
  //0x1A[CONFIG]   : 0x03 set DLPF_CFG[2-0] bits in as 3 rest are all zero anyways.
  //0x1B[GYRO_CFG] : 0x08 set GYRO_CFG_FS_SEL[4:3] = 10  range:500
  //0x1C[ACCEL_CFG]: 0x00 set ACCEL_CFG_FS_SEL[4:3] = 00 range:2g
  I2Cdev::writeBytes(0x68, 0x19, 4, config_info);
  I2Cdev::writeByte(0x68, 0x38, 0x01); //INT_EN
  
  attachInterrupt(0, mpu_interrupt, RISING);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  //cal_mpu_off(1); //If need to reconfig eeprom consts.
  cfgr_mpu_off();  //Read offsets from OFFSETS_EEPROM
}

void loop(){
  if (mint){
    I2Cdev::readByte(0x68, 0x3A, &int_status);
    I2Cdev::readWords(0x68, 0x3B, 7, (uint16_t *)acc_t_gyro);
    //from 3b to 48
    //ax ay az temp gx gy gz {all 2 byte}
    #ifdef CAL_DEBUG  //give readable o/p
      Serial.print(acc_t_gyro[0]);
      Serial.print(" ");
      Serial.print(acc_t_gyro[1]);
      Serial.print(" ");
      Serial.println(acc_t_gyro[2]);
    #endif
    //give binary o/p
    #ifndef CAL_DEBUG
      if (script_ready){
        uint8_t i, hi, lo;
        for (i=0; i<=12; i+=2){
          hi = (acc_t_gyro[i/2] >> 8);
          lo = (acc_t_gyro[i/2] & 0xFF);
          Serial.write( &hi, 1 );
          Serial.write( &lo, 1 );
          if (i==4) i=6;
        }
      }
      else if(Serial.available() > 0){
        Serial.read();
        script_ready=1;
      }
    #endif
    mint = false;
  }
  else{
    //other non-motion work!
    ;
  }
}

#define SAMPLE_COUNT 100
#define ITERATIONS 6

void cfgr_mpu_off(){
  /*
    READS FROM OFFSETS_EEPROM AND WRITES TO OFFSET REGISTERS
  */
  uint8_t i, j;
  #ifdef CAL_DEBUG
    Serial.println("\nCFGR_OFFS\n");
  #endif
  for (i=0, j=0x06; i<=10; i+=2, j+=2){ //06, 08, 0A, 13, 15, 17
    I2Cdev::writeWord(0x68, j, eeprom_read_word((uint16_t *)(OFFSETS_EEPROM+i)));
    #ifdef CAL_DEBUG
      Serial.print(j); Serial.print("\t");Serial.println((int16_t)eeprom_read_word((uint16_t *)(OFFSETS_EEPROM+i)));
    #endif
    if (j==0x0A) j=0x11;
  }
}

void cal_mpu_off(uint8_t mode){
  /*
  UPDATES EEPROM WORDS
    mode 3: both
    mode 1: accel
    mode 2: gyro
  */
  int16_t acc_gyro_offs[6] = {0};
  byte i=0, count=0;
  float axm=0, aym=0, azm=0;
  float gxm=0, gym=0, gzm=0;
  //axoff ayoff azoff gxoff gyoff gzoff
  I2Cdev::readWords(0x68, 0x06, 3, (uint16_t *)acc_gyro_offs);
  while (i < ITERATIONS){ //hope that offsets converge in 6 iterations
    I2Cdev::readWords(0x68, 0x3B, 7, (uint16_t *)acc_t_gyro);
    if (count == SAMPLE_COUNT){
      acc_gyro_offs[0] += int(axm/-6);
      acc_gyro_offs[1] += int(aym/-6);
      acc_gyro_offs[2] += int((azm+16384)/-6);
      acc_gyro_offs[3] += int(gxm/-3);
      acc_gyro_offs[4] += int(gym/-3);
      acc_gyro_offs[5] += int(gzm/-3);
      if (mode & 1){
        I2Cdev::writeWord(0x68, 0x06, acc_gyro_offs[0]);
        I2Cdev::writeWord(0x68, 0x08, acc_gyro_offs[1]);
        I2Cdev::writeWord(0x68, 0x0A, acc_gyro_offs[2]);
      }
      if (mode & 2){
        I2Cdev::writeWord(0x68, 0x13, acc_gyro_offs[3]);
        I2Cdev::writeWord(0x68, 0x15, acc_gyro_offs[4]);
        I2Cdev::writeWord(0x68, 0x17, acc_gyro_offs[5]);
      }
      count = 0;
      i++; //iteration++
    }
    else{
      axm = (axm*count + acc_t_gyro[0])/(count+1.0);
      aym = (aym*count + acc_t_gyro[1])/(count+1.0);
      azm = (azm*count + acc_t_gyro[2])/(count+1.0);
      gxm = (gxm*count + acc_t_gyro[4])/(count+1.0);
      gym = (gym*count + acc_t_gyro[5])/(count+1.0);
      gzm = (gzm*count + acc_t_gyro[6])/(count+1.0);
      count++;
    }
  }
  //Update EEPROM @OFFSETS_EEPROM
  if (mode & 1){
    eeprom_update_word((uint16_t *)(OFFSETS_EEPROM), acc_gyro_offs[0]);
    eeprom_update_word((uint16_t *)(OFFSETS_EEPROM+2), acc_gyro_offs[1]);
    eeprom_update_word((uint16_t *)(OFFSETS_EEPROM+4), acc_gyro_offs[2]);
  }
  if (mode & 2){
    eeprom_update_word((uint16_t *)(OFFSETS_EEPROM+6), acc_gyro_offs[3]);
    eeprom_update_word((uint16_t *)(OFFSETS_EEPROM+8), acc_gyro_offs[4]);
    eeprom_update_word((uint16_t *)(OFFSETS_EEPROM+10), acc_gyro_offs[5]);
  }
  #ifdef CAL_DEBUG
    Serial.println("\nACCEL******");
    Serial.print(acc_gyro_offs[0]);
    Serial.print(" ");
    Serial.print(acc_gyro_offs[1]);
    Serial.print(" ");
    Serial.println(acc_gyro_offs[2]);
    Serial.println("GYRO*******");
    Serial.print(acc_gyro_offs[3]);
    Serial.print(" ");
    Serial.print(acc_gyro_offs[4]);
    Serial.print(" ");
    Serial.println(acc_gyro_offs[5]);
    delay(500);
  #endif
}

void mpu_interrupt(){
  //This interrupt (from MPU) fires every ~4990-5000us
  mint = true;
}

/*
ACCEL******
-3122 -28 1748
GYRO*******
56 -42 -11
*/