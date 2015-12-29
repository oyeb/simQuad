/*
  27 Sep Tested with all kalman_binary.py, works like a charm
  25 Dec rectified timer_init()
  29 Dec rectified timer_init()
-------------------------------------------------------------------------------------------------------------------
MPU_CONFIGURATION
    * No fifo, raw accel and gyro
    * Accel and gyro offsets are taken from eeprom/calibrated
    * Clock Source:        X-GYRO
    * Sample rate:         200Hz (5ms)
    * DLPF bandwidth:      42Hz
    * gyro sensitivity:    500 dps
    * accel sensitivity:   2g
*/
#include "mympu.h"
#include "I2Cdev.h"
#include "Wire.h"
#include "mytimer.h"

#define CAL_DEBUG

volatile uint8_t mint=0;
void mpu_interrupt_fn(void);

uint8_t script_ready=0;
char cmd_in;
int16_t acc_gyro[6];
uint16_t count=0; //intended global variable <---> MPU

void setup() {
  Serial.begin(57600);
  timer_init(5, &mpu_interrupt_fn);
  mpu_init(0); //No interrupts from MPU reqd.
  cfgr_mpu_off();  //Read offsets from OFFSETS_EEPROM
  //cal_mpu_off(1); //If need to reconfig eeprom consts.
  
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void loop(){
  if (mint){
    #ifdef CAL_DEBUG
      Serial.println(count);
    #endif
    count = 0;
    mpu_getReadings(acc_gyro);
    //give readable o/p
    #ifdef CAL_DEBUG 
      Serial.print(acc_gyro[0]);
      Serial.print(" ");
      Serial.print(acc_gyro[1]);
      Serial.print(" ");
      Serial.println(acc_gyro[2]);
    #endif
    //give binary o/p
    #ifndef CAL_DEBUG
      if (script_ready){
        uint8_t i, hi, lo;
        for (i=0; i<=10; i+=2){
          hi = (acc_gyro[i/2] >> 8);
          lo = (acc_gyro[i/2] & 0xFF);
          Serial.write( &hi, 1);
          Serial.write( &lo, 1);
        }
      }
    #endif
    mint = 0;
  }
  else{
    //other non-motion work!
    if (Serial.available() > 0){
      cmd_in = Serial.read();
      Serial.write(cmd_in+1);
      script_ready = !script_ready;
    }
    count++;
  }
}

void mpu_interrupt_fn(void){
  mint = 1;
}
/*
STATS
ms      count
5       215
6       300
10      640
*/