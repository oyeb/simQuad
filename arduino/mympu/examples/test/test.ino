/*
  -=-=-=-=-=-=-=-=-=-=-=-=  IMPORTANT  -=-=-=-=-=-=-=-=-=-=-=-=
+ Baud rate 57600
+ Connect only 3.3V to MPU Vcc pin NOT 5V
+ Connect AD0 pin to ground
+ All  I2C reads/writes (for 1byte) take ~960us.
+ Additional reads/writes (in blocks) take 100us (per byte) more!!
+ CAL_DEBUG can be used only on Arduino IDE Serial Monitor. All .py files and quad functions fail in this mode
*/
#include "mympu.h"
#include "I2Cdev.h"
#include "Wire.h"

volatile bool mint=false;
uint8_t int_status, script_ready=0;
int16_t acc_gyro[6]; //intended global variable <---> MPU

void setup() {
  Serial.begin(57600);
  mpu_init();
  cfgr_mpu_off();  //Read offsets from OFFSETS_EEPROM
  //cal_mpu_off(1); //If need to reconfig eeprom consts.
  
  attachInterrupt(0, mpu_interrupt, RISING);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void loop(){
  if (mint){
    mpu_get_int_status(&int_status);
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
          Serial.write( &hi, 1 );
          Serial.write( &lo, 1 );
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

void mpu_interrupt(){
  //This interrupt (from MPU) fires every ~4990-5000us
  mint = true;
}