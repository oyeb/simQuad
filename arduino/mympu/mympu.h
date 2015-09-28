/*
  24 Sep Initial Commit, tested with {gyro_scope, proscilloscope, kalman_binary}.py
  27 Sep getReadings() is sub-optimal [read comment in ~.cpp to see why.]
         Interrupts from MPU are now optional via <mode> arg to mpu_init()
____________________________________________________________________________________________
FEATURES
+ Cleaner, unified method of comms. with MPU via I2CDevLib commands.
+ cal_mpu_off(mode) writes offset values into eeprom for later use. Makes bootup faster and
  these offsets determine the "home" position. If cal_mpu_off() is used at every bootup, a tilted
  starting orientation would be interpreted as flat, causing a catastrophe!
  +-----------------------------------------------------------+
  | cal_mpu_off(mode) MUST BE USED ONLY WHEN DEVICE IS LEVEL! |
  +-----------------------------------------------------------+
+ cfgr_mpu_off() MUST BE USED FOR USUAL TAKEOFFS
____________________________________________________________________________________________
NOTES
*  I2C operations are extremely costly. Minimise them in the loop() or else... 
     -  1  byte   480us
     -  2  bytes  580us
     - 12 bytes  1580us

*_* This can be reduced by a factor of ~4 if twi.h CONSTANTS are changed so that the
*_* I2C bus communicates @ 400kHz rather than standard 100kHz
*_* OR we use FastWire library for I2C rather than the default Wire library. I2Cdev works
*_* with both!!

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
*/

#include <Arduino.h>
#include "I2Cdev.h"
#include "Wire.h"
#include <avr/eeprom.h>
#include <avr/io.h>

#ifndef MYMPU6050
#define MYMPU6050

#define SAMPLE_COUNT 100
#define ITERATIONS 6
#define OFFSETS_EEPROM 12 // accel_gyro offsets from 12 thru 26
//#define CAL_DEBUG

void mpu_init(uint8_t);
void cal_mpu_off(uint8_t);
void cfgr_mpu_off();
void mpu_getReadings(int16_t *);
void mpu_get_int_status(uint8_t *);

#endif