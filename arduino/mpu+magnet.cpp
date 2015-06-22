/*-=-=-=-=-=-=-=-=-=-=-=-=  IMPORTANT  -=-=-=-=-=-=-=-=-=-=-=-=
+ Connect only 3.3V to MPU Vcc pin NOT 5V
+ Connect AD0 pin to ground
+ All  I2C reads/writes (for 1byte) take ~960us.
+ Additional reads/writes (in blocks) take 100us (per byte) more!!
*  I2C operations are extremely costly. Minimise them in the loop() or else...
     -  1  byte   480us
     -  2  bytes  580us
     - 12 bytes  1580us

*_* This can be reduced by a factor of ~4 if twi.h CONSTANTS are changed so that the
*_* I2C bus communicates @ 400kHz rather than standard 100kHz
*_* OR we use FastWire library for I2C rather than the default Wire library. I2Cdev works
*_* with both!!
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// Device default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
int16_t gx, gy, gz;
int16_t ax, ay, az;
int16_t bx, by, bz;
int16_t xoff, yoff, zoff;
float gxm, gym, gzm;
float axm, aym, azm;
int count;
unsigned int last, el, el2;
volatile bool mint=false, cint=false;
uint8_t int_status, temp;
uint16_t fifocount=24, test; //fifocount is initialised with 24. Contact Ananya to understand why. Keep it like this only. There is not enough space here to explain this.
uint8_t fifoBuffer[12];

//#define TIMING
//#define CAL_DEBUG
//#define OUTPUT_READABLE_ACCEL
#define OUTPUT_READABLE_GYRO
#define OUTPUT_READABLE_COMPASS

// function declarations here:
void calibrate_accel();
void calibrate_gyros();
void mpu_interrupt();
void compass_interrupt();

void setup() {
  Wire.begin();
  Serial.begin(57600);
  I2Cdev::writeByte(0x68, 0x6B, 0x01); //PWR_MGMT1 for clock source as X-gyro
  uint8_t temp[8] = {8, 0};//GYRO range:250, ACCEL range:2g | Refer the Datasheet if you want to change these.
  I2Cdev::writeBytes(0x68, 0x1B, 2, temp); //GYRO_CFG and ACCEL_CFG

  accelgyro.setRate(4);
  accelgyro.setDLPFMode(0x03);
  accelgyro.setFIFOEnabled(true);
  I2Cdev::writeByte(0x68, 0x23, 0x78); //FIFO_EN for ACCEL,GYRO
  accelgyro.setDMPEnabled(false);
  I2Cdev::writeByte(0x68, 0x38, 0x11); //INT_EN
  
  attachInterrupt(0, mpu_interrupt, RISING);
  attachInterrupt(1, compass_interrupt, FALLING);  

  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(0x1E); //open communication with HMC5883
  Wire.write(0x00); //select Config_Register_A: 
  Wire.write(0x58); //4-point avg. and 75Hz rate
  Wire.write(0x02); //In Config_Register_B: +-1.9G (820LSB/G)
  Wire.write(0x00); //In Mode_Register: continuous measurement mode
  Wire.endTransmission();
  pinMode(13, INPUT);
  #ifdef CAL_DEBUG
    Serial.print("Calibrating Gyros and Accel! Hold Still and Level!");
  #endif
  calibrate_gyros();
  calibrate_accel();
  accelgyro.resetFIFO();
}

void loop(){
  if (mint){
    el = micros() - last;
    last = micros();
    int_status = accelgyro.getIntStatus();
    fifocount = accelgyro.getFIFOCount();
    if ((int_status & 1) && fifocount >= 12){ //data ready! read fifo now.
        //costly operation below!! It takes 1580us!
        I2Cdev::readBytes(0x68, 0x74, 12, fifoBuffer);
        ax = (fifoBuffer[0]<<8)|fifoBuffer[1];
        ay = (fifoBuffer[2]<<8)|fifoBuffer[3];
        az = (fifoBuffer[4]<<8)|fifoBuffer[5];
        gx = (fifoBuffer[6]<<8)|fifoBuffer[7];
        gy = (fifoBuffer[8]<<8)|fifoBuffer[9];
        gz = (fifoBuffer[10]<<8)|fifoBuffer[11];
        #ifdef TIMING
          el2 = micros() - last;
        #endif
    }
    if (int_status & 0x10){ //fifo overflow
      accelgyro.resetFIFO();
    }
    #ifdef TIMING
      Serial.print(el);Serial.print(' ');
      Serial.print(int_status);Serial.print(' ');
      Serial.print(test);Serial.print(' ');
      Serial.print(el2);Serial.print(' ');
    #endif
    #ifdef OUTPUT_READABLE_ACCEL
      Serial.print(ax);Serial.print(' ');
      Serial.print(ay);Serial.print(' ');
      Serial.print(az);
      #ifdef OUTPUT_READABLE_GYRO
        Serial.print(' ');
      #else
        Serial.print('\n');
      #endif
    #endif
    #ifdef OUTPUT_READABLE_GYRO
      Serial.print(gx);Serial.print(' ');
      Serial.print(gy);Serial.print(' ');
      Serial.println(gz);
    #endif
    mint = false;
    test=0;
  }
  if (cint){
    el = micros() - last;
    last = micros();
    Wire.beginTransmission(0x1E);
    Wire.write(0x03); //select register 3, X MSB register
    Wire.endTransmission();
    Wire.requestFrom(0x1E, 6);
    if(6<=Wire.available()){
      bx = Wire.read()<<8; //X msb
      bx |= Wire.read(); //X lsb
      bz = Wire.read()<<8; //Z msb
      bz |= Wire.read(); //Z lsb
      by = Wire.read()<<8; //Y msb
      by |= Wire.read(); //Y lsb
    }
    #ifdef TIMING
      el2 = micros() - last;
      Serial.print(el);Serial.print(' ');
      Serial.print(test);Serial.print(' ');
      Serial.print(el2);Serial.print(' ');
    #endif
    #ifdef OUTPUT_READABLE_COMPASS
      Serial.print(bx);Serial.print(" ");
      Serial.print(by);Serial.print(" ");
      Serial.print(bz);Serial.println("b");
    #endif
    cint = false;
    test = 0;
  }
  else{
    //other non-motion work!
    #ifdef TIMING
      test++;
    #endif
  }
}

#define SAMPLE_COUNT 100
#define ITERATIONS 6

void calibrate_accel(){
  xoff = accelgyro.getXAccelOffset();
  yoff = accelgyro.getYAccelOffset();
  zoff = accelgyro.getZAccelOffset();
  byte i=0;
  while (i < ITERATIONS){ //hope that offsets converge in 6 iterations
    accelgyro.getAcceleration(&ax, &ay, &az);
    if (count == SAMPLE_COUNT){
      xoff += int(axm/-6);
      yoff += int(aym/-6);
      zoff += int((azm+16384)/-6);
      accelgyro.setXAccelOffset(xoff);
      accelgyro.setYAccelOffset(yoff);
      accelgyro.setZAccelOffset(zoff);
      #ifdef CAL_DEBUG
        Serial.print(axm); Serial.print(" ");
        Serial.print(aym); Serial.print(" ");
        Serial.println(azm);
        Serial.print(xoff); Serial.print(" ");
        Serial.print(yoff); Serial.print(" ");
        Serial.println(zoff);
        Serial.println("*********************");
      #endif
      count = 0;
      i++; //iteration++
      #ifdef CAL_DEBUG
        Serial.print(".");
      #endif
    }
    else{
      axm = (axm*count + ax)/(count+1.0);
      aym = (aym*count + ay)/(count+1.0);
      azm = (azm*count + az)/(count+1.0);
      count++;
    }
  }
  #ifdef CAL_DEBUG
    Serial.println(" Done.");
  #endif
}

void calibrate_gyros(){
  byte i=0;
  while (i < ITERATIONS){ //hope that offsets converge in 6 iterations
    accelgyro.getRotation(&gx, &gy, &gz);
    if (count == SAMPLE_COUNT){
      xoff += int(gxm/-3);
      yoff += int(gym/-3);
      zoff += int(gzm/-3);
      accelgyro.setXGyroOffset(xoff);
      accelgyro.setYGyroOffset(yoff);
      accelgyro.setZGyroOffset(zoff);
      #ifdef CAL_DEBUG
        Serial.print(gxm); Serial.print(" ");
        Serial.print(gym); Serial.print(" ");
        Serial.println(gzm);
        Serial.print(xoff); Serial.print(" ");
        Serial.print(yoff); Serial.print(" ");
        Serial.println(zoff);
        Serial.println("*********************");
      #endif
      count = 0;
      i++; //iteration++
      #ifdef CAL_DEBUG
        Serial.print(".");
      #endif
    }
    else{
      gxm = (gxm*count + gx)/(count+1.0);
      gym = (gym*count + gy)/(count+1.0);
      gzm = (gzm*count + gz)/(count+1.0);
      count++;
    }
  }
  #ifdef CAL_DEBUG
    Serial.println(" Done.");
  #endif
}

void mpu_interrupt(){
  //This interrupt (from MPU) fires every ~4990-5000us
  mint = true;
}

void compass_interrupt(){
  //This interrupt (from HMC5883L) fires every 13333us
  cint = true;
}