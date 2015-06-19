//-=-=-=-=-=-=-=-=-=-=-=-=  IMPORTANT  -=-=-=-=-=-=-=-=-=-=-=-=
// Connect only 3.3V to MPU Vcc pin NOT 5V
// Connect AD0 pin to ground
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
int16_t gx, gy, gz;
int16_t ax, ay, az;
int16_t xoff, yoff, zoff;
float gxm, gym, gzm;
float axm, aym, azm;
int count;
unsigned int last, el;
volatile bool mint=false;
uint8_t int_status, temp;
uint16_t fifocount;
uint8_t fifoBuffer[12];

#define VERBOSE
#define CAL_DEBUG
#define OUTPUT_READABLE_ACCEL
//#define OUTPUT_READABLE_GYRO
//#define OUTPUT_BINARY_ACCELGYRO

// function declarations here:
void calibrate_accel();
void calibrate_gyros();
void timeit();

void setup() {
  Wire.begin();
  Serial.begin(57600);

  /*accelgyro.initialize(); //.initialize does these 2 things
  */I2Cdev::writeByte(0x68, 0x6B, 0x01); //PWR_MGMT1 for clock source as X-gyro
    //accelgyro.setClockSource(0x01);
    uint8_t temp[8] = {0, 0};//8
    I2Cdev::writeBytes(0x68, 0x1B, 2, temp); //GYRO_CFG and ACCEL_CFG
    //accelgyro.setFullScaleGyroRange(0x00);
    //accelgyro.setFullScaleAccelRange(0x00);

  accelgyro.setRate(4);
  accelgyro.setDLPFMode(0x03);
  accelgyro.setFIFOEnabled(true);
  I2Cdev::writeByte(0x68, 0x23, 0x78); //FIFO_EN for ACCEL,GYRO
  accelgyro.setDMPEnabled(false);
  I2Cdev::writeByte(0x68, 0x38, 0x11); //INT_EN
  
  
  //verify connection
  //Serial.println("Testing device connections...");
  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  //Configure interrupt pin:
  //  Clear on any read
  //  active-high
  //  DMP_INT, probably when FIFO is ready. The DATA_RDY fires when all sensor regs are updated.
  //    So, which interrupt fires after FIFO update???
  /*
  Wire.beginTransmission(0x68);
  Wire.write(0x37); //INT_CFG
  Wire.write(0x70); //0111 0000
  Wire.write(0x01); //0000 0010 (DMP_INT)
  Wire.endTransmission();
  */
  attachInterrupt(0, timeit, RISING);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  #ifdef VERBOSE
    #ifndef CAL_DEBUG
      Serial.print("Calibrating Gyros! Hold Still");
    #endif
  #endif
  //calibrate_gyros();
  calibrate_accel();
  accelgyro.resetFIFO();
}

void loop(){
  if (mint){
    el = micros() - last;
    last = micros();
    int_status = accelgyro.getIntStatus();
    fifocount = accelgyro.getFIFOCount();
    if ((int_status & 1) && fifocount >= 12){ //data ready! read fifo
        I2Cdev::readBytes(0x68, 0x74, 12, fifoBuffer);
        ax = (fifoBuffer[0]<<8)|fifoBuffer[1];
        ay = (fifoBuffer[2]<<8)|fifoBuffer[3];
        az = (fifoBuffer[4]<<8)|fifoBuffer[5];
        gx = (fifoBuffer[6]<<8)|fifoBuffer[7];
        gy = (fifoBuffer[8]<<8)|fifoBuffer[9];
        gz = (fifoBuffer[10]<<8)|fifoBuffer[11];
    }
    if (int_status & 0x10) //fifo overflow
      accelgyro.resetFIFO();
    #ifdef VERBOSE_1
      Serial.print(el);Serial.print(' ');
      Serial.print(int_status);Serial.print(' ');
      Serial.println(fifocount);
    #endif
    #ifdef OUTPUT_READABLE_ACCEL
        Serial.print(ax);Serial.print(' ');
        Serial.print(ay);Serial.print(' ');
        Serial.print(az);
    #endif
    #ifdef OUTPUT_READABLE_ACCEL
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
  }
  else{
    //other non-motion work!
    ;
  }
}

#define SAMPLE_COUNT 100
#define ITERATIONS 6

void calibrate_accel(){
  //-74 -107 22
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
      i++;
      #ifdef VERBOSE
        #ifndef CAL_DEBUG
          Serial.print(".");
        #endif
      #endif
    }
    else{
      axm = (axm*count + ax)/(count+1.0);
      aym = (aym*count + ay)/(count+1.0);
      azm = (azm*count + az)/(count+1.0);
      count++;
    }
  }
  #ifdef VERBOSE
    #ifndef CAL_DEBUG
      Serial.println("\nDone.");
    #endif
  #endif
}

void calibrate_gyros(){
  byte i=0;
  while (i < ITERATIONS){ //hope that offsets converge in 6 iterations
    accelgyro.getRotation(&gx, &gy, &gz);
    if (count == SAMPLE_COUNT){
      xoff += int(gxm/-6);
      yoff += int(gym/-6);
      zoff += int(gzm/-6);
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
      i++;
      #ifdef VERBOSE
        #ifndef CAL_DEBUG
          Serial.print(".");
        #endif
      #endif
    }
    else{
      gxm = (gxm*count + gx)/(count+1.0);
      gym = (gym*count + gy)/(count+1.0);
      gzm = (gzm*count + gz)/(count+1.0);
      count++;
    }
  }
  #ifdef VERBOSE
    #ifndef CAL_DEBUG
      Serial.println("\nDone.");
    #endif
  #endif
}

void timeit(){
    mint = true;
}
