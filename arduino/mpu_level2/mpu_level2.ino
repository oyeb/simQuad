//-=-=-=-=-=-=-=-=-=-=-=-=  IMPORTANT  -=-=-=-=-=-=-=-=-=-=-=-=
// Connect only 3.3V to MPU Vcc pin NOT 5V
// Connect AD0 pin to ground
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

/*
Wire.beginTransmission(devAddr);
Wire.write((uint8_t) regAddr); // send address
for (uint8_t i = 0; i < length; i++) {
  Wire.write((uint8_t) data[i]);
}
status = Wire.endTransmission();
*/

/*
int8_t count = 0;
uint32_t t1 = millis();
for (uint8_t k = 0; k < length; k += min(length, BUFFER_LENGTH)) { //BUFFER_LENGTH is 32 in I2CDevLib.h
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    Wire.endTransmission();
    Wire.beginTransmission(devAddr);
    Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));

    for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); count++) {
        data[count] = Wire.read();
    }
  }
  if (timeout > 0 && millis() - t1 >= timeout && count < length) count = -1; 
  return count;
}
*/

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
int16_t gx, gy, gz;
int8_t xoff, yoff, zoff;
float gxm, gym, gzm;
int count;
unsigned int last, el;
volatile bool mint=false;
uint8_t int_status, temp;
uint16_t fifocount;
uint8_t fifoBuffer[12];

//#define VERBOSE
//#define CAL_DEBUG
#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_BINARY_ACCELGYRO

// function declarations here:
void calibrate();
void timeit();

void setup() {
  Wire.begin();
  Serial.begin(57600);
  /*
  Set clock_src = x_gyro
  Set gyro range 250
  Set accel rang 2g
  Disable sleep mode
  */
  accelgyro.initialize();
  //accelgyro.setIntEnabled(0x12);
  accelgyro.setRate(4);
  accelgyro.setDLPFMode(0x03);
  accelgyro.setFIFOEnabled(false);
  I2Cdev::writeByte(0x68, 0x23, 0x78);
  accelgyro.setDMPEnabled(false);
  I2Cdev::writeByte(0x68, 0x38, 0x11);
  
  
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
  calibrate();
  accelgyro.resetFIFO();
}

void loop(){
  if (mint){
    el = micros() - last;
    last = micros();
    int_status = accelgyro.getIntStatus();
    fifocount = accelgyro.getFIFOCount();
    if ((int_status & 1) && fifocount >= 12){
      for (uint8_t i=0; i<12; i++)
        I2Cdev::readByte(0x68, 0x74, fifoBuffer+i);
    }
    Serial.print(el);Serial.print(' ');
    Serial.print(int_status);Serial.print(' ');
    Serial.println(fifocount);
    #ifdef OUTPUT_READABLE_ACCELGYRO
      for (uint8_t i=0; i<12; i++){
        Serial.print(fifoBuffer[i]);Serial.print(' ');
      }
      Serial.print('\n');
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
void calibrate(){
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
