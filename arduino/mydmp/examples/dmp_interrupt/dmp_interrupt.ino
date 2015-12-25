#include "I2Cdev.h"
#include "Wire.h"
#include "mydmp.h"

#define READABLE_Q
//#define BINARY_Q

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

uint8_t script_ready=0, cmd_in;

void dmpDataReady();

void setup(){
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    Serial.begin(115200);
    mpu_init(1);
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        cfgr_mpu_off();
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpu_get_int_status(&mpuIntStatus);
        dmpReady = true;
    }
    pinMode(13, OUTPUT);
}

void loop(){
    if (!dmpReady) return;
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < dmpPacketSize) {
        // other program behavior stuff here
        if (Serial.available() > 0){
          cmd_in = Serial.read();
          Serial.write(cmd_in+1);
          script_ready = !script_ready;
          digitalWrite(13, !script_ready);
        }
    }

    mpuInterrupt = false;
    mpu_get_int_status(&mpuIntStatus);
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024){
        mpu.resetFIFO(); // if overflow happened
    }
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < dmpPacketSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, dmpPacketSize);       
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= dmpPacketSize;

        dmpGetQuaternion(&q, fifoBuffer);
        if (script_ready){
            #ifdef READABLE_Q
                Serial.print("quat\t");
                Serial.print(q.w);
                Serial.print("\t");
                Serial.print(q.x);
                Serial.print("\t");
                Serial.print(q.y);
                Serial.print("\t");
                Serial.println(q.z);
            #endif
            #ifdef BINARY_Q
                mpu_send_quat_packet(fifoBuffer);
            #endif
        }
    }
}

void dmpDataReady() {
    mpuInterrupt = true;
}