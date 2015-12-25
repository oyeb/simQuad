#include "mydmp.h"

/*
  mode <=0 No interrupts from mpu
  mode  >0 Interrupts from mpu enabled
*/
void mpu_init(uint8_t mode=0){
  Wire.begin();
  I2Cdev::writeByte(0x68, 0x6B, 0x01);    //PWR_MGMT1 for clock source as X-gyro, SLEEP{bit6} is (obviously) disabled
  uint8_t config_info[4] = {4, 3, 8, 0}; // Refer the Datasheet if you want to change these.
/*
  0x19[SMPRT_DIV]: 0x04 set SamplingRate as (clock_source / 4) = 2kHz [Accelerometer has max sample rate of 1kHz]
  0x1A[CONFIG]   : 0x03 set DLPF_CFG[2-0] bits in as 3 rest are all zero anyways.
  0x1B[GYRO_CFG] : 0x08 set GYRO_CFG_FS_SEL[4:3] = 10  range:500
  0x1C[ACCEL_CFG]: 0x00 set ACCEL_CFG_FS_SEL[4:3] = 00 range:2g
*/
  I2Cdev::writeBytes(0x68, 0x19, 4, config_info);
  if (mode > 0)
    I2Cdev::writeByte(0x68, 0x38, 0x01); //INT_EN
}

uint8_t MPU6050::dmpInitialize(){
    // reset device
    reset();
    delay(30); // wait after reset

    // disable sleep mode
    setSleepEnabled(false);

    // get MPU hardware revision
    setMemoryBank(0x10, true, true);
    setMemoryStartAddress(0x06);
    uint8_t hwRevision = readMemoryByte();
    setMemoryBank(0, false, false);

    // check OTP bank valid
    uint8_t otpValid = getOTPBankValid();

    // get X/Y/Z gyro offsets
    int8_t xgOffsetTC = getXGyroOffsetTC();
    int8_t ygOffsetTC = getYGyroOffsetTC();
    int8_t zgOffsetTC = getZGyroOffsetTC();

    // setup weird slave stuff (?)
    setSlaveAddress(0, 0x7F);
    setI2CMasterModeEnabled(false);
    setSlaveAddress(0, 0x68);
    resetI2CMaster();
    delay(20);

    // load DMP code into memory banks
    if (writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE)) {

        // write DMP configuration
        if (writeProgDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {

            setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

            setIntEnabled(0x12);

            setRate(4); // 1khz / (1 + 4) = 200 Hz

            setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

            setDLPFMode(MPU6050_DLPF_BW_188);

            setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

            setDMPConfig1(0x03);
            setDMPConfig2(0x00);

            setOTPBankValid(false);

            setXGyroOffsetTC(xgOffsetTC);
            setYGyroOffsetTC(ygOffsetTC);
            setZGyroOffsetTC(zgOffsetTC);

            //setXGyroOffset(0);
            //setYGyroOffset(0);
            //setZGyroOffset(0);

            uint8_t dmpUpdate[16], j;
            uint16_t pos = 0;
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            resetFIFO();

            uint16_t fifoCount = getFIFOCount();
            uint8_t fifoBuffer[128];

            getFIFOBytes(fifoBuffer, fifoCount);

            setMotionDetectionThreshold(2);

            setZeroMotionDetectionThreshold(156);

            setMotionDetectionDuration(80);

            setZeroMotionDetectionDuration(0);

            resetFIFO();

            setFIFOEnabled(true);

            setDMPEnabled(true);

            resetDMP();

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            while ((fifoCount = getFIFOCount()) < 3);

            getFIFOBytes(fifoBuffer, fifoCount);

            uint8_t mpuIntStatus = getIntStatus();


            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            while ((fifoCount = getFIFOCount()) < 3);


            getFIFOBytes(fifoBuffer, fifoCount);

            mpuIntStatus = getIntStatus();


            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);


            setDMPEnabled(false);

            /*if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
                return 3; // TODO: proper error code for no memory
            }*/

            resetFIFO();
        } else {
            return 2; // configuration block loading failed
        }
    } else {
        return 1; // main binary block loading failed
    }
    return 0; // success
}

uint8_t dmpGetQuaternion(Quaternion *q, const uint8_t* packet) {
    if (packet == 0) return 1;
    q -> w = (float)((packet[0] << 8) + packet[1]) / 16384.0f;
    q -> x = (float)((packet[4] << 8) + packet[5]) / 16384.0f;
    q -> y = (float)((packet[8] << 8) + packet[9]) / 16384.0f;
    q -> z = (float)((packet[12] << 8) + packet[13]) / 16384.0f;
    return 0;
}

/*
  Reads sensor registers and copies data into the global var in MAIN.
  Sensor registers could have been read "in-a-block" to save time. In this case, 
  ~68us (180us - (~112)us spent in copying) would have been saved at the cost of 15B of 
  global mem. Would need 14 byte temporary array, one loop to copy, etc for that. Not worth it
*/
void mpu_getReadings(int16_t *main_blob){
  I2Cdev::readWords(0x68, 0x3B, 3, (uint16_t *)main_blob);
  I2Cdev::readWords(0x68, 0x43, 3, (uint16_t *)(main_blob+3));
  /*
  uint8_t i;
  int16_t local_blob[7];
  I2Cdev::readWords(0x68, 0x3B, 7, (uint16_t *)local_blob);
  for (i=0; i<6; i++)
    main_blob[i] = (i<3)? local_blob[i] : local_blob[i+1];
  */
}

void mpu_send_quat_packet(uint8_t *packet){
  uint8_t i;
  for (i=0; i<14; i++)
    Serial.write(packet[i]);
}

void mpu_get_int_status(uint8_t *main_i_status){
  I2Cdev::readByte(0x68, 0x3A, main_i_status);
}

/*
  READS FROM "OFFSETS_EEPROM" AND WRITES TO OFFSET REGISTERS
*/
void cfgr_mpu_off(){
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

/*
  UPDATES EEPROM WORDS
    mode 3: both
    mode 1: accel
    mode 2: gyro
*/
void cal_mpu_off(uint8_t mode){
  int16_t acc_gyro_offs[6] = {0}, acc_t_gyro[7];
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