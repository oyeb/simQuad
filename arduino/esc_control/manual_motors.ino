#include <Servo.h>

#define MOTOR_START 1050
#define MOTOR_STOP   1040
#define MOTOR_ARM    620

Servo motor[4];
uint8_t i, hi, lo;
uint16_t pw = MOTOR_STOP;

void setup(){
  Serial.begin(115200);
  for(i=0; i<4; i++){
    motor[i].attach(i+3, 600, 2200); //3,4,5,6
    motor[i].writeMicroseconds(MOTOR_ARM);
  }
}

void loop(){
  while (Serial.available() <= 1);  // (arduino) <- (py) [ hi _ lo ]
  hi = Serial.read();
  lo = Serial.read();
  pw = (( hi << 8) | lo);
  for (i=0; i<4; i++)
    motor[i].writeMicroseconds(pw);
  Serial.write( (pw>>8) );    // hi
  Serial.write(pw & 0xFF);    // lo
}