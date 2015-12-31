/*
MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo  
MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo 
DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
REFRESH_INTERVAL    20000     // minumim time to refresh servos in microseconds
                                                                        +-------------+
Servo.h utilises timers {5, 1, 3, 4} (in that order of necessity) on an | ArduinoMega |
                                                                        +-------------+
In this project we will never require 12 servo objects and hence ...... | only timer5 |
will be used.                                                           +-------------+
*/
#include <Servo.h>

Servo m1;
uint16_t pulsewidth=620;
uint8_t ch;

void setup(){
	m1.attach(3, 600, 2200);
  m1.writeMicroseconds(pulsewidth);
  pulsewidth = 1040;
	Serial.begin(57600);
}
//540 for arming:620
//800 for stop:1055, 1055, 1044 , 1040
//   for start:1064, 1064, 1050 , 1050
void loop(){
	Serial.println(pulsewidth);
	m1.writeMicroseconds(pulsewidth);
	while (Serial.available() <= 0);
  ch = Serial.read();
  /*
  if (ch == '[')
	  pulsewidth = 2000;
  else if (ch == '/')
    pulsewidth = 1000;
  */
  if (ch == 10)
    pulsewidth += 100;
  else
    pulsewidth -= 200;
  
}
