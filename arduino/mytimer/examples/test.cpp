#include <mytimer.h>

volatile uint8_t LEDstate=0;

void setup(){
	pinMode(13, OUTPUT);
	timer_init(200, &LEDstate); //TIMEPERIOD MUST FALL IN THE CORRECT RANGE! CHECK <mytimer.h>
}

void loop(){
	digitalWrite(13, LEDstate);
}