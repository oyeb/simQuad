#include <mytimer.h>

volatile uint8_t LED_int=0;
uint8_t LEDstate = 0;

void setup(){
	pinMode(13, OUTPUT);
	timer_init(200, &LED_int, 1); //TIMEPERIOD MUST FALL IN THE CORRECT RANGE! CHECK <mytimer.h>
}

void loop(){
	if (LED_int){
		digitalWrite(13, LEDstate);
		LEDstate = !LEDstate;
		LED_int = 0;
	}
}