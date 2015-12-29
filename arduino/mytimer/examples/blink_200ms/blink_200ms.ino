#include <mytimer.h>

volatile uint8_t LED_int=0;
uint8_t LEDstate = 0, status;

void timer_operation(){
	LED_int = 1;
}

void setup(){
	pinMode(13, OUTPUT);
	status = timer_init(200, timer_operation); //TIMEPERIOD MUST FALL IN THE CORRECT RANGE! CHECK <mytimer.h>
}

void loop(){
	if (status) return;
	if (LED_int){
		digitalWrite(13, LEDstate);
		LEDstate = !LEDstate;
		LED_int = 0;
	}
}