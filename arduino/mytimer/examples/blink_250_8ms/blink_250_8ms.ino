#include <mytimer.h>

volatile uint8_t LED_int=0;
uint8_t LEDstate = 0, status;
uint16_t time_period[2] = {8, 250};

void timer_operation(){
	LED_int = 1;
}

void setup(){
	pinMode(13, OUTPUT);
	status = timer_init(time_period[1], timer_operation); //TIMEPERIOD MUST FALL IN THE CORRECT RANGE! CHECK <mytimer.h>
}

void loop(){
	if (status) return;
	if (LED_int){
		digitalWrite(13, LEDstate);
    	LEDstate = 1 - LEDstate;
		if ( timer_update(time_period[LEDstate]) ) ;//oops! could not update! {do nothing}
		LED_int = 0;
	}
}