/*
	ISR(TIMER1_COMPA_vect) flips "flag" variable.
	tp MUST BE IN THE RANGE SPECIFIED IN THE COMMENT BELOW
	                      _||_
	                      \  /
	                       \/
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>

#ifndef MYTIMER1
#define MYTIMER1

/*
  16Mhz : prescaler 64 -> 4us time period
  Max time length = 4us * 2^16 = 262.144 ms
  For 8ms, OCR1A = .008/.000004 - 1 = 1999
*/
uint16_t OCR_VALUE(timeperiod) timeperiod*250-1

void timer_init(int tp, volatile uint8_t *flag);
/*
  Initialises TIMER1 with:
  	64 prescaler
  	CTC mode               ~~-*-~~-*-~~-*-~~-*-~~-*-~~-
  	OCR1A = OCR_VALUE(tp)  |tp MUST BE IN MILLISECONDS|
*/

#endif