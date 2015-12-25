/*
  Usage tip:
  in MAIN, make a volatile uint8_t "flag" variable.
  in MAIN, call timer-init() with correct args.
  in MAIN-LOOP, keep checking for the state of this variable,
    if "true", call suitable procedure / execute block of statements
    else, obviously, do nothing.

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
#define OCR_VALUE(timeperiod) timeperiod*250-1

void timer_init(int tp, volatile uint8_t *flag, uint8_t set_value);
/*
  Initialises TIMER1 with:
  	64 prescaler
  	CTC mode               +--------------------------+
  	OCR1A = OCR_VALUE(tp)  |tp MUST BE IN MILLISECONDS|
                           +--------------------------+
*/

#endif