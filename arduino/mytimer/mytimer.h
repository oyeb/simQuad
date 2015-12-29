#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>

#ifndef MYTIMER_n
#define MYTIMER_n
/*
Utilises only 16-bit timers {1, 3, 4, 5} and provides Overflow Interrupts
Prescaler {=64} is hard-coded.
Thus, timer range is restricted to 5us to 262us

  Usage tip:
  in MAIN, make a volatile uint8_t "flag" variable.
  in MAIN, call timer-init(time-period, interrupt-operation-fn) with correct args.
  in MAIN-LOOP, keep checking for the state of this variable,
    if "true"
      call suitable procedure / execute block of statements
      reset the flag variable, forgetting this will cause a lot of headache!
    else, obviously, do nothing.

ISR(MY_TIMER_COMPA_vect) calls the interrupt-operation-fn()

time-period MUST BE IN THE RANGE SPECIFIED BELOW
                  _||_
                  \  /
                   \/
             5us -----> 262us

+-----------------------------------------+
| ONLY ONE OF THE BELOW MUST BE DEFINED!! |
+-----------------------------------------+
*/
        #define _use_my_timer_1
        //#define _use_my_timer_3
        //#define _use_my_timer_4
        //#define _use_my_timer_5

#include "mytimer_util.h"

/*
  16Mhz : prescaler 64 -> 4us time period
  Max time length = 4us * 2^16 = 262.144 ms

  For 8ms, OCR1A = .008/.000004 - 1 = 1999
*/

#define OCR_VALUE(timeperiod) timeperiod*250-1

uint8_t timer_init(uint16_t tp, void(*flag_op)(void));
/*
  Initialises TIMERn with:
    64 prescaler
    CTC mode               +--------------------------+
    OCRnA = OCR_VALUE(tp)  |tp MUST BE IN MILLISECONDS|
                           +--------------------------+
*/

uint8_t timer_update(uint16_t tp);
/*
  Updates time-period of the timer.
*/

uint8_t timer_detach(void);
/*
  Stops the timer, it can be re-initialised though...
*/

#endif