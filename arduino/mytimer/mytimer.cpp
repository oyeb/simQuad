#include "mytimer.h"

uint8_t isActive = 0;
void (* _flag_operation)(void);

uint8_t timer_init(uint16_t tp, void (*flag_op)(void)){   
    if (!isActive){
        isActive = 1;
        _flag_operation = flag_op;
        uint8_t oldSREG = SREG;
        // initialize Timer1
        cli();          // disable global interrupts
        MY_TCCRA = 0;     // set entire TCCR1A register to 0
        MY_TCCRB = 0;     // same for TCCR1B
        // enable timer compare interrupt:
        MY_TIMSK |= (1 << OCIE1A);
        // set compare match register to desired timer count:
        MY_OCRA = OCR_VALUE(tp);
        // Set CS10 and CS12 bits for 64 prescaler:
        MY_TCCRB |= (1 << CS10) | (1 << CS11);
        // turn on CTC mode:
        MY_TCCRB |= (1 << WGM12);

        // enable global interrupts:
        sei();
        SREG = oldSREG;
        return 0;
    }
    return 1;
}

uint8_t timer_update(uint16_t tp){
    // the first interrupt after the change might be missed or inaccurate
    // don't use for critical application
    // minimum tp for interrupt to be caught ~1 ms (this figure is wrong)
    if (isActive){
        uint8_t oldSREG = SREG;
        cli();
        MY_OCRA = OCR_VALUE(tp);
        sei();
        SREG = oldSREG;
        return 0;
    }
    return 1;
}

uint8_t timer_detach(void){
    if (isActive){
        MY_TIMSK = 0;
        MY_TCCRB = 0;
        isActive = 0;
        return 0;
    }
    return 1;
}

ISR(MY_TIMER_COMPA_vect)
{
    _flag_operation();
}