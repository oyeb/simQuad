#include <mytimer.h>

volatile uint8_t *flag;
void timer_init(int tp, volatile uint8_t *flag_var)
{
    flag = flag_var;
    // initialize Timer1
    cli();          // disable global interrupts
    TCCR1A = 0;     // set entire TCCR1A register to 0
    TCCR1B = 0;     // same for TCCR1B
    // enable timer compare interrupt:
    TIMSK1 |= (1 << OCIE1A);
    // set compare match register to desired timer count:
    OCR1A = OCR_VALUE(tp);
    // Set CS10 and CS12 bits for 64 prescaler:
    TCCR1B |= (1 << CS10) | (1 << CS11);
    // turn on CTC mode:
    TCCR1B |= (1 << WGM12);

    // enable global interrupts:
    sei();
}

ISR(TIMER1_COMPA_vect)
{
    *flag = !(*flag);;
}