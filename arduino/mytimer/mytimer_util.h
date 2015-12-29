#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>

#ifdef _use_my_timer_1
  #define            MY_TCNTH TCNT1H
  #define            MY_TCNTL TCNT1L
  #define             MY_TCNT TCNT1
  #define            MY_TCCRA TCCR1A
  #define            MY_TCCRB TCCR1B
  #define            MY_TCCRC TCCR1C
  #define            MY_OCRAH OCR1AH
  #define            MY_OCRAL OCR1AL
  #define             MY_OCRA OCR1A
  #define            MY_OCRBH OCR1BH
  #define            MY_OCRBL OCR1BL
  #define             MY_OCRB OCR1B
  #define            MY_OCRCH OCR1CH
  #define            MY_OCRCL OCR1CL
  #define             MY_OCRC OCR1C
  #define             MY_ICRH ICR1H
  #define             MY_ICRL ICR1L
  #define              MY_ICR ICR1
  #define             MY_TIFR TIFR1
  #define            MY_TIMSK TIMSK1
  #define MY_TIMER_COMPA_vect TIMER1_COMPA_vect
#endif

#ifdef _use_my_timer_3
  #define            MY_TCNTH TCNT3H
  #define            MY_TCNTL TCNT3L
  #define             MY_TCNT TCNT3
  #define            MY_TCCRA TCCR3A
  #define            MY_TCCRB TCCR3B
  #define            MY_TCCRC TCCR3C
  #define            MY_OCRAH OCR3AH
  #define            MY_OCRAL OCR3AL
  #define             MY_OCRA OCR3A
  #define            MY_OCRBH OCR3BH
  #define            MY_OCRBL OCR3BL
  #define             MY_OCRB OCR3B
  #define            MY_OCRCH OCR3CH
  #define            MY_OCRCL OCR3CL
  #define             MY_OCRC OCR3C
  #define             MY_ICRH ICR3H
  #define             MY_ICRL ICR3L
  #define              MY_ICR ICR3
  #define             MY_TIFR TIFR3
  #define            MY_TIMSK TIMSK3
  #define MY_TIMER_COMPA_vect TIMER3_COMPA_vect
#endif

#ifdef _use_my_timer_4
  #define            MY_TCNTH TCNT4H
  #define            MY_TCNTL TCNT4L
  #define             MY_TCNT TCNT4
  #define            MY_TCCRA TCCR4A
  #define            MY_TCCRB TCCR4B
  #define            MY_TCCRC TCCR4C
  #define            MY_OCRAH OCR4AH
  #define            MY_OCRAL OCR4AL
  #define             MY_OCRA OCR4A
  #define            MY_OCRBH OCR4BH
  #define            MY_OCRBL OCR4BL
  #define             MY_OCRB OCR4B
  #define            MY_OCRCH OCR4CH
  #define            MY_OCRCL OCR4CL
  #define             MY_OCRC OCR4C
  #define             MY_ICRH ICR4H
  #define             MY_ICRL ICR4L
  #define              MY_ICR ICR4
  #define             MY_TIFR TIFR4
  #define            MY_TIMSK TIMSK4
  #define MY_TIMER_COMPA_vect TIMER4_COMPA_vect
#endif

#ifdef _use_my_timer_5
  #define            MY_TCNTH TCNT5H
  #define            MY_TCNTL TCNT5L
  #define             MY_TCNT TCNT5
  #define            MY_TCCRA TCCR5A
  #define            MY_TCCRB TCCR5B
  #define            MY_TCCRC TCCR5C
  #define            MY_OCRAH OCR5AH
  #define            MY_OCRAL OCR5AL
  #define             MY_OCRA OCR5A
  #define            MY_OCRBH OCR5BH
  #define            MY_OCRBL OCR5BL
  #define             MY_OCRB OCR5B
  #define            MY_OCRCH OCR5CH
  #define            MY_OCRCL OCR5CL
  #define             MY_OCRC OCR5C
  #define             MY_ICRH ICR5H
  #define             MY_ICRL ICR5L
  #define              MY_ICR ICR5
  #define             MY_TIFR TIFR5
  #define            MY_TIMSK TIMSK5
  #define MY_TIMER_COMPA_vect TIMER5_COMPA_vect
#endif
