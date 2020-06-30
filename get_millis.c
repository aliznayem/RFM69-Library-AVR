// Source: https://gist.github.com/adnbr/2439125#file-counting-millis-c

#include "get_millis.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

volatile unsigned long timer1_millis;

void millis_init()
{
    // CTC mode, Clock/8
    TCCR1B |= (1 << WGM12) | (1 << CS11);
    
    // Load the high byte, then the low byte
    // into the output compare
    OCR1AH = (CTC_MATCH_OVERFLOW >> 8);
    OCR1AL = CTC_MATCH_OVERFLOW;
    sei();
    
    // Enable the compare match interrupt
    #if defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
        TIMSK1 |= (1 << OCIE1A);
    #elif defined (__AVR_ATmega64__)
        TIMSK |= (1 << OCIE1A);
    #elif defined (__AVR_ATtiny84A__)
        TIMSK1 |= (1 << OCIE1A);
    #endif
}

unsigned long millis()
{
    unsigned long millis_return;
    // ensure this cannnot be disrupted
    ATOMIC_BLOCK(ATOMIC_FORCEON)
    {
        millis_return = timer1_millis;
    }
    return millis_return;
}

ISR (TIMER1_COMPA_vect)
{
    timer1_millis++;
}
