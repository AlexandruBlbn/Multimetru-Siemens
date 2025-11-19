#include "../inc/intrerupts.h"

// Configure external interrupts INT2, INT3, and INT4
// INT2 -> PD2 (Pin 19 on Arduino Mega)
// INT3 -> PD3 (Pin 18 on Arduino Mega)
// INT4 -> PE4 (Pin 2 on Arduino Mega)
void Interrupts_Init(void) {
    DDRD &= ~((1 << DDD2) | (1 << DDD3));
    PORTD |= (1 << PORTD2) | (1 << PORTD3);
    DDRE &= ~(1 << DDE4);
    PORTE |= (1 << PORTE4);
    EICRA &= ~((1 << ISC20) | (1 << ISC21) | (1 << ISC30) | (1 << ISC31));
    EICRA |= (1 << ISC21) | (1 << ISC31);
    EICRB &= ~((1 << ISC40) | (1 << ISC41));
    EICRB |= (1 << ISC41);
    EIFR |= (1 << INTF2) | (1 << INTF3) | (1 << INTF4);
    EIMSK |= (1 << INT2) | (1 << INT3) | (1 << INT4);
}

// ISR for INT2 (Pin 19)
ISR(INT2_vect) {
    // TODO: Add interrupt handling code
}

// ISR for INT3 (Pin 18)
ISR(INT3_vect) {
    // TODO: Add interrupt handling code
}

// ISR for INT4 (Pin 2)
ISR(INT4_vect) {
    // TODO: Add interrupt handling code
}
