#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Config/i2c.h"
#include "Config/uart.h"
#include "MAX30102/max30102.h"

void AVR_init(void){ uart_init(); i2c_init(); timer_init();}

int main(void) {
	AVR_init();


    while(1) {
    MAX30102_Start();

    }
    
    return 0;
}
