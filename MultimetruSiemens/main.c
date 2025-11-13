#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "Config/i2c.h"
#include "Config/uart.h"
#include "Config/ADC.h"
#include "inc/LCD.h"
#include "inc/AD8232.h"
#include "inc/HX710B.h"
#include "inc/max30102.h"

void AVR_init(void) {
    uart_init();
    i2c_init();
    timer_init();
    LCD_Init();
    AD8232_init();
    hx710b_init();
    max30102_init();

}

int main(void) 
{
    AVR_init();
    
    _delay_ms(500);  // Give LCD time to initialize
    MAX30102_Start();
    while(1) 
    {

    }

    return 0;
}