#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "Config/i2c.h"
#include "Config/uart.h"
#include "Config/ADC.h"
#include "inc/LCD.h"
#include "inc/AD8232.h"
#include "inc/max30102.h"
#include "inc/HX710B.h"

volatile bool g_stop_measurement = false;

void AVR_init(void) {
    uart_init();
    i2c_init(); 
    adc_init();
    timer_init();
    LCD_Init();
    max30102_init();
}

int main(void) 
{
    AVR_init();
    LCD_Clear();
    LCD_WriteString("Pornit.");
    _delay_ms(1000);
    MAX30102_Start();
    
    while(1) 
    {
        // Main loop
    }
 
    return 0; 
}