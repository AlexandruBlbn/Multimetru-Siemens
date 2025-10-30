#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Config/i2c.h"
#include "Config/uart.h"
#include "LCD/LCD.h"

#include "MAX30102/max30102.h"

void AVR_init(void){ 
    uart_init(); 
    i2c_init(); 
    timer_init();
    LCD_Init();     
	LCD_Clear();
	LCD_WriteString("Initializare...");
	_delay_ms(1000);
    LCD_Clear();   
}

int main(void) {
	AVR_init();
	int optiune = 1;
	
	//TODO: meniu cu intreruperi pt functii.

    while(1) {
		MAX30102_Start();

    }
    
    return 0;
}
