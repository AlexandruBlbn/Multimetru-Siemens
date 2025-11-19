#include "../inc/intrerupts.h"
#include "../inc/LCD.h"
#include "../inc/AD8232.h"
#include "../inc/max30102.h"
#include "../inc/HX710B.h"
#include <util/delay.h>
#include "../Config/uart.h"

// Configure external interrupts INT2, INT3, and INT4
// INT2 -> PD2 (Pin 19 on Arduino Mega)
// INT3 -> PD3 (Pin 18 on Arduino Mega)
// INT4 -> PE4 (Pin 2 on Arduino Mega)

volatile int menu_option = 0;
volatile bool status = false;


void Interrupts_Init(void) {
    // Configure INT2 and INT3 pins as inputs with pull-up
    DDRD &= ~((1 << DDD2) | (1 << DDD3));
    PORTD |= (1 << PORTD2) | (1 << PORTD3);
    
    // Configure INT4 pin as input with pull-up
    DDRE &= ~(1 << DDE4);
    PORTE |= (1 << PORTE4);
    
    // Configure INT2 and INT3 to trigger on RISING edge (when button is released)
    EICRA &= ~((1 << ISC20) | (1 << ISC21) | (1 << ISC30) | (1 << ISC31));
    EICRA |= (1 << ISC21) | (1 << ISC20) | (1 << ISC31) | (1 << ISC30);  // ISC21=1, ISC20=1 -> rising edge
    
    // Configure INT4 to trigger on RISING edge (when button is released)
    EICRB &= ~((1 << ISC40) | (1 << ISC41));
    EICRB |= (1 << ISC41) | (1 << ISC40);  // ISC41=1, ISC40=1 -> rising edge
    
    // Clear interrupt flags
    EIFR |= (1 << INTF2) | (1 << INTF3) | (1 << INTF4);
    
    // Enable INT2, INT3, and INT4
    EIMSK |= (1 << INT2) | (1 << INT3) | (1 << INT4);
}


void MenuLoop(void){
        LCD_Clear();
        LCD_SetCursor(0,0); LCD_WriteString("1. SatO2  2. EKG ");
        LCD_SetCursor(1,0); LCD_WriteString("3. Tensiune ");
   
     while(1){
    if (menu_option == 1) {
        if (status == false){
            status = true;
            MAX30102_Start();
            menu_option = 0;
            MenuLoop();
        }
    }
    else if (menu_option == 2) {
            if (status == false){
            status = true;
            // AD8232_Start();
            LCD_Clear();
            LCD_SetCursor(0,0); 
            LCD_WriteString("1"); 
            if (menu_option == 0){
            MenuLoop();}

            }
    }
    else if (menu_option == 3) {
            if (status == false){
            status = true;
            HX710B_Start();
        }
    }
    

    
}}

// ISR for INT2 (Pin 19)
ISR(INT2_vect) {
        _delay_ms(50); // Debounce
    if(menu_option == 0 || menu_option == 2 || menu_option == 3){
        menu_option = 1;
    }
    else{
        menu_option = 0;
        status = false;

    }
}

// ISR for INT3 (Pin 18)
ISR(INT3_vect) {
    _delay_ms(50); // Debounce
    if(menu_option == 0 || menu_option == 1 || menu_option == 3){
        menu_option = 2;
        status = false;
    }
    else{
        menu_option = 0;
        status = false;
    }
}

// ISR for INT4 (Pin 2)
ISR(INT4_vect) {
        _delay_ms(50); // Debounce
    if(menu_option == 0 || menu_option == 1 || menu_option == 2){
        menu_option = 3;
        status = false;
    }
    else{
        menu_option = 0;
        status = false;
    }
}
