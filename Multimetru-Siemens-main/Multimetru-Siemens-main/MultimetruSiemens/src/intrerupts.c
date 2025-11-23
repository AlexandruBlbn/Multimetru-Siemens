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
extern volatile bool g_stop_measurement;
extern unsigned long millis(void);

volatile unsigned long last_button_time[3] = {0, 0, 0};
#define DEBOUNCE 200 

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


void MenuLoop(void){

    while(1){
    if (menu_option == 1) {
        if (status == false){
            status = true;
            MAX30102_Start();
            if (menu_option == 0){
                LCD_Clear();
                _delay_ms(100);
                MenuLoop();
            }
        }
    }
    if (menu_option == 2) {
        if (status == false){
            status = true;
            HX710B_Start();
            if (menu_option == 0){
                LCD_Clear();
                _delay_ms(100);
                MenuLoop();
            }
        }
    }
    if (menu_option == 3) {
        if (status == false){
            status = true;
            HX710B_Start();
            if (menu_option == 0){
                LCD_Clear();
                _delay_ms(100);
                MenuLoop();
            }
        }
    }
    if (menu_option == 0) {
        if(status==false){
            status = true;
            _delay_ms(100);
            LCD_Clear();
            LCD_SetCursor(0,0); 
            LCD_WriteString("1. SatO2");
            LCD_SetCursor(1,0); 
            LCD_WriteString("2. Tensiune ");
        }
    }
    
}
}
// ISR for INT2 (Pin 19)
ISR(INT2_vect) {
    unsigned long current_time = millis();
    if (current_time - last_button_time[0] < DEBOUNCE) {
        return;
    }
    last_button_time[0] = current_time;
    
    if(menu_option != 1){
        menu_option = 1;
        status = false;
        g_stop_measurement = false;
        uart_puts("Optiunea 1\r\n");
    }
    else if (menu_option == 1){
        menu_option = 0;
        status = false;
        g_stop_measurement = true;
        uart_puts("Meniu principal\r\n");
    }
}

// ISR for INT3 (Pin 18)
ISR(INT3_vect) {
    unsigned long current_time = millis();
    if (current_time - last_button_time[1] < DEBOUNCE) {
        return;
    }
    last_button_time[1] = current_time;
    
    if(menu_option != 2){
        menu_option = 2;
        status = false;
        uart_puts("Optiunea 2\r\n");
    }
    else if (menu_option == 2){
        menu_option = 0;
        status = false;
        uart_puts("Meniu principal\r\n");
    }
}

// ISR for INT4 (Pin 2)
ISR(INT4_vect) {
    unsigned long current_time = millis();
    if (current_time - last_button_time[2] < DEBOUNCE) {
        return;
    }
    last_button_time[2] = current_time;
    
    if(menu_option != 3){
        menu_option = 3;
        status = false;
        uart_puts("Optiunea 3\r\n");
    }
    else if (menu_option == 3){
        menu_option = 0;
        status = false;
        uart_puts("Meniu principal\r\n");
    }
}
