/*
 * LCD.c
 *
 * Created: 10/29/2025 11:46:16 AM
 *  Author: Alexandru
 */ 

#define F_CPU 16000000UL
#include "LCD.h"
#include <avr/io.h>
#include <util/delay.h>

// functie de enable al pulsului
static void LCD_PulseEnable(void) {
    LCD_E_PORT |= (1 << LCD_E_PIN);
    _delay_us(1);
    LCD_E_PORT &= ~(1 << LCD_E_PIN);
    _delay_us(50);
}


// trimite nibble 4 biti pe pinii d4-d7.
static void LCD_SendNibble(uint8_t nibble) {
    // Setează fiecare pin individual pentru că sunt pe porturi diferite
    if (nibble & 0x01) 
        LCD_D4_PORT |= (1 << LCD_D4_PIN);
    else 
        LCD_D4_PORT &= ~(1 << LCD_D4_PIN);
    
    if (nibble & 0x02) 
        LCD_D5_PORT |= (1 << LCD_D5_PIN);
    else 
        LCD_D5_PORT &= ~(1 << LCD_D5_PIN);
    
    if (nibble & 0x04) 
        LCD_D6_PORT |= (1 << LCD_D6_PIN);
    else 
        LCD_D6_PORT &= ~(1 << LCD_D6_PIN);
    
    if (nibble & 0x08) 
        LCD_D7_PORT |= (1 << LCD_D7_PIN);
    else 
        LCD_D7_PORT &= ~(1 << LCD_D7_PIN);
    
    LCD_PulseEnable();
}


//Initializarea LCD-ului prin care trimit secvente de 4 biti (4-bit mode) - 
void LCD_Init(void) {
    // Setare pinuri ca output
    LCD_RS_DDR |= (1 << LCD_RS_PIN);
    LCD_E_DDR |= (1 << LCD_E_PIN);
    LCD_D4_DDR |= (1 << LCD_D4_PIN);
    LCD_D5_DDR |= (1 << LCD_D5_PIN);
    LCD_D6_DDR |= (1 << LCD_D6_PIN);
    LCD_D7_DDR |= (1 << LCD_D7_PIN);
    
	//setare pini pe low.
    LCD_RS_PORT &= ~(1 << LCD_RS_PIN);
    LCD_E_PORT &= ~(1 << LCD_E_PIN);
    LCD_D4_PORT &= ~(1 << LCD_D4_PIN);
    LCD_D5_PORT &= ~(1 << LCD_D5_PIN);
    LCD_D6_PORT &= ~(1 << LCD_D6_PIN);
    LCD_D7_PORT &= ~(1 << LCD_D7_PIN);
    
	//secventa de initialziare 
    LCD_RS_PORT &= ~(1 << LCD_RS_PIN);  // RS = 0 pentru comanda.
    
    // Trimite 0x03 de 3 ori (8-bit interface)
    LCD_SendNibble(0x03);
    _delay_ms(5);
    
    LCD_SendNibble(0x03);
    _delay_us(150);
    
    LCD_SendNibble(0x03);
    _delay_us(150);
    
    //Acum seteaza pe 4 biti.
    LCD_SendNibble(0x02);
    _delay_us(150);
    
	
	//Trimitere de biti pentru configurare - folosirea functiilor.
    LCD_SendCommand(0x28); // Function set: 4-bit, 2 lines, 5x8 font
    LCD_SendCommand(0x0C); // Display ON, cursor OFF, blink OFF
    LCD_SendCommand(0x06); // Entry mode: increment cursor, no shift
    LCD_SendCommand(0x01); // Clear display
}

void LCD_SendCommand(uint8_t cmd) {
    LCD_RS_PORT &= ~(1 << LCD_RS_PIN); // RS = 0 (command)
    LCD_SendNibble(cmd >> 4);           // High nibble
    LCD_SendNibble(cmd);                // Low nibble
    _delay_us(100);                     // Wait for command to complete
}

void LCD_SendData(uint8_t data) {
    LCD_RS_PORT |= (1 << LCD_RS_PIN);  // RS = 1 (data)
    LCD_SendNibble(data >> 4);          // High nibble
    LCD_SendNibble(data);               // Low nibble
    _delay_us(100);                     // Wait for data to complete
}

//------------------------------------------------------
//Functii de baza

void LCD_WriteChar(char c) {
    LCD_SendData((uint8_t)c);
}

void LCD_WriteString(const char *str) {
    while (*str) {
        LCD_WriteChar(*str++);
    }
}

void LCD_Clear(void) {
    LCD_SendCommand(LCD_CLEAR);
    _delay_ms(5);  // Clear command needs more time
}

void LCD_Home(void) {
    LCD_SendCommand(LCD_HOME);
    _delay_ms(5);  // Home command needs more time
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? col : (0x40 + col);
    LCD_SendCommand(0x80 | addr);
}