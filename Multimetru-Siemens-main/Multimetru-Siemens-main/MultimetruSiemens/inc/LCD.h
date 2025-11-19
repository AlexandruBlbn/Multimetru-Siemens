/*
 * LCD.h
 *
 * Created: 10/29/2025 11:46:02 AM
 *  Author: Alexandru
 */ 


#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>

// Connected pins: RS=45, E=41, D4=29, D5=27, D6=25, D7=23 (atmega2560)

// VSS - GND, VDD - 5V, V0 - 2 resistors in series of 110 ohms - GND, A - 5V, K - 5V
//RS=43, E - 41, D4=39, D5=37, D6=35, D7=33

#define LCD_RS_PORT PORTL
#define LCD_RS_DDR  DDRL
#define LCD_RS_PIN  4        // Pin 45 (PL4) read/write

#define LCD_E_PORT PORTG
#define LCD_E_DDR  DDRG
#define LCD_E_PIN  0         // Pin 41 (PG0)

// D4-D7 - Ports for bit transfer on LCD
#define LCD_D4_PORT PORTA
#define LCD_D4_DDR  DDRA
#define LCD_D4_PIN  7        // Pin 29 (PA7) -> D4

#define LCD_D5_PORT PORTA
#define LCD_D5_DDR  DDRA
#define LCD_D5_PIN  5        // Pin 27 (PA5) -> D5

#define LCD_D6_PORT PORTA
#define LCD_D6_DDR  DDRA
#define LCD_D6_PIN  3        // Pin 25 (PA3) -> D6

#define LCD_D7_PORT PORTA
#define LCD_D7_DDR  DDRA
#define LCD_D7_PIN  1        // Pin 23 (PA1) -> D7

// LCD Commands 8-bit registers - page 24 documentation LCD table 6
#define LCD_CLEAR 00000001
#define LCD_HOME 00000010
#define LCD_ENTRY_MODE 00000100
#define LCD_DISPLAY_CONTROL 00001000
#define LCD_CURSOR_SHIFT 00010000
#define LCD_FUNCTION_SET 00100000
#define LCD_SET_CGRAM_ADDR 01000000
#define LCD_SET_DDRAM_ADDR 10000000


// Specific LCD functions (see documentation table 6) for how it works.
void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_WriteChar(char c);
void LCD_WriteString(const char *str);
void LCD_Clear(void);
void LCD_Home(void);
void LCD_SetCursor(uint8_t row, uint8_t col);




#endif /* LCD_H_ */