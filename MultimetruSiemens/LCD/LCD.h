/*
 * LCD.h
 *
 * Created: 10/29/2025 11:46:02 AM
 *  Author: Alexandru
 */ 


#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>

//pini contectati:  RS=53, E=52, D4=51, D5=50, D6=49, D7=48 (atmegA2560)

// VSS- GND, VDD - 5v, V0 - 2 rezistente in serie de 110 omhi - GND, A-5V, K-5V
#define LCD_RS_PORT PORTB
#define LCD_RS_DDR  DDRB
#define LCD_RS_PIN  0        // Pin 53 (PB0) read/write

#define LCD_E_PORT PORTB
#define LCD_E_DDR  DDRB
#define LCD_E_PIN  1         // Pin 52 (PB1)

// D4-D7 - porturi pentru transfer de biti pe LCD
#define LCD_D4_PORT PORTB
#define LCD_D4_DDR  DDRB
#define LCD_D4_PIN  2        // Pin 51 (PB2) -> D4

#define LCD_D5_PORT PORTB
#define LCD_D5_DDR  DDRB
#define LCD_D5_PIN  3        // Pin 50 (PB3) -> D5

#define LCD_D6_PORT PORTL
#define LCD_D6_DDR  DDRL
#define LCD_D6_PIN  0        // Pin 49 (PL0) -> D6

#define LCD_D7_PORT PORTL
#define LCD_D7_DDR  DDRL
#define LCD_D7_PIN  1        // Pin 48 (PL1) -> D7

// Comenzi LCD registrii de 8 biti - pagina 24 documentatie LCD tabel 6
#define LCD_CLEAR 00000001
#define LCD_HOME 00000010
#define LCD_ENTRY_MODE 00000100
#define LCD_DISPLAY_CONTROL 00001000
#define LCD_CURSOR_SHIFT 00010000
#define LCD_FUNCTION_SET 00100000
#define LCD_SET_CGRAM_ADDR 01000000
#define LCD_SET_DDRAM_ADDR 10000000

// functii SPECIFICE LCD-ului (vezi documentatie tabel 6) pentru functionare.
void LCD_Init(void);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_WriteChar(char c);
void LCD_WriteString(const char *str);
void LCD_Clear(void);
void LCD_Home(void);
void LCD_SetCursor(uint8_t row, uint8_t col);




#endif /* LCD_H_ */