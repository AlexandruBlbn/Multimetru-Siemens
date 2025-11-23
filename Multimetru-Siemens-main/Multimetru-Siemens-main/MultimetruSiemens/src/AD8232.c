/*
 * AD8232.c
 *
 * Created: 10/30/2025 10:52:37 AM
 *  Author: Alexandru
 */ 

#include "../inc/AD8232.h"
#include "../Config/ADC.h"
#include "../inc/LCD.h"
#include "../Config/uart.h"
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#define AD8232_STREAM_DELAY_MS     10      // 100 Hz sampling
#define AD8232_LCD_REFRESH_CYCLES  (1000 / AD8232_STREAM_DELAY_MS)
#define AD8232_LO_CONNECTED_MAX    150     // valori mici => electrozi conectati
#define AD8232_LO_DISCONNECTED_MIN 850     // valori mari => electrozi desprinsi

#define ADC_REF_VOLTAGE            3.3f
#define ADC_MAX_VALUE              1023.0f

uint16_t ekg_buffer[1] = {0};
uint16_t ekg_sample_count = 0;

volatile uint8_t ad8232_streaming = 0;
extern volatile bool g_stop_measurement;

// Forward declarations
uint16_t AD8232_readOutput(void);
uint16_t AD8232_readLOPlus(void);
uint16_t AD8232_readLOMinus(void);

void AD8232_init(void){
    adc_init(); // ADC initialization
}

void AD8232_Start(void){
    ekg_sample_count = 0;
    ekg_buffer[0] = AD8232_readOutput();
}

uint16_t AD8232_readOutput(void){
    adc_selectCanal(AD8232_OUTPUT_PIN); // Select OUTPUT channel
    return adc_read(); // Read OUTPUT value
}

uint16_t AD8232_readOutput_Average(uint8_t esantioane){
    adc_selectCanal(AD8232_OUTPUT_PIN); // Select OUTPUT channel
    return adc_readAverage(esantioane); // Read OUTPUT value with averaging
}

uint16_t AD8232_readLOPlus(void){
    adc_selectCanal(AD8232_LO_PLUS); // Select LO_PLUS channel
    return adc_read(); // Read LO_PLUS value
}
uint16_t AD8232_readLOMinus(void){
    adc_selectCanal(AD8232_LO_MINUS); // Select LO_MINUS channel
    return adc_read(); // Read LO_MINUS value
}

uint8_t AD8232_readLO_Ambele(uint8_t *lo_plus, uint8_t *lo_minus){
    *lo_plus = (uint8_t)AD8232_readLOPlus();
    *lo_minus = (uint8_t)AD8232_readLOMinus();
    return 1;  // Return status OK
}

float AD8232_readVoltage(void) {
    uint16_t rawValue = AD8232_readOutput();
    return (rawValue / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
}

static void AD8232_sendPacket(uint16_t ekg_value, uint16_t lo_plus, uint16_t lo_minus, bool leads_ok)
{
    char packet[40];
    snprintf(packet, sizeof(packet), "%u,%u,%u,%s\r\n",
             ekg_value, lo_plus, lo_minus, leads_ok ? "OK" : "FAIL");
    uart_puts(packet);
}

void AD8232_startStreaming(void) {
    ad8232_streaming = 1;
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_WriteString("EKG Streaming");

    uint16_t lcd_counter = 0;

    while (ad8232_streaming && !g_stop_measurement) {
        uint16_t lo_plus = AD8232_readLOPlus();
        uint16_t lo_minus = AD8232_readLOMinus();
        bool leads_ok = (lo_plus < AD8232_LO_CONNECTED_MAX) && (lo_minus < AD8232_LO_CONNECTED_MAX);
        bool leads_fail = (lo_plus > AD8232_LO_DISCONNECTED_MIN) || (lo_minus > AD8232_LO_DISCONNECTED_MIN);

        uint16_t ekg_value = AD8232_readOutput();
        AD8232_sendPacket(ekg_value, lo_plus, lo_minus, leads_ok && !leads_fail);

        if (lcd_counter == 0) {
            LCD_SetCursor(1, 0);
            if (leads_ok && !leads_fail) {
                char line[17];
                snprintf(line, sizeof(line), "Val:%4u", ekg_value);
                LCD_WriteString(line);
            } else {
                LCD_WriteString("! ELECTROZI !   ");
            }
        }

        lcd_counter++;
        if (lcd_counter >= AD8232_LCD_REFRESH_CYCLES) {
            lcd_counter = 0;
        }

        _delay_ms(AD8232_STREAM_DELAY_MS);
    }

    ad8232_streaming = 0;
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_WriteString("EKG Stopped");
    _delay_ms(500);
}

