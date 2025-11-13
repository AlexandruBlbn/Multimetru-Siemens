/*
 * Blood Pressure Monitor with HX710B Digital Pressure Sensor
 * 
 * Hardware Connections:
 * - Pump: PA0 (pin 22)
 * - HX710B DOUT: PA1 (pin 23)  
 * - HX710B SCK: PA2 (pin 24)
 * - HX710B VCC: 3.3V
 * - HX710B GND: GND
 * 
 * LCD: I2C
 * ECG: A0 (AD8232)
 */

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

// Pump control pin: PA0 (Digital Pin 22 on ATmega2560)
#define PUMP_PORT PORTA
#define PUMP_DDR DDRA
#define PUMP_PIN 0

// Target pressure for inflation (24 kPa = 180 mmHg)
#define TARGET_PRESSURE_KPA 24.0f

// Variables for measurements
volatile float bpSystolic = 0.0f;
volatile float bpDiastolic = 0.0f;
volatile float bpMean = 0.0f;
volatile uint8_t bpStatus = 0;
volatile uint8_t monitoring = 0;

// Oscillometric method variables
volatile float maxOscillation = 0.0f;
volatile float systolicPressure = 0.0f;
volatile float diastolicPressure = 0.0f;
volatile float meanPressure = 0.0f;
volatile uint8_t measurementComplete = 0;

void AVR_init(void) {
    uart_init();
    i2c_init();
    timer_init();
    LCD_Init();
    AD8232_init();
    hx710b_init();  // Initialize HX710B digital interface
	max30102_init();
    
    // Initialize pump control pin (PD0) as output
    PUMP_DDR |= (1 << PUMP_PIN);
    PUMP_PORT &= ~(1 << PUMP_PIN);  // Set to 0 (pump OFF)
    
    LCD_WriteString("iNITIalizare...");
	_delay_ms(2000);
    
    // Test HX710B sensor - read digital value
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_WriteString("Test HX710B:");
    
    _delay_ms(100);
    if(hx710b_is_ready()) {
        PressureReading_t test_reading;
        hx710b_read_raw(&test_reading);
        
        LCD_SetCursor(0, 1);
        char kpa_str[17] = "Press: 00 kPa   ";
        int kpa_val = (int)test_reading.pressure_kpa;
        if(kpa_val < 0) kpa_val = 0;
        if(kpa_val > 99) kpa_val = 99;
        kpa_str[7] = '0' + (kpa_val / 10);
        kpa_str[8] = '0' + (kpa_val % 10);
        LCD_WriteString(kpa_str);
    } else {
        LCD_SetCursor(0, 1);
        LCD_WriteString("Sensor not ready");
    }
    
    _delay_ms(5000);  // Show for 5 seconds
    
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_WriteString("BP Monitor      ");
    LCD_SetCursor(1, 0);
    LCD_WriteString("Starting in 10s ");
    _delay_ms(10000);  // Wait 10 seconds after reset
    
    LCD_Clear();
}

void display_main_screen(void) {
    // Line 1: BP Systolic
    char line1[17];
    sprintf(line1, "SYS:%3d DIA:%3d", (int)systolicPressure, (int)diastolicPressure);
    LCD_SetCursor(0, 0);
    LCD_WriteString(line1);
    
    // Line 2: Status
    char line2[17];
    const char *status_str;
    if (systolicPressure < 90) status_str = "Low     ";
    else if (systolicPressure <= 120) status_str = "Normal  ";
    else if (systolicPressure <= 140) status_str = "Elevated";
    else status_str = "High    ";
    
    sprintf(line2, "%s", status_str);
    LCD_SetCursor(1, 0);
    LCD_WriteString(line2);
}

void display_measuring_screen(float currentPressure) {
    char str1[17] = "Inflating       ";
    char str2[17] = "Press: 00 kPa   ";
    
    int pressure_int = (int)currentPressure;
    
    if (pressure_int < 10) {
        str2[7] = '0';
        str2[8] = '0' + pressure_int;
    } else if (pressure_int < 100) {
        str2[7] = '0' + (pressure_int / 10);
        str2[8] = '0' + (pressure_int % 10);
    }
    
    LCD_SetCursor(0, 0);
    LCD_WriteString(str1);
    LCD_SetCursor(0, 1);
    LCD_WriteString(str2);
}

int main(void) {
    AVR_init();
	
    // Oscillometric measurement variables
    float pressureReadings[100];  // Store pressure readings during deflation
    float oscillations[100];      // Store oscillation amplitude
    uint8_t readingIndex = 0;
    float currentPressure = 0.0f;
    float lastPressure = 0.0f;
    float maxOscillationValue = 0.0f;
    
    while(1) {

        // Clear and setup LCD for inflation
        LCD_Clear();
        LCD_SetCursor(0, 0);
        LCD_WriteString("Inflating       ");
        LCD_SetCursor(0, 1);
        LCD_WriteString("Press: 00 kPa   ");
        
        _delay_ms(100);  // Let LCD stabilize before starting pump
        
        // Start pump (set pin HIGH)
        PUMP_PORT |= (1 << PUMP_PIN);
        
        _delay_ms(200);  // Wait for pump to stabilize power consumption
        
        // Re-initialize LCD after pump starts (protects against voltage drop)
        LCD_Init();
        LCD_Clear();
        LCD_SetCursor(0, 0);
        LCD_WriteString("Inflating       ");
        LCD_SetCursor(0, 1);
        LCD_WriteString("Press: 00 kPa   ");
        
        // Read pressure until target is reached
        readingIndex = 0;
        currentPressure = 0.0f;
        uint16_t displayCounter = 0;
        
        while (currentPressure < TARGET_PRESSURE_KPA && readingIndex < 300) {
            // Read current pressure
            PressureReading_t bp_reading;
            hx710b_read_raw(&bp_reading);
            currentPressure = bp_reading.pressure_kpa;
            
            // Update display every 20 readings (2 seconds) to avoid flicker
            if (displayCounter % 20 == 0) {
                display_measuring_screen(currentPressure);
                _delay_ms(50);  // Extra delay to ensure LCD completes write
            }
            
            readingIndex++;
            displayCounter++;
            _delay_ms(100);
        }
        
        // Stop pump (set pin LOW)
        PUMP_PORT &= ~(1 << PUMP_PIN);
        
        _delay_ms(500);
        
        // Re-initialize LCD after pump stops (restore communication)
        LCD_Init();
        LCD_Clear();
        LCD_SetCursor(0, 0);
        LCD_WriteString("Deflating       ");
        LCD_SetCursor(0, 1);
        LCD_WriteString("Press: 00 kPa   ");
        
        // Deflation phase - detect oscillations
        readingIndex = 0;
        lastPressure = currentPressure;
        maxOscillationValue = 0.0f;
        displayCounter = 0;
        
        while (currentPressure > 0.0f && readingIndex < 100) {
            // Read current pressure
            PressureReading_t bp_reading;
            hx710b_read_raw(&bp_reading);
            currentPressure = bp_reading.pressure_kpa;
            
            // Store reading
            pressureReadings[readingIndex] = currentPressure;
            
            // Calculate oscillation amplitude (simplified)
            float oscillation = 0.0f;
            if (readingIndex > 0) {
                oscillation = (float)abs((int)(currentPressure - lastPressure) * 1000);
                if (oscillation > maxOscillationValue) {
                    maxOscillationValue = oscillation;
                }
            }
            oscillations[readingIndex] = oscillation;
            lastPressure = currentPressure;
            
            // Update display every 10 readings (1 second) to show progress
            if (displayCounter % 10 == 0) {
                char str[17] = "Press: 00 kPa   ";
                int pressure_int = (int)currentPressure;
                
                if (pressure_int < 10) {
                    str[7] = '0';
                    str[8] = '0' + pressure_int;
                } else if (pressure_int < 100) {
                    str[7] = '0' + (pressure_int / 10);
                    str[8] = '0' + (pressure_int % 10);
                }
                
                LCD_SetCursor(0, 1);
                LCD_WriteString(str);
                _delay_ms(50);  // Extra delay to ensure LCD completes write
            }
            
            readingIndex++;
            displayCounter++;
            _delay_ms(100);
        }
        
        // Calculate BP from oscillometric data
        if (readingIndex > 10) {
            // Find maximum oscillation point (mean pressure)
            float maxOscillationIndex = 0;
            maxOscillationValue = 0.0f;
            
            for (uint8_t i = 5; i < readingIndex - 5; i++) {
                if (oscillations[i] > maxOscillationValue) {
                    maxOscillationValue = oscillations[i];
                    maxOscillationIndex = i;
                }
            }
            
            meanPressure = pressureReadings[(uint8_t)maxOscillationIndex];
            systolicPressure = meanPressure + 15.0f;
            diastolicPressure = meanPressure - 10.0f;
        }
        
        // Display results
        LCD_Clear();
        LCD_SetCursor(0, 0);
        char sys_str[17] = "SYS:000 mmHg    ";
        int sys_val = (int)systolicPressure;
        sys_str[4] = '0' + (sys_val / 100);
        sys_str[5] = '0' + ((sys_val / 10) % 10);
        sys_str[6] = '0' + (sys_val % 10);
        LCD_WriteString(sys_str);
        
        LCD_SetCursor(0, 1);
        char dia_str[17] = "DIA:000 mmHg    ";
        int dia_val = (int)diastolicPressure;
        dia_str[4] = '0' + (dia_val / 100);
        dia_str[5] = '0' + ((dia_val / 10) % 10);
        dia_str[6] = '0' + (dia_val % 10);
        LCD_WriteString(dia_str);
        
        // Wait before next measurement
        _delay_ms(30000);
    }

    return 0;
}
