/*
 * HX710B.c
 * 
 * Implementation for HX710B Pressure Sensor Driver
 * Blood pressure measurement system
 * HX710B is a 24-bit digital ADC with serial interface
 */

#include "../inc/HX710B.h"
#include "../Config/uart.h"
#include "../inc/LCD.h"
#include <util/delay.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>  // For abs()

// Calibration values
static int32_t calibration_zero = 8388607L;  // Mid-point for zero pressure
static float calibration_span = 1.0f;

// Filtering buffers
static PressureFilter_t systolic_filter;
static PressureFilter_t diastolic_filter;

//====================================================================
// Basic functions
//====================================================================

// Initialize sensor - configure digital pins
bool hx710b_init(void) {
    // Configure SCK as output
    HX710B_SCK_DDR |= (1 << HX710B_SCK_BIT);
    HX710B_SCK_PORT &= ~(1 << HX710B_SCK_BIT);  // Set LOW
    
    // Configure DOUT as input
    HX710B_DOUT_DDR &= ~(1 << HX710B_DOUT_BIT);
    
    hx710b_filter_init(&systolic_filter);
    hx710b_filter_init(&diastolic_filter);
    
    _delay_ms(100);  // Wait for sensor to stabilize
    
    return true;
}

// Check if HX710B is ready to send data
bool hx710b_is_ready(void) {
    return (HX710B_DOUT_PIN & (1 << HX710B_DOUT_BIT)) == 0;
}

// Read 24-bit digital value from HX710B
int32_t hx710b_read_digital(void) {
    int32_t value = 0;
    
    // Wait for sensor to be ready (DOUT goes LOW)
    while((HX710B_DOUT_PIN & (1 << HX710B_DOUT_BIT)) != 0);
    
    // Read 24 bits
    for(uint8_t i = 0; i < 24; i++) {
        // Clock HIGH
        HX710B_SCK_PORT |= (1 << HX710B_SCK_BIT);
        _delay_us(1);
        
        // Shift and read bit
        value = value << 1;
        if(HX710B_DOUT_PIN & (1 << HX710B_DOUT_BIT)) {
            value++;
        }
        
        // Clock LOW
        HX710B_SCK_PORT &= ~(1 << HX710B_SCK_BIT);
        _delay_us(1);
    }
    
    // Send one more pulse to select channel/gain
    HX710B_SCK_PORT |= (1 << HX710B_SCK_BIT);
    _delay_us(1);
    HX710B_SCK_PORT &= ~(1 << HX710B_SCK_BIT);
    _delay_us(1);
    
    // Convert to signed value (24-bit two's complement)
    if(value & 0x800000) {
        value |= 0xFF000000;
    }
    
    return value;
}

// Legacy function - read raw ADC (now returns digital value scaled down)
uint16_t hx710b_read_adc(void) {
    int32_t digital = hx710b_read_digital();
    // Scale 24-bit to 16-bit for compatibility
    return (uint16_t)((digital >> 8) & 0xFFFF);
}

// Read and convert pressure data
void hx710b_read_raw(PressureReading_t *reading) {
    int32_t digital_value = hx710b_read_digital();
    
    reading->adc_raw = (uint16_t)(digital_value >> 8);  // Store scaled version
    reading->pressure_kpa = hx710b_digital_to_kpa(digital_value);
    reading->pressure_mmhg = hx710b_kpa_to_mmhg(reading->pressure_kpa);
    reading->pressure_psi = hx710b_kpa_to_psi(reading->pressure_kpa);
}

//====================================================================
// Conversion functions
//====================================================================

// Digital to kPa conversion
// HX710B outputs 24-bit value, centered around 8388607 for zero pressure
// Calibrated based on real sensor testing: offset -7900000 ≈ 30 kPa
float hx710b_digital_to_kpa(int32_t digital_value) {
    // Calculate offset from zero point
    int32_t offset = digital_value - calibration_zero;
    
    // Convert to pressure using calibrated factor (263000 per kPa)
    float kpa = abs((float)offset) / 263000.0f;
    
    // Apply calibration span
    kpa = kpa * calibration_span;
    
    // Limit to sensor range
    if(kpa > SENSOR_MAX_KPA) kpa = SENSOR_MAX_KPA;
    if(kpa < 0) kpa = 0;
    
    return kpa;
}

// Legacy ADC to kPa - now redirects to digital conversion
float hx710b_adc_to_kpa(uint16_t adc_value) {
    // Read fresh digital value
    int32_t digital = hx710b_read_digital();
    return hx710b_digital_to_kpa(digital);
}

// kPa to mmHg conversion
float hx710b_kpa_to_mmhg(float kpa) {
    return kpa * KPA_TO_MMHG;
}

// kPa to PSI conversion
float hx710b_kpa_to_psi(float kpa) {
    return kpa * KPA_TO_PSI;
}

//====================================================================
// Calibration function
//====================================================================

// Zero calibration - call when sensor at atmospheric pressure
void hx710b_calibrate(void) {
    // Read multiple samples
    int32_t sum = 0;
    for (uint8_t i = 0; i < 10; i++) {
        int32_t digital_val = hx710b_read_digital();
        sum += digital_val;
        _delay_ms(100);
    }
    
    calibration_zero = sum / 10;
    calibration_span = 1.0f;
    
    uart_puts("Calibration complete - Zero point: ");
    char buffer[12];
    sprintf(buffer, "%ld", calibration_zero);
    uart_puts(buffer);
    uart_puts("\r\n");
}

//====================================================================
// Blood pressure analysis functions
//====================================================================

// Measure blood pressure
// Note: HX710B measures absolute pressure, so external algorithm needed
// for systolic/diastolic separation (oscillometric method)
uint8_t hx710b_measure_bp(float *systolic, float *diastolic) {
    PressureReading_t reading;
    hx710b_read_raw(&reading);
    
    // Add to filters
    hx710b_filter_add(&systolic_filter, reading.pressure_mmhg);
    hx710b_filter_add(&diastolic_filter, reading.pressure_mmhg);
    
    // Get averaged values
    *systolic = hx710b_filter_get_average(&systolic_filter);
    *diastolic = hx710b_filter_get_average(&diastolic_filter);
    
    // Analyze status
    BloodPressure_t bp;
    bp.systolic = *systolic;
    bp.diastolic = *diastolic;
    
    return hx710b_analyze_bp(&bp);
}

// Analyze blood pressure values
uint8_t hx710b_analyze_bp(BloodPressure_t *bp) {
    // Calculate mean and pulse pressures
    bp->pulse = bp->systolic - bp->diastolic;
    bp->mean = bp->diastolic + (bp->pulse / 3.0f);
    
    // Determine status based on systolic pressure
    if (bp->systolic < BP_SYSTOLIC_MIN) {
        bp->status = BP_STATUS_LOW;
    }
    else if (bp->systolic >= BP_SYSTOLIC_MIN && bp->systolic <= BP_SYSTOLIC_MAX &&
             bp->diastolic >= BP_DIASTOLIC_MIN && bp->diastolic <= BP_DIASTOLIC_MAX) {
        bp->status = BP_STATUS_NORMAL;
    }
    else if (bp->systolic > BP_SYSTOLIC_MAX && bp->systolic <= 140) {
        bp->status = BP_STATUS_ELEVATED;
    }
    else {
        bp->status = BP_STATUS_HIGH;
    }
    
    return bp->status;
}

// Get status string
const char* hx710b_get_status_string(uint8_t status) {
    switch (status) {
        case BP_STATUS_LOW:
            return "Hypotension";
        case BP_STATUS_NORMAL:
            return "Normal";
        case BP_STATUS_ELEVATED:
            return "Elevated";
        case BP_STATUS_HIGH:
            return "Hypertension";
        default:
            return "Unknown";
    }
}

//====================================================================
// Filtering functions
//====================================================================

void hx710b_filter_init(PressureFilter_t *filter) {
    filter->index = 0;
    filter->count = 0;
    for (uint8_t i = 0; i < PRESSURE_BUFFER_SIZE; i++) {
        filter->buffer[i] = 0.0f;
    }
}

void hx710b_filter_add(PressureFilter_t *filter, float value) {
    filter->buffer[filter->index] = value;
    filter->index = (filter->index + 1) % PRESSURE_BUFFER_SIZE;
    
    if (filter->count < PRESSURE_BUFFER_SIZE) {
        filter->count++;
    }
}

float hx710b_filter_get_average(PressureFilter_t *filter) {
    if (filter->count == 0) {
        return 0.0f;
    }
    
    float sum = 0.0f;
    for (uint8_t i = 0; i < filter->count; i++) {
        sum += filter->buffer[i];
    }
    
    return sum / (float)filter->count;
}

//====================================================================
// Main monitoring function
//====================================================================

void hx710b_start_monitoring(void) {
    uart_puts("Blood Pressure Monitor started...\r\n");
    uart_puts("Send 'S' to stop\r\n");
    uart_puts("Sys,Dia,Mean,Pulse,Status\r\n");
    
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_WriteString("BP: -- / --     ");
    LCD_SetCursor(1, 0);
    LCD_WriteString("Status: Normal  ");
    
    _delay_ms(1000);
    
    while (1) {
        // Read pressure
        float systolic, diastolic;
        uint8_t status = hx710b_measure_bp(&systolic, &diastolic);
        
        // Create BP structure
        BloodPressure_t bp;
        bp.systolic = systolic;
        bp.diastolic = diastolic;
        bp.status = status;
        bp.timestamp = 0;  // Would use millis() if available
        
        // Update LCD
        char bp_str[17];
        int sys_int = (int)systolic;
        int dia_int = (int)diastolic;
        sprintf(bp_str, "BP: %3d / %2d    ", sys_int, dia_int);
        LCD_SetCursor(0, 0);
        LCD_WriteString(bp_str);
        
        char status_str[17];
        const char *status_name = hx710b_get_status_string(status);
        sprintf(status_str, "%-15s", status_name);
        LCD_SetCursor(1, 0);
        LCD_WriteString(status_str);
        
        // Send via UART
        uart_putInt(sys_int);
        uart_puts(",");
        uart_putInt(dia_int);
        uart_puts(",");
        uart_putInt((int)bp.mean);
        uart_puts(",");
        uart_putInt((int)bp.pulse);
        uart_puts(",");
        uart_puts(status_name);
        uart_puts("\r\n");
        
        // Check for stop command
        if ((UCSR0A & (1 << RXC0))) {
            char cmd = uart_getc();
            if (cmd == 'S' || cmd == 's') {
                uart_puts("Monitoring stopped\r\n");
                break;
            }
        }
        
        _delay_ms(1000);  // Update every 1 second
    }
}

void Tensiometer_Start(void) {
    hx710b_init();
    hx710b_calibrate();  // 0 kPa
    hx710b_start_monitoring();
}