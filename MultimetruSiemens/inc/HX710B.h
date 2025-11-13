/*
 * HX710B.h
 * 
 * Driver for HX710B Pressure Sensor
 * Measurement range: 0-40 kPa
 * Output: Digital via I2C/SPI or Analog (ADC)
 * 
 * Blood Pressure Parameters:
 * - Systolic pressure: 90-120 mmHg (12-16 kPa)
 * - Diastolic pressure: 60-80 mmHg (8-10.7 kPa)
 * - Mean arterial pressure: 70-100 mmHg (9.3-13.3 kPa)
 */

#ifndef HX710B_H_
#define HX710B_H_

#include <stdint.h>
#include <stdbool.h>

// Pressure conversion constants
#define PA_TO_MMHG 0.00750062  // 1 Pa = 0.00750062 mmHg
#define KPA_TO_MMHG 7.50062    // 1 kPa = 7.50062 mmHg
#define KPA_TO_PSI 0.145038    // 1 kPa = 0.145038 PSI

// Sensor range
#define SENSOR_MIN_KPA 0.0f
#define SENSOR_MAX_KPA 40.0f
#define SENSOR_MIN_MMHG 0.0f
#define SENSOR_MAX_MMHG 300.0f  // 40 kPa = 300 mmHg

// HX710B Digital Interface Configuration
// HX710B is a 24-bit digital ADC with serial interface (NOT analog!)
#define HX710B_DOUT_PORT PORTA  // Data OUT pin
#define HX710B_DOUT_DDR  DDRA
#define HX710B_DOUT_PIN  PINA
#define HX710B_DOUT_BIT  1      // PA1 (pin 23)

#define HX710B_SCK_PORT  PORTA  // Clock pin
#define HX710B_SCK_DDR   DDRA
#define HX710B_SCK_BIT   2      // PA2 (pin 24)

// Digital ADC values
#define HX710B_MAX_VALUE 16777216L  // 24-bit max value (2^24)
#define HX710B_ZERO_VALUE 8388607L  // Mid-point for zero pressure

// Blood pressure reference values (mmHg)
#define BP_SYSTOLIC_MIN 90
#define BP_SYSTOLIC_MAX 120
#define BP_DIASTOLIC_MIN 60
#define BP_DIASTOLIC_MAX 80
#define BP_MEAN_MIN 70
#define BP_MEAN_MAX 100

// Pressure status codes
#define BP_STATUS_LOW 0      // Hypotension
#define BP_STATUS_NORMAL 1   // Normal
#define BP_STATUS_ELEVATED 2 // Elevated
#define BP_STATUS_HIGH 3     // Hypertension

// Data structure for blood pressure readings
typedef struct {
    float systolic;      // Systolic pressure (mmHg)
    float diastolic;     // Diastolic pressure (mmHg)
    float mean;          // Mean arterial pressure (mmHg)
    float pulse;         // Pulse pressure (mmHg)
    uint8_t status;      // BP status (LOW, NORMAL, ELEVATED, HIGH)
    uint32_t timestamp;  // Reading timestamp (ms)
} BloodPressure_t;

// Data structure for raw sensor data
typedef struct {
    float pressure_kpa;  // Pressure in kPa
    float pressure_mmhg; // Pressure in mmHg
    float pressure_psi;  // Pressure in PSI
    uint16_t adc_raw;    // Raw ADC value (0-1023)
} PressureReading_t;

// Initialization functions
// hx710b_init - Initialize pressure sensor (digital interface)
// Return: true if sensor responds, false otherwise
bool hx710b_init(void);

// hx710b_is_ready - Check if HX710B has data ready
// Return: true if ready to read
bool hx710b_is_ready(void);

// hx710b_read_digital - Read 24-bit digital value from HX710B
// Return: Raw 24-bit value
int32_t hx710b_read_digital(void);

// hx710b_read_adc - Read raw ADC value from sensor (legacy name, now digital)
// Return: ADC value (0-16777216)
uint16_t hx710b_read_adc(void);

// hx710b_read_raw - Read raw pressure data
// Parameters: reading (pointer to PressureReading_t structure)
// Converts ADC value to kPa, mmHg, and PSI
void hx710b_read_raw(PressureReading_t *reading);

// Conversion functions
// hx710b_digital_to_kpa - Convert digital value to kPa
// Parameters: digital_value (24-bit value from sensor)
// Return: Pressure in kPa (0-40)
float hx710b_digital_to_kpa(int32_t digital_value);

// hx710b_adc_to_kpa - Convert ADC value to kPa (legacy, redirects to digital)
// Parameters: adc_value (ignored, uses digital read)
// Return: Pressure in kPa (0-40)
float hx710b_adc_to_kpa(uint16_t adc_value);

// hx710b_kpa_to_mmhg - Convert kPa to mmHg
// Parameters: kpa (pressure in kPa)
// Return: Pressure in mmHg
float hx710b_kpa_to_mmhg(float kpa);

// hx710b_kpa_to_psi - Convert kPa to PSI
// Parameters: kpa (pressure in kPa)
// Return: Pressure in PSI
float hx710b_kpa_to_psi(float kpa);

// Blood pressure analysis functions
// hx710b_calibrate - Calibrate sensor (zero point calibration)
// Should be called when sensor is at atmospheric pressure (0 kPa)
void hx710b_calibrate(void);

// hx710b_measure_bp - Measure and analyze blood pressure
// Parameters: systolic (pointer to systolic value), diastolic (pointer to diastolic value)
// Mean and pulse calculated automatically
// Return: BP status code (BP_STATUS_LOW, NORMAL, ELEVATED, HIGH)
uint8_t hx710b_measure_bp(float *systolic, float *diastolic);

// hx710b_analyze_bp - Analyze blood pressure values
// Parameters: bp (pointer to BloodPressure_t structure with systolic/diastolic set)
// Calculates mean and pulse pressures, determines status
uint8_t hx710b_analyze_bp(BloodPressure_t *bp);

// hx710b_get_status_string - Get human-readable status string
// Parameters: status (BP_STATUS_LOW, NORMAL, ELEVATED, HIGH)
// Return: Pointer to status string ("Hypotension", "Normal", "Elevated", "Hypertension")
const char* hx710b_get_status_string(uint8_t status);

// Filtering and averaging functions
#define PRESSURE_BUFFER_SIZE 10

typedef struct {
    float buffer[PRESSURE_BUFFER_SIZE];
    uint8_t index;
    uint8_t count;
} PressureFilter_t;

// hx710b_filter_init - Initialize pressure filter
// Parameters: filter (pointer to PressureFilter_t)
void hx710b_filter_init(PressureFilter_t *filter);

// hx710b_filter_add - Add pressure value to filter buffer
// Parameters: filter (pointer to PressureFilter_t), value (pressure in kPa)
void hx710b_filter_add(PressureFilter_t *filter, float value);

// hx710b_filter_get_average - Get averaged pressure value
// Parameters: filter (pointer to PressureFilter_t)
// Return: Averaged pressure in kPa
float hx710b_filter_get_average(PressureFilter_t *filter);

// Main streaming function
// hx710b_start_monitoring - Start continuous blood pressure monitoring
// Displays readings on LCD and sends via UART
// Send 'S' to stop monitoring
void hx710b_start_monitoring(void);

#endif /* HX710B_H_ */
