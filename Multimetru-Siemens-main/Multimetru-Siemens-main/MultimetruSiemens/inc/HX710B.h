#ifndef HX710B_H_
#define HX710B_H_

#include <stdint.h>
#include <stdbool.h>

#define HX710B_DOUT_PORT    PORTC
#define HX710B_DOUT_DDR     DDRC
#define HX710B_DOUT_PIN     PINC
#define HX710B_DOUT_BIT     4        // Pin 33 (PC4)

#define HX710B_SCK_PORT     PORTC
#define HX710B_SCK_DDR      DDRC
#define HX710B_SCK_BIT      2        // Pin 35 (PC2)

//Control PIN to turn on/off the air pomp - pin 31 (PC6)
#define HX710B_PUMP_PORT    PORTC
#define HX710B_PUMP_DDR     DDRC
#define HX710B_PUMP_PIN     6        // Pin 31 (PC6)

// Calibration constants for 0-40 kPa sensor range
// These values should be adjusted after calibration for the specific sensor
#define HX710B_VALOARE_ZERO        290000L    // ADC value at 0 kPa (measured calibration)
#define HX710B_VALOARE_40KPA       8388607L   // ADC value at 40 kPa (maximum pressure)
#define HX710B_PRESIUNE_MAX        40.0f      // Maximum pressure in kPa
// Parameters for filtering and reading stability
#define HX710B_NUMAR_CITIRI        12         // Number of readings for averaging
#define HX710B_DELAY_CITIRE        1         // Delay between readings (ms)

// Parameters for the oscillometric algorithm
#define HX710B_FILTRU_MARIME           8
#define HX710B_OSC_THRESHOLD_KPA       0.2f
#define HX710B_MIN_PULSE_INTERVAL_MS   200
#define HX710B_INFLATE_TARGET_KPA      40.0f   // Initial inflation target (kPa)
#define HX710B_INFLATE_SAFETY_KPA      40.2f   // Absolute safety threshold (kPa)
#define HX710B_RELEASE_PRESSURE_KPA    8.0f
#define HX710B_POST_INFLATE_SETTLE_MS  70
#define HX710B_SAMPLE_INTERVAL_MS      5
#define HX710B_INFLATE_MAX_LOOPS       250
#define HX710B_DEF_SAMPLE_COUNT        130

//================================================================================================================================================
// Basic functions - Communication with HX710B pressure sensor

// HX710B_Init -
// Configures DOUT as input and SCK as output
void HX710B_Init(void);

// HX710B_CitesteSenzor - Reads the raw 24-bit value from the sensor
// Return: Signed 24-bit value (two's complement)
// Uses HX710B specific serial protocol (25 SCK pulses)
int32_t HX710B_CitesteSenzor(void);

// HX710B_CitestePresiune - Reads the pressure in kPa
// Return: Pressure in kPa (float)
// Takes multiple readings, removes outliers, and applies calibration
float HX710B_CitestePresiune(void);

// HX710B_Check - Verifies if the sensor is connected and functional
// Displays message via UART
// Return: true if the sensor responds correctly, false otherwise
bool HX710B_Check(void);

// Data structure for blood pressure results
typedef struct {
	float presiune_curenta_kpa; //curent pressure kpa
	float presiune_curenta_mmhg; //curent pressure mmhg
	float oscilatie_kpa; //oscillation amplitude kpa
	bool  puls_detectat; //pulse detected
	float bpm; //beats per minute
	float sistolica_mmhg; //systolic pressure mmhg
	float diastolica_mmhg; //diastolic pressure mmhg
} HX710B_BloodPressureData;

// Resets internal filters of the oscillometric algorithm
void HX710B_ResetAlgorithm(void);

// Starts a complete measurement (inflation + analysis)
// Pump control is done on pin 31 (PC6)
// Returns true if valid data exists in the result structure
bool HX710B_StartMeasurement(HX710B_BloodPressureData *result);

// Main loop for repeated measurements
// Displays results on LCD and sends logs via UART
void HX710B_Start(void);

#endif /* HX710B_H_ */
