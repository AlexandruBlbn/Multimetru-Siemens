/*
 * AD8232.h
 *
 * Created: 10/30/2025 10:52:27 AM
 *  Author: Alexandru
 */ 

#ifndef AD8232_H_
#define AD8232_H_

#include <stdint.h>

#define AD8232_OUTPUT_PIN    0   // Digital pin connected to OUTPUT (ADC0 - A0)
// Pins for electrode detection
#define AD8232_LO_PLUS       1 // Pin ADC1
#define AD8232_LO_MINUS      2 // Pin ADC2

uint16_t AD8232_readOutput(void);
uint16_t AD8232_readOutput_Average(uint8_t esantioane);
uint16_t AD8232_readLOPlus(void);
uint16_t AD8232_readLOMinus(void);
uint8_t AD8232_readLO_Ambele(uint8_t *lo_plus, uint8_t *lo_minus);
void AD8232_init(void);
void AD8232_Start(void);

// Buffer access pentru transmitere UART
extern uint16_t ekg_buffer[];
extern uint16_t ekg_sample_count;

// from mV in range 0-1024 (0-5V)
float AD8232_readVoltage(void);

// EKG Streaming Real-Time
void AD8232_startStreaming(void);

#endif /* AD8232_H_ */