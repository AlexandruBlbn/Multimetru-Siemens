/*
 * AD8232.h
 *
 * Created: 10/30/2025 10:52:27 AM
 *  Author: Alexandru
 */ 

#ifndef AD8232_H_
#define AD8232_H_

#include <stdint.h>

#define AD8232_OUTPUT_PIN    0   // Pinul digital conectat la OUTPUT (ADC0 - A0)
//Pini pentru detectare de electrozi.
#define AD8232_LO_PLUS       1 // Pinul ADC1
#define AD8232_LO_MINUS      2 // Pinul ADC2

uint16_t AD8232_readOutput(void);
uint16_t AD8232_readOutput_Average(uint8_t esantioane);
uint16_t AD8232_readLOPlus(void);
uint16_t AD8232_readLOMinus(void);
uint8_t AD8232_readLO_Ambele(uint8_t *lo_plus, uint8_t *lo_minus);
void AD8232_init(void);


#endif /* AD8232_H_ */