/*
 * ADC.h
 *
 * Created: 10/30/2025 12:30:39 PM
 *  Author: Alexandru
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>
#include <stdint.h>


// ADC initialization
void adc_init(void);
// ADC channel selection
void adc_selectCanal(uint8_t canal);
// Read ADC
uint16_t adc_read(void);
// Read ADC with averaging (for noise reduction)
uint16_t adc_readAverage(uint8_t esantioane);




#endif /* ADC_H_ */