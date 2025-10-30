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


//initializare ADC
void adc_init(void);
//selectare canal ADC
void adc_selectCanal(uint8_t canal);
//citire adc
uint16_t adc_read(void);
//citire adc cu mediere (pentru reducerea zgomotului)
uint16_t adc_readAverage(uint8_t canal);




#endif /* ADC_H_ */