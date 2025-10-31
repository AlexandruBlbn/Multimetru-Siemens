/*
 * AD8232.c
 *
 * Created: 10/30/2025 10:52:37 AM
 *  Author: Alexandru
 */ 


//-----------------------
// Configuration and operation functions

#include "../inc/AD8232.h"
#include "../Config/ADC.h"


void AD8232_init(void){
    adc_init(); // ADC initialization
    adc_selectCanal(AD8232_OUTPUT_PIN); // Select OUTPUT channel (see AD8232.h for pin configuration)
    adc_selectCanal(AD8232_LO_PLUS); // Select LO_PLUS channel (see AD8232.h for pin configuration)
    adc_selectCanal(AD8232_LO_MINUS); // Select LO_MINUS channel (see AD8232.h for pin configuration)
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

