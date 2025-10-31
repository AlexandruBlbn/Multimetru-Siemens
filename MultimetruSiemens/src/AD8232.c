/*
 * AD8232.c
 *
 * Created: 10/30/2025 10:52:37 AM
 *  Author: Alexandru
 */ 


//-----------------------
// functii de configurare si functionare

#include "AD8232.h"
#include "../Config/ADC.h"


void AD8232_init(void){
    adc_init(); // initializare ADC
    adc_selectCanal(AD8232_OUTPUT_PIN); // selectare canal OUTPUT (vezi AD8232.h pentru configurare pini)
    adc_selectCanal(AD8232_LO_PLUS); // selectare canal LO_PLUS (vezi AD8232.h pentru configurare pini)
    adc_selectCanal(AD8232_LO_MINUS); // selectare canal LO_MINUS (vezi AD8232.h pentru configurare pini)
}

uint16_t AD8232_readOutput(void){
    adc_selectCanal(AD8232_OUTPUT_PIN); // selectare canal OUTPUT
    return adc_read(); // citire valoare OUTPUT
}

uint16_t AD8232_readOutput_Average(uint8_t esantioane){
    adc_selectCanal(AD8232_OUTPUT_PIN); // selectare canal OUTPUT
    return adc_readAverage(esantioane); // citire valoare OUTPUT cu mediere
}

uint16_t AD8232_readLOPlus(void){
    adc_selectCanal(AD8232_LO_PLUS); // selectare canal LO_PLUS
    return adc_read(); // citire valoare LO_PLUS
}
uint16_t AD8232_readLOMinus(void){
    adc_selectCanal(AD8232_LO_MINUS); // selectare canal LO_MINUS
    return adc_read(); // citire valoare LO_MINUS
}

uint8_t AD8232_readLO_Ambele(uint8_t *lo_plus, uint8_t *lo_minus){
    *lo_plus = (uint8_t)AD8232_readLOPlus();
    *lo_minus = (uint8_t)AD8232_readLOMinus();
    return 1;  // Returnează status OK
}

