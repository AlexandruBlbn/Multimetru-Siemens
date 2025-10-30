/*
 * ADC.c
 *
 * Created: 10/30/2025 1:55:37 PM
 *  Author: Alexandru
 */ 

 #include "ADC.h"


 //init ADC.
 void adc_init(void){
    ADMUX |= (1<<REFS0); // (setare referinta 3.3v)
	ADCSRA |= (1 <<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // activare adc + setare prescaler 128 (tabel 26-5)
 }
 
 //selectare canal ADC
 // 0xF0 = 1111 0000, canal - pinul dorit
void adc_selectCanal(uint8_t canal){
    ADMUX = (ADMUX & 0xF0) | canal;
}

uint16_t adc_read(void){
    ADCSRA |= (1<<ADSC); // pornire conversie
    while(ADCSRA & (1<<ADSC)); // asteptare finalizare conversie
    return ADC; // returnare valoare citita
}

//citire adc cu mediere (pentru reducerea zgomotului)
// esantioane = numarul de esantioane pentru mediere
//functionare: se face suma tuturor esantioanelor si se imparte la numarul de esantioane
uint16_t adc_readAverage(uint8_t esantioane){
    uint32_t suma = 0;
    for(uint8_t i = 0; i < esantioane; i++){
        suma += adc_read();
    }
    return suma / esantioane;
}