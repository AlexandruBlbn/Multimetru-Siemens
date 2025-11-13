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
#include "../Config/uart.h"
#include <util/delay.h>

// Configuratie sampling
#define SAMPLE_INTERVAL 10   // 10ms pentru 100 Hz

// Fara filtre - direct RAW

// ADC Reference Voltage
#define ADC_VREF 3.3f  // 5V reference (schimba la 3.3f daca folosesti 3.3V)
#define ADC_MAX 1023.0f


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


// //------------------------------------------
// // Converteste ADC la Volts
// //------------------------------------------

// float AD8232_readVoltage(void) {
//     uint16_t adcValue = AD8232_readOutput();
//     float voltage = (adcValue / ADC_MAX) * ADC_VREF;
//     return voltage;
// }


// //------------------------------------------
// // EKG Streaming Real-Time
// //------------------------------------------

// volatile uint8_t ad8232_streaming = 0;

// void AD8232_startStreaming(void) {
//     ad8232_streaming = 1;
//     uart_puts("EKG Streaming started (100 Hz, Voltage output)...\r\n");
//     uart_puts("Send 'S' to stop\r\n");
    
//     while(ad8232_streaming) {
//         // Citeste tensiunea in volti
//         float voltage = AD8232_readVoltage();
        
//         // Trimite pe UART cu 3 zecimale
//         uart_putFloat(voltage, 3);
//         uart_puts("\r\n");
        
//         // Delay pentru 100 Hz sampling (10ms)
//         _delay_ms(10);
        
//         // Verifica comanda de oprire
//         if (UCSR0A & (1 << RXC0)) {
//             char cmd = uart_getc();
//             if (cmd == 'S' || cmd == 's') {
//                 ad8232_streaming = 0;
//                 uart_puts("\r\nEKG Streaming stopped\r\n");
//             }
//         }
//     }
// }

