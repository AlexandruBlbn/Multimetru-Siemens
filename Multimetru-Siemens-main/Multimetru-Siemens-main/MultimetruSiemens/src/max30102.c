// #define F_CPU 16000000UL // Removed: defined by build system

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "../inc/max30102.h"
#include "../Config/i2c.h"
#include "../Config/uart.h"
#include "../inc/LCD.h"
#include <stdio.h>

// External stop flag from main
extern volatile bool g_stop_measurement;


//Initialize sensor parameters
#define RATE_SIZE 10
#define AVG_SIZE 5
#define SPO2_BUFFER_SIZE 100

uint8_t rates[RATE_SIZE];
uint8_t rateSpot = 0;
float beatsPerMinute = 0;
int beatAvg = 0;
int beatCount = 0;
unsigned long lastBeatTime = 0;

long irValue = 0;
long lastIR = 0;
long derivative = 0;
long lastDerivative = 0;

long irAvgBuffer[AVG_SIZE];
int avgIndex = 0;
long irSmooth = 0;

long irPeak = 0;
long irValley = 999999;
uint8_t peakDetected = 0;

uint32_t irBuffer[SPO2_BUFFER_SIZE];
uint32_t redBuffer[SPO2_BUFFER_SIZE];
int bufferIndex = 0;
uint8_t bufferReady = 0;

float SpO2 = 0;

volatile unsigned long millisCounter = 0;

void timer_init(void) {
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
	OCR1A = 249;
	TIMSK1 = (1 << OCIE1A);
	sei();
}

// millis - Returns number of milliseconds since startup
unsigned long millis(void) {
    unsigned long m;
    uint8_t oldSREG = SREG;
    cli();
    m = millisCounter;
    SREG = oldSREG;
	return m;
}

// micros - Returns number of microseconds since startup
unsigned long micros(void) {
    unsigned long m;
    uint16_t t;
    uint8_t oldSREG = SREG;
    
    cli();
    m = millisCounter;
    t = TCNT1;
    
    // Check for pending interrupt (timer wrapped but ISR didn't run yet)
    if ((TIFR1 & (1 << OCF1A)) && (t < 249)) {
        m++;
        t = TCNT1;
    }
    SREG = oldSREG;
    
    return (m * 1000) + (t * 4);
}

// ISR(TIMER1_COMPA_vect) - Timer interrupt routine
ISR(TIMER1_COMPA_vect) {
	millisCounter++;
}

//================================================================================================================================================
// Basic functions - Communication with MAX30102 sensor via I2C

// max30102_write_reg - Write a value to a sensor register
// Parameters: reg (register address), value (8-bit value to write)
// Uses I2C to communicate with the sensor at MAX30102_ADDRESS
void max30102_write_reg(uint8_t reg, uint8_t value) {
    i2c_write_register(MAX30102_ADDRESS, reg, value);
}

// max30102_read_reg - Read a value from a sensor register
// Parameters: reg (register address)
// Return: 8-bit value from register
// Uses I2C to communicate with the sensor
uint8_t max30102_read_reg(uint8_t reg) {
    return i2c_read_register(MAX30102_ADDRESS, reg);
}

// max30102_read_fifo - Read multiple bytes from sensor FIFO
// Parameters: buffer (pointer where data is stored), length (number of bytes)
// Uses I2C to read data from MAX30102_FIFODATA
void max30102_read_fifo(uint8_t *buffer, uint8_t length) {
    i2c_read_bytes(MAX30102_ADDRESS, MAX30102_FIFODATA, buffer, length);
}

// max30102_init - Check if sensor is connected and working
// Return: true if part ID is 0x15 (MAX30102 correctly detected), false otherwise
// Reads MAX30102_PARTID register to verify sensor type
bool max30102_init(void) {
    uint8_t partID = max30102_read_reg(MAX30102_PARTID);
    return (partID == 0x15);
}

// max30102_reset - Completely reset the sensor
// Writes MODE_RESET to MODECONFIG register, then waits 100ms for completion
void max30102_reset(void) {
    max30102_write_reg(MAX30102_MODECONFIG, MODE_RESET);
    _delay_ms(100);
}

// max30102_check - Check if sensor is connected and display message
// Uses max30102_init() for verification
// Displays via UART "Sensor OK" or "Sensor not detected"
// If not detected, enters infinite loop to stop execution
void max30102_check(void){
        if (!max30102_init()) {
        uart_puts("Sensor not detected \n");
        while(1);
    }
    else{
        uart_puts("Sensor OK\r\n");
    }
}

// max30102_setup - Configure sensor operating parameters
// Parameters:
//   ledBrightness - LED power (0-255), typical values 0-100
//   sampleAvg - number of averaged samples (SAMPLEAVG_1 to SAMPLEAVG_32)
//   sampleRate - sampling frequency in Hz (SAMPLERATE_50 to SAMPLERATE_3200)
//   pulseWidth - LED pulse width in microseconds (PULSEWIDTH_69 to PULSEWIDTH_411)
//   adcRange - ADC range for measurement (ADCRANGE_2048 to ADCRANGE_16384)
// Steps: reset -> FIFO config -> mode config -> particle config -> LED config -> multi-LED config -> clear FIFO
void max30102_setup(uint8_t ledBrightness, uint8_t sampleAvg, uint8_t sampleRate, uint8_t pulseWidth, uint8_t adcRange) {
    max30102_reset();
    
    max30102_write_reg(MAX30102_FIFOCONFIG, sampleAvg | ROLLOVER_ENABLE);
    
    max30102_write_reg(MAX30102_MODECONFIG, MODE_REDIRONLY);
    
    max30102_write_reg(MAX30102_PARTICLECONFIG, adcRange | sampleRate | pulseWidth);
    
    max30102_write_reg(MAX30102_LED1_PULSEAMP, ledBrightness);
    max30102_write_reg(MAX30102_LED2_PULSEAMP, ledBrightness);
    
    max30102_write_reg(MAX30102_MULTILEDCONFIG1, (SLOT_IR_LED << 4) | SLOT_RED_LED);
    max30102_write_reg(MAX30102_MULTILEDCONFIG2, 0x00);
    
    max30102_clear_fifo();
}

// max30102_clear_fifo - Clear all data from FIFO queue
// Reset pointers: FIFOWRITEPTR, FIFOOVERFLOW, FIFOREADPTR to 0
void max30102_clear_fifo(void) {
    max30102_write_reg(MAX30102_FIFOWRITEPTR, 0x00);
    max30102_write_reg(MAX30102_FIFOOVERFLOW, 0x00);
    max30102_write_reg(MAX30102_FIFOREADPTR, 0x00);
}

// max30102_get_read_ptr - Get FIFO read pointer position
// Return: pointer position (0-31)
// Reads MAX30102_FIFOREADPTR register
uint8_t max30102_get_read_ptr(void) {
    return max30102_read_reg(MAX30102_FIFOREADPTR);
}

// max30102_get_write_ptr - Get FIFO write pointer position
// Return: pointer position (0-31)
// Reads MAX30102_FIFOWRITEPTR register
uint8_t max30102_get_write_ptr(void) {
    return max30102_read_reg(MAX30102_FIFOWRITEPTR);
}

// max30102_read_sample - Read a pair of samples (red and infrared)
// Parameters: red (pointer to red value), ir (pointer to infrared value)
// Reads 6 bytes from FIFO: bytes 0-2 (red), bytes 3-5 (infrared)
// Masked to 18 bits (0x3FFFF) because sensor uses 18 bits
void max30102_read_sample(uint32_t *red, uint32_t *ir) {
    uint8_t buffer[6];
    max30102_read_fifo(buffer, 6);
    
    *red = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    *red &= 0x3FFFF;
    
    *ir = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];
    *ir &= 0x3FFFF;
}

// max30102_get_red - Get only red LED value from FIFO
// Return: red value in 18 bits
// Reads 6 bytes and extracts first 3 for red
uint32_t max30102_get_red(void) {
    uint8_t buffer[6];
    max30102_read_fifo(buffer, 6);
    // Comment
    uint32_t red = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    red &= 0x3FFFF;
    
    return red;
}

// max30102_get_ir - Get only infrared LED value from FIFO
// Return: infrared value in 18 bits
// Reads 6 bytes and extracts last 3 for infrared
uint32_t max30102_get_ir(void) {
    uint8_t buffer[6];
    max30102_read_fifo(buffer, 6);
    
    uint32_t ir = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];
    ir &= 0x3FFFF;
    
    return ir;
}

// End of basic functions
//======================================================================================================================
// Intermediate working functions and calculation functions for desired values

// Timer with increment every 1ms
// Prescaler used: 64 -> 16MHz/64 = 250kHz, 250kHz/250 = 1ms period
// Registers: TCCR1B (CTC mode), OCR1A (period), TIMSK1 (interrupt)
/*
void timer_init(void) {
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
	OCR1A = 249;
	TIMSK1 = (1 << OCIE1A);
	sei();
}

// millis - Returns number of milliseconds since startup
// Return: millisCounter value (counts at each timer interrupt)
unsigned long millis(void) {
	return millisCounter;
}

// ISR(TIMER1_COMPA_vect) - Timer interrupt routine
// Executes every 1ms and increments millisCounter
// This function is automatically called by the microcontroller
ISR(TIMER1_COMPA_vect) {
	millisCounter++;
}
*/

// BPM calculation: beatsPerMinute = 60000ms / interval between beats in ms

// calculateSpO2 - Calculate oxygen saturation (SpO2) from red and infrared buffers
// Uses irBuffer and redBuffer containing SPO2_BUFFER_SIZE samples
// Algorithm: calculates AC/DC ratio for each LED and determines SpO2
// Result stored in global variable SpO2
void calculateSpO2(void) {
    uint32_t irMax = 0, irMin = 999999;
    uint32_t redMax = 0, redMin = 999999;
    unsigned long irSum = 0, redSum = 0;
    
    // Find max, min and sum values for both LEDs
    for (int i = 0; i < SPO2_BUFFER_SIZE; i++) {
        if (irBuffer[i] > irMax) irMax = irBuffer[i];
        if (irBuffer[i] < irMin) irMin = irBuffer[i];
        irSum += irBuffer[i];
        
        if (redBuffer[i] > redMax) redMax = redBuffer[i];
        if (redBuffer[i] < redMin) redMin = redBuffer[i];
        redSum += redBuffer[i];
    }
    
    // Calculate AC and DC components
    float irAC = (float)(irMax - irMin);
    float redAC = (float)(redMax - redMin);
    float irDC = (float)irSum / (float)SPO2_BUFFER_SIZE;
    float redDC = (float)redSum / (float)SPO2_BUFFER_SIZE;
    
    // Calculeaza SpO2 din ratios
    if (irDC != 0 && redDC != 0 && irAC != 0 && irDC > 10000) { // Added check for minimum signal level
        float redRatio = redAC / redDC;
        float irRatio = irAC / irDC;
        
        if (irRatio != 0) {
            float ratio = redRatio / irRatio;
            
            // Standard Maxim Integrated correlation for SpO2
            // 104 - 17R is a linear approximation. 
            // A slightly tuned version for reflection mode often used is:
            SpO2 = 104.0 - 17.0 * ratio;
            
            // Filter out unrealistic jumps
            if (SpO2 > 100) SpO2 = 100;
            if (SpO2 < 60) SpO2 = 0; // Raised floor to 60 to hide noise artifacts
        }
    } else {
        SpO2 = 0;
    }
}

// End of intermediate working functions
    


// resetStats - reseteaza toti buferii si variabilele de calcul
// Goleste: rates[], irBuffer[], redBuffer[], irAvgBuffer[]
// Reseteaza: beatsPerMinute, beatAvg, beatCount, SpO2, etc.
void resetStats(void) {
    beatsPerMinute = 0;
    beatAvg = 0;
    beatCount = 0;
    lastBeatTime = 0;
    rateSpot = 0;
    SpO2 = 0;
    bufferIndex = 0;
    bufferReady = 0;
    
    for (uint8_t x = 0; x < RATE_SIZE; x++) {
        rates[x] = 0;
    }
    for (int i = 0; i < SPO2_BUFFER_SIZE; i++) {
        irBuffer[i] = 0;
        redBuffer[i] = 0;
    }
    for (int i = 0; i < AVG_SIZE; i++) {
        irAvgBuffer[i] = 0;
    }
}

//================================================================================================================================================
// Functia principala - MAX30102_Start
// Aceasta functie porneste senzorul, verifica conexiunea si citeste date intr-o bucla infinita
// Afiseaza BPM si SpO2 pe LCD si trimite datele si prin UART

void MAX30102_Start(void){
    // Verifica daca senzorul este conectat
    max30102_check();
    
    // Configureaza parametrii senzorului:
    // - ledBrightness=40 (puterea LED-ului)
    // - sampleAvg=SAMPLEAVG_4 (medieaza 4 esantioane)
    // - sampleRate=SAMPLERATE_100 (100 Hz = 100 esantioane/secunda)
    // - pulseWidth=PULSEWIDTH_411 (411 microsecunde)
    // - adcRange=ADCRANGE_4096 (interval ADC 4096)
    max30102_setup(0x50, SAMPLEAVG_4, SAMPLERATE_50, PULSEWIDTH_411, ADCRANGE_4096);
    
    // Setare luminozitate LED-uri
    max30102_write_reg(MAX30102_LED1_PULSEAMP, 0x1F);
    max30102_write_reg(MAX30102_LED2_PULSEAMP, 40);
    
    // Initializeaza LCD si afiseaza mesajele initiale
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_WriteString("BPM: --         ");
    LCD_SetCursor(1, 0);
    LCD_WriteString("SpO2: --        ");
    
    _delay_ms(2000);
    uart_puts("Astept deget\r\n");
    resetStats();
    
    // Bucla infinita de citire si procesare
    while(1) {
        // Check for stop request
        if (g_stop_measurement) {
            uart_puts("Stop solicitat, iesire din masurare SPO2\r\n");
            break;
        }
        
        // Verifica daca sunt date noi in FIFO
        uint8_t readPtr = max30102_get_read_ptr();
        uint8_t writePtr = max30102_get_write_ptr();
        
        // Daca pointerii sunt egali, FIFO-ul este gol, asteapta
        if (readPtr == writePtr) {
            _delay_ms(1);
            continue;
        }
        
        // Citeste o pereche de mostre (red si infrarosu)
        uint32_t redValue, irValue32;
        max30102_read_sample(&redValue, &irValue32);
        irValue = (long)irValue32;
        long redVal = (long)redValue;
        
        // Daca semnalul IR este prea mic, probabil degetul nu e pe senzor
        if (irValue < 50000) {
            resetStats();
            _delay_ms(500);
            continue;
        }
        
        // Adauga valoarea IR la buffer pentru mediare
        irAvgBuffer[avgIndex] = irValue;
        avgIndex = (avgIndex + 1) % AVG_SIZE;
        
        // Calculeaza media valorilor IR
        irSmooth = 0;
        for (int i = 0; i < AVG_SIZE; i++) {
            irSmooth += irAvgBuffer[i];
        }
        irSmooth /= AVG_SIZE;
        
        // Calculeaza derivata pentru detactia unui puls/inima
        derivative = irSmooth - lastIR;

        // Detectare puls: derivata trece de la pozitiva la negativa
        if (lastDerivative > 20 && derivative < -20 && irSmooth > 100000) {
            unsigned long currentTime = millis();
            unsigned long timeDiff = currentTime - lastBeatTime;
            
            // Verifica daca intervalul dintre batai e rezonabil (350-2500ms)
            if (timeDiff > 350 && timeDiff < 2500 && lastBeatTime > 0) {
                // Calculeaza BPM: 60000ms / intervalul in ms
                beatsPerMinute = 60000.0 / (float)timeDiff;
                
                // Verifica daca BPM e in intervalul valid (40-180)
                if (beatsPerMinute >= 40 && beatsPerMinute <= 180) {
                    // Adauga la buffer de rate
                    rates[rateSpot++] = (uint8_t)beatsPerMinute;
                    rateSpot %= RATE_SIZE;
                    
                    // Calculeaza media BPM din ultimele RATE_SIZE valori
                    beatAvg = 0;
                    int validBeats = 0;
                    for (uint8_t x = 0; x < RATE_SIZE; x++) {
                        if (rates[x] > 0) {
                            beatAvg += rates[x];
                            validBeats++;
                        }
                    }
                    if (validBeats > 0) {
                        beatAvg /= validBeats;
                    }
                    
                    beatCount++;
                }
            }
            
            lastBeatTime = currentTime;
        }
        
        // Salveaza valorile anterioare pentru urmatoarea iteratie
        lastDerivative = derivative;
        lastIR = irSmooth;
        
        // Adauga mostre in buffere pentru calcul SpO2
        irBuffer[bufferIndex] = irValue32;
        redBuffer[bufferIndex] = redValue;
        bufferIndex++;
        
        // Cand bufferul e plin, marcheaza ca gata pentru calcul
        if (bufferIndex >= SPO2_BUFFER_SIZE) {
            bufferIndex = 0;
            bufferReady = 1;
        }
        
        // Calculeaza SpO2 daca avem suficiente date
        if (bufferReady && beatCount >= 3) {
            calculateSpO2();
        }
        
        // Actualizeaza LCD cu valorile curente
        char bpmBuffer[17];
        char spo2Buffer[17];
        
        // Linia 1 - BPM (16 caractere)
        if (beatAvg > 0) {
            sprintf(bpmBuffer, "BPM: %3d        ", beatAvg);
        } else {
            sprintf(bpmBuffer, "BPM: --         ");
        }
        LCD_SetCursor(0, 0);
        LCD_WriteString(bpmBuffer);
        
        // Linia 2 - SpO2 (16 caractere)
        if (SpO2 >= 70 && SpO2 <= 100) {
            int spo2Int = (int)SpO2;
            int spo2Dec = (int)((SpO2 - spo2Int) * 10);
            sprintf(spo2Buffer, "SpO2: %2d.%1d%%     ", spo2Int, spo2Dec);
        } else {
            sprintf(spo2Buffer, "SpO2: --        ");
        }
        LCD_SetCursor(1, 0);
        LCD_WriteString(spo2Buffer);
        
        // Trimite valorile si prin UART pentru debugging
        uart_puts("BPM:");
        if (beatAvg > 0) {
            uart_putInt(beatAvg);
        } else {
            uart_puts("--");
        }
        uart_puts(" SpO2:");
        if (SpO2 >= 70 && SpO2 <= 100) {
            int spo2Int = (int)SpO2;
            int spo2Dec = (int)((SpO2 - spo2Int) * 10);
            uart_putInt(spo2Int);
            uart_putc('.');
            uart_putInt(spo2Dec);
        } else {
            uart_puts("--");
        }
        uart_puts("%\r\n");
    }
}





