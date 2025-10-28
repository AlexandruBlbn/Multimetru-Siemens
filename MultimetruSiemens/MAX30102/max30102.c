#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "max30102.h"
#include "../Config/i2c.h"
#include "../Config/uart.h"

//init parametrii necesari de functionare a senzorului.
#define RATE_SIZE 4
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

//================================================================================================================================================
//functii de baza

void max30102_write_reg(uint8_t reg, uint8_t value) {
    i2c_write_register(MAX30102_ADDRESS, reg, value);
}

uint8_t max30102_read_reg(uint8_t reg) {
    return i2c_read_register(MAX30102_ADDRESS, reg);
}

void max30102_read_fifo(uint8_t *buffer, uint8_t length) {
    i2c_read_bytes(MAX30102_ADDRESS, MAX30102_FIFODATA, buffer, length);
}

bool max30102_init(void) {
    uint8_t partID = max30102_read_reg(MAX30102_PARTID);
    return (partID == 0x15);
}

void max30102_reset(void) {
    max30102_write_reg(MAX30102_MODECONFIG, MODE_RESET);
    _delay_ms(100);
}

//verifica daca senzorul este conectat si functioneaza corect.
void max30102_check(void){
        if (!max30102_init()) {
        uart_puts("Senzor nedetectat \n");
        while(1);
    }
    else{
        uart_puts("Senzor OK\r\n");
    }
}

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

void max30102_clear_fifo(void) {
    max30102_write_reg(MAX30102_FIFOWRITEPTR, 0x00);
    max30102_write_reg(MAX30102_FIFOOVERFLOW, 0x00);
    max30102_write_reg(MAX30102_FIFOREADPTR, 0x00);
}

uint8_t max30102_get_read_ptr(void) {
    return max30102_read_reg(MAX30102_FIFOREADPTR);
}

uint8_t max30102_get_write_ptr(void) {
    return max30102_read_reg(MAX30102_FIFOWRITEPTR);
}

void max30102_read_sample(uint32_t *red, uint32_t *ir) {
    uint8_t buffer[6];
    max30102_read_fifo(buffer, 6);
    
    *red = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    *red &= 0x3FFFF;
    
    *ir = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];
    *ir &= 0x3FFFF;
}

uint32_t max30102_get_red(void) {
    uint8_t buffer[6];
    max30102_read_fifo(buffer, 6);
    
    uint32_t red = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    red &= 0x3FFFF;
    
    return red;
}

uint32_t max30102_get_ir(void) {
    uint8_t buffer[6];
    max30102_read_fifo(buffer, 6);
    
    uint32_t ir = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];
    ir &= 0x3FFFF;
    
    return ir;
}

//sfarsit functii de baza.
//======================================================================================================================
//Functii de lucru, intermediare(necesare pentru functionarea altora) si calcul a valorilor dorite.


//-----------------------------
//functii de timer cu incrementare la fiecare 1ms.
//prescaler folosit 64 - 16mhz/64 - 250khz, 250khz/250 = 1ms - perioada de 1ms
void timer_init(void) {
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
	OCR1A = 249;
	TIMSK1 = (1 << OCIE1A);
	sei();
}

unsigned long millis(void) {
	return millisCounter;
}

ISR(TIMER1_COMPA_vect) {
	millisCounter++;
}

//bpm = 60.000ms / intervalul dintre batai in ms

//-----------------------------

//Calcul Saturatie O2
void calculateSpO2(void) {
    uint32_t irMax = 0, irMin = 999999;
    uint32_t redMax = 0, redMin = 999999;
    unsigned long irSum = 0, redSum = 0;
    
    for (int i = 0; i < SPO2_BUFFER_SIZE; i++) {
        if (irBuffer[i] > irMax) irMax = irBuffer[i];
        if (irBuffer[i] < irMin) irMin = irBuffer[i];
        irSum += irBuffer[i];
        
        if (redBuffer[i] > redMax) redMax = redBuffer[i];
        if (redBuffer[i] < redMin) redMin = redBuffer[i];
        redSum += redBuffer[i];
    }
    
    float irAC = (float)(irMax - irMin);
    float redAC = (float)(redMax - redMin);
    float irDC = (float)irSum / (float)SPO2_BUFFER_SIZE;
    float redDC = (float)redSum / (float)SPO2_BUFFER_SIZE;
    
    if (irDC != 0 && redDC != 0 && irAC != 0) {
        float redRatio = redAC / redDC;
        float irRatio = irAC / irDC;
        
        if (irRatio != 0) {
            float ratio = redRatio / irRatio;
            SpO2 = 104.0 - 17.0 * ratio;
            
            if (SpO2 > 100) SpO2 = 100;
            if (SpO2 < 70) SpO2 = 0;
        }
    }
}

//-----------------------------
//reset bufferi pentru O2 si BPM
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
//-----------------------------

//Logica implementare functie senzor.

void MAX30102_Start(void){
    max30102_check();
    max30102_setup(40, SAMPLEAVG_4, SAMPLERATE_100, PULSEWIDTH_411, ADCRANGE_4096);
    max30102_write_reg(MAX30102_LED1_PULSEAMP, 0x1F);
    max30102_write_reg(MAX30102_LED2_PULSEAMP, 40);
    _delay_ms(2000);
    uart_puts("Astept deget\r\n");
    resetStats();
    
    while(1) {
        uint8_t readPtr = max30102_get_read_ptr();
        uint8_t writePtr = max30102_get_write_ptr();
        
        if (readPtr == writePtr) {
            _delay_ms(1);
            continue;
        }
        
        uint32_t redValue, irValue32;
        max30102_read_sample(&redValue, &irValue32);
        irValue = (long)irValue32;
        long redVal = (long)redValue;
        
        if (irValue < 50000) {
            resetStats();
            _delay_ms(500);
            continue;
        }
        
        irAvgBuffer[avgIndex] = irValue;
        avgIndex = (avgIndex + 1) % AVG_SIZE;
        
        irSmooth = 0;
        for (int i = 0; i < AVG_SIZE; i++) {
            irSmooth += irAvgBuffer[i];
        }
        irSmooth /= AVG_SIZE;
        
        derivative = irSmooth - lastIR;

        if (lastDerivative > 20 && derivative < -20 && irSmooth > 100000) {
            unsigned long currentTime = millis();
            unsigned long timeDiff = currentTime - lastBeatTime;
            
            if (timeDiff > 350 && timeDiff < 2500 && lastBeatTime > 0) {
                //calcul BPM
                beatsPerMinute = 60000.0 / (float)timeDiff;
                
                if (beatsPerMinute >= 40 && beatsPerMinute <= 180) {
                    rates[rateSpot++] = (uint8_t)beatsPerMinute;
                    rateSpot %= RATE_SIZE;
                    
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
        
        lastDerivative = derivative;
        lastIR = irSmooth;
        
        irBuffer[bufferIndex] = irValue32;
        redBuffer[bufferIndex] = redValue;
        bufferIndex++;
        
        if (bufferIndex >= SPO2_BUFFER_SIZE) {
            bufferIndex = 0;
            bufferReady = 1;
        }
        
        if (bufferReady && beatCount >= 3) {
            calculateSpO2();
        }
        
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




