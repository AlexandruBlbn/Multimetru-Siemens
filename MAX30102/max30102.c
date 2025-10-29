#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "max30102.h"
#include "../Config/i2c.h"
#include "../Config/uart.h"
#include "../LCD/LCD.h"
#include <stdio.h>


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
// Functii de baza - Comunicare cu senzorul MAX30102 via I2C

// max30102_write_reg - scrie o valoare intr-un registru al senzorului
// Parametri: reg (adresa registrului), value (valoarea de scris pe 8 biti)
// Foloseste I2C pentru a comunica cu senzorul la adresa MAX30102_ADDRESS
void max30102_write_reg(uint8_t reg, uint8_t value) {
    i2c_write_register(MAX30102_ADDRESS, reg, value);
}

// max30102_read_reg - citeste o valoare dintr-un registru al senzorului
// Parametri: reg (adresa registrului)
// Returnare: valoarea din registru pe 8 biti
// Foloseste I2C pentru a comunica cu senzorul
uint8_t max30102_read_reg(uint8_t reg) {
    return i2c_read_register(MAX30102_ADDRESS, reg);
}

// max30102_read_fifo - citeste mai multi bytes din FIFO-ul senzorului
// Parametri: buffer (pointer unde se stocheaza datele), length (numarul de bytes)
// Foloseste I2C pentru a citi datele din MAX30102_FIFODATA
void max30102_read_fifo(uint8_t *buffer, uint8_t length) {
    i2c_read_bytes(MAX30102_ADDRESS, MAX30102_FIFODATA, buffer, length);
}

// max30102_init - verifica daca senzorul este conectat si functioneaza
// Returnare: true daca ID-ul partii este 0x15 (MAX30102 corect detectat), false altfel
// Citeste registrul MAX30102_PARTID pentru a verifica tipul senzorului
bool max30102_init(void) {
    uint8_t partID = max30102_read_reg(MAX30102_PARTID);
    return (partID == 0x15);
}

// max30102_reset - reseteaza complet senzorul
// Scrie MODE_RESET in registrul MODECONFIG, apoi asteapta 100ms pentru finalizare
void max30102_reset(void) {
    max30102_write_reg(MAX30102_MODECONFIG, MODE_RESET);
    _delay_ms(100);
}

// max30102_check - verifica daca senzorul este conectat si afiseaza mesaj
// Foloseste max30102_init() pentru verificare
// Afiseaza prin UART "Senzor OK" sau "Senzor nedetectat"
// Daca nu e detectat, intra in bucla infinita pentru a opri executia
void max30102_check(void){
        if (!max30102_init()) {
        uart_puts("Senzor nedetectat \n");
        while(1);
    }
    else{
        uart_puts("Senzor OK\r\n");
    }
}

// max30102_setup - configureaza parametrii de functionare ai senzorului
// Parametri:
//   ledBrightness - puterea LED-ului (0-255), valori tipice 0-100
//   sampleAvg - numarul de mostre mediate (SAMPLEAVG_1 pana SAMPLEAVG_32)
//   sampleRate - frecventa de esantionare in Hz (SAMPLERATE_50 pana SAMPLERATE_3200)
//   pulseWidth - latimea pulsului LED in microsecunde (PULSEWIDTH_69 pana PULSEWIDTH_411)
//   adcRange - intervalul ADC pentru masurare (ADCRANGE_2048 pana ADCRANGE_16384)
// Etape: reset -> FIFO config -> mode config -> particle config -> LED config -> multi-LED config -> clear FIFO
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

// max30102_clear_fifo - sterge toate datele din coada FIFO
// Reseteaza pointerii: FIFOWRITEPTR, FIFOOVERFLOW, FIFOREADPTR la 0
void max30102_clear_fifo(void) {
    max30102_write_reg(MAX30102_FIFOWRITEPTR, 0x00);
    max30102_write_reg(MAX30102_FIFOOVERFLOW, 0x00);
    max30102_write_reg(MAX30102_FIFOREADPTR, 0x00);
}

// max30102_get_read_ptr - obtine pozitia pointerului de citire din FIFO
// Returnare: pozitia pointerului (0-31)
// Citeste registrul MAX30102_FIFOREADPTR
uint8_t max30102_get_read_ptr(void) {
    return max30102_read_reg(MAX30102_FIFOREADPTR);
}

// max30102_get_write_ptr - obtine pozitia pointerului de scriere in FIFO
// Returnare: pozitia pointerului (0-31)
// Citeste registrul MAX30102_FIFOWRITEPTR
uint8_t max30102_get_write_ptr(void) {
    return max30102_read_reg(MAX30102_FIFOWRITEPTR);
}

// max30102_read_sample - citeste o pereche de mostre (red si infrarosu)
// Parametri: red (pointer la valoare red), ir (pointer la valoare infrarosu)
// Citeste 6 bytes din FIFO: bytes 0-2 (red), bytes 3-5 (infrarosu)
// Mascare la 18 biti (0x3FFFF) deoarece senzorul foloseste 18 biti
void max30102_read_sample(uint32_t *red, uint32_t *ir) {
    uint8_t buffer[6];
    max30102_read_fifo(buffer, 6);
    
    *red = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    *red &= 0x3FFFF;
    
    *ir = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];
    *ir &= 0x3FFFF;
}

// max30102_get_red - obtine doar valoarea LED-ului rosu din FIFO
// Returnare: valoare red pe 18 biti
// Citeste 6 bytes si extrage primii 3 pentru red
uint32_t max30102_get_red(void) {
    uint8_t buffer[6];
    max30102_read_fifo(buffer, 6);
    //Comentariu
    uint32_t red = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    red &= 0x3FFFF;
    
    return red;
}

// max30102_get_ir - obtine doar valoarea LED-ului infrarosu din FIFO
// Returnare: valoare infrarosu pe 18 biti
// Citeste 6 bytes si extrage ultimi 3 pentru infrarosu
uint32_t max30102_get_ir(void) {
    uint8_t buffer[6];
    max30102_read_fifo(buffer, 6);
    
    uint32_t ir = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];
    ir &= 0x3FFFF;
    
    return ir;
}

//sfarsit functii de baza.
//======================================================================================================================
// Functii de lucru intermediare si functii de calcul pentru valorile dorite

// Timer cu incrementare la fiecare 1ms
// Prescaler folosit: 64 -> 16MHz/64 = 250kHz, 250kHz/250 = 1ms perioada
// Registri: TCCR1B (mod CTC), OCR1A (perioada), TIMSK1 (intrerupere)
void timer_init(void) {
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
	OCR1A = 249;
	TIMSK1 = (1 << OCIE1A);
	sei();
}

// millis - returneaza numarul de milisecunde de la pornire
// Returnare: valoarea millisCounter (numara la fiecare intrerupere a timerului)
unsigned long millis(void) {
	return millisCounter;
}

// ISR(TIMER1_COMPA_vect) - rutina de intrerupere a timerului
// Se executa la fiecare 1ms si incrementeaza millisCounter
// Aceasta functie este apelata automat de microcontroler
ISR(TIMER1_COMPA_vect) {
	millisCounter++;
}

// Calcul BPM: beatsPerMinute = 60000ms / intervalul dintre batai in ms

// calculateSpO2 - calculeaza saturatia de oxigen (SpO2) din bufferele red si infrarosu
// Foloseste bufferele irBuffer si redBuffer care contin SPO2_BUFFER_SIZE mostre
// Algoritm: calculeaza AC/DC ratio pentru fiecare LED si determina SpO2
// Rezultat stocat in variabila globala SpO2
void calculateSpO2(void) {
    uint32_t irMax = 0, irMin = 999999;
    uint32_t redMax = 0, redMin = 999999;
    unsigned long irSum = 0, redSum = 0;
    
    // Gaseste valori max, min si suma pentru ambele LED-uri
    for (int i = 0; i < SPO2_BUFFER_SIZE; i++) {
        if (irBuffer[i] > irMax) irMax = irBuffer[i];
        if (irBuffer[i] < irMin) irMin = irBuffer[i];
        irSum += irBuffer[i];
        
        if (redBuffer[i] > redMax) redMax = redBuffer[i];
        if (redBuffer[i] < redMin) redMin = redBuffer[i];
        redSum += redBuffer[i];
    }
    
    // Calculeaza componente AC si DC
    float irAC = (float)(irMax - irMin);
    float redAC = (float)(redMax - redMin);
    float irDC = (float)irSum / (float)SPO2_BUFFER_SIZE;
    float redDC = (float)redSum / (float)SPO2_BUFFER_SIZE;
    
    // Calculeaza SpO2 din ratios
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
    // - sampleAvg=SAMPLEAVG_4 (medieaza 4 mostre)
    // - sampleRate=SAMPLERATE_100 (100 Hz = 100 mostre/secunda)
    // - pulseWidth=PULSEWIDTH_411 (411 microsecunde)
    // - adcRange=ADCRANGE_4096 (interval ADC 4096)
    max30102_setup(40, SAMPLEAVG_4, SAMPLERATE_100, PULSEWIDTH_411, ADCRANGE_4096);
    
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





