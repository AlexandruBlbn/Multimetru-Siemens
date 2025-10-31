#ifndef MAX30102_H_
#define MAX30102_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX30102_ADDRESS    0x57

// Registri Status - pagina 14-15 din datasheet MAX30102
// Acesti registri indica starea senzorului si intreruperile
#define MAX30102_INTSTAT1        0x00
#define MAX30102_INTSTAT2        0x01
#define MAX30102_INTENABLE1      0x02
#define MAX30102_INTENABLE2      0x03

// Registri FIFO (First In First Out) - pagina 16-17 din datasheet
// FIFO este o coada de date pentru citirea mostrei senzorului
#define MAX30102_FIFOWRITEPTR    0x04
#define MAX30102_FIFOOVERFLOW    0x05
#define MAX30102_FIFOREADPTR     0x06
#define MAX30102_FIFODATA        0x07

// Registri Configurare - pagina 18-20 din datasheet
// Acesti registri configureaza modul de functionare al senzorului
#define MAX30102_FIFOCONFIG      0x08
#define MAX30102_MODECONFIG      0x09
#define MAX30102_PARTICLECONFIG  0x0A
#define MAX30102_LED1_PULSEAMP   0x0C
#define MAX30102_LED2_PULSEAMP   0x0D
#define MAX30102_MULTILEDCONFIG1 0x11
#define MAX30102_MULTILEDCONFIG2 0x12

// Registri ID - pagina 21 din datasheet
// Acesti registri contin informatii despre versiunea hardware/software
#define MAX30102_REVISIONID      0xFE
#define MAX30102_PARTID          0xFF

// Registru biti - FIFO Config (pagina 18 din datasheet)
// SAMPLEAVG - numarul de mostre care sunt mediate
#define SAMPLEAVG_1      0x00
#define SAMPLEAVG_2      0x20
#define SAMPLEAVG_4      0x40
#define SAMPLEAVG_8      0x60
#define SAMPLEAVG_16     0x80
#define SAMPLEAVG_32     0xA0

// ROLLOVER - comportamentul cand FIFO este plin
#define ROLLOVER_ENABLE  0x10
#define ROLLOVER_DISABLE 0x00

// Masti si Valori - Mode Config (pagina 19 din datasheet)
// MODE - modul de operare al senzorului
#define MODE_SHUTDOWN    0x80
#define MODE_RESET       0x40
#define MODE_REDONLY     0x02
#define MODE_REDIRONLY   0x03

// Masti si Valori - Particle Config (pagina 20 din datasheet - SpO2)
// ADCRANGE - intervalul de masurare al ADC
#define ADCRANGE_2048    0x00
#define ADCRANGE_4096    0x20
#define ADCRANGE_8192    0x40
#define ADCRANGE_16384   0x60

// SAMPLERATE - frecventa de esantionare (Hz)
#define SAMPLERATE_50    0x00
#define SAMPLERATE_100   0x04
#define SAMPLERATE_200   0x08
#define SAMPLERATE_400   0x0C
#define SAMPLERATE_800   0x10
#define SAMPLERATE_1000  0x14
#define SAMPLERATE_1600  0x18
#define SAMPLERATE_3200  0x1C

// PULSEWIDTH - latimea pulsului LED in microsecunde
#define PULSEWIDTH_69    0x00
#define PULSEWIDTH_118   0x01
#define PULSEWIDTH_215   0x02
#define PULSEWIDTH_411   0x03

// Multi-LED Slots (pagina 21 din datasheet)
// Tipurile de LED-uri care pot fi configurate
#define SLOT_NONE        0x00
#define SLOT_RED_LED     0x01
#define SLOT_IR_LED      0x02

// Functii
// max30102_write_reg - scrie valoare in registrul specificat
// Parametri: reg (adresa registrului), value (valoarea de scris)
void max30102_write_reg(uint8_t reg, uint8_t value);

// max30102_read_reg - citeste valoare din registrul specificat
// Parametri: reg (adresa registrului)
// Returnare: valoarea din registru (uint8_t)
uint8_t max30102_read_reg(uint8_t reg);

// max30102_read_fifo - citeste date din FIFO
// Parametri: buffer (pointer la buffer unde se stocheaza datele), length (numarul de bytes)
void max30102_read_fifo(uint8_t *buffer, uint8_t length);

// max30102_init - initializeaza si verifica senzorul
// Returnare: true daca senzorul este gasit si functioneaza, false altfel
bool max30102_init(void);

// max30102_reset - reseteaza complet senzorul
// Nu are parametri, Nu returneaza nimic
void max30102_reset(void);

// max30102_setup - configureaza parametrii de functionare ai senzorului
// Parametri: 
//   ledBrightness - luminozitatea LED-ului (0-255)
//   sampleAvg - medierea mostrei (SAMPLEAVG_1 pana la SAMPLEAVG_32)
//   sampleRate - frecventa de esantionare (SAMPLERATE_50 pana la SAMPLERATE_3200)
//   pulseWidth - latimea pulsului LED (PULSEWIDTH_69 pana la PULSEWIDTH_411)
//   adcRange - intervalul ADC (ADCRANGE_2048 pana la ADCRANGE_16384)
void max30102_setup(uint8_t ledBrightness, uint8_t sampleAvg, uint8_t sampleRate, uint8_t pulseWidth, uint8_t adcRange);

// max30102_clear_fifo - sterge toate datele din FIFO
// Nu are parametri, Nu returneaza nimic
void max30102_clear_fifo(void);

// max30102_read_sample - citeste o pereche de mostre (red si infrarosu)
// Parametri: red (pointer la valoare red), ir (pointer la valoare infrarosu)
void max30102_read_sample(uint32_t *red, uint32_t *ir);

// max30102_get_red - obtine doar valoarea red din FIFO
// Returnare: valoarea red (uint32_t)
uint32_t max30102_get_red(void);

// max30102_get_ir - obtine doar valoarea infrarosu din FIFO
// Returnare: valoarea infrarosu (uint32_t)
uint32_t max30102_get_ir(void);

// max30102_get_read_ptr - obtine pointerul de citire FIFO
// Returnare: pozitia pointerului de citire (uint8_t)
uint8_t max30102_get_read_ptr(void);

// max30102_get_write_ptr - obtine pointerul de scriere FIFO
// Returnare: pozitia pointerului de scriere (uint8_t)
uint8_t max30102_get_write_ptr(void);

// millis - returneaza numarul de milisecunde de la pornire
// Returnare: timp in milisecunde (unsigned long)
unsigned long millis(void);

// timer_init - initializeaza timerul pentru millis()
// Nu are parametri, Nu returneaza nimic
void timer_init(void);

// MAX30102_Start - functia principala care porneste senzorul si citeste date
// Aceasta functie contine o bucla infinita de citire si afisare date pe LCD
// Nu are parametri, Nu returneaza nimic (bucla infinita)
void MAX30102_Start(void);

#endif /* MAX30102_H_ */