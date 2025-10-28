#ifndef MAX30102_H_
#define MAX30102_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX30102_ADDRESS    0x57

// Registri Status
#define MAX30102_INTSTAT1        0x00
#define MAX30102_INTSTAT2        0x01
#define MAX30102_INTENABLE1      0x02
#define MAX30102_INTENABLE2      0x03

// Registri FIFO
#define MAX30102_FIFOWRITEPTR    0x04
#define MAX30102_FIFOOVERFLOW    0x05
#define MAX30102_FIFOREADPTR     0x06
#define MAX30102_FIFODATA        0x07

// Registri Configurare
#define MAX30102_FIFOCONFIG      0x08
#define MAX30102_MODECONFIG      0x09
#define MAX30102_PARTICLECONFIG  0x0A
#define MAX30102_LED1_PULSEAMP   0x0C
#define MAX30102_LED2_PULSEAMP   0x0D
#define MAX30102_MULTILEDCONFIG1 0x11
#define MAX30102_MULTILEDCONFIG2 0x12

// Registri ID
#define MAX30102_REVISIONID      0xFE
#define MAX30102_PARTID          0xFF

// Masti si Valori - FIFO Config
#define SAMPLEAVG_1      0x00
#define SAMPLEAVG_2      0x20
#define SAMPLEAVG_4      0x40
#define SAMPLEAVG_8      0x60
#define SAMPLEAVG_16     0x80
#define SAMPLEAVG_32     0xA0

#define ROLLOVER_ENABLE  0x10
#define ROLLOVER_DISABLE 0x00

// Masti si Valori - Mode Config
#define MODE_SHUTDOWN    0x80
#define MODE_RESET       0x40
#define MODE_REDONLY     0x02
#define MODE_REDIRONLY   0x03

// Masti si Valori - Particle Config (SpO2)
#define ADCRANGE_2048    0x00
#define ADCRANGE_4096    0x20
#define ADCRANGE_8192    0x40
#define ADCRANGE_16384   0x60

#define SAMPLERATE_50    0x00
#define SAMPLERATE_100   0x04
#define SAMPLERATE_200   0x08
#define SAMPLERATE_400   0x0C
#define SAMPLERATE_800   0x10
#define SAMPLERATE_1000  0x14
#define SAMPLERATE_1600  0x18
#define SAMPLERATE_3200  0x1C

#define PULSEWIDTH_69    0x00
#define PULSEWIDTH_118   0x01
#define PULSEWIDTH_215   0x02
#define PULSEWIDTH_411   0x03

// Multi-LED Slots
#define SLOT_NONE        0x00
#define SLOT_RED_LED     0x01
#define SLOT_IR_LED      0x02

// Functii
void max30102_write_reg(uint8_t reg, uint8_t value);
uint8_t max30102_read_reg(uint8_t reg);
void max30102_read_fifo(uint8_t *buffer, uint8_t length);

bool max30102_init(void);
void max30102_reset(void);
void max30102_setup(uint8_t ledBrightness, uint8_t sampleAvg, uint8_t sampleRate, uint8_t pulseWidth, uint8_t adcRange);
void max30102_clear_fifo(void);

void max30102_read_sample(uint32_t *red, uint32_t *ir);
uint32_t max30102_get_red(void);
uint32_t max30102_get_ir(void);
uint8_t max30102_get_read_ptr(void);
uint8_t max30102_get_write_ptr(void);

unsigned long millis(void);
void timer_init(void);
void MAX30102_Start(void);

#endif /* MAX30102_H_ */