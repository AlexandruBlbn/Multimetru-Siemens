// #define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../inc/HX710B.h"
#include "../Config/uart.h"
#include "../inc/LCD.h"

extern volatile int menu_option;

//methodology:
//COde implements a oscilometric method. The HX710B detects pressure fluctuations
// caused by arterial pulsations during cuff deflation.
//HOw it is calculated:
// It is used the Heuristic approach and the systolic pressure is calculated as the pressure
// at which the maximum oscillation occurs, while the diastolic pressure is estimated
// as a percentage of the systolic pressure (typically around 67%). 



//================================================================================================================================================
// Internal state for oscillometric calculation
static float hx710b_history[HX710B_FILTRU_MARIME] = {0};
static uint8_t hx710b_history_index = 0;
static bool hx710b_history_full = false;
static float hx710b_prev_pressure = 0.0f;
static float hx710b_prev_osc = 0.0f;
static float hx710b_max_osc = 0.0f;
static float hx710b_pressure_at_max = 0.0f;
static uint32_t hx710b_last_pulse_time = 0;
static uint32_t hx710b_first_pulse_time = 0;
static uint8_t hx710b_pulse_count = 0;
static float hx710b_latest_pressure = 0.0f;
static float hx710b_latest_osc = 0.0f;
static bool hx710b_latest_pulse = false;
static float hx710b_bpm = 0.0f;
static float hx710b_moving_avg_pressure = 0.0f;
static uint16_t hx710b_sample_counter = 0;


// Forward declarations
static float HX710B_ApplyFilter(float new_value);
static float HX710B_DetectOscillation(float pressure_kpa);
static void HX710B_PumpControl(bool enabled);

//================================================================================================================================================
// Basic functions - Communication with HX710B pressure sensor

// HX710B_Init - Initialize pins for HX710B communication
// Configures DOUT as input and SCK as output, sets SCK to LOW
void HX710B_Init(void) {
    // Configure DOUT as input (no pull-up)
    HX710B_DOUT_DDR &= ~(1 << HX710B_DOUT_BIT);
    
    // Configure SCK as output and set LOW
    HX710B_SCK_DDR |= (1 << HX710B_SCK_BIT);
    HX710B_SCK_PORT &= ~(1 << HX710B_SCK_BIT);
    
    // Configure pump pin as output (default OFF)
    HX710B_PUMP_DDR |= (1 << HX710B_PUMP_PIN);
    HX710B_PUMP_PORT &= ~(1 << HX710B_PUMP_PIN);
    
    _delay_ms(100); // Wait for sensor to stabilize
    
    // Reset sensor by sending 50+ clock pulses
    for (uint8_t i = 0; i < 50; i++) {
        HX710B_SCK_PORT |= (1 << HX710B_SCK_BIT);
        _delay_us(1);
        HX710B_SCK_PORT &= ~(1 << HX710B_SCK_BIT);
        _delay_us(1);
    }
    
    _delay_ms(100); // Wait for reset to complete
    HX710B_ResetAlgorithm();
}

// HX710B_CitesteSenzor - Read 24-bit raw value from sensor
// Return: Signed 24-bit value (two's complement)
// Protocol: Wait for DOUT to go LOW, then clock in 24 bits + 3 extra pulses (Channel A, gain 128)
int32_t HX710B_CitesteSenzor(void) {
    uint32_t timeout = 100000;
    while ((HX710B_DOUT_PIN & (1 << HX710B_DOUT_BIT)) && timeout > 0) {
        _delay_us(1);
        timeout--;
    }
    if (timeout == 0) {
        return 0; // Timeout - sensor not responding
    }

    int32_t result = 0;

    // Read 24 bits using the same pattern as the Arduino reference
    for (uint8_t i = 0; i < 24; i++) {
        HX710B_SCK_PORT |= (1 << HX710B_SCK_BIT);
        _delay_us(1);
        HX710B_SCK_PORT &= ~(1 << HX710B_SCK_BIT);
        _delay_us(1);

        result = result << 1;
        if (HX710B_DOUT_PIN & (1 << HX710B_DOUT_BIT)) {
            result++;
        }
    }

    // Two's complement sign extension
    if (result & 0x800000) {
        result |= 0xFF000000;
    }

    // Three extra pulses to keep using Channel A, gain 128
    for (uint8_t i = 0; i < 3; i++) {
        HX710B_SCK_PORT |= (1 << HX710B_SCK_BIT);
        _delay_us(1);
        HX710B_SCK_PORT &= ~(1 << HX710B_SCK_BIT);
        _delay_us(1);
    }

    return result;
}

// HX710B_CitestePresiune - Read pressure in kPa
// Return: Pressure in kPa (float)
// Takes multiple readings, removes outliers, applies calibration
float HX710B_CitestePresiune(void) {
    int32_t citiri[HX710B_NUMAR_CITIRI];
    uint8_t sample_count = 0;

    // Collect valid readings (skip timeouts)
    while (sample_count < HX710B_NUMAR_CITIRI) {
        int32_t sample = HX710B_CitesteSenzor();
        if (sample == 0) {
            continue; // retry on timeout
        }

        citiri[sample_count++] = sample;
        _delay_ms(HX710B_DELAY_CITIRE);
    }
    
    // Sort readings (simple Bubble sort)
    for (uint8_t i = 0; i < HX710B_NUMAR_CITIRI - 1; i++) {
        for (uint8_t j = i + 1; j < HX710B_NUMAR_CITIRI; j++) {
            if (citiri[i] > citiri[j]) {
                int32_t temp = citiri[i];
                citiri[i] = citiri[j];
                citiri[j] = temp;
            }
        }
    }
    
    // Average middle values (remove 2 min and 2 max)
    int32_t suma_citiri = 0;
    for (uint8_t i = 2; i < HX710B_NUMAR_CITIRI - 2; i++) {
        suma_citiri += citiri[i];
    }
    int32_t valoare_medie = suma_citiri / (HX710B_NUMAR_CITIRI - 4);
    
    // Apply linear calibration
    // Formula: Pressure = (Value - Zero) * (Max_Pressure / (Max_Value - Zero))
    int32_t valoare_tarata = valoare_medie - HX710B_VALOARE_ZERO;
    float factor_scalare = HX710B_PRESIUNE_MAX / (float)(HX710B_VALOARE_40KPA - HX710B_VALOARE_ZERO);
    float presiune_kpa = valoare_tarata * factor_scalare;
    
    // Limit pressure to valid range
    if (presiune_kpa < 0) presiune_kpa = 0;
    if (presiune_kpa > HX710B_PRESIUNE_MAX) presiune_kpa = HX710B_PRESIUNE_MAX;
    
    return presiune_kpa;
}

// HX710B_Check - Check if sensor is connected and display message
// Return: true if sensor responds, false otherwise
// Uses HX710B_CitesteSenzor() for verification
bool HX710B_Check(void) {
    int32_t test_value = HX710B_CitesteSenzor();

    char debug_buffer[50];
    sprintf(debug_buffer, "HX710B raw value: %ld\r\n", test_value);
    uart_puts(debug_buffer);

    if (test_value == 0) {
        uart_puts("HX710B pressure sensor - NOT detected (timeout)\r\n");
        return false;
    }

    uart_puts("HX710B pressure sensor - OK\r\n");
    return true;
}

//================================================================================================================================================
// Pump control helper
static void HX710B_PumpControl(bool enabled) {
    if (enabled) {
        HX710B_PUMP_PORT |= (1 << HX710B_PUMP_PIN);
    } else {
        HX710B_PUMP_PORT &= ~(1 << HX710B_PUMP_PIN);
    }
}

//================================================================================================================================================
// Oscillometric helper functions
void HX710B_ResetAlgorithm(void) {
    for (uint8_t i = 0; i < HX710B_FILTRU_MARIME; i++) {
        hx710b_history[i] = 0.0f;
    }
    hx710b_history_index = 0;
    hx710b_history_full = false;
    hx710b_prev_pressure = 0.0f;
    hx710b_prev_osc = 0.0f;
    hx710b_max_osc = 0.0f;
    hx710b_pressure_at_max = 0.0f;
    hx710b_last_pulse_time = 0;
    hx710b_first_pulse_time = 0;
    hx710b_pulse_count = 0;
    hx710b_latest_pressure = 0.0f;
    hx710b_latest_osc = 0.0f;
    hx710b_latest_pulse = false;
    hx710b_bpm = 0.0f;
    hx710b_moving_avg_pressure = 0.0f;
    hx710b_sample_counter = 0;
}

static float HX710B_ApplyFilter(float new_value) {
    hx710b_history[hx710b_history_index] = new_value;
    hx710b_history_index = (hx710b_history_index + 1) % HX710B_FILTRU_MARIME;
    if (!hx710b_history_full && hx710b_history_index == 0) {
        hx710b_history_full = true;
    }

    if (!hx710b_history_full) {
        return new_value;
    }

    float sum = 0.0f;
    for (uint8_t i = 0; i < HX710B_FILTRU_MARIME; i++) {
        sum += hx710b_history[i];
    }
    return sum / HX710B_FILTRU_MARIME;
}


static float HX710B_DetectOscillation(float pressure_kpa) {
    if (hx710b_moving_avg_pressure == 0.0f) {
        hx710b_moving_avg_pressure = pressure_kpa;
        return 0.0f;
    }
    // MODIFICAT: Alpha 0.25 pentru urmarire si mai rapida a pantei
    hx710b_moving_avg_pressure = (hx710b_moving_avg_pressure * 0.75f) + (pressure_kpa * 0.25f);
    
    float raw_oscillation = fabsf(pressure_kpa - hx710b_moving_avg_pressure);
    float osc_filtered = (raw_oscillation * 0.4f) + (hx710b_prev_osc * 0.6f);
    hx710b_prev_osc = osc_filtered;
    hx710b_prev_pressure = pressure_kpa;
    
    return osc_filtered;
}


static void HX710B_ProcessSample(float pressure_kpa, uint32_t time_ms) {
    float filtered = HX710B_ApplyFilter(pressure_kpa);
    
    hx710b_sample_counter++;

    // MODIFICAT: Redus la 5 esantioane
    // Deoarece avem delay-ul de 2 secunde inainte de start, nu mai e nevoie sa ignoram mult aici
    if (hx710b_sample_counter < 5) {
        if (hx710b_moving_avg_pressure == 0.0f) hx710b_moving_avg_pressure = filtered;
        else hx710b_moving_avg_pressure = (hx710b_moving_avg_pressure * 0.75f) + (filtered * 0.25f);
        return; 
    }

    float oscillation = HX710B_DetectOscillation(filtered);
    hx710b_latest_pressure = filtered;
    hx710b_latest_osc = oscillation;
 
    // MODIFICAT: Limita relaxata la 24.0 kPa (approx 180 mmHg MAP)
    // Acum ca presiunea tinta e mai mica (26kPa), riscul de zgomot extrem e redus
    if (oscillation > 0.05f && pressure_kpa > 5.3f && pressure_kpa < 24.0f) {
        // Daca oscilatia curenta e mai mare decat maximul gasit pana acum
        if (oscillation > hx710b_max_osc) {
            hx710b_max_osc = oscillation;
            hx710b_pressure_at_max = pressure_kpa; 
        }
    }
}

static void HX710B_FinalizeResult(HX710B_BloodPressureData *result) {
    if (!result) {
        return;
    }

    result->presiune_curenta_kpa = hx710b_latest_pressure;
    result->presiune_curenta_mmhg = hx710b_latest_pressure * 7.50062f; 
    result->oscilatie_kpa = hx710b_latest_osc;
    result->puls_detectat = hx710b_latest_pulse;
    result->bpm = hx710b_bpm;
    float map_mmhg = hx710b_pressure_at_max * 7.50062f;

    if (map_mmhg > 0) {
        // MODIFICAT: Coeficienti echilibrati
        // Sistolica = MAP * 1.30
        // Diastolica = MAP * 0.80
        result->sistolica_mmhg = map_mmhg * 1.30f; 
        result->diastolica_mmhg = map_mmhg * 0.80f;
        
        if (result->sistolica_mmhg - result->diastolica_mmhg < 25) {
            result->sistolica_mmhg = result->diastolica_mmhg + 25;
        }
    } else {

        result->sistolica_mmhg = result->presiune_curenta_mmhg;
        result->diastolica_mmhg = result->presiune_curenta_mmhg * 0.7f;
    }
}

bool HX710B_StartMeasurement(HX710B_BloodPressureData *result) {
    if (!result) {
        return false;
    }

    uint32_t simulated_time = 0;
    float pressure = 0.0f;
    LCD_Clear();
    LCD_SetCursor(0,0);
    LCD_WriteString("Umflare");
    LCD_SetCursor(1,0);
    LCD_WriteString("Nu te misca");

    // Asiguram ca senzorul e treaz si stabil
    HX710B_CitesteSenzor();
    _delay_ms(10);
    HX710B_ResetAlgorithm();

    // Inflate quickly to the target, monitoring only for safety to avoid sensor saturation
    HX710B_PumpControl(true);
    bool reached_target = false;
    for (uint16_t i = 0; i < HX710B_INFLATE_MAX_LOOPS; i++) {
        pressure = HX710B_CitestePresiune();
        char debug_buffer[40];
        char pressure_str[16];
        dtostrf(pressure, 6, 2, pressure_str);
        snprintf(debug_buffer, sizeof(debug_buffer), "Inflate: %s kPa\r\n", pressure_str);
        uart_puts(debug_buffer);
        simulated_time += HX710B_SAMPLE_INTERVAL_MS;

        if (pressure >= HX710B_INFLATE_SAFETY_KPA) {
            reached_target = true;
            break;
        }

        if (pressure >= HX710B_INFLATE_TARGET_KPA) {
            reached_target = true;
            break;
        }
    }
    HX710B_PumpControl(false);

    if (!reached_target) {
        uart_puts("eroare");
    }

    // Allow cuff to stabilize before sampling oscillations during deflation
    _delay_ms(HX710B_POST_INFLATE_SETTLE_MS);

    HX710B_ResetAlgorithm();
    simulated_time = 0;

    // Measurement phase while deflating (pump off, oscillometric window)
    for (uint16_t i = 0; i < HX710B_DEF_SAMPLE_COUNT; i++) {
        pressure = HX710B_CitestePresiune();
        HX710B_ProcessSample(pressure, simulated_time);
        simulated_time += HX710B_SAMPLE_INTERVAL_MS;

        if (pressure <= HX710B_RELEASE_PRESSURE_KPA) {
            break;
        }
    }

    HX710B_FinalizeResult(result);

    return (result->sistolica_mmhg > 0.0f);
}

void HX710B_Start(void) {
    HX710B_BloodPressureData bp_data;
    char value_buffer[16];
    char sist_buffer[24];
    char dias_buffer[24];
    char bpm_buffer[24];

    bool measurement_ok = HX710B_StartMeasurement(&bp_data);

    LCD_Clear();
    LCD_SetCursor(0, 0);
    if (measurement_ok) {
        dtostrf(bp_data.sistolica_mmhg, 6, 1, value_buffer);
        snprintf(sist_buffer, sizeof(sist_buffer), "Sist:%smmHg", value_buffer);
        LCD_WriteString(sist_buffer);

        LCD_SetCursor(1, 0);
        dtostrf(bp_data.diastolica_mmhg, 6, 1, value_buffer);
        snprintf(dias_buffer, sizeof(dias_buffer), "Dias:%smmHg", value_buffer);
        LCD_WriteString(dias_buffer);

        snprintf(bpm_buffer, sizeof(bpm_buffer), "BPM: %.1f", bp_data.bpm);

    } else {
        LCD_WriteString("Masurare esuata");
        LCD_SetCursor(1, 0);
        LCD_WriteString("Reincercare...  ");
        uart_puts("HX710B: masurare esuata\r\n");
    }
    menu_option = 2;
    while (menu_option == 2) {
        _delay_ms(100);
    }
    _delay_ms(500);
}
