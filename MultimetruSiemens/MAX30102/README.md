# Documentația MAX30102 Driver

## Funcții de Bază

### `void max30102_write_reg(uint8_t reg, uint8_t value)`
**Descriere:** Scrie o valoare într-un registru al senzorului MAX30102.  
**Parametri:**
- `reg` (uint8_t) - Adresa registrului
- `value` (uint8_t) - Valoarea de scris

**Output:** Niciuna (void)

---

### `uint8_t max30102_read_reg(uint8_t reg)`
**Descriere:** Citește valoarea dintr-un registru al senzorului MAX30102.  
**Parametri:**
- `reg` (uint8_t) - Adresa registrului

**Output:** 
- `uint8_t` - Valoarea din registru

---

### `void max30102_read_fifo(uint8_t *buffer, uint8_t length)`
**Descriere:** Citește date brute din FIFO-ul senzorului.  
**Parametri:**
- `*buffer` (uint8_t pointer) - Buffer unde se stochează datele
- `length` (uint8_t) - Numărul de bytes de citit

**Output:** Niciuna (void)

---

### `bool max30102_init(void)`
**Descriere:** Verifică dacă senzorul este conectat citind ID-ul piesei (Part ID).  
**Parametri:** Niciuno

**Output:**
- `bool` - `true` dacă Part ID este 0x15 (senzor valid), `false` altfel

---

### `void max30102_reset(void)`
**Descriere:** Resetează senzorul prin setarea bitului de reset în registrul MODECONFIG. Se așteptă 100ms pentru stabilizare.  
**Parametri:** Niciuno

**Output:** Niciuna (void)

---

### `void max30102_check(void)`
**Descriere:** Verifică dacă senzorul este detectat. Dacă nu, trimite mesaj pe UART și blocheaza execuția (infinite loop).  
**Parametri:** Niciuno

**Output:** Niciuna (void) - Sau hang dacă senzor nu detectat

---

## Funcții de Configurare

### `void max30102_setup(uint8_t ledBrightness, uint8_t sampleAvg, uint8_t sampleRate, uint8_t pulseWidth, uint8_t adcRange)`
**Descriere:** Configurează senzorul cu parametrii optimi pentru măsurarea pulsului și SpO2.  
**Configurare:**
- Resetează senzorul
- Setează parametrii FIFO (sample averaging, rollover enable)
- Setează modul: RED + IR LED
- Configură gama ADC, rata de eșantionare, lățimea impulsului LED
- Setează intensitatea LED-urilor
- Goleșteaza FIFO-ul

**Parametri:**
- `ledBrightness` (uint8_t) - Intensitate LED (0-255, tipic 40)
- `sampleAvg` (uint8_t) - Media eșantioanelor FIFO (1, 2, 4, 8, 16, 32)
- `sampleRate` (uint8_t) - Rata de eșantionare (50, 100, 200, 400, 800, 1000 Hz)
- `pulseWidth` (uint8_t) - Lățimea impulsului LED (69, 118, 215, 411 μs)
- `adcRange` (uint8_t) - Gama ADC (2048, 4096, 8192, 16384)

**Output:** Niciuna (void)

---

### `void max30102_clear_fifo(void)`
**Descriere:** Goleșteaza buffer-ul FIFO prin resetarea pointerilor de citire și scriere.  
**Configurare:**
- Write pointer = 0x00
- Overflow counter = 0x00
- Read pointer = 0x00

**Parametri:** Niciuno

**Output:** Niciuna (void)

---

## Funcții de Citire FIFO

### `uint8_t max30102_get_read_ptr(void)`
**Descriere:** Obține poziția curentă a pointerului de citire din FIFO.  
**Parametri:** Niciuno

**Output:**
- `uint8_t` - Valoarea pointerului de citire (0-31)

---

### `uint8_t max30102_get_write_ptr(void)`
**Descriere:** Obține poziția curentă a pointerului de scriere în FIFO.  
**Parametri:** Niciuno

**Output:**
- `uint8_t` - Valoarea pointerului de scriere (0-31)

---

### `void max30102_read_sample(uint32_t *red, uint32_t *ir)`
**Descriere:** Citește simultan o mostră de 6 bytes din FIFO și le procesează în două valori de 18-bit pentru LED-ul roșu și infraroșu.  
**Parametri:**
- `*red` (uint32_t pointer) - Pointer unde se stochează valoarea LED roșu
- `*ir` (uint32_t pointer) - Pointer unde se stochează valoarea LED infraroșu

**Output:** Niciuna (void) - Valorile sunt stocate în pointeri

---

### `uint32_t max30102_get_red(void)`
**Descriere:** Citește și returnează doar valoarea LED-ului roșu din FIFO (18-bit).  
**Parametri:** Niciuno

**Output:**
- `uint32_t` - Valoarea LED roșu (0-262143)

---

### `uint32_t max30102_get_ir(void)`
**Descriere:** Citește și returnează doar valoarea LED-ului infraroșu din FIFO (18-bit).  
**Parametri:** Niciuno

**Output:**
- `uint32_t` - Valoarea LED infraroșu (0-262143)

---

## Funcții de Timer

### `void timer_init(void)`
**Descriere:** Inițializează Timer1 în modul CTC (Clear Timer on Compare) pentru a genera o întrerupere la fiecare 1ms.  
**Configurare:**
- Mode: CTC (WGM12=1)
- Prescaler: 64 (CS11=1, CS10=1)
- Frecvență CPU: 16 MHz → 250 kHz după prescaler
- Compare value: 249 → 250kHz / 250 = 1 kHz = 1ms
- Enableaza întreruperi globale (sei)

**Parametri:** Niciuno

**Output:** Niciuna (void)

---

### `unsigned long millis(void)`
**Descriere:** Returnează numărul de milisecunde scurse de la inițializarea timerului.  
**Parametri:** Niciuno

**Output:**
- `unsigned long` - Milisecunde scurse

---

### `ISR(TIMER1_COMPA_vect)`
**Descriere:** Rutina de întrerupere pentru Timer1. Se execută la fiecare 1ms și incrementează contorul de milisecunde.  
**Acțiune:** `millisCounter++`

---

## Funcții de Procesare Semnal

### `void calculateSpO2(void)`
**Descriere:** Calculează saturația de oxigen (SpO2) pe baza valorilor LED-urilor roșu și infraroșu din buffer (100 eșantioane).  
**Algoritm:**
1. Găsește max și min pentru IR și RED din buffer
2. Calculează componenta AC (max - min) și DC (medie) pentru fiecare
3. Calculează raportul: `(RED_AC / RED_DC) / (IR_AC / IR_DC)`
4. Aplică formula: `SpO2 = 104 - 17 × ratio`
5. Limitează rezultatul între 70-100%

**Parametri:** Niciuno

**Output:** Niciuna (void) - Stochează rezultatul în variabila globală `SpO2`

---

### `void resetStats(void)`
**Descriere:** Resetează toate variabilele și bufferele pentru măsurări noi (BPM, SpO2).  
**Resetează:**
- Contoare BPM: `beatsPerMinute`, `beatAvg`, `beatCount`, `lastBeatTime`
- Contoare SpO2: `SpO2`, buffer index-uri
- Vectori: `rates[]`, `irBuffer[]`, `redBuffer[]`, `irAvgBuffer[]`

**Parametri:** Niciuno

**Output:** Niciuna (void)

---

## Funcția Principală

### `void MAX30102_Start(void)`
**Descriere:** Funcția principală care inițializează senzorul și execută bucla infinită de măsurare, calcul BPM/SpO2 și transmisie pe UART.  

**Inițializare:**
1. Verifică detecția senzorului (`max30102_check`)
2. Configurează senzorul (100 Hz, 4096 ADC range, 411 μs pulse width, 4x averaging)
3. Setează intensitate LED: RED=31mA (0x1F), IR=40mA
4. Așteaptă 2 secunde și resetează statisticile

**Bucla principală (infinite):**
1. Citește pointerul FIFO (citire/scriere)
2. Dacă FIFO nu are date, așteptă 1ms și reîncepe
3. Citește o mostră (RED și IR)
4. Dacă IR < 50000 (nu e deget pe senzor), resetează și continuă
5. Aplică filtru de mediere mobilă pe IR (5 eșantioane)
6. Calculează derivata pentru detecția vârfului
7. **Detecție BPM:** Dacă derivata schimbă semn (vârf detectat):
   - Calculează intervalul de timp dintre bătăi (ms)
   - Validează interval: 350-2500ms (40-180 BPM)
   - Calculează BPM: `60000 / interval`
   - Stochează în circular buffer de 4 valori și face media
8. Stochează IR și RED în bufferele SpO2 (100 eșantioane)
9. După 100 eșantioane și minimum 3 bătăi detectate, calculează SpO2
10. **Transmisie UART:** Format: `BPM:XX SpO2:YY.Y%`
    - BPM: Valoarea medie sau "--" dacă nu valid
    - SpO2: Valoare cu o zecimală (70-100%) sau "--"

**Parametri:** Niciuno

**Output:** Niciuna (void) - Execuție infinită cu output pe UART

---

## Variabile Globale

| Variabilă | Tip | Descriere |
|-----------|-----|-----------|
| `rates[]` | uint8_t[4] | Buffer circular cu ultimele 4 valori BPM |
| `beatsPerMinute` | float | BPM calculat din intervalul ultimei bătăi |
| `beatAvg` | int | Media ultimelor 4 valori BPM |
| `beatCount` | int | Numărul total de bătăi detectate |
| `lastBeatTime` | unsigned long | Timestamp-ul ultimei bătăi (ms) |
| `irValue` | long | Valoarea curentă IR din FIFO |
| `irSmooth` | long | Valoarea IR după filtru de mediere |
| `derivative` | long | Derivata IR (irSmooth - lastIR) |
| `irBuffer[]` | uint32_t[100] | Buffer cu ultimele 100 valori IR pentru SpO2 |
| `redBuffer[]` | uint32_t[100] | Buffer cu ultimele 100 valori RED pentru SpO2 |
| `SpO2` | float | Valoarea saturației de oxigen (%) |
| `millisCounter` | volatile unsigned long | Contor milisecunde (incrementat de ISR) |

---

## Parametrii de Configurare Implicite (din `MAX30102_Start`)

- **Rata de eșantionare:** 100 Hz (10ms între eșantioane)
- **Gama ADC:** 4096 (18-bit resolution)
- **Lățimea impulsului LED:** 411 μs (cea mai mare, pentru semnal mai bun)
- **Media FIFO:** 4 eșantioane consecutive
- **Intensitate LED RED:** 0x1F (31mA)
- **Intensitate LED IR:** 40mA (0x28)

---

## Fluxul de Execuție

```
main.c → MAX30102_Start()
         ↓
    Inițializare senzor
         ↓
    Loop infinit:
         ↓
    Citire FIFO (6 bytes RED+IR)
         ↓
    Filtru mediere (5 eșantioane)
         ↓
    Detecție vârf (derivata)
    ├─→ BPM: interval timp între vârfuri
    └─→ SpO2: max/min pe 100 eșantioane
         ↓
    Output UART: "BPM:XX SpO2:YY.Y%"
         ↓
    (repeat)
```

