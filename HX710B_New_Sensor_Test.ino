// --- Definim pinii ---
const int DOUT_PIN = 22; 
const int SCK_PIN = 24;  

// --- CALIBRARE pentru senzor 0-40 kPa ---
// Calibrare la 2 puncte: 0 kPa și 40 kPa (capacitate maximă senzor)
const long VALOARE_ZERO = 300800;        // Citire la 0 kPa (fără presiune)
const long VALOARE_40KPA = 8388607;      // Citire la 40 kPa (presiune maximă)
const float PRESIUNE_MAX_SENZOR = 40.0;  // Capacitate maximă: 40 kPa = 300 mmHg
// --- SFÂRȘIT CALIBRARE ---

// NOTĂ: Pentru tensiometru, 40 kPa (300 mmHg) este suficient pentru:
// - Tensiune normală: 120/80 mmHg (16/10.7 kPa)
// - Tensiune ridicată: 160/100 mmHg (21.3/13.3 kPa)
// - Presiune maximă manșetă: 200-250 mmHg (26.7-33.3 kPa)

// --- SETĂRI PENTRU DETECTARE PULS ---
const int MARIME_FILTRU = 8;             // Mărime filtru mediu glisant
const float PRAG_OSCILATIE = 0.2;        // Prag minim pentru detectare puls (kPa) - ajustat pentru 40kPa
const unsigned long TIMP_MINIM_PULS = 400; // Timp minim între pulsuri (ms) - maxim 150 BPM
const float PRESIUNE_OPTIMA_MASURARE = 16.0; // ~120 mmHg - presiune optimă pentru detectare puls

// ===================================================================
// === MOD DE OPERARE ===
// 0 = Serial Monitor (text complet cu detectare puls)
// 1 = Serial Plotter (grafic simplu presiune)
// 2 = Serial Plotter Advanced (presiune + oscilații + puls)
// ===================================================================
const int MOD_OPERARE = 2;
// ===================================================================

// Variabile pentru calibrare liniară cu 2 puncte
const float FACTOR_SCALARE = 40.0 / (float)(VALOARE_40KPA - VALOARE_ZERO);

// Variabile pentru filtrare și detectare puls
float istoricPresiune[MARIME_FILTRU] = {0};
int indexIstoric = 0;
bool filtruInitializat = false;

float presiuneAnterioara = 0;
float oscilatieAnterioara = 0;
unsigned long timpUltimPuls = 0;
int contorPuls = 0;
unsigned long timpStartMasurare = 0;
float presiuneMaxOscilatie = 0;
float presiuneLaMaxOscilatie = 0;

void setup() {
  Serial.begin(115200);
  pinMode(DOUT_PIN, INPUT);
  pinMode(SCK_PIN, OUTPUT);
  
  timpStartMasurare = millis();
  
  // Afișăm setările doar în modul Monitor
  if (MOD_OPERARE == 0) {
    Serial.println("=== TENSIOMETRU - Senzor HX710B (0-40 kPa) ===");
    Serial.print("Factor de scalare: ");
    Serial.println(FACTOR_SCALARE, 10);
    Serial.println("Calibrare: 0 kPa si 40 kPa (300 mmHg max)");
    Serial.println("Prag detectare puls: " + String(PRAG_OSCILATIE) + " kPa");
    Serial.println("Presiune optima masurare: " + String(PRESIUNE_OPTIMA_MASURARE) + " kPa (" + String(PRESIUNE_OPTIMA_MASURARE * 7.5, 0) + " mmHg)");
    Serial.println("==============================================");
    Serial.println();
  } else if (MOD_OPERARE == 2) {
    // Etichetele pentru Serial Plotter Advanced
    Serial.println("Presiune:, Oscilatie:, Puls:");
  }
}

void loop() {
  // Facem media a 10 citiri și eliminăm valorile extreme pentru precizie maximă
  const int numar_citiri = 10;
  long citiri[numar_citiri];
  
  // Colectăm citirile
  for (int i = 0; i < numar_citiri; i++) {
    citiri[i] = citesteSenzor();
    delay(5); // Pauză mai mică pentru citiri mai rapide
  }
  
  // Sortăm citirile pentru a elimina valorile extreme
  for (int i = 0; i < numar_citiri - 1; i++) {
    for (int j = i + 1; j < numar_citiri; j++) {
      if (citiri[i] > citiri[j]) {
        long temp = citiri[i];
        citiri[i] = citiri[j];
        citiri[j] = temp;
      }
    }
  }
  
  // Eliminăm cele 2 valori cele mai mici și cele 2 cele mai mari
  // și facem media valorilor rămase (metoda trimmed mean)
  long suma_citiri = 0;
  for (int i = 2; i < numar_citiri - 2; i++) {
    suma_citiri += citiri[i];
  }
  long valoare_bruta = suma_citiri / (numar_citiri - 4);

  // Aplicăm calibrarea
  long valoare_tarata = valoare_bruta - VALOARE_ZERO;
  float presiune_bruta = valoare_tarata * FACTOR_SCALARE;
  
  // Aplicăm filtru mediu glisant pentru o citire mai stabilă
  float presiune_kpa = aplicaFiltruMediuGlisant(presiune_bruta);
  
  // Detectăm oscilațiile (variații) pentru identificarea pulsului
  float oscilatie = detecteazaOscilatie(presiune_kpa);
  
  // Detectăm pulsul bazat pe oscilații
  bool pulsDetectat = detecteazaPuls(oscilatie);
  
  // Verificăm dacă suntem în zona optimă de măsurare
  static bool inZonaOptima = false;
  if (presiune_kpa >= 10.0 && presiune_kpa <= 20.0) { // 75-150 mmHg
    if (!inZonaOptima && MOD_OPERARE == 0) {
      Serial.println(">>> IN ZONA OPTIMA DE MASURARE <<<");
      inZonaOptima = true;
    }
  } else {
    inZonaOptima = false;
  }
  
  // Calculăm BPM și estimăm tensiunea
  static unsigned long timpAfisareBPM = 0;
  if (millis() - timpAfisareBPM > 5000) { // La fiecare 5 secunde
    timpAfisareBPM = millis();
    if (contorPuls > 0 && MOD_OPERARE == 0) {
      float bpm = (contorPuls * 60000.0) / (millis() - timpStartMasurare);
      Serial.print("BPM: ");
      Serial.print(bpm, 1);
      Serial.print(" | Sistolica estimata: ");
      Serial.print(presiuneLaMaxOscilatie * 7.5, 1); // Conversie kPa -> mmHg
      Serial.print(" mmHg | Presiune curenta: ");
      Serial.print(presiune_kpa * 7.5, 1);
      Serial.println(" mmHg");
    }
  }
  
  // Afișare în funcție de modul ales
  afiseazaDatele(presiune_kpa, oscilatie, pulsDetectat);
}

// --- Funcția de citire (Varianta CORECTATĂ) ---
long citesteSenzor() {
  while (digitalRead(DOUT_PIN)) {}

  long result = 0;
  for (int i = 0; i < 24; i++) {
    digitalWrite(SCK_PIN, HIGH);
    digitalWrite(SCK_PIN, LOW);
    result = result << 1;
    if (digitalRead(DOUT_PIN)) {
      result++;
    }
  }

  // Corectarea complementului față de 2 (Sign Extension)
  if (result & 0x800000) {
    result |= 0xFF000000; 
  }

  // 3 pulsuri pentru următoarea citire
  for (char i = 0; i < 3; i++) {
    digitalWrite(SCK_PIN, HIGH);
    digitalWrite(SCK_PIN, LOW);
  }

  return result;
}

// --- FUNCȚII PENTRU PROCESARE SEMNAL ---

// Filtru mediu glisant pentru reducerea zgomotului
float aplicaFiltruMediuGlisant(float valoare_noua) {
  istoricPresiune[indexIstoric] = valoare_noua;
  indexIstoric = (indexIstoric + 1) % MARIME_FILTRU;
  
  // Dacă filtrul nu e complet, returnăm valoarea directă
  if (!filtruInitializat) {
    if (indexIstoric == 0) filtruInitializat = true;
    return valoare_noua;
  }
  
  // Calculăm media
  float suma = 0;
  for (int i = 0; i < MARIME_FILTRU; i++) {
    suma += istoricPresiune[i];
  }
  return suma / MARIME_FILTRU;
}

// Detectează oscilațiile (variațiile) în presiune
float detecteazaOscilatie(float presiune_curenta) {
  float oscilatie = abs(presiune_curenta - presiuneAnterioara);
  presiuneAnterioara = presiune_curenta;
  
  // Aplicăm un filtru simplu și pe oscilație
  float oscilatie_filtrata = (oscilatie + oscilatieAnterioara) / 2.0;
  oscilatieAnterioara = oscilatie_filtrata;
  
  return oscilatie_filtrata;
}

// Detectează pulsul bazat pe oscilații
bool detecteazaPuls(float oscilatie) {
  unsigned long timpCurent = millis();
  bool pulsDetectat = false;
  
  // Detectăm un puls când oscilația depășește pragul
  // și a trecut suficient timp de la ultimul puls
  if (oscilatie > PRAG_OSCILATIE && 
      (timpCurent - timpUltimPuls) > TIMP_MINIM_PULS) {
    
    pulsDetectat = true;
    timpUltimPuls = timpCurent;
    contorPuls++;
    
    // Salvăm presiunea la oscilația maximă (probabil sistolică)
    if (oscilatie > presiuneMaxOscilatie) {
      presiuneMaxOscilatie = oscilatie;
      presiuneLaMaxOscilatie = presiuneAnterioara;
    }
  }
  
  return pulsDetectat;
}

// Afișează datele în funcție de modul ales
void afiseazaDatele(float presiune, float oscilatie, bool puls) {
  if (MOD_OPERARE == 0) {
    // Modul Monitor - Text complet
    Serial.print("Presiune: ");
    Serial.print(presiune, 2);
    Serial.print(" kPa (");
    Serial.print(presiune * 7.5, 1); // Conversie în mmHg
    Serial.print(" mmHg)");
    
    // Afișăm procentajul din capacitatea maximă
    float procent = (presiune / PRESIUNE_MAX_SENZOR) * 100.0;
    Serial.print(" [");
    Serial.print(procent, 0);
    Serial.print("%] | Oscilatie: ");
    Serial.print(oscilatie, 3);
    Serial.print(" kPa");
    if (puls) {
      Serial.print(" | ♥ PULS!");
    }
    Serial.println();
    
  } else if (MOD_OPERARE == 1) {
    // Modul Plotter simplu - Doar presiunea
    Serial.println(presiune, 2);
    
  } else if (MOD_OPERARE == 2) {
    // Modul Plotter avansat - Presiune + Oscilație + Marcaj puls
    Serial.print(presiune, 2);
    Serial.print(",");
    Serial.print(oscilatie * 50, 2); // Amplificăm oscilația pentru vizibilitate
    Serial.print(",");
    Serial.println(puls ? presiune : 0, 2); // Marcaj vizual când e puls
  }
}