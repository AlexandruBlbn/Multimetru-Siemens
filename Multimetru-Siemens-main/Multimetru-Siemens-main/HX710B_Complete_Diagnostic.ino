/*
 * HX710B Complete Diagnostic Test
 * Step-by-step verification of sensor functionality
 * 
 * This test will check:
 * 1. Pin configuration
 * 2. DOUT pin state monitoring
 * 3. Basic read attempts
 * 4. Timing verification
 * 
 * Connections:
 * HX710B VCC  -> 3.3V
 * HX710B GND  -> GND
 * HX710B OUT  -> Pin 23 (PA1)
 * HX710B SCK  -> Pin 24 (PA2)
 */

#include "HX710AB.h"

#define HX710B_DOUT 23
#define HX710B_SCK  24

HX710AB sensor(HX710B_DOUT, HX710B_SCK);

void setup() {
  Serial.begin(115200);
  delay(2000);  // Wait for Serial to be ready
  
  Serial.println("\n========================================");
  Serial.println("   HX710B COMPLETE DIAGNOSTIC TEST");
  Serial.println("========================================\n");
  
  Serial.println("This test will verify:");
  Serial.println("- Pin configuration");
  Serial.println("- DOUT pin state");
  Serial.println("- Sensor communication");
  Serial.println("- Timing and protocol\n");
  
  delay(1000);
  
  // TEST 1: Pin Configuration
  Serial.println("=== TEST 1: PIN CONFIGURATION ===");
  pinMode(HX710B_DOUT, INPUT);
  pinMode(HX710B_SCK, OUTPUT);
  digitalWrite(HX710B_SCK, LOW);
  
  Serial.print("DOUT pin (");
  Serial.print(HX710B_DOUT);
  Serial.println("): Configured as INPUT");
  Serial.print("SCK pin (");
  Serial.print(HX710B_SCK);
  Serial.println("): Configured as OUTPUT (LOW)");
  Serial.println("✓ Pin configuration complete\n");
  
  delay(1000);
  
  // TEST 2: DOUT Pin State Monitoring
  Serial.println("=== TEST 2: DOUT PIN STATE MONITORING ===");
  Serial.println("Monitoring DOUT pin for 5 seconds...");
  Serial.println("(Sensor should toggle between HIGH/LOW)\n");
  
  int high_count = 0;
  int low_count = 0;
  int transitions = 0;
  int last_state = digitalRead(HX710B_DOUT);
  
  unsigned long monitor_start = millis();
  while(millis() - monitor_start < 5000) {
    int current_state = digitalRead(HX710B_DOUT);
    
    if(current_state == HIGH) high_count++;
    else low_count++;
    
    if(current_state != last_state) {
      transitions++;
      Serial.print("  [");
      Serial.print(millis() - monitor_start);
      Serial.print(" ms] DOUT changed: ");
      Serial.print(last_state ? "HIGH" : "LOW");
      Serial.print(" -> ");
      Serial.println(current_state ? "HIGH" : "LOW");
    }
    
    last_state = current_state;
    delay(10);
  }
  
  Serial.print("\nResults:\n");
  Serial.print("  HIGH count: ");
  Serial.print(high_count);
  Serial.print(" (");
  Serial.print((high_count * 100) / (high_count + low_count));
  Serial.println("%)");
  Serial.print("  LOW count: ");
  Serial.print(low_count);
  Serial.print(" (");
  Serial.print((low_count * 100) / (high_count + low_count));
  Serial.println("%)");
  Serial.print("  Transitions: ");
  Serial.println(transitions);
  
  Serial.println("\nAnalysis:");
  if(transitions == 0) {
    if(high_count > low_count) {
      Serial.println("  ❌ DOUT STUCK HIGH");
      Serial.println("  Problem: Sensor not powered or defective");
      Serial.println("  Check: VCC=3.3V, GND connected");
    } else {
      Serial.println("  ❌ DOUT STUCK LOW");
      Serial.println("  Problem: Unusual - sensor may be defective");
    }
  } else if(transitions < 5) {
    Serial.println("  ⚠️  FEW TRANSITIONS");
    Serial.println("  Problem: Sensor partially working");
    Serial.println("  Check: Power supply stability");
  } else {
    Serial.println("  ✓ DOUT TOGGLING - Sensor appears active!");
  }
  
  Serial.println();
  delay(2000);
  
  // TEST 3: Library Initialization
  Serial.println("=== TEST 3: LIBRARY INITIALIZATION ===");
  sensor.begin();
  Serial.println("✓ HX710AB library initialized\n");
  
  delay(1000);
  
  // TEST 4: Wait for Ready State
  Serial.println("=== TEST 4: WAITING FOR READY STATE ===");
  Serial.println("Waiting up to 10 seconds for sensor ready...\n");
  
  unsigned long wait_start = millis();
  bool became_ready = false;
  
  while(millis() - wait_start < 10000) {
    if(sensor.is_ready()) {
      Serial.print("✓ Sensor became READY after ");
      Serial.print(millis() - wait_start);
      Serial.println(" ms");
      became_ready = true;
      break;
    }
    
    if((millis() - wait_start) % 1000 == 0) {
      Serial.print("  ");
      Serial.print((millis() - wait_start) / 1000);
      Serial.println(" seconds - still waiting...");
    }
    delay(10);
  }
  
  if(!became_ready) {
    Serial.println("\n❌ Sensor NEVER became ready!");
    Serial.println("\nPOSSIBLE CAUSES:");
    Serial.println("1. Wrong pins - check DOUT=23, SCK=24");
    Serial.println("2. Sensor not powered - check VCC=3.3V");
    Serial.println("3. Defective sensor");
    Serial.println("4. DOUT/SCK swapped");
    Serial.println("\nTEST FAILED - Cannot continue");
    while(1) delay(1000);  // Stop here
  }
  
  Serial.println();
  delay(1000);
  
  // TEST 5: Read Attempts
  Serial.println("=== TEST 5: SENSOR READ ATTEMPTS ===");
  Serial.println("Attempting 10 reads...\n");
  
  int successful_reads = 0;
  long first_value = 0;
  long min_value = 999999999;
  long max_value = -999999999;
  
  for(int i = 0; i < 10; i++) {
    Serial.print("Read #");
    Serial.print(i+1);
    Serial.print(": ");
    
    if(sensor.is_ready()) {
      long value = sensor.read();
      
      if(i == 0) first_value = value;
      if(value < min_value) min_value = value;
      if(value > max_value) max_value = value;
      
      Serial.print("SUCCESS - Raw: ");
      Serial.print(value);
      Serial.print(" (0x");
      Serial.print(value, HEX);
      Serial.println(")");
      
      successful_reads++;
    } else {
      Serial.println("FAILED - Sensor not ready");
    }
    
    delay(200);
  }
  
  Serial.print("\nSuccessful reads: ");
  Serial.print(successful_reads);
  Serial.println("/10");
  
  if(successful_reads > 0) {
    Serial.print("Value range: ");
    Serial.print(min_value);
    Serial.print(" to ");
    Serial.println(max_value);
    Serial.print("Variation: ");
    Serial.println(max_value - min_value);
    
    if(max_value - min_value < 100) {
      Serial.println("\n✓ Values stable - sensor working correctly!");
    } else {
      Serial.println("\n⚠️  Large variation - sensor may be noisy");
    }
  }
  
  Serial.println("\n========================================");
  Serial.println("        DIAGNOSTIC COMPLETE");
  Serial.println("========================================\n");
  
  if(successful_reads >= 8) {
    Serial.println("✓✓✓ SENSOR IS WORKING! ✓✓✓");
    Serial.println("\nYou can now proceed with normal operation.");
    Serial.println("Starting continuous monitoring in 3 seconds...\n");
    delay(3000);
  } else {
    Serial.println("❌ SENSOR NOT WORKING PROPERLY");
    Serial.println("\nPlease check connections and try again.");
    while(1) delay(1000);
  }
}

void loop() {
  // Continuous monitoring mode
  static int reading_number = 0;
  
  if(sensor.is_ready()) {
    reading_number++;
    long raw = sensor.read();
    
    Serial.print("#");
    Serial.print(reading_number);
    Serial.print(": Raw=");
    Serial.print(raw);
    
    // Show approximate pressure (using 263k factor)
    float kpa = abs((float)(raw - 8388607)) / 263000.0;
    Serial.print("\t~");
    Serial.print(kpa, 2);
    Serial.println(" kPa");
  } else {
    Serial.println("Waiting for sensor...");
  }
  
  delay(500);
}
