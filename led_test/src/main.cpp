#include <Arduino.h>

// Mapped from Schematic Pin 21
#define TEST_PIN 29 

void setup() {
  Serial.begin(115200);
  pinMode(TEST_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Force the pin to 3.3V
  digitalWrite(TEST_PIN, HIGH);
  
  // Built-in LED on to confirm the Teensy hasn't crashed
  digitalWrite(LED_BUILTIN, HIGH); 
  
  Serial.println("Pin 29 is now forced HIGH (3.3V).");
  Serial.println("Measure between Hole 21 and GND.");
}

void loop() {
  // Maintaining state
  delay(1000);
}