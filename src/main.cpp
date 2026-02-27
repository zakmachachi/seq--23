// Minimal main for SimpleSequencer
#include <Arduino.h>
#include "SimpleSequencer.h"

SimpleSequencer seq;

void setup(){
  seq.begin();
}

void loop(){
  seq.loop();
  // small delay to keep CPU usage low
  delay(1);
}
