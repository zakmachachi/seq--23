#include "SimpleSequencer.h"
#include <IntervalTimer.h>

// Background Hardware Timer for flawless MIDI clock
static IntervalTimer midiClockTimer;
static volatile bool midiTimerRunning = false;

// Engine timer (1ms) to decouple MIDI processing from UI drawing
static IntervalTimer engineTimer;
static volatile bool stepAdvanceRequested = false; // set by internalClockTick

// forward wrapper so ISR stays tiny
static void internalClockTickWrapper();

void sendClockISR() {
  // ISR must be as tiny as possible: emit MIDI Clock and advance internal tick counter
  Serial8.write(0xF8);
  internalClockTickWrapper();
}
// MIDI clock timing (24 PPQN)
static uint32_t lastMidiClockMicros = 0;

// external MIDI clock state
static bool externalMidiClockActive = false;
static uint32_t lastExternalClockMillis = 0;
static uint8_t midiStepTickCounter = 0; // counts MIDI clock ticks toward a 16th (6 ticks)

// No special auto-channel mapping: send notes on per-track channels by default

// Absolute Timestamp Window for Bulletproof BPM
#define BPM_TICK_WINDOW 49 // 49 timestamps = exactly 48 gaps (2 full beats)
static uint32_t tickTimestamps[BPM_TICK_WINDOW];
static uint8_t tickIndex = 0;
static uint8_t validTicks = 0;
static float smoothedBpm = 120.0f;



// Note length in exact MIDI Clock Ticks (96 = Whole, 48 = Half, 24 = Quarter, 12 = Eighth, 6 = Sixteenth)
static const uint8_t noteLenTicks[] = { 96, 48, 24, 12, 6 };
static const char* noteLenNames[] = { "1", "1/2", "1/4", "1/8", "1/16" };

// Division printable names
static const char* divisionNames[] = { "Whole", "Half", "Quarter", "Eighth", "Sixteenth" };

static float getDivisionFactor(SimpleSequencer::Division d){
  switch(d){
    case SimpleSequencer::DIV_WHOLE: return 4.0f;
    case SimpleSequencer::DIV_HALF: return 2.0f;
    case SimpleSequencer::DIV_QUARTER: return 1.0f;
    case SimpleSequencer::DIV_EIGHTH: return 0.5f;
    case SimpleSequencer::DIV_SIXTEENTH: return 0.25f;
  }
  return 0.25f;
}

SimpleSequencer::SimpleSequencer()
  : bpm(200), lastStepMillis(0), currentStep(0), selectedChannel(0)
{
  // Default base pitch per channel

  for (uint8_t c=0;c<NUM_CHANNELS;c++){
    pulses[c]=4;
    euclidOffset[c] = 0;
    retrig[c]=1;
    euclidEnabled[c]=false;
    muted[c]=false; // <-- All channels start unmuted
    noteOffTick[c]=0;
    for (uint8_t s=0; s<NUM_STEPS; s++) fillStep[c][s] = false;
    for(uint8_t s=0;s<NUM_STEPS;s++){
      steps[c][s]=false;
      euclidPattern[c][s]=false;
      // --- THE FIX: 255 means "Use Global Pitch" ---
      pitch[c][s] = 255;
      // --- THE FIX: 255 means "Use Global Length" ---
      noteLen[c][s] = 255;
      // ratchet default: off
      stepRatchet[c][s] = 0;
      pendingToggle[s] = false;
    }
    channelPitch[c] = 36; // Default each channel's base pitch to C2
    // ratchet engine defaults
    ratchetIntervalTicks[c] = 0;
    lastNotePlaying[c] = 255;
  }
  lastMidiClockMicros = 0;
  noteLenIdx = 4; // default to 1/16 (use shorter gate to avoid envelope collisions)
  absoluteTickCounter = 0;
}

// define per-button ISR forwarders (attachInterrupt requires a no-arg function)
static void isr_btn_0() { if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(0); }
static void isr_btn_1() { if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(1); }
static void isr_btn_2() { if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(2); }
static void isr_btn_3() { if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(3); }
static void isr_btn_4() { if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(4); }
static void isr_btn_5() { if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(5); }
static void isr_btn_6() { if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(6); }
static void isr_btn_7() { if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(7); }
static void isr_btn_8() { if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(8); }
static void isr_btn_9() { if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(9); }
static void isr_btn_10(){ if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(10); }
static void isr_btn_11(){ if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(11); }
static void isr_btn_12(){ if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(12); }
static void isr_btn_13(){ if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(13); }
static void isr_btn_14(){ if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(14); }
static void isr_btn_15(){ if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->handleButtonIRQ(15); }

static void (* const isr_table[NUM_STEPS])() = {
  isr_btn_0, isr_btn_1, isr_btn_2, isr_btn_3,
  isr_btn_4, isr_btn_5, isr_btn_6, isr_btn_7,
  isr_btn_8, isr_btn_9, isr_btn_10, isr_btn_11,
  isr_btn_12, isr_btn_13, isr_btn_14, isr_btn_15
};

void SimpleSequencer::begin(){
  // set instance pointer for ISRs
  SimpleSequencer::instancePtr = this;
  setupPins();
  // euclidean functionality removed for simplified MIDI test
  lastStepMillis = millis();
  analogWriteResolution(12); // use full resolution where supported
  Serial.begin(115200);
  // init display
  Wire.begin();
  Wire.setClock(400000); // speed up I2C to 400kHz to reduce OLED blocking time
  // quick I2C scan to help debug wiring/address
  Serial.println("Scanning I2C bus...");
  bool any = false;
  // simple I2C scan
  for (uint8_t addr = 1; addr < 127; addr++){
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0){
      Serial.print("I2C device found at 0x"); Serial.println(addr, HEX);
      any = true;
    }
  }
  if (!any) Serial.println("No I2C devices found");

  // initialize display
  display.begin(0x3C);
  displayTest();

  // Initialize hardware Serial8 for MIDI at 31250 baud
  Serial8.begin(31250);
  // initialize high-resolution clock reference for internal MIDI output
  lastMidiClockMicros = micros();

  // MIDI clock timing handled by global `lastMidiClockMicros`

  // start the 1ms engine timer which will process MIDI RX, note-offs and step advancement
  engineTimer.begin([](){ if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->runEngine(); }, 1000);

  // attempt to auto-load saved state from EEPROM
  loadState();

}

void SimpleSequencer::midiSendByte(uint8_t b){
  // Use hardware Serial8 for MIDI output (31250 baud)
  Serial8.write(b);
}

void SimpleSequencer::midiSendNoteOn(uint8_t channel, uint8_t note, uint8_t vel){
  uint8_t status = 0x90 | (channel & 0x0F);
  midiSendByte(status);
  midiSendByte(note & 0x7F);
  midiSendByte(vel & 0x7F);
}

void SimpleSequencer::midiSendNoteOff(uint8_t channel, uint8_t note, uint8_t vel){
  // Some Elektron devices expect Note-Offs as Note-On with velocity 0.
  // Send a Note-On (0x90) with velocity 0 to be compatible.
  uint8_t status = 0x90 | (channel & 0x0F);
  midiSendByte(status);
  midiSendByte(note & 0x7F);
  midiSendByte(0);
}

void SimpleSequencer::setupPins(){
  // buttons use interrupt-driven handlers
  for (uint8_t i=0;i<NUM_STEPS;i++){
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }
  // channel modifier button (hold + step 1-4 to quick-select channel)
  pinMode(CHANNEL_BTN_PIN, INPUT_PULLUP);
  // encoder pins
  for (uint8_t e=0;e<4;e++){
    pinMode(ENC_A[e], INPUT_PULLUP);
    pinMode(ENC_B[e], INPUT_PULLUP);
    pinMode(ENC_SW[e], INPUT_PULLUP);
  }
  // start/stop button
  pinMode(START_STOP_PIN, INPUT_PULLUP);
  // MIDI RX handled by hardware Serial8; no manual pin/interrupt here
  // CV/Gate pins removed; using MIDI only
  // debug LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  debugLedFlag = false;
  debugLedOffTime = 0;
}

// static instance pointer for ISR forwarding
SimpleSequencer* SimpleSequencer::instancePtr = nullptr;

void SimpleSequencer::handleButtonIRQ(uint8_t idx){
  // (no-op when using Bounce2 polling). Kept for backward compatibility.
}

void SimpleSequencer::loop(){
  // --- TRACK THE FILL PERFORMANCE BUTTON (Now on Pin 28) ---
  fillModeActive = (digitalRead(CHANNEL_BTN_PIN) == LOW);

  // UI-only loop: read controls and update display. Time-critical MIDI work runs in engine timer.
  readButtons();
  readEncoders();
  // handle start/stop button debounce (Arduino-style)
  unsigned long now = millis();

  bool startReading = (digitalRead(START_STOP_PIN) == LOW);
  if (startReading != startLastReading){
    startLastDebounceTime = now;
  }
  if ((now - startLastDebounceTime) > debounceMs){
    if (startReading != startState){
      startState = startReading;
      if (startState){
        // PRESSED: Reset the modifier flag
        startStopModifierFlag = false;
      } else {
        // RELEASED: Only toggle transport if we DID NOT use it to mute a track
        if (!startStopModifierFlag) {
          // toggle running state
          isRunning = !isRunning;
          if (isRunning){
            midiStepTickCounter = 0;
            stepAdvanceRequested = false;
            // reset absolute tick counter so internal timing/ratchets start aligned
            absoluteTickCounter = 0;
            midiSendByte(0xFA); // MIDI Start
            midiSendByte(0xF8); // MIDI Clock
            currentStep = 0;
            for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
              bool isActive = euclidEnabled[ch] ? euclidPattern[ch][currentStep] : steps[ch][currentStep];
              if (isActive) triggerChannel(ch);
            }
            if (!externalMidiClockActive && !midiTimerRunning) {
              uint32_t interval = (60000000UL / bpm) / 24;
              midiClockTimer.begin(sendClockISR, interval);
              midiTimerRunning = true;
            }
          } else {
            for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
              if (noteOffTick[ch]){
                if (lastNotePlaying[ch] < 128) midiSendNoteOff(ch, lastNotePlaying[ch], 0);
                noteOffTick[ch] = 0;
              }
            }
            midiSendByte(0xFC); // MIDI Stop
            if (midiTimerRunning) { midiClockTimer.end(); midiTimerRunning = false; }
            currentStep = 0;
            midiStepTickCounter = 0;
            stepAdvanceRequested = false;
          }
        }
      }
    }
  }
  startLastReading = startReading;
  // serial command: 't' to run a 10s switch test
  if (Serial.available()){
    char c = Serial.read();
    if (c == 't' || c == 'T') runSwitchTest(10000);
    if (c == 'd' || c == 'D'){
      // cycle division
      stepDivision = (Division)((stepDivision + 1) % 5);
      Serial.print("Division: "); Serial.println(divisionNames[(int)stepDivision]);
    }
    if (c == 'p' || c == 'P'){
      // play test note C3 on channel 0 immediately
      Serial.println("Play C3 (ch1)");
      triggerChannel(0);
    }
    if (c == 'r' || c == 'R'){
      printEncoderRaw();
    }
    if (c == 'm' || c == 'M'){
      runMidiPinMonitor(2000);
    }
    if (c == 'e' || c == 'E'){
      // run encoder switch test for 10s
      runEncoderSwitchTest(10000);
    }
  }
  // MIDI clock generation and external MIDI handling moved to `runEngine()` only to avoid race conditions.

  // Time-critical MIDI processing (advancing steps/note-offs/MIDI RX) now runs in the engine timer.
  // update display at configured refresh interval
  if (millis() - lastDisplayMillis > displayRefreshMs){
    drawDisplay();
    lastDisplayMillis = millis();
  }
}

void SimpleSequencer::readButtons(){
  // polling-based debounce via Bounce2
  for (uint8_t i=0;i<NUM_STEPS;i++){
    // Arduino-style millis debounce (per-button)
    static bool buttonState[NUM_STEPS] = {0};        // the debounced/stable state
    static bool lastReading[NUM_STEPS] = {0};        // last raw reading
    static unsigned long lastDebounceTime[NUM_STEPS] = {0};
    // pendingToggle moved to member variable to allow encoder access
    unsigned long now = millis();
    bool reading = (digitalRead(BUTTON_PINS[i]) == LOW); // pressed = LOW
    if (reading != lastReading[i]){
      lastDebounceTime[i] = now;
    }
    if ((now - lastDebounceTime[i]) > debounceMs){
      if (reading != buttonState[i]){
        buttonState[i] = reading;
        if (buttonState[i]){ // PRESSED
          bool chanModHeld = (digitalRead(CHANNEL_BTN_PIN) == LOW);

          // 1. CHANNEL SELECT INTERCEPT: Pin 28 + Buttons 1-4
          if (chanModHeld && i < NUM_CHANNELS) {
            selectedChannel = i;
          }
          // 2. MUTE INTERCEPT: Start/Stop + Buttons 1-4
          else if (startState && i < NUM_CHANNELS) {
            muted[i] = !muted[i];
            startStopModifierFlag = true;
          }
          // 3. NORMAL STEP TOGGLE / P-LOCK HOLD
          else {
            pendingToggle[i] = true;
            heldStep = i;
          }
        } else { // released
          // perform the toggle now (on release) if it was pending
          if (pendingToggle[i]){
            steps[selectedChannel][i] = !steps[selectedChannel][i];
            // THE ERASER: If step turned OFF, reset it to Global defaults (255) and clear Fill & Ratchet
            if (!steps[selectedChannel][i]) {
              noteLen[selectedChannel][i] = 255;
              pitch[selectedChannel][i] = 255;
              fillStep[selectedChannel][i] = false;
              stepRatchet[selectedChannel][i] = 0;
            }
            Serial.print("Ch"); Serial.print(selectedChannel+1);
            Serial.print(" Step "); Serial.print(i);
            Serial.print(" = "); Serial.println(steps[selectedChannel][i]);
            pendingToggle[i] = false;
          }
          if (heldStep == (int8_t)i) heldStep = -1;
        }
      }
    }
    lastReading[i] = reading;
  }
}

void SimpleSequencer::readEncoders(){
  static uint8_t lastState[4] = {0,0,0,0};
  static unsigned long lastSwDebounce[4] = {0,0,0,0};
  static bool lastSwState[4] = {0,0,0,0};
  static bool lastRawSwState[4] = {0,0,0,0}; // <-- THE FIX: Missing raw tracker added!
  
  const int8_t encTable[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static bool encInitialized = false;
  
  if (!encInitialized){
    for (uint8_t e=0;e<4;e++){
      uint8_t a = digitalRead(ENC_A[e])==HIGH ? 1:0;
      uint8_t b = digitalRead(ENC_B[e])==HIGH ? 1:0;
      lastState[e] = (a<<1) | b;
    }
    encInitialized = true;
  }
  
  for (uint8_t e=0;e<4;e++){
    uint8_t a = digitalRead(ENC_A[e])==HIGH ? 1:0;
    uint8_t b = digitalRead(ENC_B[e])==HIGH ? 1:0;
    uint8_t st = (a<<1) | b;
    uint8_t idx = (lastState[e] << 2) | st;
    int8_t delta = encTable[idx & 0x0F];
    
      if (delta != 0){
      static int8_t encAcc1 = 0;
      static int encAcc2 = 0;
      static int encAcc3 = 0;
      static int encAcc4 = 0;
      int encSteps = 0; 

      if (e == 0){
        encAcc1 += delta;
        if (abs(encAcc1) >= 2) { encSteps = encAcc1 / 2; encAcc1 %= 2; }
      } else if (e == 1){
        if (heldStep >= 0){
          encAcc2 += delta;
          if (abs(encAcc2) >= 4) { encSteps = encAcc2 / 4; encAcc2 %= 4; }
        } else {
          encAcc2 -= delta; 
          if (abs(encAcc2) >= 20) { encSteps = encAcc2 / 20; encAcc2 %= 20; }
        }
      } else if (e == 2){
        encAcc3 += delta;
        if (heldStep >= 0){
          if (abs(encAcc3) >= 4) { encSteps = encAcc3 / 4; encAcc3 %= 4; }
        } else {
          if (abs(encAcc3) >= 20) { encSteps = encAcc3 / 20; encAcc3 %= 20; }
        }
      } else if (e == 3) { // ENCODER 4 GEARBOX
        encAcc4 += delta;
        if (abs(encAcc4) >= 4) { 
          encSteps = encAcc4 / 4; // 4 pulses = 1 physical click
          encAcc4 %= 4; 
        }
      } else {
        encSteps = delta;
      }

      if (encSteps != 0){
        if (e == 0){ // Encoder 1: BPM or Ratchet when a step is held
          if (heldStep >= 0){
            // RATCHET GEARBOX
            static int ratchetAcc = 0;
            ratchetAcc += encSteps;
            
            if (abs(ratchetAcc) >= 2) { // 2 encSteps = 1 full physical click
              int rSteps = ratchetAcc / 2;
              ratchetAcc %= 2;
              
              pendingToggle[heldStep] = false;
              steps[selectedChannel][heldStep] = true;
              int val = (int)stepRatchet[selectedChannel][heldStep] + rSteps;
              stepRatchet[selectedChannel][heldStep] = (uint8_t)constrain(val, 0, 5);
            }
          } else {
            int newBpm = (int)bpm + encSteps;
            if (newBpm < 20) newBpm = 20;
            if (newBpm > 300) newBpm = 300;
            bpm = newBpm;
            if (isRunning && !externalMidiClockActive && midiTimerRunning){
              uint32_t interval = (60000000UL / bpm) / 24;
              midiClockTimer.update(interval);
            }
          }
        } else if (e == 1){ // encoder 2: PITCH
          if (heldStep >= 0){
            pendingToggle[heldStep] = false;
            steps[selectedChannel][heldStep] = true;
            if (pitch[selectedChannel][heldStep] == 255) {
              pitch[selectedChannel][heldStep] = channelPitch[selectedChannel];
            }
            int note = (int)pitch[selectedChannel][heldStep] + encSteps;
            pitch[selectedChannel][heldStep] = (uint8_t)constrain(note, 0, 127);
          } else {
            if (encSteps != 0){
              int note = (int)channelPitch[selectedChannel] + encSteps;
              channelPitch[selectedChannel] = (uint8_t)constrain(note, 0, 127);
            }
          }
        } else if (e == 2){ // encoder 3: NOTE LENGTH
          if (heldStep >= 0){
            pendingToggle[heldStep] = false;
            steps[selectedChannel][heldStep] = true;
            if (noteLen[selectedChannel][heldStep] == 255) {
              noteLen[selectedChannel][heldStep] = noteLenIdx;
            }
            int idxn = (int)noteLen[selectedChannel][heldStep] + encSteps;
            int maxIdx = (int)(sizeof(noteLenTicks)/sizeof(noteLenTicks[0])) - 1;
            noteLen[selectedChannel][heldStep] = (uint8_t)constrain(idxn, 0, maxIdx);
          } else {
            int idxn = (int)noteLenIdx + encSteps;
            int maxIdx = (int)(sizeof(noteLenTicks)/sizeof(noteLenTicks[0])) - 1;
            noteLenIdx = (uint8_t)constrain(idxn, 0, maxIdx);
          }
        } else if (e == 3){ // encoder 4: EUCLID PULSES or OFFSET
          if (heldStep < 0) { 
            if (euclidEnabled[selectedChannel]){
              bool chanModHeld = (digitalRead(CHANNEL_BTN_PIN) == LOW);
              
              if (chanModHeld) {
                // Adjust the Shift Offset
                int o = (int)euclidOffset[selectedChannel] + encSteps;
                while (o < 0) o += NUM_STEPS; // Safe negative wrapping
                euclidOffset[selectedChannel] = (uint8_t)(o % NUM_STEPS);
              } else {
                // Adjust the Hit Pulses
                int p = (int)pulses[selectedChannel] + encSteps;
                if (p < 0) p = 0;
                if (p > NUM_STEPS) p = NUM_STEPS;
                pulses[selectedChannel] = p;
              }
              updateEuclid(selectedChannel);
            }
          }
        }
      }
    }
    lastState[e] = st;
    
    // --- THE FIX: Correctly structured switch debounce logic ---
    bool sw = (digitalRead(ENC_SW[e]) == LOW);
    unsigned long now = millis();
    
    // Compare against raw state!
    if (sw != lastRawSwState[e]){
      lastSwDebounce[e] = now;
    }
    
    if ((now - lastSwDebounce[e]) > debounceMs){
      if (sw != lastSwState[e]){
        lastSwState[e] = sw;
        if (sw){
          // PRESSED
          if (e == 0) { // ENCODER 1 SWITCH: Save State
            saveState();
          }
          else if (e == 1) { // ENCODER 2 SWITCH (Fill Toggle)
            if (heldStep >= 0) {
              // Toggle Fill, force step ON, cancel release toggle
              fillStep[selectedChannel][heldStep] = !fillStep[selectedChannel][heldStep];
              steps[selectedChannel][heldStep] = true;
              pendingToggle[heldStep] = false;
            }
          }
          else if (e == 3){
            // toggle euclid for selected channel
            euclidEnabled[selectedChannel] = !euclidEnabled[selectedChannel];
            updateEuclid(selectedChannel);
          }
        }
      }
    }
    // Record raw state for next loop
    lastRawSwState[e] = sw;
  }
}


void SimpleSequencer::saveState() {
  SaveData data;
  data.magicNumber = 87654321; // Unique signature
  data.savedBpm = bpm;
  data.savedNoteLenIdx = noteLenIdx;
  
  for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
    data.savedChannelPitch[c] = channelPitch[c];
    data.savedMuted[c] = muted[c];
    data.savedEuclidEnabled[c] = euclidEnabled[c];
    data.savedPulses[c] = pulses[c];
    data.savedEuclidOffset[c] = euclidOffset[c];
    
    for (uint8_t s = 0; s < NUM_STEPS; s++) {
      data.savedSteps[c][s] = steps[c][s];
      data.savedPitch[c][s] = pitch[c][s];
      data.savedNoteLen[c][s] = noteLen[c][s];
      data.savedFillStep[c][s] = fillStep[c][s];
      data.savedStepRatchet[c][s] = stepRatchet[c][s];
    }
  }
  EEPROM.put(0, data); // Write to address 0

  // Flash the OLED
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(24, 24);
  display.print("SAVED!");
  display.display();
  delay(600);
}

void SimpleSequencer::loadState() {
  SaveData data;
  EEPROM.get(0, data);

  if (data.magicNumber == 87654321) {
    bpm = data.savedBpm;
    noteLenIdx = data.savedNoteLenIdx;

    for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
      channelPitch[c] = data.savedChannelPitch[c];
      muted[c] = data.savedMuted[c];
      euclidEnabled[c] = data.savedEuclidEnabled[c];
      pulses[c] = data.savedPulses[c];
      euclidOffset[c] = data.savedEuclidOffset[c];
      
      for (uint8_t s = 0; s < NUM_STEPS; s++) {
        steps[c][s] = data.savedSteps[c][s];
        pitch[c][s] = data.savedPitch[c][s];
        noteLen[c][s] = data.savedNoteLen[c][s];
        fillStep[c][s] = data.savedFillStep[c][s];
        stepRatchet[c][s] = data.savedStepRatchet[c][s];
      }
      // Regenerate Euclidean patterns if enabled
      if (euclidEnabled[c]) updateEuclid(c);
    }
    Serial.println("State loaded from EEPROM.");
  } else {
    Serial.println("No saved state found. Booting blank.");
  }
}



void SimpleSequencer::updateEuclid(uint8_t ch){
  uint8_t k = pulses[ch];
  uint8_t n = NUM_STEPS;
  uint8_t offset = euclidOffset[ch];
  if (k == 0){
    for (uint8_t i=0;i<n;i++) euclidPattern[ch][i]=false;
    return;
  }
  if (k >= n){
    for (uint8_t i=0;i<n;i++) euclidPattern[ch][i]=true;
    return;
  }

  bool tempPattern[NUM_STEPS];
  for (uint8_t j=0;j<n;j++){
    int x = (j * k) / n;
    int y = ((j+1) * k) / n;
    tempPattern[j] = (y > x);
  }

  // Apply the rotation offset wrapping around NUM_STEPS
  for (uint8_t j=0;j<n;j++){
    euclidPattern[ch][(j + offset) % n] = tempPattern[j];
  }
}

// Advance the internal MIDI tick counter (called from MIDI clock ISR)
void SimpleSequencer::internalClockTick(){
  // increment absolute tick counter
  absoluteTickCounter++;

  // 1) Process tick-based note-offs FIRST so they clear before a new step triggers
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++){
    if (noteOffTick[ch] > 0 && absoluteTickCounter >= noteOffTick[ch]){
      if (lastNotePlaying[ch] < 128){
        midiSendNoteOff(ch, lastNotePlaying[ch], 0);
      }
      noteOffTick[ch] = 0;
      lastNotePlaying[ch] = 255;
    }

    // 1b) Process pure-tick Ratchet Note-Ons
    if (ratchetIntervalTicks[ch] > 0 && absoluteTickCounter >= ratchetNextTick[ch]) {
      if (absoluteTickCounter < ratchetEndTick[ch]) {
        // Fire next ratchet
        midiSendNoteOn(ch, ratchetPitch[ch], 100);
        lastNotePlaying[ch] = ratchetPitch[ch];
        // Schedule its crisp note-off
        uint32_t offOffset = ratchetIntervalTicks[ch] / 2;
        if (offOffset == 0) offOffset = 1;
        noteOffTick[ch] = absoluteTickCounter + offOffset;
        // Schedule next hit
        ratchetNextTick[ch] += ratchetIntervalTicks[ch];
      } else {
        ratchetIntervalTicks[ch] = 0; // Burst finished
      }
    }
  }

  // 2) Advance the sequencer step using PPQN counting
  midiStepTickCounter++;
  if (midiStepTickCounter >= 6){
    midiStepTickCounter = 0;
    stepAdvanceRequested = true;
  }
}

// Small static wrapper to keep ISR tiny
static void internalClockTickWrapper(){ if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->internalClockTick(); }

// Engine runs at 1ms from hardware timer. It processes incoming MIDI bytes,
// handles external/internal clock state, advances steps when requested, and
// services scheduled note-offs. This function is intentionally minimal and
// avoids USB Serial printing to keep timing deterministic.
void SimpleSequencer::runEngine(){
  // Use micros() for timing inside the engine to avoid reliance on millis()
  uint32_t nowMicros = micros();
  uint32_t nowMs = nowMicros / 1000;

  // 1) Process any MIDI bytes from hardware Serial8
  while (Serial8.available() > 0){
    uint8_t b = Serial8.read();
    if (b == 0xF8){
      externalMidiClockActive = true;
      lastExternalClockMillis = nowMs;
      if (midiTimerRunning){ midiClockTimer.end(); midiTimerRunning = false; }
      // update timestamp window for BPM calculation
      uint32_t currentMicros = nowMicros;
      tickTimestamps[tickIndex] = currentMicros;
      if (validTicks < BPM_TICK_WINDOW) {
        validTicks++;
      } else {
        uint8_t oldestIndex = (tickIndex + 1) % BPM_TICK_WINDOW;
        uint32_t elapsedMicros = currentMicros - tickTimestamps[oldestIndex];
        if (elapsedMicros > 0){
          float calculatedBpm = 120000000.0f / (float)elapsedMicros;
          smoothedBpm = (smoothedBpm * 0.40f) + (calculatedBpm * 0.60f);
          bpm = (uint32_t)(smoothedBpm + 0.5f);
        }
      }
      tickIndex = (tickIndex + 1) % BPM_TICK_WINDOW;
      // advance internal tick counter for this incoming clock
      internalClockTick();
    }
    else if (b == 0xFA){
      // MIDI Start
      externalMidiClockActive = true;
      lastExternalClockMillis = nowMs;
      midiStepTickCounter = 0; validTicks = 0; tickIndex = 0;
      absoluteTickCounter = 0;
      if (midiTimerRunning){ midiClockTimer.end(); midiTimerRunning = false; }
      // start playback
      isRunning = true;
      currentStep = 0;
      // immediately trigger steps at position 0
      for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
        bool isActive = euclidEnabled[ch] ? euclidPattern[ch][currentStep] : steps[ch][currentStep];
        if (isActive) triggerChannel(ch);
      }
    }
    else if (b == 0xFB){
      // MIDI Continue
      externalMidiClockActive = true;
      lastExternalClockMillis = nowMs;
      if (midiTimerRunning){ midiClockTimer.end(); midiTimerRunning = false; }
      // resume without resetting position
      isRunning = true;
    }
    else if (b == 0xFC){
      // MIDI Stop
      externalMidiClockActive = true;
      if (midiTimerRunning){ midiClockTimer.end(); midiTimerRunning = false; }
      isRunning = false;
      // silence any playing notes immediately
      for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
        if (lastNotePlaying[ch] < 128){ midiSendNoteOff(ch, lastNotePlaying[ch], 0); lastNotePlaying[ch] = 255; }
        noteOffTick[ch] = 0;
      }
      // reset metronome counters on external Stop
      midiStepTickCounter = 0;
      stepAdvanceRequested = false;
      absoluteTickCounter = 0;
    }
    else {
      // other MIDI bytes ignored by engine to keep it tight
    }
  }

  // 2) Detect loss of external clock and fall back to internal timer if needed
  if (externalMidiClockActive){
    if ((nowMs - lastExternalClockMillis) > 2000){
      externalMidiClockActive = false;
      midiStepTickCounter = 0;
      validTicks = 0; tickIndex = 0;
      // restart internal hardware timer if needed
      if (isRunning && !midiTimerRunning){
        uint32_t interval = (60000000UL / bpm) / 24;
        midiClockTimer.begin(sendClockISR, interval);
        midiTimerRunning = true;
      }
    }
  }

  // 3) Advance step when requested (set by internalClockTick)
  if (stepAdvanceRequested){
    stepAdvanceRequested = false;
    if (isRunning){
      currentStep = (currentStep + 1) % NUM_STEPS;
      // trigger channels that have the step enabled
        for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
          bool isActive = euclidEnabled[ch] ? euclidPattern[ch][currentStep] : steps[ch][currentStep];
          if (isActive) triggerChannel(ch);
        }
    }
  }

  // Note-offs are now handled in `internalClockTick()` on MIDI ticks.

  // Note-offs are now handled in `internalClockTick()` on MIDI ticks.
}

void SimpleSequencer::triggerChannel(uint8_t ch){
  // 1. If a previous note is playing, turn it off immediately
  if (noteOffTick[ch] > 0 && absoluteTickCounter < noteOffTick[ch]){
    if (lastNotePlaying[ch] < 128) midiSendNoteOff(ch, lastNotePlaying[ch], 0);
    noteOffTick[ch] = 0;
  }

  // 2. THE NORMAL MUTE BLOCK
  if (muted[ch]) return;

  // 3. THE FILL CONDITION BLOCK (This was missing!)
  // If this step is a Fill Step, but the Fill button (Pin 28) is NOT held, abort!
  if (fillStep[ch][currentStep] && !fillModeActive) return;

  // Fire the new Note On
  uint8_t p = pitch[ch][currentStep];
  if (p == 255) p = channelPitch[ch];

  uint8_t note = constrain(p, 0, 127);
  uint8_t vel = 100;
  midiSendNoteOn(ch, note, vel);
  lastNotePlaying[ch] = note;

  // RATCHET: If ratchet is disabled for this step, use regular tick-based note-off
  uint8_t lenIdx = noteLen[ch][currentStep];
  if (lenIdx == 255) lenIdx = noteLenIdx;

  uint8_t rIdx = stepRatchet[ch][currentStep];
  if (rIdx > 0) {
    const uint8_t rTicks[] = {0, 6, 4, 3, 2, 1}; // Exact tick intervals for 16, 24, 32, 48, 96
    uint8_t ticksPerHit = rTicks[rIdx];
    
    ratchetIntervalTicks[ch] = ticksPerHit;
    ratchetNextTick[ch] = absoluteTickCounter + ticksPerHit;
    ratchetEndTick[ch] = absoluteTickCounter + 6; // Strictly constrain burst to exactly one 16th-note step (6 ticks)
    ratchetPitch[ch] = note;
    
    // Schedule crisp note off halfway through the tick interval
    uint32_t offOffset = ticksPerHit / 2;
    if (offOffset == 0) offOffset = 1; 
    noteOffTick[ch] = absoluteTickCounter + offOffset;
  } else {
    // Normal single-hit logic
    ratchetIntervalTicks[ch] = 0; 
    // THE GATE GAP FIX: Subtract 1 tick from the duration to let analog envelopes reset
    uint32_t ticks = noteLenTicks[lenIdx];
    uint32_t gateLength = (ticks > 1) ? (ticks - 1) : 1;
    noteOffTick[ch] = absoluteTickCounter + gateLength;
  }
}

// CV/Gate functions removed; using MIDI out only

void SimpleSequencer::drawDisplay(){
  display.clearDisplay();

  // 1. TOP INFO BAR
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("BPM:"); display.print(bpm);
  display.setCursor(42, 0);
  display.print("C:"); display.print(selectedChannel + 1);

  // Draw Current Channel Base Note
  const char* noteNames[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  uint8_t cp = channelPitch[selectedChannel];
  display.setCursor(60, 0);
  display.print("N:"); display.print(noteNames[cp % 12]); display.print((cp / 12) - 1);

  // Draw Mute Indicators
  for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
    if (muted[c]) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
    else display.setTextColor(SH110X_WHITE, SH110X_BLACK);
    display.setCursor(92 + (c * 9), 0);
    display.print(c + 1);
  }
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  // Separator Line
  display.drawLine(0, 10, 128, 10, SH110X_WHITE);

  // 2. SEQUENCER GRID (2 rows of 8 for the selected channel)
  const int stepW = 12; // Big chunky boxes
  const int stepH = 12;
  const int startX = 6;
  const int startY = 16;
  const int spacingX = 3;
  const int spacingY = 4;

  for (uint8_t i = 0; i < NUM_STEPS; i++) {
    int col = i % 8;
    int row = i / 8;
    int x = startX + col * (stepW + spacingX);
    int y = startY + row * (stepH + spacingY);

    // Draw filled box if step is active (supports Euclidean mode)
    bool stepActive = euclidEnabled[selectedChannel] ? euclidPattern[selectedChannel][i] : steps[selectedChannel][i];
    if (stepActive) {
      display.fillRect(x, y, stepW, stepH, SH110X_WHITE);
      // THE FIX: Draw a distinct "Hollow Core" if it is a Fill Step
      if (fillStep[selectedChannel][i]) {
        display.fillRect(x + 3, y + 3, stepW - 6, stepH - 6, SH110X_BLACK);
      }
    } else {
      display.drawRect(x, y, stepW, stepH, SH110X_WHITE);
    }

    // Draw a thick playhead indicator underneath the current step
    if (i == currentStep) {
      display.drawFastHLine(x, y + stepH + 2, stepW, SH110X_WHITE);
      display.drawFastHLine(x, y + stepH + 3, stepW, SH110X_WHITE);
    }
  }

  // 3. PARAMETER LOCK UI OVERLAY
  if (heldStep >= 0) {
    // Black out a larger space at the bottom for the edit menu (two lines)
    display.fillRect(0, 42, 128, 22, SH110X_BLACK);
    display.drawLine(0, 41, 128, 41, SH110X_WHITE); // Separator

    uint8_t p = pitch[selectedChannel][heldStep];
    uint8_t lenIdx = noteLen[selectedChannel][heldStep];
    // Fallback to channel base pitch for display if no p-lock exists
    if (p == 255) p = channelPitch[selectedChannel];
    if (lenIdx == 255) lenIdx = noteLenIdx;

    const char* noteNames[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
    const char* rNames[] = {"OFF", "16", "24", "32", "48", "96"};

    // First line: STP / Note / Length
    display.setCursor(2, 45);
    display.print("STP:"); display.print(heldStep + 1);
    display.print(" ");
    display.print(noteNames[p % 12]); display.print((p / 12) - 1);
    display.print(" L:"); display.print(noteLenNames[lenIdx]);

    // Second line: Fill and Ratchet
    display.setCursor(2, 55);
    display.print("F:"); display.print(fillStep[selectedChannel][heldStep] ? "ON" : "OFF");
    display.print(" ");
    uint8_t r = stepRatchet[selectedChannel][heldStep];
    display.print("RATC:"); display.print(rNames[r]);
  }

  else if (euclidEnabled[selectedChannel]) {
    // Black out the bottom for the Euclidean menu
    display.fillRect(0, 42, 128, 22, SH110X_BLACK);
    display.drawLine(0, 41, 128, 41, SH110X_WHITE);
    
    display.setCursor(2, 45);
    display.print("--- EUCLIDEAN ---");
    
    display.setCursor(2, 55);
    display.print("HIT:"); display.print(pulses[selectedChannel]);
    display.print("  SFT:"); display.print(euclidOffset[selectedChannel]);
  }

  display.display();
}

void SimpleSequencer::runSwitchTest(uint32_t ms){
  Serial.print("Starting switch test for "); Serial.print(ms); Serial.println(" ms");
  Serial.println("Press buttons to see state changes.");
  bool lastState[NUM_STEPS];
  for (uint8_t i=0;i<NUM_STEPS;i++) lastState[i] = (digitalRead(BUTTON_PINS[i])==LOW);
  bool lastStart = (digitalRead(START_STOP_PIN) == LOW);
  uint32_t start = millis();
  while (millis() - start < ms){
    // buttons
    for (uint8_t i=0;i<NUM_STEPS;i++){
      bool s = (digitalRead(BUTTON_PINS[i])==LOW);
      if (s != lastState[i]){
        Serial.print("Button "); Serial.print(i); Serial.print(s?" pressed":" released"); Serial.println();
        // blink built-in LED briefly
        digitalWrite(LED_BUILTIN, HIGH);
        delay(30);
        digitalWrite(LED_BUILTIN, LOW);
        lastState[i] = s;
      }
    }
    // start/stop button
    bool sr = (digitalRead(START_STOP_PIN) == LOW);
    if (sr != lastStart){
      Serial.print("Start button "); Serial.print(sr?"pressed":"released"); Serial.println();
      digitalWrite(LED_BUILTIN, HIGH);
      delay(40);
      digitalWrite(LED_BUILTIN, LOW);
      lastStart = sr;
    }
    delay(8); // poll interval ~8ms
  }
  Serial.println("Switch test finished");
}

void SimpleSequencer::runEncoderSwitchTest(uint32_t ms){
  Serial.print("Starting encoder-switch test for "); Serial.print(ms); Serial.println(" ms");
  Serial.println("Press encoder buttons to see state changes.");
  bool lastState[4];
  for (uint8_t i=0;i<4;i++) lastState[i] = (digitalRead(ENC_SW[i])==LOW);
  uint32_t start = millis();
  while (millis() - start < ms){
    for (uint8_t i=0;i<4;i++){
      bool s = (digitalRead(ENC_SW[i])==LOW);
      if (s != lastState[i]){
        Serial.print("Enc button "); Serial.print(i+1); Serial.print(s?" pressed":" released"); Serial.println();
        // blink built-in LED briefly
        digitalWrite(LED_BUILTIN, HIGH);
        delay(30);
        digitalWrite(LED_BUILTIN, LOW);
        lastState[i] = s;
      }
    }
    delay(8);
  }
  Serial.println("Encoder switch test finished");
}

void SimpleSequencer::printEncoderRaw(){
  Serial.println("Encoder raw states (A B SW):");
  for (uint8_t e=0;e<4;e++){
    int a = digitalRead(ENC_A[e]);
    int b = digitalRead(ENC_B[e]);
    int sw = digitalRead(ENC_SW[e]);
    Serial.print("Enc"); Serial.print(e+1); Serial.print(": ");
    Serial.print(a); Serial.print(" "); Serial.print(b); Serial.print(" "); Serial.println(sw==LOW?"PRESSED":"RELEASED");
  }
}

void SimpleSequencer::runMidiPinMonitor(uint32_t ms){
  Serial.print("Monitoring MIDI RX pin for "); Serial.print(ms); Serial.println(" ms");
  unsigned long start = millis();
  int last = digitalRead(MIDI_RX_PIN);
  while (millis() - start < ms){
    int v = digitalRead(MIDI_RX_PIN);
    if (v != last){
      Serial.print("MIDI_RX changed: "); Serial.println(v);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(20);
      digitalWrite(LED_BUILTIN, LOW);
      last = v;
    }
    delay(1);
  }
  Serial.println("Monitor finished");
}

// MIDI input handlers removed — processing consolidated in runEngine() to avoid concurrent Serial reads.

void SimpleSequencer::displayTest(){
  // flash full screen a few times and show text
  for (int i=0;i<3;i++){
    display.clearDisplay();
    display.fillRect(0,0,128,64,SH110X_WHITE);
    display.display();
    delay(200);
    display.clearDisplay();
    display.display();
    delay(200);
  }
  // show test text
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10,20);
  display.print("OLED");
  display.setCursor(10,42);
  display.print("TEST");
  display.display();
  delay(800);
}
