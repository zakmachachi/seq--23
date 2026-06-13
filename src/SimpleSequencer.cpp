#include "SimpleSequencer.h"
#include <IntervalTimer.h>
#include <math.h>

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
  MIDI_SERIAL.write(0xF8);
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



// Note length in exact MIDI Clock Ticks. 96 ticks = whole note (96 PPQN at 4/4).
// Granular set: whole, dotted-half, half, dotted-quarter, quarter, dotted-eighth,
// eighth, dotted-sixteenth, sixteenth, sixteenth-triplet, thirty-second.
static const uint8_t noteLenTicks[] = { 96, 72, 48, 36, 24, 18, 12, 9, 6, 4, 3 };
static const char* noteLenNames[] = { "1", "3/4", "1/2", "3/8", "1/4", "3/16", "1/8", "3/32", "1/16", "1/24", "1/32" };
static const uint8_t NOTE_LEN_COUNT = sizeof(noteLenTicks)/sizeof(noteLenTicks[0]);
static const uint8_t NOTE_LEN_DEFAULT_IDX = 8; // 1/16 in the new array

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
  : bpm(200), lastStepMillis(0), currentStep(0), selectedChannel(0),
    ledStrip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800)
{
  for (uint8_t c=0;c<NUM_CHANNELS;c++){
    pulses[c]=4;
    euclidOffset[c] = 0;
    retrig[c]=1;
    euclidEnabled[c]=false;
    euclidScaleMode[c] = 0;
    muted[c]=false;
    noteOffTick[c]=0;
    for (uint16_t s=0; s<TOTAL_STEPS; s++){
      fillState[c][s] = 0;
      steps[c][s]=false;
      euclidPattern[c][s]=false;
      pitch[c][s] = 255;
      noteLen[c][s] = 255;
      stepRatchet[c][s] = 0;
      stepVelocity[c][s] = 255;
      stepSlide[c][s] = false;
      machineOverlay[c][s] = 0;
    }
    for (uint8_t s=0; s<NUM_STEPS; s++){
      pendingToggle[s] = false;
    }
    numPages[c] = 1;
    editPage[c] = 0;
    channelPitch[c] = 33; // default A1 — matches Rytm MK2 default TRIG NOTE
    channelVelocity[c] = 100;
    midiChannel[c] = c; // default: CH1->MIDI ch1, CH2->ch2, ... (0-indexed = MIDI ch 1-6)
    ratchetIntervalTicks[c] = 0;
    lastNotePlaying[c] = 255;
    // Generative defaults
    randomSlideProb[c] = 0;   // 0% slide by default
    octaveSpread[c]    = 0;   // 0 = no octave spread (notes stay in root octave)
    lastScaleMode[c]   = 1;   // remember Major as last-active scale
    // Trigger machine defaults
    trigMachine[c] = TM_OFF;
    trigDensity[c] = 50;
    trigShift[c]   = 0;
    for (uint8_t s = 0; s < NUM_STEPS; s++){
      machinePattern[c][s] = false;
      machineRatchet[c][s] = 0;
    }
    // Kick-specific defaults
    kickNoteSpread[c] = 0;
    kickRatchetProb[c] = 0;
    kickExtrasAreFills[c] = 0;
    // Gate length default per channel
    noteLenIdx[c] = NOTE_LEN_DEFAULT_IDX;
  }
  for (uint8_t k=0;k<MATRIX_KEYS;k++){
    matrixRawState[k] = 0;
    matrixState[k] = false;
    matrixLastDebounce[k] = 0;
  }
  lastMidiClockMicros = 0;
  absoluteTickCounter = 0;
  globalPage = 0;
}

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
  display.setRotation(2); // PCB mounts the OLED upside down — flip 180°

  // second I2C bus + second OLED (overview screen)
  Wire1.begin();
  Wire1.setClock(400000);
  // probe address 0x3C on Wire1 before begin() — avoids long blocking init if absent
  Wire1.beginTransmission(0x3C);
  if (Wire1.endTransmission() == 0){
    display2.begin(0x3C);
    display2.setRotation(2); // same upside-down mounting on PCB
    display2Present = true;
    Serial.println("OLED2 detected on Wire1");
  } else {
    Serial.println("OLED2 NOT found on Wire1 (skipping)");
  }
  // Initialize WS2812 step LED strip (16 LEDs at LED_PIN)
  ledStrip.begin();
  ledStrip.setBrightness(LED_BRIGHTNESS);
  ledStrip.clear();
  ledStrip.show();
  bootAnimation();

  // Initialize hardware MIDI_SERIAL for MIDI at 31250 baud
  MIDI_SERIAL.begin(31250);
  // initialize high-resolution clock reference for internal MIDI output
  lastMidiClockMicros = micros();

  // MIDI clock timing handled by global `lastMidiClockMicros`

  // start the 1ms engine timer which will process MIDI RX, note-offs and step advancement
  engineTimer.begin([](){ if (SimpleSequencer::instancePtr) SimpleSequencer::instancePtr->runEngine(); }, 1000);

  // attempt to auto-load saved state from EEPROM
  loadState();

}

// Removed helper setStepLED and refreshStepLEDs; using updateLEDs() below.

void SimpleSequencer::midiSendByte(uint8_t b){
  // Use hardware MIDI_SERIAL for MIDI output (31250 baud)
  MIDI_SERIAL.write(b);
}

void SimpleSequencer::midiSendNoteOn(uint8_t channel, uint8_t note, uint8_t vel){
  uint8_t status = 0x90 | (channel & 0x0F);
  midiSendByte(status);
  midiSendByte(note & 0x7F);
  midiSendByte(vel & 0x7F);
  // DIAGNOSTIC — confirm notes are being sent. Remove once verified working.
  Serial.print("NOTE_ON  ch="); Serial.print(channel + 1);
  Serial.print(" note="); Serial.print(note);
  Serial.print(" vel="); Serial.println(vel);
}

void SimpleSequencer::midiSendNoteOff(uint8_t channel, uint8_t note, uint8_t vel){
  // Some Elektron devices expect Note-Offs as Note-On with velocity 0.
  uint8_t status = 0x90 | (channel & 0x0F);
  midiSendByte(status);
  midiSendByte(note & 0x7F);
  midiSendByte(0);
  Serial.print("NOTE_OFF ch="); Serial.print(channel + 1);
  Serial.print(" note="); Serial.println(note);
}

void SimpleSequencer::setupPins(){
  // 1. Setup encoder pins FIRST
  // Pot push buttons (used as encoder switches) are configured later; no quadrature encoders present
  // (analog pot pins do not need pinMode). Ensure pot push buttons are inputs:
  for (uint8_t p=0; p< (sizeof(POT_BTN_PINS)/sizeof(POT_BTN_PINS[0])); p++){
    pinMode(POT_BTN_PINS[p], INPUT_PULLUP);
  }
  // 2. Setup matrix pins (rows inputs, cols outputs)
  for (uint8_t c=0; c< (sizeof(MATRIX_COL_PINS)/sizeof(MATRIX_COL_PINS[0])); c++){
    pinMode(MATRIX_COL_PINS[c], OUTPUT);
    digitalWrite(MATRIX_COL_PINS[c], MATRIX_COL_IDLE);
  }
  for (uint8_t r=0; r< (sizeof(MATRIX_ROW_PINS)/sizeof(MATRIX_ROW_PINS[0])); r++){
    // Use pulldown on rows; columns are driven HIGH when active
    pinMode(MATRIX_ROW_PINS[r], INPUT_PULLDOWN);
  }
  // Note: START / CHANNEL modifiers are now matrix buttons (see MATRIX_BTN_START_INDEX / MATRIX_BTN_CHANNEL_INDEX)

  // 3. Handle LED_BUILTIN conflict (Pin 13)
  bool isPin13Used = false;
  for (uint8_t p=0; p< (sizeof(POT_BTN_PINS)/sizeof(POT_BTN_PINS[0])); p++){
    if (POT_BTN_PINS[p] == 13) { isPin13Used = true; break; }
  }

  if (!isPin13Used) {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    // Ensure Pin 13 is strictly an input if used by an encoder
    pinMode(13, INPUT_PULLUP);
    Serial.println("Pin 13 LED disabled to support Encoder 3 Switch.");
  }

  debugLedFlag = false;
  debugLedOffTime = 0;
}

// static instance pointer for ISR forwarding
SimpleSequencer* SimpleSequencer::instancePtr = nullptr;

void SimpleSequencer::handleButtonIRQ(uint8_t idx){
  // (no-op when using Bounce2 polling). Kept for backward compatibility.
}

void SimpleSequencer::loop(){
  // --- TRACK THE FILL PERFORMANCE BUTTON (matrix mapped) ---
  bool fillNow = isFillHeld();
  if (fillNow != fillBtnLastState){
    Serial.print("FILL_BTN ");
    Serial.print(fillNow ? "HELD" : "RELEASED");
    Serial.print(" (matrix idx=");
    Serial.print(MATRIX_BTN_FILL_INDEX);
    Serial.println(")");
    fillBtnLastState = fillNow;
  }
  fillModeActive = fillNow;

  // UI-only loop: read controls and update display. Time-critical MIDI work runs in engine timer.
  readButtons();
  readEncoders();
  // handle start/stop button debounce (Arduino-style)
  // NOTE: Start/Stop now requires BOTH the FN and FILL buttons held together (pins 27 + 28)
  unsigned long now = millis();

  // require both START and CHANNEL matrix buttons to be held for a transport toggle
  bool startReading = isFunctionHeld() && matrixState[MATRIX_BTN_PAGE_INDEX];
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
          // Trigger play/stop OLED splash for ~900ms
          transportAnimIsPlay = isRunning;
          transportAnimEndMs = millis() + 900;
          Serial.print("TRANSPORT "); Serial.println(isRunning ? "PLAY" : "STOP");
          if (isRunning){
            midiStepTickCounter = 0;
            stepAdvanceRequested = false;
            // reset absolute tick counter so internal timing/ratchets start aligned
            absoluteTickCounter = 0;
            midiSendByte(0xFA); // MIDI Start
            midiSendByte(0xF8); // MIDI Clock
            currentStep = 0;
            globalPage = 0;
            for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
              bool isActive = isStepActive(ch, playIdx(ch, currentStep));
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
                if (lastNotePlaying[ch] < 128) midiSendNoteOff(midiChannel[ch] & 0x0F, lastNotePlaying[ch], 0);
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
  // serial commands: drain buffer and execute only the latest typed command
  if (Serial.available()){
    char c = 0;
    while (Serial.available()){
      char ch = Serial.read();
      if (ch == '\n' || ch == '\r') continue;
      c = ch;
    }
    if (c == 't' || c == 'T') runSwitchTest(10000);
    if (c == 'd' || c == 'D'){
      // cycle division
      stepDivision = (Division)((stepDivision + 1) % 5);
      Serial.print("Division: "); Serial.println(divisionNames[(int)stepDivision]);
    }
    if (c == 'c' || c == 'C'){
      // Clear saved EEPROM state (one-time clear)
      SaveData z = {};
      z.magicNumber = 0;
      EEPROM.put(0, z);
      Serial.println("Saved state cleared (EEPROM).");
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
    // MIDI test notes: send one NoteOn at a known pitch on ch 1, then NoteOff
    // 100ms later, ignoring sequencer state. Used to find which MIDI note
    // actually triggers a pad on the Rytm. Press the corresponding key in
    // the serial monitor and listen.
    if (c == '1' || c == '2' || c == '3' || c == '4' || c == '5' || c == '6'){
      static const uint8_t testNotes[6] = {24, 36, 48, 60, 72, 84}; // C1..C6
      uint8_t n = testNotes[c - '1'];
      Serial.print("TEST note "); Serial.print(n); Serial.println(" on ch 1");
      midiSendNoteOn(0, n, 100);
      delay(120);
      midiSendNoteOff(0, n, 0);
    }
    if (c == 'g' || c == 'G'){
      // Diagnostic: dump current channel state to track down silent triggers
      Serial.println("--- DIAG ---");
      Serial.print("running="); Serial.print(isRunning);
      Serial.print(" bpm="); Serial.print(bpm);
      Serial.print(" sel=CH"); Serial.print(selectedChannel + 1);
      Serial.print(" gateIdx="); Serial.println(noteLenIdx[selectedChannel]);
      for (uint8_t cc = 0; cc < NUM_CHANNELS; cc++){
        uint8_t activeSteps = 0, fillSteps = 0;
        uint16_t base = (uint16_t)editPage[cc] * NUM_STEPS;
        for (uint8_t s = 0; s < NUM_STEPS; s++){
          if (steps[cc][base + s]) activeSteps++;
          if (fillState[cc][base + s] == 1) fillSteps++;
        }
        Serial.print("CH"); Serial.print(cc + 1);
        Serial.print(" muted="); Serial.print(muted[cc]);
        Serial.print(" eucEn="); Serial.print(euclidEnabled[cc]);
        Serial.print(" scale="); Serial.print(euclidScaleMode[cc]);
        Serial.print(" pulses="); Serial.print(pulses[cc]);
        Serial.print(" rootPitch="); Serial.print(channelPitch[cc]);
        Serial.print(" vel="); Serial.print(channelVelocity[cc]);
        Serial.print(" activeSteps="); Serial.print(activeSteps);
        Serial.print(" fillSteps="); Serial.println(fillSteps);
      }
      Serial.print("fillModeActive="); Serial.println(fillModeActive);
      Serial.println("------------");
    }
  }
  // MIDI clock generation and external MIDI handling moved to `runEngine()` only to avoid race conditions.

  // Time-critical MIDI processing (advancing steps/note-offs/MIDI RX) now runs in the engine timer.
  // Update display + LEDs together at the configured refresh interval.
  // Pushing WS2812 too often disables interrupts during the bit-bang and starves
  // the matrix scan; the original 60Hz-ish cadence was correct.
  if (millis() - lastDisplayMillis > displayRefreshMs){
    updateLEDs();
    drawDisplay();
    if (display2Present) drawOverview();
    lastDisplayMillis = millis();
  }
}

// Map row/col to linear button index (0..29)
static inline uint8_t matrixIndex(uint8_t row, uint8_t col){
  return (row * MATRIX_COLS) + col;
}

void SimpleSequencer::scanMatrixStep(){
  uint32_t now = millis();
  uint8_t col = matrixScanCol;

  digitalWrite(MATRIX_COL_PINS[col], MATRIX_COL_ACTIVE);
  delayMicroseconds(5);

  for (uint8_t r = 0; r < MATRIX_ROWS; r++){
    uint8_t raw = (digitalRead(MATRIX_ROW_PINS[r]) == HIGH) ? 1 : 0; // pressed when HIGH (match teensi.ino)
    uint8_t idx = matrixIndex(r, col);
    if (raw != matrixRawState[idx]){
      matrixRawState[idx] = raw;
      matrixLastDebounce[idx] = now;
    }
    if ((now - matrixLastDebounce[idx]) > debounceMs){
      bool pressed = (matrixRawState[idx] == 1);
      if (pressed != matrixState[idx]){
        matrixState[idx] = pressed;
        if (pressed) onKeyPress(r, col); else onKeyRelease(r, col);
      }
    }
  }

  // restore column to idle and advance
  digitalWrite(MATRIX_COL_PINS[col], MATRIX_COL_IDLE);
  matrixScanCol = (matrixScanCol + 1) % MATRIX_COLS;
}

void SimpleSequencer::readButtons(){
  // Scan all 6 columns each call (~250us total). Matrix is now sampled at
  // full speed every loop iteration, including just after the 17ms OLED
  // I2C transfer, so no key-press goes longer than one loop iteration
  // before being seen.
  for (uint8_t c = 0; c < MATRIX_COLS; c++){
    scanMatrixStep();
  }
}

// Called when a debounced press is detected
void SimpleSequencer::onKeyPress(uint8_t row, uint8_t col){
  uint8_t i = matrixIndex(row, col);
  Serial.print("KEY "); Serial.println(i); // debug — remove once confirmed working

  // --- Channel select buttons ---
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++){
    if (i == MATRIX_BTN_CH[ch]){
      if (isFunctionHeld()){
        muted[ch] = !muted[ch];
        startStopModifierFlag = true;
        muteAnimCh    = ch;
        muteAnimMuted = muted[ch];
        muteAnimEndMs = millis() + 600;
        Serial.print("MUTE CH"); Serial.print(ch+1);
        Serial.println(muted[ch] ? " ON" : " OFF");
      } else {
        selectedChannel = ch;
        heldChannel = (int8_t)ch; // tracks held channel for OLED focus + Pot1 MIDI-out edit
        Serial.print("SEL CH"); Serial.println(ch+1);
      }
      return;
    }
  }

  // --- Function button ---
  // Modifier in most contexts. Elektron-style: while a step is held, tapping
  // Function flips that step's Fill state (0 = normal, 1 = fill-only).
  // Also: 5 quick taps in a row (no step held) saves the patch to EEPROM.
  if (i == MATRIX_BTN_FUNCTION_INDEX){
    if (heldStep >= 0 && heldStep < (int8_t)NUM_STEPS){
      uint16_t eHs = editIdx(selectedChannel, (uint8_t)heldStep);
      uint8_t &fs = fillState[selectedChannel][eHs];
      fs = (fs == 1) ? 0 : 1;
      steps[selectedChannel][eHs] = true;
      pendingToggle[heldStep] = false;
      fillAnimStep = (uint8_t)heldStep;
      fillAnimSet  = (fs == 1);
      fillAnimEndMs = millis() + 700;
      Serial.print("FILL Ch"); Serial.print(selectedChannel+1);
      Serial.print(" Step ");  Serial.print(heldStep+1);
      Serial.print(" = ");     Serial.println(fs);
    } else {
      // Tap-counter for save: 5 quick taps within 2s of each other.
      static uint8_t fnTapCount = 0;
      static uint32_t fnLastTapMs = 0;
      uint32_t now = millis();
      if (now - fnLastTapMs > 2000) fnTapCount = 0;
      fnTapCount++;
      fnLastTapMs = now;
      Serial.print("FN tap "); Serial.print(fnTapCount); Serial.println("/5");
      if (fnTapCount >= 5){
        fnTapCount = 0;
        saveState(); // shows its own SAVED splash
      }
    }
    return;
  }

  // --- Fill button: held = performance modifier; Function + Fill = clear track ---
  if (i == MATRIX_BTN_FILL_INDEX){
    if (isFunctionHeld()){
      clearTrack(selectedChannel);
      Serial.print("CLEAR CH"); Serial.println(selectedChannel + 1);
    }
    return;
  }

  // --- Page button: modifier only, no action ---
  if (i == MATRIX_BTN_PAGE_INDEX){
    return;
  }

  // --- Menu buttons ---
  if (i == MATRIX_BTN_MENU1_INDEX){
    activeMenu = 1;  // Notes/Scale page
    heldStep = -1; focusEncoder = 0;
    Serial.print("MENU1 -> activeMenu=1 (Notes)"); Serial.println();
    return;
  }
  if (i == MATRIX_BTN_MENU2_INDEX){
    activeMenu = 5;  // Pages mode (replaces old Step Visualizer)
    heldStep = -1; focusEncoder = 0;
    Serial.println("MENU2 -> activeMenu=5 (Pages)");
    return;
  }
  if (i == MATRIX_BTN_MENU3_INDEX){
    activeMenu = 2;  // Euclid page
    heldStep = -1; focusEncoder = 0;
    Serial.print("MENU3 -> activeMenu=2 (Euclid)"); Serial.println();
    return;
  }
  if (i == MATRIX_BTN_MENU4_INDEX){
    activeMenu = 4;  // Trigger Machines page
    heldStep = -1; focusEncoder = 0;
    Serial.println("MENU4 -> activeMenu=4 (Trigger Machines)");
    return;
  }

  // --- Step buttons 0-15 only ---
  if (i < NUM_STEPS){
    // Pages mode: step buttons 1..4 directly jump to that page (auto-extending
    // numPages if the target is beyond the current page count).
    if (activeMenu == 5){
      if (i < MAX_PAGES){
        uint8_t target = (uint8_t)i;
        if (target >= numPages[selectedChannel]){
          numPages[selectedChannel] = target + 1;
        }
        editPage[selectedChannel] = target;
        Serial.print("CH"); Serial.print(selectedChannel+1);
        Serial.print(" goto page "); Serial.println(target+1);
      }
      return;
    }
    if (activeMenu == 4 && trigMachine[selectedChannel] != TM_OFF){
      uint8_t &ov = machineOverlay[selectedChannel][editIdx(selectedChannel, i)];
      ov = (ov + 1) % 3;
      heldStep = (int8_t)i;
      lastEncoderMoveTime = millis();
      focusEncoder = 0;
      pendingToggle[i] = false;  // suppress the on-release toggle below
      Serial.print("OVERLAY Ch"); Serial.print(selectedChannel+1);
      Serial.print(" Step "); Serial.print(i+1);
      Serial.print(" = ");
      Serial.println(ov == 0 ? "AUTO" : (ov == 1 ? "FORCE-ON" : "FORCE-OFF"));
      return;
    }
    pendingToggle[i] = true;
    heldStep = (int8_t)i;
    lastEncoderMoveTime = millis();
    focusEncoder = 0;
    Serial.print("STEP_PRESS idx="); Serial.print(i);
    Serial.print(" -> step "); Serial.println(i + 1);
    return;
  }

  // Fell through everything: orphan key, print so user knows it was ignored
  Serial.print("UNHANDLED KEY "); Serial.println(i);
}

// Called when a debounced release is detected
void SimpleSequencer::onKeyRelease(uint8_t row, uint8_t col){
  uint8_t i = matrixIndex(row,col);
  // Clear held-channel latch when its button is released
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++){
    if (i == MATRIX_BTN_CH[ch] && heldChannel == (int8_t)ch){
      heldChannel = -1;
      break;
    }
  }
  if (i >= NUM_STEPS){
    if (heldStep == (int8_t)i) heldStep = -1;
    return;
  }
  if (pendingToggle[i]){
    bool startHeld = isStartHeld();
    if (startHeld) {
      pendingToggle[i] = false;
    } else {
      uint16_t eI = editIdx(selectedChannel, i);
      if (euclidEnabled[selectedChannel]){
        bool newState = !euclidPattern[selectedChannel][eI];
        euclidPattern[selectedChannel][eI] = newState;
        steps[selectedChannel][eI] = newState;
        if (newState){
          if (pitch[selectedChannel][eI] == 255) pitch[selectedChannel][eI] = channelPitch[selectedChannel];
          if (noteLen[selectedChannel][eI] == 255) noteLen[selectedChannel][eI] = noteLenIdx[selectedChannel];
          if (stepVelocity[selectedChannel][eI] == 255) stepVelocity[selectedChannel][eI] = channelVelocity[selectedChannel];
        }
      } else {
        steps[selectedChannel][eI] = !steps[selectedChannel][eI];
        if (!steps[selectedChannel][eI]){
          noteLen[selectedChannel][eI] = 255;
          pitch[selectedChannel][eI] = 255;
          fillState[selectedChannel][eI] = 0;
          stepRatchet[selectedChannel][eI] = 0;
          stepVelocity[selectedChannel][eI] = 255;
          stepSlide[selectedChannel][eI] = false;
        }
      }
      Serial.print("Ch"); Serial.print(selectedChannel+1);
      Serial.print(" Pg"); Serial.print(editPage[selectedChannel]+1);
      Serial.print(" Step "); Serial.print(i);
      Serial.print(" = "); Serial.println(steps[selectedChannel][eI]);
      pendingToggle[i] = false;
    }
  }
  // Always release the held-step latch when this step is released, otherwise a
  // later Function tap would keep marking the last-touched step as a fill.
  if (heldStep == (int8_t)i) heldStep = -1;
}
// --- Modifier accessors using matrix indices -----------------
bool SimpleSequencer::isFunctionHeld(){
  return (MATRIX_BTN_FUNCTION_INDEX < MATRIX_KEYS) && matrixState[MATRIX_BTN_FUNCTION_INDEX];
}

bool SimpleSequencer::isFillHeld(){
  return (MATRIX_BTN_FILL_INDEX < MATRIX_KEYS) && matrixState[MATRIX_BTN_FILL_INDEX];
}

// Aliases so existing runEngine/loop code compiles without changes
bool SimpleSequencer::isStartHeld()   { return isFunctionHeld() && matrixState[MATRIX_BTN_PAGE_INDEX]; }
bool SimpleSequencer::isChannelHeld() { return matrixState[MATRIX_BTN_PAGE_INDEX]; }

void SimpleSequencer::readEncoders(){
  // Replaced quadrature encoder handling with 6 potentiometers (infinite-scroll style)
  const uint8_t POT_COUNT = (uint8_t)(sizeof(POT_A_PINS)/sizeof(POT_A_PINS[0]));
  static float potPrevAngle[6] = {0};
  static float potAccumulator[6] = {0};
  static bool potFirstRun[6] = {true, true, true, true, true, true};
  static float potMinTick[6] = {0.08f,0.08f,0.08f,0.08f,0.08f,0.08f};
  const float TICK_ANGLE_DEFAULT = 0.08f;
  const float MIN_TICK = 0.03f;
  const float MAX_TICK = 0.20f;
  const float TICK_DECAY = 0.96f;
  const float SMALL_TICK_DIVIDER = 10.0f;
  static int potLastTick[6] = {0,0,0,0,0,0};

  // Pot button debounce (matches teensi.ino logic)
  static bool potBtnState[6] = {false,false,false,false,false,false};
  static bool lastPotBtnState[6] = {false,false,false,false,false,false};
  static unsigned long lastPotBtnChange[6] = {0,0,0,0,0,0};
  const unsigned long POT_BTN_DEBOUNCE_MS = 10;

  // Scan pot buttons (active LOW)
  for (uint8_t i=0;i<POT_COUNT;i++){
    bool pressed = (digitalRead(POT_BTN_PINS[i]) == LOW);
    if (pressed != lastPotBtnState[i]){
      lastPotBtnChange[i] = millis();
      lastPotBtnState[i] = pressed;
    } else if (pressed != potBtnState[i]) {
      if ((millis() - lastPotBtnChange[i]) >= POT_BTN_DEBOUNCE_MS){
        potBtnState[i] = pressed;
        if (pressed) {
          onPotButtonPress(i);
        }
      }
    }
  }

  // Scan pots (infinite scroll algorithm)
  for (uint8_t i=0;i<POT_COUNT;i++){
    int valA = analogRead(POT_A_PINS[i]);
    int valB = analogRead(POT_B_PINS[i]);

    float a = (valA - 512.0f) / 512.0f;
    float b = (valB - 512.0f) / 512.0f;
    float angle = atan2f(b, a);

    if (potFirstRun[i]){ potPrevAngle[i] = angle; potFirstRun[i] = false; }

    float delta = angle - potPrevAngle[i];
    if (delta > M_PI) delta -= 2.0f * M_PI;
    if (delta < -M_PI) delta += 2.0f * M_PI;
    delta = -delta; // match original direction

    float absDelta = fabsf(delta);
    if (absDelta > potMinTick[i]){
      potMinTick[i] = fmaxf(MIN_TICK, potMinTick[i] * 0.7f);
    } else if (absDelta > 0.01f){
      potMinTick[i] = fmaxf(MIN_TICK, potMinTick[i] * 0.97f);
    } else {
      potMinTick[i] = fminf(MAX_TICK, potMinTick[i] * TICK_DECAY + TICK_ANGLE_DEFAULT * (1.0f - TICK_DECAY));
    }

    float effectiveTick = potMinTick[i];
    if (effectiveTick == MIN_TICK) effectiveTick *= SMALL_TICK_DIVIDER;

    potAccumulator[i] += delta;
    int ticks = 0;
    while (potAccumulator[i] >= effectiveTick){ ticks++; potAccumulator[i] -= effectiveTick; }
    while (potAccumulator[i] <= -effectiveTick){ ticks--; potAccumulator[i] += effectiveTick; }

    if (ticks != 0){
      handlePotRotation(i, ticks);
      int target = potLastTick[i] + ticks;
      potLastTick[i] = target;
    }

    if (ticks == 0) potLastTick[i] = 0;
    potPrevAngle[i] = angle;
  }
}

// --- POT BUTTON PRESS HANDLER: Context-dependent actions -------
void SimpleSequencer::onPotButtonPress(uint8_t pot){
  if (activeMenu == 5){
    // Pages mode: Pot 1 button cycles numPages 1..MAX_PAGES
    if (pot == 0){
      uint8_t ch = selectedChannel;
      numPages[ch] = (numPages[ch] % MAX_PAGES) + 1;
      if (editPage[ch] >= numPages[ch]) editPage[ch] = numPages[ch] - 1;
      Serial.print("CH"); Serial.print(ch+1);
      Serial.print(" numPages="); Serial.println(numPages[ch]);
    }
    return;
  }
  if (activeMenu == 2){
    // Euclid page: Pot 1 button toggles euclid on/off
    if (pot == 0){
      euclidEnabled[selectedChannel] = !euclidEnabled[selectedChannel];
      if (euclidEnabled[selectedChannel]) updateEuclid(selectedChannel);
      Serial.print("EUCLID CH"); Serial.print(selectedChannel+1);
      Serial.println(euclidEnabled[selectedChannel] ? " ON" : " OFF");
    }
  } else if (activeMenu == 1){
    // Notes page generative controls
    if (pot == 0){
      // Toggle Generative Mode: swap euclidScaleMode between 0 and lastScaleMode
      uint8_t ch = selectedChannel;
      if (euclidScaleMode[ch] == 0){
        if (lastScaleMode[ch] == 0) lastScaleMode[ch] = 1; // safety: fall back to Major
        euclidScaleMode[ch] = lastScaleMode[ch];
        // First-time helper: if the user has no rhythm set on this channel
        // (no manual steps and Euclid off), enable all 16 so they hear
        // something immediately. Otherwise respect their existing pattern.
        if (!euclidEnabled[ch]){
          bool anyActive = false;
          uint16_t base = (uint16_t)editPage[ch] * NUM_STEPS;
          for (uint8_t s = 0; s < NUM_STEPS; s++){
            if (steps[ch][base + s]){ anyActive = true; break; }
          }
          if (!anyActive){
            for (uint8_t s = 0; s < NUM_STEPS; s++){
              steps[ch][base + s] = true;
            }
          }
        }
        randomizeEuclidMelody(ch);
        Serial.print("GEN CH"); Serial.print(ch+1); Serial.println(" ON");
      } else {
        lastScaleMode[ch] = euclidScaleMode[ch];
        euclidScaleMode[ch] = 0;
        randomizeEuclidMelody(ch); // clears per-step pitches back to default
        Serial.print("GEN CH"); Serial.print(ch+1); Serial.println(" OFF");
      }
    } else if (pot == 1){
      // Regenerate notes using current settings
      randomizeEuclidMelody(selectedChannel);
      Serial.print("REGEN CH"); Serial.println(selectedChannel+1);
    }
  }
}

// --- POT ROTATION HANDLER: Context-dependent parameter control -------
void SimpleSequencer::handlePotRotation(uint8_t pot, int ticks){
  // Per-menu, per-pot sensitivity divisors (1 = native, higher = slower).
  // Notes page: pot 4 (slide%) stays at native speed; everything else dampened.
  // Euclid page: pulses=fast(3), offset=fast(3), scale=slow(5), velocity=med(3), gate=med(3).
  static const uint8_t divNotes[6]       = {3, 12, 3, 1, 3, 3};
  static const uint8_t divEuclid[6]      = {3, 3, 5, 3, 3, 1}; // pot 6 = slide%, native sensitivity
  static const uint8_t divTrigMachine[6] = {5, 1, 2, 3, 3, 3}; // density (pot2) fast
  static const uint8_t divDefault[6]     = {3, 12, 3, 3, 3, 3};
  static int potAcc[6] = {0,0,0,0,0,0};
  static uint8_t lastMenu = 0;
  if (lastMenu != activeMenu){
    for (uint8_t i = 0; i < 6; i++) potAcc[i] = 0;
    lastMenu = activeMenu;
  }
  const uint8_t* divTable = divDefault;
  if (activeMenu == 1) divTable = divNotes;
  else if (activeMenu == 2) divTable = divEuclid;
  else if (activeMenu == 4) divTable = divTrigMachine;
  int dv = (pot < 6 && divTable[pot] > 0) ? (int)divTable[pot] : 1;
  potAcc[pot] += ticks;
  int forward = potAcc[pot] / dv;
  potAcc[pot] -= forward * dv;
  if (forward == 0) return;
  ticks = forward;

  // Track which pot was last actually rotated so screen 2 can focus on it.
  lastTouchedPot  = (int8_t)pot;
  lastPotTouchMs  = millis();

  // --- GLOBAL MODIFIER: Function + Pot 1 = BPM (anywhere, in 1-BPM steps) ---
  if (pot == 0 && isFunctionHeld()){
    int newBpm = (int)bpm + ticks;
    if (newBpm < 20) newBpm = 20;
    if (newBpm > 300) newBpm = 300;
    bpm = (uint32_t)newBpm;
    if (isRunning && !externalMidiClockActive && midiTimerRunning){
      uint32_t interval = (60000000UL / bpm) / 24;
      midiClockTimer.update(interval);
    }
    bpmFocusEndMs = millis() + 1500;
    Serial.print("BPM="); Serial.println(bpm);
    return;
  }

  // --- GLOBAL MODIFIER: Channel-held + Pot 1 = per-channel MIDI Out channel ---
  if (pot == 0 && heldChannel >= 0 && heldChannel < (int)NUM_CHANNELS){
    int v = (int)midiChannel[heldChannel] + ticks;
    if (v < 0) v = 0;
    if (v > 15) v = 15;
    midiChannel[heldChannel] = (uint8_t)v;
    Serial.print("CH"); Serial.print(heldChannel+1);
    Serial.print(" MIDI_OUT="); Serial.println(midiChannel[heldChannel]+1);
    return;
  }

  if (activeMenu == 1){
    uint8_t ch = selectedChannel;
    switch (pot){
      case 0: { // Root note — TRANSPOSE existing per-step notes (no regenerate)
        int oldRoot = (int)channelPitch[ch];
        int p = oldRoot + ticks;
        p = constrain(p, 0, 127);
        int delta = p - oldRoot;
        channelPitch[ch] = (uint8_t)p;
        if (delta != 0) transposeChannelNotes(ch, delta);
        Serial.print("ROOT="); Serial.println(channelPitch[ch]);
        break;
      }
      case 1: { // Scale selection — store only (next regenerate applies it)
        int mode = (int)euclidScaleMode[ch];
        if (mode <= 0) mode = 1;
        mode += ticks;
        const int N = 6;
        mode = ((mode - 1) % N + N) % N + 1;
        euclidScaleMode[ch] = (uint8_t)mode;
        lastScaleMode[ch]   = (uint8_t)mode;
        Serial.print("SCALE="); Serial.println(mode);
        break;
      }
      case 2: { // Gate length (per-channel)
        int prev = noteLenIdx[ch];
        noteLenIdx[ch] = (uint8_t)constrain(prev + ticks, 0, (int)NOTE_LEN_COUNT - 1);
        if (noteLenIdx[ch] != prev && euclidScaleMode[ch] != 0){
          uint16_t base = (uint16_t)editPage[ch] * NUM_STEPS;
          for (uint8_t s = 0; s < NUM_STEPS; s++) noteLen[ch][base + s] = noteLenIdx[ch];
        }
        Serial.print("GATE CH"); Serial.print(ch+1);
        Serial.print("="); Serial.println(noteLenIdx[ch]);
        break;
      }
      case 3: { // Random Slide probability — re-roll slides immediately
        randomSlideProb[ch] = (uint8_t)constrain(
          (int)randomSlideProb[ch] + ticks, 0, 100);
        rerollSlides(ch);
        Serial.print("SLIDE%="); Serial.println(randomSlideProb[ch]);
        break;
      }
      case 4: { // Base velocity 0..127
        channelVelocity[ch] = (uint8_t)constrain(
          (int)channelVelocity[ch] + ticks, 0, 127);
        Serial.print("VEL="); Serial.println(channelVelocity[ch]);
        break;
      }
      case 5: { // Octave spread 0..60 semitones — store only (next regenerate applies it)
        octaveSpread[ch] = (uint8_t)constrain(
          (int)octaveSpread[ch] + ticks, 0, 60);
        Serial.print("SPRD="); Serial.println(octaveSpread[ch]);
        break;
      }
    }
    return;
  }

  // --- Menu 5: Pages mode ---
  if (activeMenu == 5){
    uint8_t ch = selectedChannel;
    if (pot == 0){
      int v = (int)editPage[ch] + ticks;
      uint8_t np = numPages[ch] > 0 ? numPages[ch] : 1;
      v = ((v % np) + np) % np;
      editPage[ch] = (uint8_t)v;
      Serial.print("CH"); Serial.print(ch+1);
      Serial.print(" editPage="); Serial.println(editPage[ch]+1);
    } else if (pot == 1){
      int v = (int)numPages[ch] + ticks;
      if (v < 1) v = 1;
      if (v > MAX_PAGES) v = MAX_PAGES;
      numPages[ch] = (uint8_t)v;
      if (editPage[ch] >= numPages[ch]) editPage[ch] = numPages[ch] - 1;
      Serial.print("CH"); Serial.print(ch+1);
      Serial.print(" numPages="); Serial.println(numPages[ch]);
    }
    return;
  }

  // --- Menu 4: Trigger Machines page ---
  if (activeMenu == 4){
    uint8_t ch = selectedChannel;
    switch (pot){
      case 0: { // Pot 1: machine type cycle
        int v = (int)trigMachine[ch] + ticks;
        const int N = (int)TM_COUNT;
        v = ((v % N) + N) % N;
        trigMachine[ch] = (uint8_t)v;
        if (trigMachine[ch] == TM_EUCLID) updateEuclid(ch);
        regenerateMachinePattern(ch);
        Serial.print("MACHINE CH"); Serial.print(ch+1);
        Serial.print(" = "); Serial.println(trigMachine[ch]);
        break;
      }
      case 1: { // Pot 2: density
        trigDensity[ch] = (uint8_t)constrain((int)trigDensity[ch] + ticks, 0, 100);
        regenerateMachinePattern(ch);
        Serial.print("DENSITY="); Serial.println(trigDensity[ch]);
        break;
      }
      case 2: { // Pot 3: shift
        int v = (int)trigShift[ch] + ticks;
        v = ((v % NUM_STEPS) + NUM_STEPS) % NUM_STEPS;
        trigShift[ch] = (uint8_t)v;
        regenerateMachinePattern(ch);
        Serial.print("SHIFT="); Serial.println(trigShift[ch]);
        break;
      }
      // Pots 4-6: per-machine extras. Currently only KICK uses them.
      case 3: { // Pot 4: kick note spread (0..5 semitones)
        if (trigMachine[ch] == TM_KICK){
          kickNoteSpread[ch] = (uint8_t)constrain((int)kickNoteSpread[ch] + ticks, 0, 5);
          regenerateMachinePattern(ch);
          Serial.print("KICK SPR="); Serial.println(kickNoteSpread[ch]);
        }
        break;
      }
      case 4: { // Pot 5: kick ratchet probability (0..100 %)
        if (trigMachine[ch] == TM_KICK){
          kickRatchetProb[ch] = (uint8_t)constrain((int)kickRatchetProb[ch] + ticks, 0, 100);
          regenerateMachinePattern(ch);
          Serial.print("KICK RCH%="); Serial.println(kickRatchetProb[ch]);
        }
        break;
      }
      case 5: { // Pot 6: kick "extras as fill" toggle
        if (trigMachine[ch] == TM_KICK){
          int v = (int)kickExtrasAreFills[ch] + ticks;
          if (v < 0) v = 0;
          if (v > 1) v = 1;
          if ((uint8_t)v != kickExtrasAreFills[ch]){
            kickExtrasAreFills[ch] = (uint8_t)v;
            Serial.print("KICK FILL=");
            Serial.println(kickExtrasAreFills[ch] ? "ON" : "OFF");
          }
        }
        break;
      }
      default: break;
    }
    return;
  }

  if (activeMenu != 2) return;  // Other menus: only Euclid handles rotation

  // Euclid page controls (reorganized: P1=pulses, P2=offset, P3=scale, P4=vel, P5=gate)
  switch (pot){
    case 0:  // Pot 1: pulses (number of triggers)
      pulses[selectedChannel] = (uint8_t)constrain(
        (int)pulses[selectedChannel] + ticks, 0, NUM_STEPS);
      updateEuclid(selectedChannel);
      Serial.print("PULSES="); Serial.println(pulses[selectedChannel]);
      break;

    case 1:  // Pot 2: offset
      euclidOffset[selectedChannel] =
        (euclidOffset[selectedChannel] + ticks + NUM_STEPS) % NUM_STEPS;
      updateEuclid(selectedChannel);
      Serial.print("OFFSET="); Serial.println(euclidOffset[selectedChannel]);
      break;

    case 2: { // Pot 3: cycle scale mode (0=OFF .. 6=Atonal, 7 modes)
      int m = (int)euclidScaleMode[selectedChannel] + ticks;
      const int N = 7;
      m = ((m % N) + N) % N;
      euclidScaleMode[selectedChannel] = (uint8_t)m;
      if (euclidEnabled[selectedChannel]) randomizeEuclidMelody(selectedChannel);
      Serial.print("SCALE="); Serial.println(euclidScaleMode[selectedChannel]);
      break;
    }

    case 3:  // Pot 4: velocity
      channelVelocity[selectedChannel] = (uint8_t)constrain(
        (int)channelVelocity[selectedChannel] + ticks, 0, 127);
      Serial.print("VEL="); Serial.println(channelVelocity[selectedChannel]);
      break;

    case 4:  // Pot 5: gate length (per-channel)
      noteLenIdx[selectedChannel] = (uint8_t)constrain(
        (int)noteLenIdx[selectedChannel] + ticks, 0, (int)NOTE_LEN_COUNT - 1);
      Serial.print("GATE CH"); Serial.print(selectedChannel + 1);
      Serial.print("="); Serial.println(noteLenIdx[selectedChannel]);
      break;

    case 5: { // Pot 6: slide probability — re-rolls slides immediately (mirrors Menu 1)
      randomSlideProb[selectedChannel] = (uint8_t)constrain(
        (int)randomSlideProb[selectedChannel] + ticks, 0, 100);
      rerollSlides(selectedChannel);
      Serial.print("SLIDE%="); Serial.println(randomSlideProb[selectedChannel]);
      break;
    }
  }
}

void SimpleSequencer::saveState() {
  SaveData data;
  data.magicNumber = 13572476; // Unique signature (v10 — pages mode)
  data.savedBpm = bpm;

  for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
    data.savedNoteLenIdx[c] = noteLenIdx[c];
    data.savedChannelPitch[c] = channelPitch[c];
    data.savedMuted[c] = muted[c];
    data.savedEuclidEnabled[c] = euclidEnabled[c];
    data.savedPulses[c] = pulses[c];
    data.savedEuclidOffset[c] = euclidOffset[c];
    data.savedEuclidScaleMode[c] = euclidScaleMode[c];

    for (uint16_t s = 0; s < TOTAL_STEPS; s++) {
      data.savedSteps[c][s] = steps[c][s];
      data.savedPitch[c][s] = pitch[c][s];
      data.savedNoteLen[c][s] = noteLen[c][s];
      data.savedFillStep[c][s] = fillState[c][s];
      data.savedStepRatchet[c][s] = stepRatchet[c][s];
      data.savedStepVelocity[c][s] = stepVelocity[c][s];
      data.savedStepSlide[c][s] = stepSlide[c][s] ? 1 : 0;
      data.savedMachineOverlay[c][s] = machineOverlay[c][s];
    }
    data.savedChannelVelocity[c] = channelVelocity[c];
    data.savedRandomSlideProb[c] = randomSlideProb[c];
    data.savedOctaveSpread[c]    = octaveSpread[c];
    data.savedMidiChannel[c]     = midiChannel[c];
    data.savedTrigMachine[c]     = trigMachine[c];
    data.savedTrigDensity[c]     = trigDensity[c];
    data.savedTrigShift[c]       = trigShift[c];
    data.savedKickNoteSpread[c]     = kickNoteSpread[c];
    data.savedKickRatchetProb[c]    = kickRatchetProb[c];
    data.savedKickExtrasAreFills[c] = kickExtrasAreFills[c];
    data.savedNumPages[c]           = numPages[c];
  }
  // Write to EEPROM
  EEPROM.put(0, data);

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

  if (data.magicNumber == 13572476) {
    bpm = data.savedBpm;

    for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
      noteLenIdx[c] = data.savedNoteLenIdx[c];
      if (noteLenIdx[c] >= NOTE_LEN_COUNT) noteLenIdx[c] = NOTE_LEN_DEFAULT_IDX;
      channelPitch[c] = data.savedChannelPitch[c];
      muted[c] = data.savedMuted[c];
      euclidEnabled[c] = data.savedEuclidEnabled[c];
      pulses[c] = data.savedPulses[c];
      euclidOffset[c] = data.savedEuclidOffset[c];
      euclidScaleMode[c] = data.savedEuclidScaleMode[c];

      for (uint16_t s = 0; s < TOTAL_STEPS; s++) {
        steps[c][s] = data.savedSteps[c][s];
        pitch[c][s] = data.savedPitch[c][s];
        noteLen[c][s] = data.savedNoteLen[c][s];
        if (noteLen[c][s] != 255 && noteLen[c][s] >= NOTE_LEN_COUNT) noteLen[c][s] = 255;
        fillState[c][s] = data.savedFillStep[c][s];
        stepRatchet[c][s] = data.savedStepRatchet[c][s];
        stepVelocity[c][s] = data.savedStepVelocity[c][s];
        if (stepVelocity[c][s] != 255 && stepVelocity[c][s] > 127) stepVelocity[c][s] = 100;
        stepSlide[c][s] = (data.savedStepSlide[c][s] != 0);
        uint8_t ov = data.savedMachineOverlay[c][s];
        machineOverlay[c][s] = (ov <= 2) ? ov : 0;
      }
      channelVelocity[c] = data.savedChannelVelocity[c];
      if (channelVelocity[c] > 127) channelVelocity[c] = 100;
      randomSlideProb[c] = (data.savedRandomSlideProb[c] <= 100) ? data.savedRandomSlideProb[c] : 0;
      octaveSpread[c]    = (data.savedOctaveSpread[c] <= 60)     ? data.savedOctaveSpread[c]    : 0;
      midiChannel[c]     = (data.savedMidiChannel[c] < 16)       ? data.savedMidiChannel[c]     : c;
      trigMachine[c]     = (data.savedTrigMachine[c] < TM_COUNT) ? data.savedTrigMachine[c]     : TM_OFF;
      trigDensity[c]     = (data.savedTrigDensity[c] <= 100)     ? data.savedTrigDensity[c]     : 50;
      trigShift[c]       = (data.savedTrigShift[c] < NUM_STEPS)  ? data.savedTrigShift[c]       : 0;
      kickNoteSpread[c]     = (data.savedKickNoteSpread[c] <= 5)     ? data.savedKickNoteSpread[c]     : 0;
      kickRatchetProb[c]    = (data.savedKickRatchetProb[c] <= 100)  ? data.savedKickRatchetProb[c]    : 0;
      kickExtrasAreFills[c] = (data.savedKickExtrasAreFills[c] <= 1) ? data.savedKickExtrasAreFills[c] : 0;
      uint8_t np = data.savedNumPages[c];
      numPages[c] = (np >= 1 && np <= MAX_PAGES) ? np : 1;
      editPage[c] = 0;
      if (euclidScaleMode[c] > 0 && euclidScaleMode[c] <= 6) lastScaleMode[c] = euclidScaleMode[c];
      if (euclidEnabled[c]) updateEuclid(c);
      regenerateMachinePattern(c);
    }
    Serial.println("State loaded from EEPROM (v10).");
  } else {
    Serial.println("No saved state (v10) found. Booting blank.");
  }
}

// --- TRIGGER MACHINE WEIGHT TABLES ----------------------------------
// Each table is 16 entries (one per step, 0-indexed). Higher weight = more
// likely to be active for a given density. A step is on iff weight >= (100-density).
// Weight 100 means "always on at any density >= 0".
// Step indices below are 0-based: step 1 in the user's terminology = idx 0.

// KICK: 4-on-the-floor skeleton (idx 0,4,8,12 at weight 100, always on).
// Other steps weighted lower so density needs to climb high before any
// extras come in — much sparser feel suited to live performance.
static const uint8_t W_KICK[16] = {
  100, 10, 30, 10, 100, 10, 30, 10, 100, 10, 30, 10, 100, 10, 30, 18
};

// HIHAT: starts dense at high density, erodes at low density. Offbeats (2,6,10,14)
// are most resistant; downbeats (0,4,8,12) next; other 1/16ths least resistant.
static const uint8_t W_HIHAT[16] = {
  80, 50, 100, 50, 80, 50, 100, 50, 80, 50, 100, 50, 80, 50, 100, 50
};

// SNARE: backbeats on idx 4 and 12 dominate. Light fills around them.
static const uint8_t W_SNARE[16] = {
  10, 10, 10, 20, 100, 20, 10, 25, 10, 10, 10, 25, 100, 25, 15, 30
};

// ANTI-KICK: triggers between kicks. 1/8 offbeats (2,6,10,14) are primary;
// 1/16 in-betweens fill in at higher density.
static const uint8_t W_ANTIKICK[16] = {
  10, 40, 100, 40, 10, 40, 100, 40, 10, 40, 100, 40, 10, 40, 100, 40
};

// PERC: even 1/16ths with weighting toward both onbeats and offbeats.
static const uint8_t W_PERC[16] = {
  90, 50, 90, 50, 90, 50, 90, 50, 90, 50, 90, 50, 90, 50, 90, 50
};

static const uint8_t* getMachineWeights(uint8_t machine){
  switch (machine){
    case SimpleSequencer::TM_KICK:     return W_KICK;
    case SimpleSequencer::TM_HIHAT:    return W_HIHAT;
    case SimpleSequencer::TM_SNARE:    return W_SNARE;
    case SimpleSequencer::TM_ANTIKICK: return W_ANTIKICK;
    case SimpleSequencer::TM_PERC:     return W_PERC;
    default: return nullptr;
  }
}

void SimpleSequencer::regenerateMachinePattern(uint8_t ch){
  uint8_t m = trigMachine[ch];
  uint8_t density = trigDensity[ch];
  uint8_t shift = trigShift[ch] % NUM_STEPS;

  // Threshold: a step is on iff weight >= (100 - density).
  // density 0   -> threshold 100 -> only weight==100 steps on (machine "skeleton")
  // density 100 -> threshold 0   -> every step with weight > 0 on
  int threshold = 100 - (int)density;

  if (m == TM_OFF || m >= TM_COUNT){
    for (uint8_t s = 0; s < NUM_STEPS; s++) machinePattern[ch][s] = false;
    return;
  }
  if (m == TM_EUCLID){
    // Euclid uses the live euclidPattern[] for whichever page is currently
    // playing. shift rotates the visible pattern.
    for (uint8_t s = 0; s < NUM_STEPS; s++){
      uint8_t src = (s + NUM_STEPS - shift) % NUM_STEPS;
      machinePattern[ch][s] = euclidPattern[ch][playIdx(ch, src)];
    }
    return;
  }
  const uint8_t* w = getMachineWeights(m);
  if (!w){
    for (uint8_t s = 0; s < NUM_STEPS; s++) machinePattern[ch][s] = false;
    return;
  }
  for (uint8_t s = 0; s < NUM_STEPS; s++){
    uint8_t src = (s + NUM_STEPS - shift) % NUM_STEPS;
    machinePattern[ch][s] = ((int)w[src] >= threshold);
    machineRatchet[ch][s] = 0;
  }

  // KICK extras: for every step that's not a 4-on-the-floor base, roll
  // independently for note spread (transposes the pitch up by 1..N
  // semitones) and a ratchet (random 1..3 hits). The fill toggle and
  // visibility/play decision is handled later in isStepActive().
  if (m == TM_KICK){
    uint8_t spread = kickNoteSpread[ch] > 5 ? 5 : kickNoteSpread[ch];
    uint8_t ratProb = kickRatchetProb[ch] > 100 ? 100 : kickRatchetProb[ch];
    // Pitch overrides go on the current play page (whichever page will play
    // next at this index). machineRatchet stays a 16-cell live array.
    for (uint8_t s = 0; s < NUM_STEPS; s++){
      uint16_t pI = playIdx(ch, s);
      if (!machinePattern[ch][s]){
        pitch[ch][pI] = 255;
        continue;
      }
      uint8_t src = (s + NUM_STEPS - shift) % NUM_STEPS;
      bool isBase = (W_KICK[src] == 100);
      if (isBase){
        pitch[ch][pI] = 255;
        machineRatchet[ch][s] = 0;
        continue;
      }
      if (spread > 0){
        uint8_t off = (uint8_t)random(1, (int)spread + 1);
        int pn = (int)channelPitch[ch] + (int)off;
        pitch[ch][pI] = (uint8_t)constrain(pn, 0, 127);
      } else {
        pitch[ch][pI] = 255;
      }
      if (ratProb > 0 && (uint8_t)random(0, 100) < ratProb){
        machineRatchet[ch][s] = (uint8_t)random(1, 4);
      } else {
        machineRatchet[ch][s] = 0;
      }
    }
  }
}

// Combined activity check: machine + overlay, falling back to legacy
// euclid/steps[] when no machine is active. `step` is now an ABSOLUTE step
// index in 0..TOTAL_STEPS-1 — caller maps via editIdx()/playIdx() depending
// on whether the check is for the visible edit page or the live playback.
bool SimpleSequencer::isStepActive(uint8_t ch, uint16_t step){
  if (step >= TOTAL_STEPS) return false;
  uint8_t s = (uint8_t)(step % NUM_STEPS); // step-within-page (0..15)
  if (trigMachine[ch] != TM_OFF){
    uint8_t ov = machineOverlay[ch][step];
    if (ov == 1) return true;
    if (ov == 2) return false;
    bool on = machinePattern[ch][s];
    // Kick "extras as fill" toggle: non-base kicks only fire while Fill is held
    if (on && trigMachine[ch] == TM_KICK && kickExtrasAreFills[ch]){
      uint8_t src = (s + NUM_STEPS - trigShift[ch]) % NUM_STEPS;
      bool isBase = (W_KICK[src] == 100);
      if (!isBase && !fillModeActive) return false;
    }
    return on;
  }
  if (euclidEnabled[ch]) return euclidPattern[ch][step];
  return steps[ch][step];
}

// Helper: get the active scale array for a given mode. Returns size.
static uint8_t getScale(uint8_t mode, const uint8_t** outScale){
  static const uint8_t maj[] = {0,2,4,5,7,9,11,12};
  static const uint8_t min[] = {0,2,3,5,7,8,10,12};
  static const uint8_t pen[] = {0,2,4,7,9,12};
  static const uint8_t loc[] = {0,1,3,5,6,8,10,12};
  static const uint8_t dim[] = {0,1,3,4,6,7,9,10,12};
  static const uint8_t ato[] = {0,1,2,3,4,5,6,7,8,9,10,11,12};
  switch (mode) {
    case 1: *outScale = maj; return 8;
    case 2: *outScale = min; return 8;
    case 3: *outScale = pen; return 6;
    case 4: *outScale = loc; return 8;
    case 5: *outScale = dim; return 9;
    default: *outScale = ato; return 13;
  }
}

// Re-roll only the slide flags based on randomSlideProb. Leaves notes untouched.
// Operates on the current edit page only.
void SimpleSequencer::rerollSlides(uint8_t ch){
  uint8_t prob = randomSlideProb[ch];
  uint16_t base = (uint16_t)editPage[ch] * NUM_STEPS;
  for (uint8_t s = 0; s < NUM_STEPS; s++){
    stepSlide[ch][base + s] = (random(0, 100) < prob);
  }
}

// Transpose every per-step pitch ACROSS ALL PAGES by `semitones`. Root changes
// affect every page on the channel — the melody shifts globally.
void SimpleSequencer::transposeChannelNotes(uint8_t ch, int semitones){
  if (semitones == 0) return;
  for (uint16_t s = 0; s < TOTAL_STEPS; s++){
    if (pitch[ch][s] == 255) continue;
    int n = (int)pitch[ch][s] + semitones;
    pitch[ch][s] = (uint8_t)constrain(n, 0, 127);
  }
}

void SimpleSequencer::randomizeEuclidMelody(uint8_t ch) {
  uint8_t mode = euclidScaleMode[ch];
  uint16_t base = (uint16_t)editPage[ch] * NUM_STEPS;
  if (mode == 0) {
    for (uint8_t s = 0; s < NUM_STEPS; s++) {
      pitch[ch][base + s]       = 255;
      stepSlide[ch][base + s]   = false;
      stepVelocity[ch][base + s] = 255;
      noteLen[ch][base + s]     = 255;
    }
    return;
  }

  uint8_t root = channelPitch[ch];
  const uint8_t* scale;
  uint8_t size = getScale(mode, &scale);

  // octaveSpread now acts in semitones (0..60). For each step we sample a
  // semitone offset above root with exponential bias toward 0, then snap
  // to the nearest scale degree at-or-below that offset so notes stay in key.
  const uint8_t MAX_SPREAD = 60;
  uint8_t spread = octaveSpread[ch];
  if (spread > MAX_SPREAD) spread = MAX_SPREAD;

  // Build the list of valid scale-degree offsets within `spread` semitones.
  uint8_t valid[64];
  uint8_t nValid = 0;
  for (int oct = 0; oct * 12 <= (int)spread && nValid < 64; oct++){
    for (uint8_t i = 0; i < size && nValid < 64; i++){
      int off = oct * 12 + (int)scale[i];
      if (off > (int)spread) break;
      valid[nValid++] = (uint8_t)off;
    }
  }
  if (nValid == 0){ valid[0] = 0; nValid = 1; }

  for (uint8_t s = 0; s < NUM_STEPS; s++) {
    // Exponential bias: pick from valid[] with weight exp(-i/T) where T is half the count.
    // Falls back to uniform when spread = 0.
    uint8_t pick;
    if (spread == 0 || nValid <= 1){
      pick = 0;
    } else {
      float T = (float)nValid * 0.5f;
      float total = 0.0f;
      float cum[64];
      for (uint8_t i = 0; i < nValid; i++){
        total += expf(-(float)i / T);
        cum[i] = total;
      }
      float r = ((float)random(1, 1000001)) * (total / 1000000.0f);
      pick = nValid - 1;
      for (uint8_t i = 0; i < nValid; i++){
        if (r <= cum[i]){ pick = i; break; }
      }
    }
    int note = (int)root + (int)valid[pick];
    pitch[ch][base + s] = (uint8_t)constrain(note, 0, 127);

    stepSlide[ch][base + s]    = (random(0, 100) < randomSlideProb[ch]);
    int v = (int)channelVelocity[ch] + random(-10, 10);
    stepVelocity[ch][base + s] = (uint8_t)constrain(v, 0, 127);
    noteLen[ch][base + s]      = noteLenIdx[ch];
    // Note: steps[] is intentionally NOT touched here. Whether a step fires
    // is the user's rhythm decision (manual toggle or Euclid). Generative
    // mode only paints the pitches.
  }
}

// Shift all euclid-generated notes up/down by "steps" scale degrees for channel ch.
void SimpleSequencer::shiftEuclidNotes(uint8_t ch, int steps){
  uint8_t mode = euclidScaleMode[ch];
  const uint8_t *scale = nullptr;
  uint8_t len = 12; // default chromatic
  static const uint8_t locrian[] = {0,1,3,5,6,8,10};
  static const uint8_t diminished[] = {0,1,3,4,6,7,9,10};
  static const uint8_t atonal[] = {0,1,2,3,4,5,6,7,8,9,10,11};

  if (mode == 1){ scale = locrian; len = sizeof(locrian)/sizeof(locrian[0]); }
  else if (mode == 2){ scale = diminished; len = sizeof(diminished)/sizeof(diminished[0]); }
  else { scale = atonal; len = sizeof(atonal)/sizeof(atonal[0]); }

  // Helper to convert a note to a global index (scale-step count)
  auto noteToGlobalIndex = [&](int note)->int{
    int bestGlobal = (note/12) * len; // fallback
    int bestDist = 1000;
    int baseOct = note / 12;
    for (int oct = baseOct-2; oct <= baseOct+2; oct++){
      for (int i=0;i<(int)len;i++){
        int cand = oct*12 + scale[i];
        if (cand < 0 || cand > 127) continue;
        int d = abs(cand - note);
        if (d < bestDist){ bestDist = d; bestGlobal = oct * len + i; }
      }
    }
    return bestGlobal;
  };

  // Helper to convert global index back to MIDI note
  auto globalIndexToNote = [&](int gidx)->int{
    int oct = gidx / len;
    int idx = gidx % len;
    if (idx < 0) { idx += len; oct -= 1; }
    int cand = oct*12 + scale[idx];
    if (cand < 0) cand = 0;
    if (cand > 127) cand = 127;
    return cand;
  };

  // Shift per-step pitches if present (across all pages)
  for (uint16_t s=0; s<TOTAL_STEPS; s++){
    if (pitch[ch][s] == 255) continue;
    int g = noteToGlobalIndex((int)pitch[ch][s]);
    int ng = g + steps;
    int nn = globalIndexToNote(ng);
    pitch[ch][s] = (uint8_t)nn;
  }

  // Shift channelPitch as well
  int groot = noteToGlobalIndex((int)channelPitch[ch]);
  int ngroot = groot + steps;
  int nroot = globalIndexToNote(ngroot);
  channelPitch[ch] = (uint8_t)nroot;
}



void SimpleSequencer::updateEuclid(uint8_t ch){
  // Apply the euclid rhythm to all pages so the loop is consistent across them.
  uint8_t k = pulses[ch];
  uint8_t n = NUM_STEPS;
  uint8_t offset = euclidOffset[ch];
  for (uint16_t s = 0; s < TOTAL_STEPS; s++) euclidPattern[ch][s] = false;
  if (k == 0) return;
  if (k >= n){
    for (uint16_t s = 0; s < TOTAL_STEPS; s++) euclidPattern[ch][s] = true;
    return;
  }

  bool tempPattern[NUM_STEPS];
  for (uint8_t j=0;j<n;j++){
    int x = (j * k) / n;
    int y = ((j+1) * k) / n;
    tempPattern[j] = (y > x);
  }

  for (uint8_t pg = 0; pg < MAX_PAGES; pg++){
    uint16_t base = (uint16_t)pg * NUM_STEPS;
    for (uint8_t j=0;j<n;j++){
      euclidPattern[ch][base + ((j + offset) % n)] = tempPattern[j];
    }
  }
}

// Advance the internal MIDI tick counter (called from MIDI clock ISR)
void SimpleSequencer::internalClockTick(){
  // increment absolute tick counter
  absoluteTickCounter++;

  // 1) Process tick-based note-offs FIRST so they clear before a new step triggers
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++){
    uint8_t mch = midiChannel[ch] & 0x0F;
    if (noteOffTick[ch] > 0 && absoluteTickCounter >= noteOffTick[ch]){
      if (lastNotePlaying[ch] < 128){
        midiSendNoteOff(mch, lastNotePlaying[ch], 0);
      }
      noteOffTick[ch] = 0;
      lastNotePlaying[ch] = 255;
    }

    // 1b) Process pure-tick Ratchet Note-Ons
    if (ratchetIntervalTicks[ch] > 0 && absoluteTickCounter >= ratchetNextTick[ch]) {
      if (absoluteTickCounter < ratchetEndTick[ch]) {
        // Fire next ratchet
        midiSendNoteOn(mch, ratchetPitch[ch], 100);
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

  // 1) Process any MIDI bytes from hardware MIDI_SERIAL
  while (MIDI_SERIAL.available() > 0){
    uint8_t b = MIDI_SERIAL.read();
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
      globalPage = 0;
      // immediately trigger steps at position 0
      for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
        bool isActive = isStepActive(ch, playIdx(ch, currentStep));
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
        if (lastNotePlaying[ch] < 128){ midiSendNoteOff(midiChannel[ch] & 0x0F, lastNotePlaying[ch], 0); lastNotePlaying[ch] = 255; }
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
      if (currentStep == 0){
        // Just wrapped a full 16-step bar — advance page (mod MAX_PAGES).
        globalPage = (globalPage + 1) % MAX_PAGES;
      }
      // Regenerate machine pattern for any channel whose page changed
      if (currentStep == 0){
        for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++){
          if (trigMachine[ch] != TM_OFF) regenerateMachinePattern(ch);
        }
      }
      // trigger channels that have the step enabled
        for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
          bool isActive = isStepActive(ch, playIdx(ch, currentStep));
          if (isActive) triggerChannel(ch);
        }
    }
  }

  // Note-offs are now handled in `internalClockTick()` on MIDI ticks.

  // Note-offs are now handled in `internalClockTick()` on MIDI ticks.
}

void SimpleSequencer::triggerChannel(uint8_t ch){
  // 1. THE NORMAL MUTE & FILL BLOCK — per-step values resolve via playIdx()
  if (muted[ch]) return;
  uint16_t pIdx = playIdx(ch, currentStep);
  uint8_t fstate = fillState[ch][pIdx];
  if (fstate == 1 && !fillModeActive) return;
  if (fstate == 2 && fillModeActive) return;
  uint8_t p = pitch[ch][pIdx];
  if (p == 255) p = channelPitch[ch];
  uint8_t note = constrain(p, 0, 127);

  uint8_t vel = stepVelocity[ch][pIdx];
  if (vel == 255) vel = channelVelocity[ch];

  // 2. THE MONOSYNTH LEGATO MAGIC — route to per-channel MIDI Out
  static bool prevSlide[NUM_CHANNELS] = {false};
  bool isSlidingIntoThis = prevSlide[ch];
  uint8_t mch = midiChannel[ch] & 0x0F;

  if (lastNotePlaying[ch] < 128) {
    if (isSlidingIntoThis) {
      midiSendNoteOn(mch, note, vel);
      midiSendNoteOff(mch, lastNotePlaying[ch], 0);
    } else {
      midiSendNoteOff(mch, lastNotePlaying[ch], 0);
      midiSendNoteOn(mch, note, vel);
    }
  } else {
    midiSendNoteOn(mch, note, vel);
  }

  lastNotePlaying[ch] = note;
  prevSlide[ch] = stepSlide[ch][pIdx] || encoderSlideHold;

  // 3. RATCHET & GATE LENGTH
  uint8_t lenIdx = noteLen[ch][pIdx];
  if (lenIdx == 255) lenIdx = noteLenIdx[ch];

  uint8_t rIdx = stepRatchet[ch][pIdx];
  // Merge machine-driven ratchets (e.g., kick fill notes) with user P-Locks
  if (trigMachine[ch] != TM_OFF){
    uint8_t mr = machineRatchet[ch][currentStep];
    if (mr > rIdx) rIdx = mr;
  }
  if (rIdx > 0) {
    const uint8_t rTicks[] = {0, 6, 4, 3, 2, 1};
    uint8_t ticksPerHit = rTicks[rIdx];

    ratchetIntervalTicks[ch] = ticksPerHit;
    ratchetNextTick[ch] = absoluteTickCounter + ticksPerHit;
    ratchetEndTick[ch] = absoluteTickCounter + 6; 
    ratchetPitch[ch] = note;
    uint32_t offOffset = ticksPerHit / 2;
    if (offOffset == 0) offOffset = 1;
    noteOffTick[ch] = absoluteTickCounter + offOffset;


  } else {
    // Normal single-hit logic
    ratchetIntervalTicks[ch] = 0;
    uint32_t ticks = noteLenTicks[lenIdx];
    uint32_t gateLength;

    if (stepSlide[ch][pIdx]) {
      gateLength = (ticks < 7) ? 7 : (ticks + 1);
    } else {
      gateLength = (ticks > 1) ? (ticks - 1) : 1;
    }
    noteOffTick[ch] = absoluteTickCounter + gateLength;


  }
}

// CV/Gate functions removed; using MIDI out only

void SimpleSequencer::drawDisplay(){
  // Transport splash — overlays everything else for ~600ms after a play/stop edge
  uint32_t nowMs = millis();
  if (transportAnimEndMs && nowMs < transportAnimEndMs){
    // Full-screen inverse splash so it's unmistakable
    display.clearDisplay();
    display.fillRect(0, 0, 128, 64, SH110X_WHITE);
    // Icon on the left, text fits within the right side (4 chars * 18px = 72px).
    // Cursor at x=54 puts visible glyphs in 54..120, well inside 128.
    if (transportAnimIsPlay){
      display.fillTriangle(12, 12, 46, 32, 12, 52, SH110X_BLACK);
      display.setTextColor(SH110X_BLACK);
      display.setTextSize(3);
      display.setCursor(54, 22);
      display.print("PLAY");
    } else {
      display.fillRect(14, 16, 32, 32, SH110X_BLACK);
      display.setTextColor(SH110X_BLACK);
      display.setTextSize(3);
      display.setCursor(54, 22);
      display.print("STOP");
    }
    display.display();
    return;
  } else if (transportAnimEndMs && nowMs >= transportAnimEndMs){
    transportAnimEndMs = 0;
  }

  // Fill-mark splash — shows clearly which step was just toggled
  if (fillAnimEndMs && nowMs < fillAnimEndMs){
    display.clearDisplay();
    display.drawRect(0, 0, 128, 64, SH110X_WHITE);
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(2);
    display.setCursor(8, 6);
    display.print("STEP "); display.print(fillAnimStep + 1);
    display.setTextSize(3);
    display.setCursor(8, 30);
    display.print(fillAnimSet ? "FILL" : "NORM");
    if (fillAnimSet){
      display.setTextSize(1);
      display.setCursor(80, 38);
      display.print("on FILL");
      display.setCursor(80, 48);
      display.print("only");
    }
    display.display();
    return;
  } else if (fillAnimEndMs && nowMs >= fillAnimEndMs){
    fillAnimEndMs = 0;
  }

  // Mute toggle splash
  if (muteAnimEndMs && nowMs < muteAnimEndMs){
    display.clearDisplay();
    display.drawRect(0, 0, 128, 64, SH110X_WHITE);
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(2);
    display.setCursor(8, 6);
    display.print("CH "); display.print(muteAnimCh + 1);
    display.setTextSize(3);
    display.setCursor(8, 30);
    display.print(muteAnimMuted ? "MUTE" : "ON");
    if (muteAnimMuted){
      // Strikethrough decorate
      display.drawLine(8, 44, 80, 44, SH110X_WHITE);
    }
    display.display();
    return;
  } else if (muteAnimEndMs && nowMs >= muteAnimEndMs){
    muteAnimEndMs = 0;
  }

  // Clear-track splash
  if (clearAnimEndMs && nowMs < clearAnimEndMs){
    display.clearDisplay();
    display.fillRect(0, 0, 128, 64, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setTextSize(3);
    display.setCursor(8, 8);
    display.print("CLEAR");
    display.setTextSize(2);
    display.setCursor(8, 40);
    display.print("CH "); display.print(clearAnimCh + 1);
    display.display();
    return;
  } else if (clearAnimEndMs && nowMs >= clearAnimEndMs){
    clearAnimEndMs = 0;
  }

  // Function + Pot1 BPM splash: shows current BPM while editing
  if (bpmFocusEndMs && nowMs < bpmFocusEndMs){
    display.clearDisplay();
    display.drawRect(0, 0, 128, 64, SH110X_WHITE);
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(2);
    display.setCursor(8, 6);
    display.print("BPM");
    display.setTextSize(4);
    display.setCursor(8, 28);
    display.print(bpm);
    display.display();
    return;
  } else if (bpmFocusEndMs && nowMs >= bpmFocusEndMs){
    bpmFocusEndMs = 0;
  }

  // Channel-held focus: shows the channel's mute state and MIDI Out channel
  if (heldChannel >= 0 && heldChannel < (int)NUM_CHANNELS){
    display.clearDisplay();
    display.drawRect(0, 0, 128, 64, SH110X_WHITE);
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(2);
    display.setCursor(8, 4);
    display.print("CH "); display.print(heldChannel + 1);
    display.setTextSize(1);
    display.setCursor(70, 4);
    display.print(muted[heldChannel] ? "MUTED" : "ON");
    display.setCursor(8, 28);
    display.print("MIDI OUT");
    display.setTextSize(3);
    display.setCursor(8, 40);
    display.print("ch ");
    display.print((int)midiChannel[heldChannel] + 1);
    display.setTextSize(1);
    display.setCursor(78, 56);
    display.print("(Pot1)");
    display.display();
    return;
  }

  if (activeMenu == 1){ drawNotesView(); return; }
  if (activeMenu == 2){ drawEuclidView(); return; }
  if (activeMenu == 3){ drawStepVisualiser(); return; }
  if (activeMenu == 4){ drawTrigMachineView(); return; }
  if (activeMenu == 5){ drawPagesView();      return; }

  display.clearDisplay();

  uint32_t now = millis();
  bool focused = (focusEncoder != 0) && ((now - lastEncoderMoveTime) < focusTimeout);

  // Global visual for Encoder 3 held slide-all
  if (encoderSlideHold) {
    display.setTextSize(2);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(16, 24);
    display.print("SLIDE ALL");
    display.display();
    updateLEDs();
    return;
  }


  const char* noteNames[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  const char* scaleNames[] = {"OFF", "LOC", "DIM", "ATO"};
  const char* ratchetNames[] = {"OFF", "1/16", "1/24", "1/32", "1/48", "1/96"};

  // ── DEBUG MODE: Hold both FN + FILL to show full grid ──────────
  bool debugHold = isFunctionHeld() && isFillHeld();
  if (debugHold){
    drawDebugGrid();
    // Thin status line at top
    display.fillRect(0, 0, 128, 10, SH110X_BLACK);
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(2, 1);
    display.print("DEBUG  CH"); display.print(selectedChannel + 1);
    display.setCursor(80, 1);
    display.print("BPM "); display.print(bpm);
    display.display();
    updateLEDs();
    return;
  }

  // Global Fill indicator (small vertical bar at top-right)
  if (fillModeActive) {
    // Draw a compact white bar to indicate Fill is active without taking space
    display.fillRect(120, 2, 6, 10, SH110X_WHITE);
  }

  // ── FOCUSED ENCODER VIEWS ─────────────────────────────────────
  if (focused){
    uint8_t fe = focusEncoder - 1;

    // ── ENCODER 1 ────────────────────────────────────────────────
    if (fe == 0){
      if (heldStep >= 0){
        // P-LOCK: Full-screen retrig rate
        uint8_t r = stepRatchet[selectedChannel][heldStep];
        display.setTextSize(2);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(4, 2);
        display.print("RETRIG");
        display.setTextSize(1);
        display.setCursor(90, 6);
        display.print("STP "); display.print(heldStep + 1);
        display.setTextSize(4);
        display.setCursor(4, 26);
        display.print(ratchetNames[r]);
      } else {
        // BPM — big
        display.setTextSize(2);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(4, 2);
        display.print("BPM");
        display.setTextSize(4);
        display.setCursor(4, 26);
        display.print(bpm);
      }
      display.display();
      updateLEDs();
      return;
    }

    // ── ENCODER 2 ────────────────────────────────────────────────
    if (fe == 1){
      if (heldStep >= 0){
        bool startHeld = isStartHeld();
        if (startHeld) {
          // ACCENT UI
          uint8_t v = stepVelocity[selectedChannel][heldStep];
          if (v == 255) v = channelVelocity[selectedChannel];
          display.setTextSize(2); display.setTextColor(SH110X_WHITE);
          display.setCursor(4, 2); display.print("ACCENT");
          display.setTextSize(1); display.setCursor(90, 6);
          display.print("STP "); display.print(heldStep + 1);
          display.setTextSize(4); display.setCursor(4, 26);
          display.print(v);
        } else {
          // PITCH UI
          uint8_t p = pitch[selectedChannel][heldStep];
          if (p == 255) p = channelPitch[selectedChannel];
          display.setTextSize(2); display.setTextColor(SH110X_WHITE);
          display.setCursor(4, 2); display.print("NOTE");
          display.setTextSize(1); display.setCursor(90, 6);
          display.print("STP "); display.print(heldStep + 1);
          display.setTextSize(4); display.setCursor(4, 26);
          display.print(noteNames[p % 12]);
          display.print((p / 12) - 1);
        }
      } else if (euclidEnabled[selectedChannel]){
        // Euclid scale shift — show grid + shift info
        drawDebugGrid();
        display.fillRect(0, 0, 128, 12, SH110X_BLACK);
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(2, 2);
        display.print("SHIFT ");
        display.print(noteNames[channelPitch[selectedChannel] % 12]);
        display.print((channelPitch[selectedChannel] / 12) - 1);
        display.setCursor(80, 2);
        display.print("SCL:");
        display.print(scaleNames[euclidScaleMode[selectedChannel] % 4]);
      } else {
        // Channel note — big
        uint8_t cp = channelPitch[selectedChannel];
        display.setTextSize(2);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(4, 2);
        display.print("NOTE");
        display.setTextSize(4);
        display.setCursor(4, 26);
        display.print(noteNames[cp % 12]);
        display.print((cp / 12) - 1);
      }
      display.display();
      updateLEDs();
      return;
    }

    // ── ENCODER 3 ────────────────────────────────────────────────
    if (fe == 2){
      if (heldStep >= 0){
        bool startHeld = isStartHeld();
        if (startHeld) {
          // SLIDE UI
          display.setTextSize(2); display.setTextColor(SH110X_WHITE);
          display.setCursor(4, 2); display.print("SLIDE");
          display.setTextSize(1); display.setCursor(90, 6);
          display.print("STP "); display.print(heldStep + 1);
          display.setTextSize(4); display.setCursor(4, 26);
          display.print(stepSlide[selectedChannel][heldStep] ? "ON" : "OFF");
        } else {
          // GATE UI
          uint8_t lenIdx = noteLen[selectedChannel][heldStep];
          if (lenIdx == 255) lenIdx = noteLenIdx[selectedChannel];
          display.setTextSize(2); display.setTextColor(SH110X_WHITE);
          display.setCursor(4, 2); display.print("GATE");
          display.setTextSize(1); display.setCursor(90, 6);
          display.print("STP "); display.print(heldStep + 1);
          display.setTextSize(4); display.setCursor(4, 26);
          display.print(noteLenNames[lenIdx]);
        }
      } else {
        // Channel default gate length — big
        display.setTextSize(2);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(4, 2);
        display.print("GATE");
        display.setTextSize(4);
        display.setCursor(4, 26);
        display.print(noteLenNames[noteLenIdx[selectedChannel]]);
      }
      display.display();
      updateLEDs();
      return;
    }

    // ── ENCODER 4 ────────────────────────────────────────────────
    if (fe == 3){
      if (euclidEnabled[selectedChannel]){
        // Euclid active — show grid + params
        drawDebugGrid();
        display.fillRect(0, 0, 128, 12, SH110X_BLACK);
        display.setTextSize(1);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(2, 2);
        display.print("EUCLID");
        display.setCursor(48, 2);
        display.print("H:"); display.print(pulses[selectedChannel]);
        display.setCursor(80, 2);
        display.print("O:"); display.print(euclidOffset[selectedChannel]);
      } else {
        // Euclid off
        display.setTextSize(2);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(4, 2);
        display.print("EUCLID");
        display.setTextSize(3);
        display.setCursor(4, 28);
        display.print("OFF");
      }
      display.display();
      updateLEDs();
      return;
    }
  }

  // ── DEFAULT OVERVIEW ──────────────────────────────────────────
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  // Row 1: Channel indicator boxes (mute state)
  for (uint8_t c = 0; c < NUM_CHANNELS; c++){
    // 7 channels across 128px: each tab is 17px wide with 1px gap = 125px total
    int bx = 2 + c * 17;
    if (c == selectedChannel){
      display.fillRect(bx, 0, 16, 11, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK, SH110X_WHITE);
    } else {
      if (muted[c]){
        display.drawRect(bx, 0, 16, 11, SH110X_WHITE);
        display.drawLine(bx, 5, bx + 14, 5, SH110X_WHITE);
      } else {
        display.drawRect(bx, 0, 16, 11, SH110X_WHITE);
      }
      display.setTextColor(SH110X_WHITE, SH110X_BLACK);
    }
    display.setTextSize(1);
    display.setCursor(bx + 3, 2);
    display.print(c + 1);
  }
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  // Row 2: Channel note + note length (note length slightly smaller)
  uint8_t cp = channelPitch[selectedChannel];
  display.setTextSize(3);
  display.setCursor(4, 16);
  display.print(noteNames[cp % 12]); display.print((cp / 12) - 1);
  // Note length: reduce font to avoid awkward overflow
  display.setTextSize(2);
  display.setCursor(76, 18);
  display.print(noteLenNames[noteLenIdx[selectedChannel]]);

  // Row 3: Euclid status (BPM tucked bottom-right)
  display.setTextSize(1);
  // Place BPM a bit more left to avoid wrapping/overlap
  display.setCursor(84, 44);
  display.print("BPM "); display.print(bpm);

  if (euclidEnabled[selectedChannel]){
    display.setCursor(52, 44);
    display.print("EUC H:"); display.print(pulses[selectedChannel]);
    display.print(" O:"); display.print(euclidOffset[selectedChannel]);
  }

  // Row 4: Scale + P-lock indicator (always show scale)
  display.setCursor(4, 55);
  display.print("SCL:"); display.print(scaleNames[euclidScaleMode[selectedChannel] % 4]);
  if (heldStep >= 0){
    display.setCursor(100, 55);
    display.print("P:"); display.print(heldStep + 1);
    // Show per-step P-Lock VEL and SLD
    uint8_t v = stepVelocity[selectedChannel][heldStep];
    if (v == 255) v = channelVelocity[selectedChannel];
    display.setCursor(4, 55);
    display.print("VEL:"); display.print(v);
    display.setCursor(52, 55);
    display.print("SLD:"); display.print(stepSlide[selectedChannel][heldStep] ? "ON" : "OFF");
  }

  display.display();
  updateLEDs();
}

// LEDs disabled: no-op implementation so calls are safe during bring-up
void SimpleSequencer::updateLEDs(){
  // One LED per step. Show the active channel's pattern + playhead.
  // Per-channel hue distinguishes which channel is selected.
  // Hues spaced widely to be visually distinct on WS2812s.
  static const uint32_t channelColors[NUM_CHANNELS] = {
    0xFF0000, // CH1 red
    0xFF6000, // CH2 amber
    0xE0D000, // CH3 yellow
    0x00C040, // CH4 green
    0x00B0B0, // CH5 teal
    0x0060FF, // CH6 blue
    0xC000FF  // CH7 magenta
  };
  uint32_t chCol = channelColors[selectedChannel % NUM_CHANNELS];
  uint8_t cr = (chCol >> 16) & 0xFF;
  uint8_t cg = (chCol >> 8) & 0xFF;
  uint8_t cb = chCol & 0xFF;

  // Slow pulse phase for blinking decorations (0..255 sine-ish ramp)
  uint8_t pulse = (uint8_t)((millis() / 4) & 0xFF);
  uint8_t pulseBri = (pulse < 128) ? (pulse * 2) : (255 - (pulse - 128) * 2);

  for (uint8_t s = 0; s < LED_COUNT && s < NUM_STEPS; s++){
    bool active = isStepActive(selectedChannel, editIdx(selectedChannel, s));
    bool isPlayhead = isRunning && (s == currentStep);

    if (isPlayhead){
      // Playhead: white normally, green when Fill is held (so you see when fill is active)
      if (fillModeActive){
        ledStrip.setPixelColor(s, ledStrip.Color(0, 255, 80));
      } else {
        ledStrip.setPixelColor(s, ledStrip.Color(255, 255, 255));
      }
    } else if (active){
      ledStrip.setPixelColor(s, ledStrip.Color(cr, cg, cb));
    } else {
      // Dim hint so the grid is visible even at rest
      ledStrip.setPixelColor(s, ledStrip.Color(cr / 16, cg / 16, cb / 16));
    }

    // Decorations for Menu 4 overlay state and fill marks
    uint16_t eI = editIdx(selectedChannel, s);
    if (activeMenu == 4 && trigMachine[selectedChannel] != TM_OFF){
      uint8_t ov = machineOverlay[selectedChannel][eI];
      if (!isPlayhead){
        if (ov == 1){
          ledStrip.setPixelColor(s, ledStrip.Color(180, 180, 180));
        } else if (ov == 2){
          ledStrip.setPixelColor(s, ledStrip.Color(40, 0, 0));
        }
      }
    } else if (fillState[selectedChannel][eI] == 1 && !isPlayhead){
      // Fill-only step: solid green when Fill is held (the step will trigger),
      // gentle pulse green when Fill not held (so you can see fills exist).
      if (fillModeActive){
        ledStrip.setPixelColor(s, ledStrip.Color(0, 255, 60));
      } else {
        uint8_t g = 40 + (pulseBri / 3); // 40..125 pulsing
        ledStrip.setPixelColor(s, ledStrip.Color(0, g, 0));
      }
      if (active && fillModeActive){
        ledStrip.setPixelColor(s, ledStrip.Color(0, 255, 0));
      }
    }
  }
  ledStrip.show();
}

void SimpleSequencer::drawDebugGrid(){
  // replicate previous grid drawing for debugging
  const int stepW = 12, stepH = 12, startX = 6, startY = 16, spacingX = 3, spacingY = 4;
  for (uint8_t i = 0; i < NUM_STEPS; i++){
    int col = i % 8; int row = i / 8;
    int x = startX + col * (stepW + spacingX);
    int y = startY + row * (stepH + spacingY);
    bool stepActive = euclidEnabled[selectedChannel] ? euclidPattern[selectedChannel][i] : steps[selectedChannel][i];
    if (stepActive){ 
      display.fillRect(x, y, stepW, stepH, SH110X_WHITE);
      uint8_t fs = fillState[selectedChannel][i];
      if (fs == 1) display.fillRect(x+3, y+3, stepW-6, stepH-6, SH110X_BLACK);
      else if (fs == 2){ display.drawRect(x+2, y+2, stepW-4, stepH-4, SH110X_WHITE); display.fillRect(x+4, y+4, 4, 4, SH110X_WHITE); }
    }
    else display.drawRect(x, y, stepW, stepH, SH110X_WHITE);
    if (i == currentStep){ display.drawFastHLine(x, y + stepH + 2, stepW, SH110X_WHITE); display.drawFastHLine(x, y + stepH + 3, stepW, SH110X_WHITE); }
  }
}

void SimpleSequencer::runSwitchTest(uint32_t ms){
  Serial.print("Starting switch test for "); Serial.print(ms); Serial.println(" ms");
  Serial.println("Press buttons to see state changes. Press 'e' to jump to pot test, or any other key to cancel.");
  // drop any stale queued chars before starting
  while (Serial.available()) Serial.read();
  // Build initial snapshot by scanning entire matrix
  bool lastState[MATRIX_KEYS];
  for (uint8_t k=0;k<MATRIX_KEYS;k++) lastState[k] = false;
    for (uint8_t c=0;c<MATRIX_COLS;c++){
      digitalWrite(MATRIX_COL_PINS[c], MATRIX_COL_ACTIVE);
      delayMicroseconds(30);
      for (uint8_t r=0;r<MATRIX_ROWS;r++){
        uint8_t idx = matrixIndex(r,c);
        lastState[idx] = (digitalRead(MATRIX_ROW_PINS[r]) == HIGH);
      }
      digitalWrite(MATRIX_COL_PINS[c], MATRIX_COL_IDLE);
    }
  bool lastStart = isStartHeld();
  uint32_t start = millis();
  while (millis() - start < ms){
    if (Serial.available()){
      char cmd = Serial.read();
      if (cmd == 'e' || cmd == 'E'){
        Serial.println("Switch test interrupted -> pot-button test");
        while (Serial.available()) Serial.read();
        runEncoderSwitchTest(10000);
        return;
      }
      Serial.println("Switch test cancelled");
      while (Serial.available()) Serial.read();
      return;
    }
    // full matrix scan (blocking) for test
    for (uint8_t c=0;c<MATRIX_COLS;c++){
      digitalWrite(MATRIX_COL_PINS[c], MATRIX_COL_ACTIVE);
      delayMicroseconds(30);
      for (uint8_t r=0;r<MATRIX_ROWS;r++){
        uint8_t idx = matrixIndex(r,c);
        bool s = (digitalRead(MATRIX_ROW_PINS[r]) == HIGH);
        if (s != lastState[idx]){
          Serial.print("Button "); Serial.print(idx); Serial.print(s?" pressed":" released"); Serial.println();
          digitalWrite(LED_BUILTIN, HIGH);
          delay(30);
          digitalWrite(LED_BUILTIN, LOW);
          lastState[idx] = s;
        }
      }
      digitalWrite(MATRIX_COL_PINS[c], MATRIX_COL_IDLE);
    }
    // start/stop button (matrix-mapped)
    bool sr = isStartHeld();
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
  // Legacy name kept for compatibility; now uses exact teensi.ino debounce logic.
  Serial.print("Starting pot-button test for "); Serial.print(ms); Serial.println(" ms");
  Serial.println("Press pot buttons to see state changes.");

  const uint8_t POT_BTN_COUNT = sizeof(POT_BTN_PINS)/sizeof(POT_BTN_PINS[0]);
  bool potBtnState[6] = {0};
  bool lastPotBtnState[6] = {0};
  unsigned long lastPotBtnChange[6] = {0};
  const unsigned long POT_BTN_DEBOUNCE_MS = 10;

  // initialize last raw states
  for (uint8_t i = 0; i < POT_BTN_COUNT; i++) {
    bool pressed = (digitalRead(POT_BTN_PINS[i]) == LOW);
    lastPotBtnState[i] = pressed;
    potBtnState[i] = pressed;
    lastPotBtnChange[i] = millis();
  }

  uint32_t start = millis();
  while (millis() - start < ms){
    for (uint8_t i = 0; i < POT_BTN_COUNT; i++) {
      bool pressed = (digitalRead(POT_BTN_PINS[i]) == LOW); // Active LOW
      if (pressed != lastPotBtnState[i]) {
        lastPotBtnChange[i] = millis();
        lastPotBtnState[i] = pressed;
      } else if (pressed != potBtnState[i]) {
        if (millis() - lastPotBtnChange[i] >= POT_BTN_DEBOUNCE_MS) {
          potBtnState[i] = pressed;
          if (pressed) {
            Serial.print("Potentiometer Button ");
            Serial.print(i + 1);
            Serial.println(" pressed");
          } else {
            Serial.print("Potentiometer Button ");
            Serial.print(i + 1);
            Serial.println(" released");
          }
        }
      }
    }
    delay(1);
  }
  Serial.println("Pot-button test finished");
}

void SimpleSequencer::printEncoderRaw(){
  // Legacy name kept for compatibility; print raw potentiometer readings.
  Serial.println("Pot raw states (A B BTN):");
  const uint8_t potCount = sizeof(POT_A_PINS)/sizeof(POT_A_PINS[0]);
  for (uint8_t e=0;e<potCount;e++){
    int a = analogRead(POT_A_PINS[e]);
    int b = analogRead(POT_B_PINS[e]);
    int sw = digitalRead(POT_BTN_PINS[e]);
    Serial.print("Pot"); Serial.print(e+1); Serial.print(": ");
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

void SimpleSequencer::bootAnimation() {
  // Spiral boot animation — restored from the original design.
  // Screen 1 draws a fractal spiral in white-on-black; screen 2 mirrors it
  // horizontally and inverts the colours (black-on-white).
  // LEDs pulse out from the centre in red or blue at max brightness.
  display.clearDisplay();
  if (display2Present){
    display2.clearDisplay();
    display2.fillRect(0, 0, 128, 64, SH110X_WHITE); // white background
  }
  ledStrip.clear();
  ledStrip.setBrightness(255); // max brightness for boot
  randomSeed(analogRead(0));

  // --- LED zones (mirror around the centre of the 16-LED strip) ---
  bool useRed = (random(0, 2) == 0);
  const uint8_t spread[4][4] = {
    {3, 4, 11, 12}, // Zone 0: centre
    {2, 5, 10, 13}, // Zone 1: mid-inner
    {1, 6, 9, 14},  // Zone 2: mid-outer
    {0, 7, 8, 15}   // Zone 3: outer edges
  };

  // --- Spiral DNA: random each boot for a unique look ---
  const int cx = 64, cy = 32;
  int branches = random(2, 6);
  float angleStep    = random(5, 20) / 100.0f;
  float radiusStep   = random(10, 50) / 100.0f;
  float fractalTwist = random(10, 50) / 10.0f;
  float angle = 0, radius = 0;

  // ── MAIN LOOP: 150 frames at ~12ms each (~1.8s) ─────────────────
  for (int frame = 0; frame < 150; frame++){
    // 1) LED pulse: a peak sweeps from centre (d=0) to edge (d=3) and back.
    float peak = 1.5f - 1.5f * cosf(frame * (TWO_PI / 150.0f));
    float globalFade = 1.0f;
    if (frame > 120) globalFade = 1.0f - ((frame - 120) / 30.0f);
    for (int d = 0; d < 4; d++){
      float dist = fabsf(peak - (float)d);
      float intensity = constrain(1.0f - (dist * 0.7f), 0.0f, 1.0f);
      int val = (int)(255.0f * intensity * intensity * intensity * globalFade);
      uint8_t r = useRed ? (uint8_t)val : (uint8_t)((val * 180) / 255);
      uint8_t b = useRed ? 0 : (uint8_t)val;
      for (int i = 0; i < 4; i++){
        ledStrip.setPixelColor(spread[d][i], ledStrip.Color(r, 0, b));
      }
    }
    ledStrip.show();

    // 2) Spiral geometry — two iterations per frame for density.
    for (int iter = 0; iter < 2; iter++){
      angle  += angleStep;
      radius += radiusStep;
      for (int b_idx = 0; b_idx < branches; b_idx++){
        float armAngle = angle + (b_idx * (TWO_PI / branches));
        int x  = cx + (int)(radius * cosf(armAngle));
        int y  = cy + (int)(radius * sinf(armAngle));
        int fx = x + (int)((radius * 0.3f) * cosf(armAngle * fractalTwist));
        int fy = y + (int)((radius * 0.3f) * sinf(armAngle * fractalTwist));
        // Screen 1: white on black
        display.drawPixel(x,  y,  SH110X_WHITE);
        display.drawPixel(fx, fy, SH110X_WHITE);
        // Screen 2: mirrored horizontally, black on white
        if (display2Present){
          display2.drawPixel(127 - x,  y,  SH110X_BLACK);
          display2.drawPixel(127 - fx, fy, SH110X_BLACK);
        }
      }
    }

    // Push frames every other tick to avoid choking the I2C buses.
    if (frame % 2 == 0){
      display.display();
      if (display2Present) display2.display();
    }
    delay(12);
  }

  delay(400);

  // ── FINALE: clear and show a small boot card on both screens ──────
  display.clearDisplay();
  if (display2Present) display2.clearDisplay();
  ledStrip.clear();
  ledStrip.show();

  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(44, 20); display.print("seq-23");
  display.setCursor(16, 32); display.print("made by Bob and Zak");
  display.setCursor(28, 44); display.print("v. prototype");
  display.display();

  if (display2Present){
    display2.fillRect(0, 0, 128, 64, SH110X_WHITE);
    display2.setTextColor(SH110X_BLACK);
    display2.setTextSize(1);
    display2.setCursor(44, 20); display2.print("seq-23");
    display2.setCursor(16, 32); display2.print("made by Bob and Zak");
    display2.setCursor(28, 44); display2.print("v. prototype");
    display2.display();
  }
  delay(900);

  // Restore normal LED brightness for runtime
  ledStrip.setBrightness(LED_BRIGHTNESS);
  ledStrip.clear();
  ledStrip.show();
  display.clearDisplay();
  display.display();
  if (display2Present){
    display2.clearDisplay();
    display2.display();
  }
}


void SimpleSequencer::clearTrack(uint8_t ch) {
  // Clear every page slot for this channel
  for (uint16_t s = 0; s < TOTAL_STEPS; s++) {
    steps[ch][s] = false;
    pitch[ch][s] = 255;
    noteLen[ch][s] = 255;
    fillState[ch][s] = 0;
    stepRatchet[ch][s] = 0;
    stepVelocity[ch][s] = 255;
    stepSlide[ch][s] = false;
    euclidPattern[ch][s] = false;
    machineOverlay[ch][s] = 0;
  }
  for (uint8_t s = 0; s < NUM_STEPS; s++){
    machinePattern[ch][s] = false;
    machineRatchet[ch][s] = 0;
  }
  numPages[ch] = 1;
  editPage[ch] = 0;
  euclidEnabled[ch] = false;
  euclidScaleMode[ch] = 0;
  pulses[ch] = 4;
  euclidOffset[ch] = 0;
  trigMachine[ch] = TM_OFF;
  trigDensity[ch] = 50;
  trigShift[ch] = 0;
  kickNoteSpread[ch] = 0;
  kickRatchetProb[ch] = 0;
  kickExtrasAreFills[ch] = 0;
  // Silence any sustaining note on this channel
  if (lastNotePlaying[ch] < 128){
    midiSendNoteOff(midiChannel[ch] & 0x0F, lastNotePlaying[ch], 0);
    lastNotePlaying[ch] = 255;
  }
  noteOffTick[ch] = 0;
  ratchetIntervalTicks[ch] = 0;
  // Splash for visual feedback
  clearAnimCh = ch;
  clearAnimEndMs = millis() + 700;
}

void SimpleSequencer::drawNotesView(){
  // Menu 1: Generative performance page for the selected channel
  display.clearDisplay();
  const char* noteNames[]   = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  const char* scaleNames[]  = {"OFF","MAJOR","MINOR","PENTA","LOCR","DIM","ATONL"};
  uint8_t ch  = selectedChannel;
  uint8_t sm  = euclidScaleMode[ch];
  if (sm > 6) sm = 6;
  bool genOn = (sm != 0);

  // ── CHANNEL STRIP: all 7 with selected + mute state ─────────
  for (uint8_t c = 0; c < NUM_CHANNELS; c++){
    int bx = 2 + c * 17;
    bool sel = (c == selectedChannel);
    if (sel){
      display.fillRect(bx, 0, 16, 10, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.drawRect(bx, 0, 16, 10, SH110X_WHITE);
      display.setTextColor(SH110X_WHITE);
    }
    display.setTextSize(1);
    display.setCursor(bx + 4, 1);
    display.print(c + 1);
    if (muted[c]){
      display.drawLine(bx + 1, 5, bx + 14, 5,
                       sel ? SH110X_BLACK : SH110X_WHITE);
    }
  }
  display.setTextColor(SH110X_WHITE);

  // ── ROOT + SCALE ROW ────────────────────────────────────────
  uint8_t p = channelPitch[ch];
  display.setTextSize(2);
  display.setCursor(2, 13);
  display.print(noteNames[p % 12]); display.print((p / 12) - 1);

  // Scale name in size-1 so it doesn't fight the GEN chip for the right side
  display.setTextSize(1);
  display.setCursor(44, 17);
  display.print(scaleNames[sm]);

  // GEN / OFF chip (top right) — pulled in 4px so it stops at x=124
  if (genOn){
    display.fillRect(102, 12, 20, 11, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setTextSize(1);
    display.setCursor(104, 14);
    display.print("GEN");
    display.setTextColor(SH110X_WHITE);
  } else {
    display.drawRect(102, 12, 20, 11, SH110X_WHITE);
    display.setTextSize(1);
    display.setCursor(104, 14);
    display.print("OFF");
  }
  display.drawFastHLine(2, 31, 124, SH110X_WHITE);

  // ── PARAM ROWS ──────────────────────────────────────────────
  display.setTextSize(1);
  display.setCursor(2, 35);
  display.print("SLD:"); display.print(randomSlideProb[ch]); display.print("%");
  display.setCursor(64, 35);
  display.print("SPRD:"); display.print(octaveSpread[ch]);

  display.setCursor(2, 46);
  display.print("VEL:"); display.print(channelVelocity[ch]);
  display.setCursor(64, 46);
  display.print("GT:"); display.print(noteLenNames[noteLenIdx[ch]]);

  display.setCursor(2, 57);
  display.print("BPM:"); display.print(bpm);

  display.display();
}

void SimpleSequencer::drawEuclidView(){
  // Menu 2: show euclid params + per-channel mute state for all tracks
  display.clearDisplay();
  uint32_t now = millis();

  // ── TOP BAR: 7 channel tabs with selected + mute state ───────────
  // Each chip 17px wide, 11 tall
  for (uint8_t c = 0; c < NUM_CHANNELS; c++){
    int bx = 2 + c * 17;
    bool sel = (c == selectedChannel);
    if (sel){
      display.fillRect(bx, 0, 16, 11, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.drawRect(bx, 0, 16, 11, SH110X_WHITE);
      display.setTextColor(SH110X_WHITE);
    }
    display.setTextSize(1);
    display.setCursor(bx + 4, 2);
    display.print(c + 1);
    if (muted[c]){
      display.drawLine(bx + 1, 5, bx + 14, 5,
                       sel ? SH110X_BLACK : SH110X_WHITE);
    }
  }
  display.setTextColor(SH110X_WHITE);

  // ── PARAMS ROW ───────────────────────────────────────────────────
  display.setTextSize(1);
  display.setCursor(2, 14);
  display.print("H:"); display.print(pulses[selectedChannel]);
  display.setCursor(28, 14);
  display.print("O:"); display.print(euclidOffset[selectedChannel]);
  display.setCursor(56, 14);
  const char* scaleNames[] = {"OFF","MAJ","MIN","PEN","LOC","DIM","ATO"};
  uint8_t sm = euclidScaleMode[selectedChannel];
  if (sm > 6) sm = 6;
  display.print("S:"); display.print(scaleNames[sm]);
  display.setCursor(96, 14);
  display.print(euclidEnabled[selectedChannel] ? "ON" : "OFF");

  // ── PATTERN GRID: 16 steps as small squares ──────────────────────
  // sq=7 + gap=1 = 8 per cell. 16 cells = 128 — at the edge. Shrink to sq=6
  // (gap stays 1) so the grid is 16*7 - 1 = 111px, sitting inside startX=4.
  const uint8_t sq = 6, gap = 1, startX = 4, startY = 26;
  for (uint8_t s = 0; s < NUM_STEPS; s++){
    int x = startX + s * (sq + gap);
    bool active = euclidEnabled[selectedChannel]
                  ? euclidPattern[selectedChannel][s]
                  : steps[selectedChannel][s];
    bool isHead = isRunning && (s == currentStep);
    if (isHead){
      display.fillRect(x, startY, sq, sq, SH110X_WHITE);
      if ((now / 125) % 2 == 0)
        display.fillRect(x+2, startY+2, 3, 3, SH110X_BLACK);
    } else if (active){
      display.fillRect(x, startY, sq, sq, SH110X_WHITE);
    } else {
      display.drawRect(x, startY, sq, sq, SH110X_WHITE);
    }
  }

  // ── VEL + GATE + SLD ROW ─────────────────────────────────────────
  display.setCursor(2, 38);
  display.print("Vel:"); display.print(channelVelocity[selectedChannel]);
  display.setCursor(56, 38);
  display.print("Gate:"); display.print(noteLenNames[noteLenIdx[selectedChannel]]);
  display.setCursor(2, 46);
  display.print("Sld:"); display.print(randomSlideProb[selectedChannel]); display.print("%");

  // ── BOTTOM: spinner + BPM ────────────────────────────────────────
  const char spinFrames[] = {'-','\\','|','/'};
  display.setCursor(2, 56);
  display.print(isRunning ? spinFrames[(now/120)%4] : '.');
  display.setCursor(84, 56);
  display.print("BPM:"); display.print(bpm);

  display.display();
}

void SimpleSequencer::drawStepVisualiser(){
  // Original single-channel grid layout, restored.
  display.clearDisplay();
  uint32_t now = millis();

  // ── TOP BAR: 7 channel tabs with selected + mute state ───────────
  for (uint8_t c = 0; c < NUM_CHANNELS; c++){
    int bx = 2 + c * 17;
    bool isSelected = (c == selectedChannel);
    bool isMuted    = muted[c];
    if (isSelected){
      display.fillRect(bx, 0, 16, 9, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.drawRect(bx, 0, 16, 9, SH110X_WHITE);
      display.setTextColor(SH110X_WHITE);
      if (isMuted){
        display.drawLine(bx+1, 4, bx+14, 4, SH110X_WHITE);
      }
    }
    display.setTextSize(1);
    display.setCursor(bx + 3, 1);
    display.print(c + 1);
  }
  display.setTextColor(SH110X_WHITE);

  // ── STEP GRID: 16 steps in 2 rows of 8 ───────────────────────
  // Each cell is 14px wide x 16px tall with 2px gap. gridX=1 to fit cleanly.
  const uint8_t cellW = 14, cellH = 16, gapX = 2, gapY = 3;
  const uint8_t gridX = 1, gridY = 13;

  for (uint8_t s = 0; s < NUM_STEPS; s++){
    uint8_t col = s % 8;
    uint8_t row = s / 8;
    int x = gridX + col * (cellW + gapX);
    int y = gridY + row * (cellH + gapY);

    bool active = isStepActive(selectedChannel, editIdx(selectedChannel, s));
    bool isPlayhead = isRunning && (s == currentStep);

    if (isPlayhead){
      display.fillRect(x, y, cellW, cellH, SH110X_WHITE);
      if ((now / 125) % 2 == 0){
        display.fillRect(x+4, y+5, 6, 6, SH110X_BLACK);
      }
    } else if (active){
      display.fillRect(x, y, cellW, cellH, SH110X_WHITE);
      uint16_t eI = editIdx(selectedChannel, s);
      uint8_t fs = fillState[selectedChannel][eI];
      if (fs == 1){
        display.setTextColor(SH110X_BLACK);
        display.setTextSize(1);
        display.setCursor(x + 4, y + 4);
        display.print('F');
      } else if (fs == 2){
        display.drawLine(x+2, y+3, x+cellW-3, y+cellH-4, SH110X_BLACK);
        display.drawLine(x+cellW-3, y+3, x+2, y+cellH-4, SH110X_BLACK);
      }
      if (stepSlide[selectedChannel][eI]){
        display.fillTriangle(x+cellW-4, y+cellH-1,
                             x+cellW-1, y+cellH-4,
                             x+cellW-1, y+cellH-1, SH110X_BLACK);
      }
    } else {
      display.drawRect(x, y, cellW, cellH, SH110X_WHITE);
    }

    uint16_t eIr = editIdx(selectedChannel, s);
    uint8_t rIdx = stepRatchet[selectedChannel][eIr];
    if (rIdx == 0 && trigMachine[selectedChannel] != TM_OFF) rIdx = machineRatchet[selectedChannel][s];
    if (rIdx > 0){
      display.fillRect(x+1, y+1, 2, 2, active ? SH110X_BLACK : SH110X_WHITE);
    }
  }

  // ── BOTTOM BAR ───────────────────────────────────────────────
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  const char spinFrames[] = {'-','\\','|','/'};
  uint8_t spinFrame = (now / 120) % 4;
  display.setCursor(2, 57);
  display.print(isRunning ? spinFrames[spinFrame] : '.');
  display.setCursor(34, 57);
  display.print("BPM:");
  display.print(bpm);
  if (fillModeActive && ((now / 250) % 2 == 0)){
    display.setCursor(98, 57);
    display.print("FILL");
  } else if (isRunning){
    display.setCursor(98, 57);
    display.print(currentStep + 1);
    display.print("/");
    display.print(NUM_STEPS);
  }

  display.display();
}

void SimpleSequencer::drawTrigMachineView(){
  display.clearDisplay();
  static const char* machineNames[TM_COUNT] = {
    "OFF", "KICK", "HIHAT", "SNARE", "ANTIK", "PERC", "EUCL"
  };

  // Top: channel tabs
  for (uint8_t c = 0; c < NUM_CHANNELS; c++){
    int bx = 2 + c * 17;
    if (c == selectedChannel){
      display.fillRect(bx, 0, 16, 9, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.drawRect(bx, 0, 16, 9, SH110X_WHITE);
      display.setTextColor(SH110X_WHITE);
      if (muted[c]) display.drawLine(bx+1, 4, bx+14, 4, SH110X_WHITE);
    }
    display.setTextSize(1);
    display.setCursor(bx + 3, 1);
    display.print(c + 1);
  }
  display.setTextColor(SH110X_WHITE);

  // Param row: machine | density | shift  (+ kick extras when KICK is active)
  uint8_t ch = selectedChannel;
  uint8_t m = trigMachine[ch];
  if (m >= TM_COUNT) m = 0;
  display.setTextSize(1);
  if (m == TM_KICK){
    // Compact kick layout — fits all six in 120 px.
    display.setCursor(2, 12);
    display.print("KCK D"); display.print(trigDensity[ch]);
    display.print(" S"); display.print(trigShift[ch]);
    display.print(" P"); display.print(kickNoteSpread[ch]);
    display.print(" R"); display.print(kickRatchetProb[ch]);
    display.print(kickExtrasAreFills[ch] ? " F" : "");
  } else {
    display.setCursor(2, 12);
    display.print("M:"); display.print(machineNames[m]);
    display.setCursor(56, 12);
    display.print("D:"); display.print(trigDensity[ch]);
    display.setCursor(94, 12);
    display.print("S:"); display.print(trigShift[ch]);
  }

  // Step grid: 2 rows x 8 cols, showing machine + overlay state
  // Empty box = machine off, no override
  // Filled box = machine on, no override (auto-on)
  // Box with X = forced on by overlay
  // Box with - through it = forced off by overlay
  // Cell width 13 + 2px gap = 15 per step. 8 cells = 118 px starting at x=4
  // → last cell ends at x=122, inside the safe area.
  const uint8_t cellW = 13, cellH = 16, gapX = 2, gapY = 3;
  const uint8_t startX = 4, startY = 24;
  for (uint8_t s = 0; s < NUM_STEPS; s++){
    uint8_t col = s % 8;
    uint8_t row = s / 8;
    int x = startX + col * (cellW + gapX);
    int y = startY + row * (cellH + gapY);
    uint16_t eIdx = editIdx(ch, s);
    bool autoOn = (m != TM_OFF) ? machinePattern[ch][s] : steps[ch][eIdx];
    uint8_t ov = machineOverlay[ch][eIdx];
    bool finalOn = isStepActive(ch, eIdx);

    if (finalOn){
      display.fillRect(x, y, cellW, cellH, SH110X_WHITE);
      if (ov == 1){
        // Force-on marker: small black plus inside
        display.drawLine(x+5, y+5, x+5, y+10, SH110X_BLACK);
        display.drawLine(x+3, y+7, x+9, y+7, SH110X_BLACK);
      }
    } else {
      display.drawRect(x, y, cellW, cellH, SH110X_WHITE);
      if (ov == 2){
        // Force-off marker: line through it
        display.drawLine(x+1, y+cellH/2, x+cellW-2, y+cellH/2, SH110X_WHITE);
      } else if (autoOn){
        // Hint: machine wanted this on but overlay turned it off
        display.drawPixel(x + cellW/2, y + cellH/2, SH110X_WHITE);
      }
    }
    // Current step playhead
    if (s == currentStep){
      display.drawFastHLine(x, y + cellH + 1, cellW, SH110X_WHITE);
    }
  }

  display.display();
}

// ── Menu 5: Pages mode — screen 1 view ─────────────────────────────────
void SimpleSequencer::drawPagesView(){
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  // Channel tab strip at top
  for (uint8_t c = 0; c < NUM_CHANNELS; c++){
    int bx = 2 + c * 17;
    if (c == selectedChannel){
      display.fillRect(bx, 0, 16, 9, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.drawRect(bx, 0, 16, 9, SH110X_WHITE);
      display.setTextColor(SH110X_WHITE);
      if (muted[c]) display.drawLine(bx+1, 4, bx+14, 4, SH110X_WHITE);
    }
    display.setTextSize(1);
    display.setCursor(bx + 5, 1);
    display.print(c + 1);
  }
  display.setTextColor(SH110X_WHITE);

  // Big "PAGE X/Y" indicator for the selected channel
  uint8_t ch = selectedChannel;
  uint8_t cur = editPage[ch] + 1;
  uint8_t tot = numPages[ch];
  display.setTextSize(3);
  char buf[8];
  snprintf(buf, sizeof(buf), "%u/%u", cur, tot);
  int tw = (int)strlen(buf) * 18;
  display.setCursor((128 - tw) / 2, 16);
  display.print(buf);

  // 4 page chips at the bottom — filled = selected edit page,
  //                            ring = exists (within numPages),
  //                            empty = not allocated.
  // Also highlight the playing page when running.
  uint8_t playPg = (numPages[ch] > 0) ? (globalPage % numPages[ch]) : 0;
  const int chipW = 24, chipH = 14, gapX = 4;
  int totalW = MAX_PAGES * chipW + (MAX_PAGES - 1) * gapX;
  int startX = (128 - totalW) / 2;
  int y = 46;
  display.setTextSize(1);
  for (uint8_t p = 0; p < MAX_PAGES; p++){
    int x = startX + p * (chipW + gapX);
    bool exists = (p < numPages[ch]);
    bool isEdit = (p == editPage[ch]);
    bool isPlay = (isRunning && p == playPg);

    if (isEdit){
      display.fillRect(x, y, chipW, chipH, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else if (exists){
      display.drawRect(x, y, chipW, chipH, SH110X_WHITE);
      display.setTextColor(SH110X_WHITE);
    } else {
      display.drawRect(x, y, chipW, chipH, SH110X_WHITE);
      // Dim "not allocated": just a single dot in the middle
      display.drawPixel(x + chipW/2, y + chipH/2, SH110X_WHITE);
      display.setTextColor(SH110X_WHITE);
    }
    if (exists){
      display.setCursor(x + 9, y + 4);
      display.print(p + 1);
    }
    // Playhead marker = small chevron above the chip
    if (isPlay){
      display.fillTriangle(x + chipW/2 - 3, y - 4,
                           x + chipW/2 + 3, y - 4,
                           x + chipW/2,     y - 1, SH110X_WHITE);
    }
  }
  display.setTextColor(SH110X_WHITE);
  display.setCursor(2, 57);
  display.print("Pot1: page  Pot1 btn: count");
  display.display();
}

// --- Secondary OLED: global overview dashboard ------------------------------
// Layout (128x64):
//   Top bar  (y=0..11):  ▶/■ transport, BPM, step counter
//   Playhead arrow at y=12 above the grid
//   Channel grid (y=14..48): 7 rows × 16 steps, channel number on left, mute marker right
//   Footer (y=52..63):  active menu name + held channel
// Draw a stylised machine icon centred at (cx, cy). Used for Menu 4 screen 2.
static void drawMachineIcon(Adafruit_SH1106G& d, int cx, int cy, uint8_t machine){
  switch (machine){
    case SimpleSequencer::TM_KICK: {
      // Solid filled circle — heavy thump
      d.fillCircle(cx, cy, 14, SH110X_WHITE);
      break;
    }
    case SimpleSequencer::TM_HIHAT: {
      // Bold X — closed hat sticks
      for (int o = -1; o <= 1; o++){
        d.drawLine(cx - 13 + o, cy - 13, cx + 13 + o, cy + 13, SH110X_WHITE);
        d.drawLine(cx + 13 + o, cy - 13, cx - 13 + o, cy + 13, SH110X_WHITE);
      }
      break;
    }
    case SimpleSequencer::TM_SNARE: {
      // Filled triangle pointing up — snare crack
      d.fillTriangle(cx, cy - 14, cx - 14, cy + 12, cx + 14, cy + 12, SH110X_WHITE);
      break;
    }
    case SimpleSequencer::TM_ANTIKICK: {
      // Hollow ring with a dot — kick's shadow
      d.drawCircle(cx, cy, 14, SH110X_WHITE);
      d.drawCircle(cx, cy, 13, SH110X_WHITE);
      d.fillCircle(cx, cy, 3, SH110X_WHITE);
      break;
    }
    case SimpleSequencer::TM_PERC: {
      // Four small filled circles — scattered percussion
      d.fillCircle(cx - 7, cy - 7, 3, SH110X_WHITE);
      d.fillCircle(cx + 7, cy - 7, 3, SH110X_WHITE);
      d.fillCircle(cx - 7, cy + 7, 3, SH110X_WHITE);
      d.fillCircle(cx + 7, cy + 7, 3, SH110X_WHITE);
      d.fillCircle(cx, cy, 2, SH110X_WHITE);
      break;
    }
    case SimpleSequencer::TM_EUCLID: {
      // Concentric rings — Euclidean rotation
      d.drawCircle(cx, cy, 14, SH110X_WHITE);
      d.drawCircle(cx, cy, 9, SH110X_WHITE);
      d.drawCircle(cx, cy, 4, SH110X_WHITE);
      break;
    }
    default: { // TM_OFF — two thick horizontal lines (mute mark)
      d.fillRect(cx - 14, cy - 2, 28, 4, SH110X_WHITE);
      break;
    }
  }
}

void SimpleSequencer::drawOverview(){
  display2.clearDisplay();
  display2.setTextColor(SH110X_WHITE);
  uint32_t now = millis();
  uint8_t ch = (heldChannel >= 0) ? (uint8_t)heldChannel : selectedChannel;

  // ── Menu 4: Trigger Machines — stripped-down focus view ─────────
  if (activeMenu == 4){
    uint8_t m = trigMachine[ch];
    if (m >= TM_COUNT) m = 0;
    static const char* names[TM_COUNT] = {
      "OFF", "KICK", "HIHAT", "SNARE", "ANTIKICK", "PERC", "EUCLID"
    };
    // Big machine icon centred horizontally at the top
    drawMachineIcon(display2, 64, 18, m);

    // Machine name (size 2, centred). Moves up when KICK to leave room for
    // the live-params row.
    const char* nm = names[m];
    int textW = (int)strlen(nm) * 12;
    int tx = (128 - textW) / 2; if (tx < 0) tx = 0;
    display2.setTextSize(2);
    display2.setCursor(tx, m == TM_KICK ? 30 : 38);
    display2.print(nm);

    // Kick live params row (only when KICK is the active machine).
    // Compact text + tiny segmented bars under the title.
    if (m == TM_KICK){
      display2.setTextColor(SH110X_WHITE);
      display2.setTextSize(1);
      // Spread (0..5) as 5 small filled boxes
      display2.setCursor(2, 46);
      display2.print("SP");
      for (int i = 0; i < 5; i++){
        int bx = 14 + i * 4;
        if (i < (int)kickNoteSpread[ch]){
          display2.fillRect(bx, 47, 3, 5, SH110X_WHITE);
        } else {
          display2.drawRect(bx, 47, 3, 5, SH110X_WHITE);
        }
      }
      // Ratchet probability (0..100) as a small bar
      display2.setCursor(40, 46);
      display2.print("RT");
      int rbX = 52, rbW = 36, rbH = 5;
      display2.drawRect(rbX, 47, rbW, rbH, SH110X_WHITE);
      int rfw = (kickRatchetProb[ch] * (rbW - 2) + 50) / 100;
      if (rfw > rbW - 2) rfw = rbW - 2;
      if (rfw > 0) display2.fillRect(rbX + 1, 48, rfw, rbH - 2, SH110X_WHITE);
      // Fill toggle indicator
      if (kickExtrasAreFills[ch]){
        display2.fillRect(94, 46, 18, 7, SH110X_WHITE);
        display2.setTextColor(SH110X_BLACK);
        display2.setCursor(96, 47);
        display2.print("FILL");
        display2.setTextColor(SH110X_WHITE);
      } else {
        display2.drawRect(94, 46, 18, 7, SH110X_WHITE);
        display2.setCursor(96, 47);
        display2.print("LIVE");
      }
    }

    // Density fill bar at the bottom — Digitakt-style 8-segment block.
    // Live density on the channel; a thin "leading edge" pulse animates the
    // filled portion so you can see density changes as you sweep Pot 2.
    const int barX = 8, barY = 56, barW = 112, barH = 7;
    display2.drawRect(barX, barY, barW, barH, SH110X_WHITE);
    int dens = trigDensity[ch];
    int fillW = ((dens * (barW - 2)) + 50) / 100;
    if (fillW > barW - 2) fillW = barW - 2;
    if (fillW > 0){
      display2.fillRect(barX + 1, barY + 1, fillW, barH - 2, SH110X_WHITE);
      for (int i = 1; i < 8; i++){
        int sx = barX + 1 + (i * (barW - 2)) / 8;
        if (sx < barX + 1 + fillW){
          display2.drawFastVLine(sx, barY + 1, barH - 2, SH110X_BLACK);
        }
      }
      int pulsePos = barX + 1 + fillW - 1;
      if ((now / 100) % 2 == 0){
        display2.drawFastVLine(pulsePos, barY, barH, SH110X_WHITE);
      }
    }

    display2.display();
    return;
  }

  // ── Menu 3 (activeMenu==2): Euclid extension ────────────────────
  if (activeMenu == 2){
    bool enabled = euclidEnabled[ch];

    // 16 dot positions on a circle of radius 18 around centre (64, 22).
    // Precomputed to avoid sin/cos in the inner loop.
    static const int8_t offsX[16] = { 0,  7, 13, 17, 18, 17, 13,  7,  0, -7,-13,-17,-18,-17,-13, -7};
    static const int8_t offsY[16] = {-18,-17,-13, -7,  0,  7, 13, 17, 18, 17, 13,  7,  0, -7,-13,-17};
    const int cx = 64, cy = 22;

    // Pass 1: draw "slide chains" UNDER the dots. For each active sliding
    // step, draw a line from it to the next active step's dot so the user
    // can see exactly which notes glide into which.
    if (enabled){
      for (uint8_t s = 0; s < NUM_STEPS; s++){
        uint16_t eI = editIdx(ch, s);
        if (!euclidPattern[ch][eI]) continue;
        if (!stepSlide[ch][eI]) continue;
        uint8_t next = s;
        for (uint8_t i = 1; i <= NUM_STEPS; i++){
          uint8_t cand = (s + i) % NUM_STEPS;
          if (euclidPattern[ch][editIdx(ch, cand)]){ next = cand; break; }
        }
        if (next == s) continue;
        int x1 = cx + offsX[s];
        int y1 = cy + offsY[s];
        int x2 = cx + offsX[next];
        int y2 = cy + offsY[next];
        display2.drawLine(x1, y1, x2, y2, SH110X_WHITE);
      }
    }

    // Pass 2: dots + playhead on top of the chains
    for (uint8_t s = 0; s < NUM_STEPS; s++){
      int dx = cx + offsX[s];
      int dy = cy + offsY[s];
      bool active = enabled ? euclidPattern[ch][editIdx(ch, s)] : false;

      if (active){
        display2.fillCircle(dx, dy, 2, SH110X_WHITE);
      } else {
        display2.drawPixel(dx, dy, SH110X_WHITE);
      }

      // Playhead: ring around the current step
      if (enabled && isRunning && s == currentStep){
        display2.drawCircle(dx, dy, 4, SH110X_WHITE);
      }
    }

    // Centre dot tinted by enabled state — quick visual on/off cue
    if (enabled){
      display2.fillCircle(cx, cy, 1, SH110X_WHITE);
    }

    // Status text below the circle (size 2 if OFF, size 1 status line if ON)
    display2.setTextColor(SH110X_WHITE);
    if (!enabled){
      // Big "OFF" centred
      display2.setTextSize(2);
      const char* txt = "OFF";
      int tw = (int)strlen(txt) * 12;
      display2.setCursor((128 - tw) / 2, 44);
      display2.print(txt);
    } else {
      // "P 6/16  O 2" status line
      char buf[16];
      snprintf(buf, sizeof(buf), "P %u/16  O %u",
               (unsigned)pulses[ch], (unsigned)euclidOffset[ch]);
      int tw = (int)strlen(buf) * 6;
      display2.setTextSize(1);
      display2.setCursor((128 - tw) / 2, 46);
      display2.print(buf);
    }

    // Pulses fill bar at the bottom (Digitone-style 16 segments)
    const int barX = 4, barY = 56, barW = 120, barH = 7;
    display2.drawRect(barX, barY, barW, barH, SH110X_WHITE);
    int p = enabled ? pulses[ch] : 0;
    if (p > NUM_STEPS) p = NUM_STEPS;
    int fillW = (p * (barW - 2)) / NUM_STEPS;
    if (fillW > 0){
      display2.fillRect(barX + 1, barY + 1, fillW, barH - 2, SH110X_WHITE);
      for (int i = 1; i < NUM_STEPS; i++){
        int sx = barX + 1 + (i * (barW - 2)) / NUM_STEPS;
        if (sx < barX + 1 + fillW){
          display2.drawFastVLine(sx, barY + 1, barH - 2, SH110X_BLACK);
        }
      }
      // Pulsing leading edge to give a "filling" feel when sweeping Pot 1
      int pulsePos = barX + 1 + fillW - 1;
      if ((now / 100) % 2 == 0){
        display2.drawFastVLine(pulsePos, barY, barH, SH110X_WHITE);
      }
    }

    display2.display();
    return;
  }

  // ── Menu 1: Notes — focused-parameter view following last-rotated pot ──
  if (activeMenu == 1){
    static const char* noteNames[]  = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
    static const char* scaleNames[] = {"OFF","MAJOR","MINOR","PENTA","LOCRIAN","DIM","ATONAL"};
    static const char* gateNames[]  = {"1","3/4","1/2","3/8","1/4","3/16","1/8","3/32","1/16","1/24","1/32"};

    // Persistent focus: once any pot has been turned the screen stays on that
    // panel until another pot is touched. No timeout. The summary only shows
    // before the very first rotation.
    bool focused = (lastTouchedPot >= 0);

    // Common header: small label naming the focused parameter
    static const char* labels[6] = {
      "ROOT", "SCALE", "GATE", "SLIDE %", "VELOCITY", "SPREAD"
    };

    display2.setTextColor(SH110X_WHITE);

    if (!focused){
      // Default Notes overview — show channel, root, scale, and the 4 main
      // generative params as compact bars so the user has a recap to glance at.
      display2.setTextSize(1);
      display2.setCursor(2, 1);
      display2.print("CH"); display2.print(ch + 1);
      display2.print(" >M"); display2.print(midiChannel[ch] + 1);
      uint8_t rootN = channelPitch[ch];
      display2.setCursor(58, 1);
      display2.print(noteNames[rootN % 12]); display2.print((int)((rootN/12) - 1));
      display2.setCursor(86, 1);
      uint8_t sm = euclidScaleMode[ch]; if (sm > 6) sm = 0;
      display2.print(scaleNames[sm]);
      display2.drawFastHLine(2, 11, 124, SH110X_WHITE);

      // 4 mini bars. Layout sized so a 3-digit value (max "127") fits.
      auto miniBar = [&](const char* lbl, int v, int max, int y){
        display2.setTextSize(1);
        display2.setCursor(2, y); display2.print(lbl);
        int bx = 28, bw = 72, bh = 7;
        display2.drawRect(bx, y, bw, bh, SH110X_WHITE);
        if (v > 0 && max > 0){
          int fw = ((v * (bw - 2)) + max/2) / max;
          if (fw > bw - 2) fw = bw - 2;
          if (fw > 0) display2.fillRect(bx + 1, y + 1, fw, bh - 2, SH110X_WHITE);
        }
        // Right-align value in a 3-digit slot that ends at x=124 (safe edge).
        char vbuf[6]; snprintf(vbuf, sizeof(vbuf), "%d", v);
        int vw = (int)strlen(vbuf) * 6;
        display2.setCursor(124 - vw, y);
        display2.print(vbuf);
      };
      miniBar("VEL",  channelVelocity[ch], 127, 16);
      miniBar("GAT",  noteLenIdx[ch],       10, 26);
      miniBar("SLD",  randomSlideProb[ch], 100, 36);
      miniBar("SPR",  octaveSpread[ch],     60, 46);
      // Hint at the bottom
      display2.setCursor(2, 57); display2.print("TURN A POT TO FOCUS");
      display2.display();
      return;
    }

    // FOCUS HEADER
    display2.setTextSize(1);
    display2.setCursor(2, 1);
    display2.print(labels[lastTouchedPot]);
    display2.setCursor(108, 1);
    display2.print("P"); display2.print((int)(lastTouchedPot + 1));
    display2.drawFastHLine(2, 10, 124, SH110X_WHITE);

    switch (lastTouchedPot){
      case 0: { // ROOT NOTE
        uint8_t n = channelPitch[ch];
        const char* nm = noteNames[n % 12];
        int oct = (int)(n / 12) - 1;
        // Big note glyph centred
        display2.setTextSize(3);
        char big[6];
        snprintf(big, sizeof(big), "%s%d", nm, oct);
        int bw = (int)strlen(big) * 18;
        display2.setCursor((128 - bw) / 2, 16);
        display2.print(big);
        // Small piano keyboard at the bottom with root highlighted
        // White keys: 0,2,4,5,7,9,11; black keys: 1,3,6,8,10
        const int kbX = 4, kbY = 50, kw = 8, kh = 12;
        int idx = n % 12;
        static const uint8_t whiteOrder[7] = {0,2,4,5,7,9,11};
        for (int i = 0; i < 7; i++){
          int x = kbX + i * kw;
          if (whiteOrder[i] == idx){
            display2.fillRect(x, kbY, kw - 1, kh, SH110X_WHITE);
          } else {
            display2.drawRect(x, kbY, kw - 1, kh, SH110X_WHITE);
          }
        }
        // Black keys overlay
        const int8_t blackPos[5] = {0, 1, 3, 4, 5}; // white-key index before the black key
        const uint8_t blackPC[5] = {1, 3, 6, 8, 10};
        for (int i = 0; i < 5; i++){
          int x = kbX + blackPos[i] * kw + kw/2 + 1;
          bool sel = (blackPC[i] == idx);
          if (sel){
            display2.fillRect(x, kbY, kw - 2, kh * 2 / 3, SH110X_WHITE);
          } else {
            display2.fillRect(x, kbY, kw - 2, kh * 2 / 3, SH110X_BLACK);
            display2.drawRect(x, kbY, kw - 2, kh * 2 / 3, SH110X_WHITE);
          }
        }
        break;
      }
      case 1: { // SCALE
        uint8_t sm = euclidScaleMode[ch]; if (sm > 6) sm = 0;
        const char* nm = scaleNames[sm];
        display2.setTextSize(2);
        int tw = (int)strlen(nm) * 12;
        display2.setCursor((128 - tw) / 2, 18);
        display2.print(nm);
        // Chromatic intervals row — 12 squares, highlighted ones belong to the scale
        static const uint16_t scaleMask[7] = {
          0,                          // OFF
          0b101010110101,             // Major:   0,2,4,5,7,9,11
          0b010101101101,             // Minor:   0,2,3,5,7,8,10
          0b001010010101,             // Penta:   0,2,4,7,9
          0b010101101011,             // Locrian: 0,1,3,5,6,8,10
          0b011011011011,             // Dim:     0,1,3,4,6,7,9,10
          0b111111111111              // Atonal:  all 12
        };
        const int sqW = 8, sqH = 8, sqY = 40, sqX = 16;
        uint16_t mask = scaleMask[sm];
        for (int i = 0; i < 12; i++){
          int x = sqX + i * (sqW);
          bool on = (mask >> i) & 1;
          if (on){
            display2.fillRect(x, sqY, sqW - 1, sqH, SH110X_WHITE);
          } else {
            display2.drawRect(x, sqY, sqW - 1, sqH, SH110X_WHITE);
          }
        }
        display2.setTextSize(1);
        display2.setCursor(2, 54);
        display2.print("C D E F G A B");
        break;
      }
      case 2: { // GATE (per-channel)
        display2.setTextSize(3);
        uint8_t gi = noteLenIdx[ch] % 11;
        const char* nm = gateNames[gi];
        int tw = (int)strlen(nm) * 18;
        display2.setCursor((128 - tw) / 2, 18);
        display2.print(nm);
        // Mini horizontal bar showing length relative to whole note
        int ticks = (int)noteLenTicks[gi];
        const int bx = 4, by = 50, bw = 120, bh = 9;
        display2.drawRect(bx, by, bw, bh, SH110X_WHITE);
        int fw = (ticks * (bw - 2)) / 96;
        if (fw > bw - 2) fw = bw - 2;
        if (fw > 0) display2.fillRect(bx + 1, by + 1, fw, bh - 2, SH110X_WHITE);
        break;
      }
      case 3: { // SLIDE %
        int v = randomSlideProb[ch];
        display2.setTextSize(3);
        char buf[6]; snprintf(buf, sizeof(buf), "%d%%", v);
        int tw = (int)strlen(buf) * 18;
        display2.setCursor((128 - tw) / 2, 16);
        display2.print(buf);
        // Slide-chain visualization: 16 dots in a row, every Nth connected
        // by a line based on probability (every step has v% chance of slide
        // — render a representative pattern using a simple stride).
        const int dy = 50, dx = 8;
        int spacing = 7;
        for (int i = 0; i < 16; i++){
          int x = dx + i * spacing;
          display2.fillCircle(x, dy, 2, SH110X_WHITE);
        }
        // Draw connecting lines proportional to slide probability.
        int linked = (v * 16 + 50) / 100;
        for (int i = 0; i < linked && i < 15; i++){
          int x1 = dx + i * spacing;
          int x2 = dx + (i + 1) * spacing;
          display2.drawFastHLine(x1, dy, x2 - x1, SH110X_WHITE);
        }
        break;
      }
      case 4: { // VELOCITY
        int v = channelVelocity[ch];
        display2.setTextSize(3);
        char buf[6]; snprintf(buf, sizeof(buf), "%d", v);
        int tw = (int)strlen(buf) * 18;
        display2.setCursor((128 - tw) / 2, 16);
        display2.print(buf);
        // Big segmented bar (Digitakt style, 16 segments)
        const int bx = 4, by = 50, bw = 120, bh = 11;
        display2.drawRect(bx, by, bw, bh, SH110X_WHITE);
        int fw = ((v * (bw - 2)) + 63) / 127;
        if (fw > bw - 2) fw = bw - 2;
        if (fw > 0){
          display2.fillRect(bx + 1, by + 1, fw, bh - 2, SH110X_WHITE);
          for (int i = 1; i < 16; i++){
            int sx = bx + 1 + (i * (bw - 2)) / 16;
            if (sx < bx + 1 + fw){
              display2.drawFastVLine(sx, by + 1, bh - 2, SH110X_BLACK);
            }
          }
          int pulsePos = bx + 1 + fw - 1;
          if ((now / 100) % 2 == 0){
            display2.drawFastVLine(pulsePos, by, bh, SH110X_WHITE);
          }
        }
        break;
      }
      case 5: { // SPREAD (octave/semitone spread)
        int v = octaveSpread[ch];
        display2.setTextSize(3);
        char buf[8]; snprintf(buf, sizeof(buf), "%d st", v);
        int tw = (int)strlen(buf) * 18;
        display2.setCursor((128 - tw) / 2, 16);
        display2.print(buf);
        // Range visualization on a horizontal piano-like strip showing how
        // far above the root the spread reaches.
        const int by = 50, bh = 10, bx = 4, bw = 120;
        display2.drawRect(bx, by, bw, bh, SH110X_WHITE);
        // Map spread 0..60 semitones onto the bar
        int fw = (v * (bw - 2)) / 60;
        if (fw > bw - 2) fw = bw - 2;
        if (fw > 0) display2.fillRect(bx + 1, by + 1, fw, bh - 2, SH110X_WHITE);
        // Octave markers every 12 semitones
        for (int oct = 1; oct <= 5; oct++){
          int sx = bx + (oct * 12 * (bw - 2)) / 60;
          display2.drawFastVLine(sx, by - 2, bh + 4, SH110X_WHITE);
        }
        break;
      }
    }
    display2.display();
    return;
  }

  // ── Menu 2 (activeMenu==3): Step Visualizer — full 7-channel overview ──
  if (activeMenu == 3){
    // Layout
    //   y0..9   : header (step number + a few markers)
    //   y12..60 : 7 rows of 6px cell + 1px gap = 7*7 = 49 px
    //   y62..63 : reserved for future indicators
    display2.setTextColor(SH110X_WHITE);
    display2.setTextSize(1);

    // ── Header ───────────────────────────────────────────────────
    if (isRunning){
      char buf[12];
      snprintf(buf, sizeof(buf), "STEP %02u/%02u",
               (unsigned)(currentStep + 1), (unsigned)NUM_STEPS);
      display2.setCursor(2, 1);
      display2.print(buf);
    } else {
      display2.setCursor(2, 1);
      display2.print("STOPPED");
    }
    // Right side: fill indicator
    if (fillModeActive && ((now / 250) % 2 == 0)){
      display2.fillRect(108, 0, 18, 9, SH110X_WHITE);
      display2.setTextColor(SH110X_BLACK);
      display2.setCursor(110, 1);
      display2.print("FILL");
      display2.setTextColor(SH110X_WHITE);
    }
    display2.drawFastHLine(2, 10, 124, SH110X_WHITE);

    // ── Multi-channel grid ───────────────────────────────────────
    const int gridX = 10;   // step cells start here (channel digit lives in x=0..7)
    const int gridY = 13;
    const int cellW = 6, cellH = 6, gapX = 1, gapY = 1;

    // Playhead column emphasis — bright vertical guide across all rows
    if (isRunning){
      int phX = gridX + currentStep * (cellW + gapX);
      // 2 px wide soft guide around the cell column for high contrast
      display2.drawFastVLine(phX - 1, gridY, 7 * (cellH + gapY) - gapY, SH110X_WHITE);
      display2.drawFastVLine(phX + cellW, gridY, 7 * (cellH + gapY) - gapY, SH110X_WHITE);
    }

    for (uint8_t c = 0; c < NUM_CHANNELS; c++){
      int y = gridY + c * (cellH + gapY);
      bool isSelected = (c == selectedChannel);

      // Channel digit / mute marker on the left
      if (isSelected){
        display2.fillRect(0, y, 8, cellH, SH110X_WHITE);
        display2.setTextColor(SH110X_BLACK);
      } else {
        display2.setTextColor(SH110X_WHITE);
      }
      display2.setCursor(2, y);
      display2.print(c + 1);
      if (muted[c]){
        display2.drawLine(0, y + cellH/2, 7, y + cellH/2,
                          isSelected ? SH110X_BLACK : SH110X_WHITE);
      }
      display2.setTextColor(SH110X_WHITE);

      // Step cells for this channel
      for (uint8_t s = 0; s < NUM_STEPS; s++){
        int x = gridX + s * (cellW + gapX);
        bool active = isStepActive(c, editIdx(c, s));
        bool isPhCol = isRunning && (s == currentStep);

        if (isPhCol && active){
          // Playhead + active = full bright (the cell stands out clearly inside
          // the bracketed playhead guide drawn above)
          display2.fillRect(x, y, cellW, cellH, SH110X_WHITE);
        } else if (isPhCol){
          // Playhead but inactive: light an outline so the column still reads
          display2.drawRect(x, y, cellW, cellH, SH110X_WHITE);
        } else if (active){
          display2.fillRect(x, y, cellW, cellH, SH110X_WHITE);
          // Fill-only marker = small dark dot
          if (fillState[c][editIdx(c, s)] == 1){
            display2.drawPixel(x + cellW/2, y + cellH/2, SH110X_BLACK);
          }
        } else {
          // Inactive: 4 corner dots for grid readability
          display2.drawPixel(x, y, SH110X_WHITE);
          display2.drawPixel(x + cellW - 1, y, SH110X_WHITE);
          display2.drawPixel(x, y + cellH - 1, SH110X_WHITE);
          display2.drawPixel(x + cellW - 1, y + cellH - 1, SH110X_WHITE);
        }

        // Ratchet pip in the top-left corner of any cell that has ratchets
        uint8_t rIdx = stepRatchet[c][editIdx(c, s)];
        if (rIdx == 0 && trigMachine[c] != TM_OFF) rIdx = machineRatchet[c][s];
        if (rIdx > 0){
          display2.drawPixel(x + 1, y + 1,
                             (active || isPhCol) ? SH110X_BLACK : SH110X_WHITE);
        }
      }
    }
    display2.display();
    return;
  }

  // ── Menu 5: Pages mode — full pattern overview across pages ─────
  if (activeMenu == 5){
    display2.setTextColor(SH110X_WHITE);
    display2.setTextSize(1);
    // Top: channel + page summary
    display2.setCursor(2, 1);
    display2.print("CH"); display2.print(ch + 1);
    display2.print("  "); display2.print(numPages[ch]); display2.print(" PAGE");
    if (numPages[ch] > 1) display2.print("S");
    // Right side: edit page indicator
    display2.setCursor(98, 1);
    display2.print("EDIT P"); display2.print(editPage[ch] + 1);
    display2.drawFastHLine(2, 10, 124, SH110X_WHITE);

    // Show all pages stacked vertically — each page is one row of 16 cells.
    // 4 rows of 8 px height max (32px total), with channel page label on the left.
    const int gridY = 13;
    const int rowH = 12; // 10 cell + 2 gap
    const int cellW = 6, cellH = 9, gapX = 1;
    const int labelW = 12;
    for (uint8_t pg = 0; pg < MAX_PAGES; pg++){
      int y = gridY + pg * rowH;
      bool exists = (pg < numPages[ch]);
      bool isEdit = (pg == editPage[ch]);
      bool isPlay = isRunning && (pg == (uint8_t)(numPages[ch] > 0 ? (globalPage % numPages[ch]) : 0));

      // Label box
      if (isEdit){
        display2.fillRect(0, y, labelW, cellH, SH110X_WHITE);
        display2.setTextColor(SH110X_BLACK);
      } else if (exists){
        display2.drawRect(0, y, labelW, cellH, SH110X_WHITE);
        display2.setTextColor(SH110X_WHITE);
      } else {
        display2.drawPixel(labelW/2, y + cellH/2, SH110X_WHITE);
        display2.setTextColor(SH110X_WHITE);
      }
      if (exists){
        display2.setCursor(3, y + 1);
        display2.print("P"); display2.print(pg + 1);
      }
      // Playhead chevron above the row
      if (isPlay){
        display2.drawPixel(labelW/2 - 1, y - 2, SH110X_WHITE);
        display2.drawPixel(labelW/2,     y - 2, SH110X_WHITE);
        display2.drawPixel(labelW/2 + 1, y - 2, SH110X_WHITE);
      }
      display2.setTextColor(SH110X_WHITE);

      // Step cells for this page
      uint16_t base = (uint16_t)pg * NUM_STEPS;
      for (uint8_t s = 0; s < NUM_STEPS; s++){
        int x = labelW + 2 + s * (cellW + gapX);
        bool on;
        if (!exists){
          on = false;
        } else if (trigMachine[ch] != TM_OFF){
          // For machines: the live machinePattern only reflects current play page.
          // Show overlays distinctly; pattern only meaningful on play page.
          uint8_t ov = machineOverlay[ch][base + s];
          if (ov == 1) on = true;
          else if (ov == 2) on = false;
          else on = (pg == (uint8_t)(globalPage % numPages[ch])) && machinePattern[ch][s];
        } else if (euclidEnabled[ch]){
          on = euclidPattern[ch][base + s];
        } else {
          on = steps[ch][base + s];
        }
        // Playhead column highlight only on the currently-playing page
        bool ph = isPlay && (s == currentStep);
        if (ph && on){
          display2.fillRect(x, y, cellW, cellH, SH110X_WHITE);
        } else if (ph){
          display2.drawRect(x, y, cellW, cellH, SH110X_WHITE);
        } else if (on){
          display2.fillRect(x, y, cellW, cellH, SH110X_WHITE);
        } else if (exists){
          // Faint grid dot
          display2.drawPixel(x + cellW/2, y + cellH/2, SH110X_WHITE);
        }
      }
    }
    display2.display();
    return;
  }

  // ── Other menus: placeholder until we redesign each ─────────────
  display2.setTextSize(1);
  display2.setCursor(2, 2);
  display2.print("MENU ");
  display2.print((int)activeMenu);
  display2.display();
}
