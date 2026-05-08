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
  : bpm(200), lastStepMillis(0), currentStep(0), selectedChannel(0),
    ledStrip(0, 0, NEO_GRB + NEO_KHZ800)
{
  for (uint8_t c=0;c<NUM_CHANNELS;c++){
    pulses[c]=4;
    euclidOffset[c] = 0;
    retrig[c]=1;
    euclidEnabled[c]=false;
    euclidScaleMode[c] = 0;
    muted[c]=false;
    noteOffTick[c]=0;
    for (uint8_t s=0; s<NUM_STEPS; s++){
      fillState[c][s] = 0;
      steps[c][s]=false;
      euclidPattern[c][s]=false;
      pitch[c][s] = 255;
      noteLen[c][s] = 255;
      stepRatchet[c][s] = 0;
      stepVelocity[c][s] = 255;
      stepSlide[c][s] = false;
      pendingToggle[s] = false;
    }
    channelPitch[c] = 36;
    channelVelocity[c] = 96;
    ratchetIntervalTicks[c] = 0;
    lastNotePlaying[c] = 255;
    // Generative defaults
    randomSlideProb[c] = 0;   // 0% slide by default
    octaveSpread[c]    = 2;   // index 2 → 0 octave offset
    lastScaleMode[c]   = 1;   // remember Major as last-active scale
  }
  for (uint8_t k=0;k<MATRIX_KEYS;k++){
    matrixRawState[k] = 0;
    matrixState[k] = false;
    matrixLastDebounce[k] = 0;
  }
  lastMidiClockMicros = 0;
  noteLenIdx = 4;
  absoluteTickCounter = 0;
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
  // LEDs disabled for now (hardware bring-up)
  // ledStrip initialization and boot animation removed.

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
  fillModeActive = isFillHeld();

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
  }
  // MIDI clock generation and external MIDI handling moved to `runEngine()` only to avoid race conditions.

  // Time-critical MIDI processing (advancing steps/note-offs/MIDI RX) now runs in the engine timer.
  // update display at configured refresh interval
  if (millis() - lastDisplayMillis > displayRefreshMs){
    updateLEDs();
    drawDisplay();
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
  // Matrix scanning step: non-blocking single-column scan
  scanMatrixStep();
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
        Serial.print("MUTE CH"); Serial.println(ch+1);
      } else {
        selectedChannel = ch;
        Serial.print("SEL CH"); Serial.println(ch+1);
      }
      return;
    }
  }

  // --- Function button: modifier only, no action on press ---
  if (i == MATRIX_BTN_FUNCTION_INDEX){ return; }

  // --- Fill button: state read via isFillHeld() in loop() ---
  if (i == MATRIX_BTN_FILL_INDEX){ return; }

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
    activeMenu = 3;  // Step page
    heldStep = -1; focusEncoder = 0;
    Serial.print("MENU2 -> activeMenu=3 (Step)"); Serial.println();
    return;
  }
  if (i == MATRIX_BTN_MENU3_INDEX){
    activeMenu = 2;  // Euclid page
    heldStep = -1; focusEncoder = 0;
    Serial.print("MENU3 -> activeMenu=2 (Euclid)"); Serial.println();
    return;
  }
  if (i == MATRIX_BTN_MENU4_INDEX){ return; }

  // --- Step buttons 0-15 only ---
  if (i < NUM_STEPS){
    pendingToggle[i] = true;
    heldStep = (int8_t)i;
    lastEncoderMoveTime = millis();
    focusEncoder = 0;
  }
}

// Called when a debounced release is detected
void SimpleSequencer::onKeyRelease(uint8_t row, uint8_t col){
  uint8_t i = matrixIndex(row,col);
  if (i >= NUM_STEPS){
    // Non-step button released — just ensure heldStep is cleared if it was this button
    if (heldStep == (int8_t)i) heldStep = -1;
    return;
  }
  if (pendingToggle[i]){
    bool startHeld = isStartHeld();
    if (startHeld) {
      pendingToggle[i] = false;
    } else {
      if (euclidEnabled[selectedChannel]){
        bool newState = !euclidPattern[selectedChannel][i];
        euclidPattern[selectedChannel][i] = newState;
        steps[selectedChannel][i] = newState;
        if (newState){
          if (pitch[selectedChannel][i] == 255) pitch[selectedChannel][i] = channelPitch[selectedChannel];
          if (noteLen[selectedChannel][i] == 255) noteLen[selectedChannel][i] = noteLenIdx;
          if (stepVelocity[selectedChannel][i] == 255) stepVelocity[selectedChannel][i] = channelVelocity[selectedChannel];
        }
      } else {
        steps[selectedChannel][i] = !steps[selectedChannel][i];
        if (!steps[selectedChannel][i]){
          noteLen[selectedChannel][i] = 255;
          pitch[selectedChannel][i] = 255;
          fillState[selectedChannel][i] = 0;
          stepRatchet[selectedChannel][i] = 0;
          stepVelocity[selectedChannel][i] = 255;
          stepSlide[selectedChannel][i] = false;
        }
      }
      Serial.print("Ch"); Serial.print(selectedChannel+1);
      Serial.print(" Step "); Serial.print(i);
      Serial.print(" = "); Serial.println(steps[selectedChannel][i]);
      pendingToggle[i] = false;
    }
  }
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
  if (activeMenu == 1){
    uint8_t ch = selectedChannel;
    switch (pot){
      case 0: { // Channel select
        selectedChannel = (selectedChannel + ticks + NUM_CHANNELS) % NUM_CHANNELS;
        Serial.print("SEL CH"); Serial.println(selectedChannel+1);
        break;
      }
      case 1: { // Scale selection (cycle modes 1..6: Maj/Min/Pent/Loc/Dim/Atonal)
        int mode = (int)euclidScaleMode[ch];
        if (mode <= 0) mode = 1;
        mode += ticks;
        // Wrap into 1..6 inclusive
        const int N = 6;
        mode = ((mode - 1) % N + N) % N + 1;
        euclidScaleMode[ch] = (uint8_t)mode;
        lastScaleMode[ch]   = (uint8_t)mode;
        randomizeEuclidMelody(ch);
        Serial.print("SCALE="); Serial.println(mode);
        break;
      }
      case 2: { // Gate length (global)
        int prev = noteLenIdx;
        noteLenIdx = (uint8_t)constrain((int)noteLenIdx + ticks, 0, 4);
        // Propagate to all generated steps so live tweaks are audible immediately
        if (noteLenIdx != prev && euclidScaleMode[ch] != 0){
          for (uint8_t s = 0; s < NUM_STEPS; s++) noteLen[ch][s] = noteLenIdx;
        }
        Serial.print("GATE="); Serial.println(noteLenIdx);
        break;
      }
      case 3: { // Random Slide probability 0..100
        randomSlideProb[ch] = (uint8_t)constrain(
          (int)randomSlideProb[ch] + ticks, 0, 100);
        Serial.print("SLIDE%="); Serial.println(randomSlideProb[ch]);
        break;
      }
      case 4: { // Base velocity 0..127
        channelVelocity[ch] = (uint8_t)constrain(
          (int)channelVelocity[ch] + ticks, 0, 127);
        Serial.print("VEL="); Serial.println(channelVelocity[ch]);
        break;
      }
      case 5: { // Octave spread 0..5 (mapped -2..+3)
        octaveSpread[ch] = (uint8_t)constrain(
          (int)octaveSpread[ch] + ticks, 0, 5);
        Serial.print("OCT="); Serial.println((int)octaveSpread[ch] - 2);
        break;
      }
    }
    return;
  }

  if (activeMenu != 2) return;  // Other menus: only Euclid handles rotation

  // Euclid page controls
  switch (pot){
    case 0:  // Pot 1: cycle through channels
      selectedChannel = (selectedChannel + ticks + NUM_CHANNELS) % NUM_CHANNELS;
      Serial.print("SEL CH"); Serial.println(selectedChannel+1);
      break;
      
    case 1:  // Pot 2: adjust pulses (hits)
      pulses[selectedChannel] = (uint8_t)constrain(
        (int)pulses[selectedChannel] + ticks, 0, NUM_STEPS);
      updateEuclid(selectedChannel);
      Serial.print("PULSES="); Serial.println(pulses[selectedChannel]);
      break;
      
    case 2:  // Pot 3: adjust offset
      euclidOffset[selectedChannel] = 
        (euclidOffset[selectedChannel] + ticks + NUM_STEPS) % NUM_STEPS;
      updateEuclid(selectedChannel);
      Serial.print("OFFSET="); Serial.println(euclidOffset[selectedChannel]);
      break;
      
    case 3:  // Pot 4: cycle scale mode
      euclidScaleMode[selectedChannel] = (euclidScaleMode[selectedChannel] + ticks + 4) % 4;
      if (euclidEnabled[selectedChannel]) randomizeEuclidMelody(selectedChannel);
      Serial.print("SCALE="); Serial.println(euclidScaleMode[selectedChannel]);
      break;
      
    case 4:  // Pot 5: adjust velocity
      channelVelocity[selectedChannel] = (uint8_t)constrain(
        (int)channelVelocity[selectedChannel] + ticks, 0, 127);
      Serial.print("VEL="); Serial.println(channelVelocity[selectedChannel]);
      break;
      
    case 5:  // Pot 6: adjust note length
      noteLenIdx = (uint8_t)constrain(
        (int)noteLenIdx + ticks, 0, 4);
      Serial.print("GATE="); Serial.println(noteLenIdx);
      break;
  }
}

void SimpleSequencer::saveState() {
  SaveData data;
  data.magicNumber = 13572470; // Unique signature (v4 — adds generative params)
  data.savedBpm = bpm;
  data.savedNoteLenIdx = noteLenIdx;

  for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
    data.savedChannelPitch[c] = channelPitch[c];
    data.savedMuted[c] = muted[c];
    data.savedEuclidEnabled[c] = euclidEnabled[c];
    data.savedPulses[c] = pulses[c];
    data.savedEuclidOffset[c] = euclidOffset[c];
    data.savedEuclidScaleMode[c] = euclidScaleMode[c];

    for (uint8_t s = 0; s < NUM_STEPS; s++) {
      data.savedSteps[c][s] = steps[c][s];
      data.savedPitch[c][s] = pitch[c][s];
      data.savedNoteLen[c][s] = noteLen[c][s];
      data.savedFillStep[c][s] = fillState[c][s];
      data.savedStepRatchet[c][s] = stepRatchet[c][s];
      data.savedStepVelocity[c][s] = stepVelocity[c][s];
      data.savedStepSlide[c][s] = stepSlide[c][s] ? 1 : 0;
    }
    data.savedChannelVelocity[c] = channelVelocity[c];
    data.savedRandomSlideProb[c] = randomSlideProb[c];
    data.savedOctaveSpread[c]    = octaveSpread[c];
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

  if (data.magicNumber == 13572470) {
    bpm = data.savedBpm;
    noteLenIdx = data.savedNoteLenIdx;

    for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
      channelPitch[c] = data.savedChannelPitch[c];
      muted[c] = data.savedMuted[c];
      euclidEnabled[c] = data.savedEuclidEnabled[c];
      pulses[c] = data.savedPulses[c];
      euclidOffset[c] = data.savedEuclidOffset[c];
      euclidScaleMode[c] = data.savedEuclidScaleMode[c];

      for (uint8_t s = 0; s < NUM_STEPS; s++) {
        steps[c][s] = data.savedSteps[c][s];
        pitch[c][s] = data.savedPitch[c][s];
        noteLen[c][s] = data.savedNoteLen[c][s];
        fillState[c][s] = data.savedFillStep[c][s];
        stepRatchet[c][s] = data.savedStepRatchet[c][s];
        stepVelocity[c][s] = data.savedStepVelocity[c][s];
        // Normalize suspicious saved per-step velocities (preserve 255 sentinel)
        if (stepVelocity[c][s] != 255 && stepVelocity[c][s] > 120) stepVelocity[c][s] = 96;
        stepSlide[c][s] = (data.savedStepSlide[c][s] != 0);
      }
      channelVelocity[c] = data.savedChannelVelocity[c];
      if (channelVelocity[c] > 120) channelVelocity[c] = 96;
      randomSlideProb[c] = (data.savedRandomSlideProb[c] <= 100) ? data.savedRandomSlideProb[c] : 0;
      octaveSpread[c]    = (data.savedOctaveSpread[c] <= 5)      ? data.savedOctaveSpread[c]    : 2;
      // lastScaleMode not persisted — derived from euclidScaleMode if non-zero
      if (euclidScaleMode[c] > 0 && euclidScaleMode[c] <= 6) lastScaleMode[c] = euclidScaleMode[c];
      // Regenerate Euclidean patterns if enabled
      if (euclidEnabled[c]) updateEuclid(c);
    }
    Serial.println("State loaded from EEPROM (v4).");
  } else {
    Serial.println("No saved state (v4) found. Booting blank.");
  }
}

void SimpleSequencer::randomizeEuclidMelody(uint8_t ch) {
  uint8_t mode = euclidScaleMode[ch];
  if (mode == 0) {
    // OFF: clear per-step generative overrides; steps fall back to channel pitch
    for (uint8_t s = 0; s < NUM_STEPS; s++) {
      pitch[ch][s]       = 255;
      stepSlide[ch][s]   = false;
      stepVelocity[ch][s] = 255; // use channel default
      noteLen[ch][s]     = 255; // use global gate
    }
    return;
  }

  uint8_t root = channelPitch[ch];

  // Scale Definitions (mode 1..6 → Major/Minor/Pent/Locrian/Dim/Atonal)
  static const uint8_t maj[] = {0,2,4,5,7,9,11,12};
  static const uint8_t min[] = {0,2,3,5,7,8,10,12};
  static const uint8_t pen[] = {0,2,4,7,9,12};
  static const uint8_t loc[] = {0,1,3,5,6,8,10,12};
  static const uint8_t dim[] = {0,1,3,4,6,7,9,10,12};
  static const uint8_t ato[] = {0,1,2,3,4,5,6,7,8,9,10,11,12};

  const uint8_t* scale;
  uint8_t size;
  switch (mode) {
    case 1: scale = maj; size = 8;  break;
    case 2: scale = min; size = 8;  break;
    case 3: scale = pen; size = 6;  break;
    case 4: scale = loc; size = 8;  break;
    case 5: scale = dim; size = 9;  break;
    default: scale = ato; size = 13; break; // mode 6 (Atonal) and any > 6
  }

  for (uint8_t s = 0; s < NUM_STEPS; s++) {
    int octOffset = ((int)octaveSpread[ch] - 2) * 12; // -2..+3 octaves
    int note = (int)root + scale[random(0, size)] + octOffset;
    pitch[ch][s] = (uint8_t)constrain(note, 0, 127);

    stepSlide[ch][s]    = (random(0, 101) <= randomSlideProb[ch]);
    int v = (int)channelVelocity[ch] + random(-10, 10);
    stepVelocity[ch][s] = (uint8_t)constrain(v, 0, 127);
    noteLen[ch][s]      = noteLenIdx;
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

  // Shift per-step pitches if present
  for (uint8_t s=0; s<NUM_STEPS; s++){
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
  // Melody generation is decoupled from rhythm changes: do not regenerate here.
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
  // 1. THE NORMAL MUTE & FILL BLOCK
  if (muted[ch]) return;
  uint8_t fstate = fillState[ch][currentStep];
  if (fstate == 1 && !fillModeActive) return;
  if (fstate == 2 && fillModeActive) return;
  uint8_t p = pitch[ch][currentStep];
  if (p == 255) p = channelPitch[ch];
  uint8_t note = constrain(p, 0, 127);

  uint8_t vel = stepVelocity[ch][currentStep];
  if (vel == 255) vel = channelVelocity[ch];

  // 2. THE MONOSYNTH LEGATO MAGIC
  static bool prevSlide[NUM_CHANNELS] = {false};
  bool isSlidingIntoThis = prevSlide[ch];

  if (lastNotePlaying[ch] < 128) {
    if (isSlidingIntoThis) {
      // LEGATO: Fire the new note BEFORE killing the old one to trigger portamento
      midiSendNoteOn(ch, note, vel);
      midiSendNoteOff(ch, lastNotePlaying[ch], 0);
    } else {
      // NORMAL: Kill the old note BEFORE firing the new one (Crisp re-trigger)
      midiSendNoteOff(ch, lastNotePlaying[ch], 0);
      midiSendNoteOn(ch, note, vel);
    }
  } else {
    // No overlapping note, just fire
    midiSendNoteOn(ch, note, vel);
  }

  // Save the new state for the NEXT step
  lastNotePlaying[ch] = note;
  // Consider encoder held slide as an active slide for the next step
  prevSlide[ch] = stepSlide[ch][currentStep] || encoderSlideHold;

  // 3. RATCHET & GATE LENGTH
  uint8_t lenIdx = noteLen[ch][currentStep];
  if (lenIdx == 255) lenIdx = noteLenIdx;

  uint8_t rIdx = stepRatchet[ch][currentStep];
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

    if (stepSlide[ch][currentStep]) {
      // FORCE OVERLAP: If this step is sliding, ensure it bleeds past the 6-tick boundary
      gateLength = (ticks < 7) ? 7 : (ticks + 1);
    } else {
      // NORMAL: Cut it short to leave a gap for envelopes to reset
      gateLength = (ticks > 1) ? (ticks - 1) : 1;
    }
    noteOffTick[ch] = absoluteTickCounter + gateLength;


  }
}

// CV/Gate functions removed; using MIDI out only

void SimpleSequencer::drawDisplay(){
  if (activeMenu == 1){ drawNotesView(); return; }
  if (activeMenu == 2){ drawEuclidView(); return; }
  if (activeMenu == 3){ drawStepVisualiser(); return; }

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
          if (lenIdx == 255) lenIdx = noteLenIdx;
          display.setTextSize(2); display.setTextColor(SH110X_WHITE);
          display.setCursor(4, 2); display.print("GATE");
          display.setTextSize(1); display.setCursor(90, 6);
          display.print("STP "); display.print(heldStep + 1);
          display.setTextSize(4); display.setCursor(4, 26);
          display.print(noteLenNames[lenIdx]);
        }
      } else {
        // Global gate length — big
        display.setTextSize(2);
        display.setTextColor(SH110X_WHITE);
        display.setCursor(4, 2);
        display.print("GATE");
        display.setTextSize(4);
        display.setCursor(4, 26);
        display.print(noteLenNames[noteLenIdx]);
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
    // 6 channels across 128px: each tab is 20px wide with 1px gap = 126px total
    int bx = c * 21;
    if (c == selectedChannel){
      display.fillRect(bx, 0, 20, 11, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK, SH110X_WHITE);
    } else {
      if (muted[c]){
        display.drawRect(bx, 0, 20, 11, SH110X_WHITE);
        display.drawLine(bx, 5, bx + 18, 5, SH110X_WHITE);
      } else {
        display.drawRect(bx, 0, 20, 11, SH110X_WHITE);
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
  display.print(noteLenNames[noteLenIdx]);

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
  (void)0;
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
  // LED boot animation disabled during bring-up; keep OLED intact
  (void)0;
}

// LEDs disabled: keep updateLEDs as no-op (defined earlier)


void SimpleSequencer::clearTrack(uint8_t ch) {
  for (uint8_t s = 0; s < NUM_STEPS; s++) {
    steps[ch][s] = false;
    pitch[ch][s] = 255;
    noteLen[ch][s] = 255;
    fillState[ch][s] = 0;
    stepRatchet[ch][s] = 0;
    stepVelocity[ch][s] = 255;
    stepSlide[ch][s] = false;
  }
  euclidEnabled[ch] = false;
  pulses[ch] = 4;
  euclidOffset[ch] = 0;
  
  display.clearDisplay();
  display.fillRect(0, 0, 128, 64, SH110X_WHITE);
  display.display();
  delay(30);
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

  display.setTextColor(SH110X_WHITE);

  // ── TOP ROW: Channel + Root note ─────────────────────────────
  uint8_t p = channelPitch[ch];
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("CH"); display.print(ch+1);
  // Root note bigger on the right
  display.setTextSize(2);
  display.setCursor(48, 0);
  display.print(noteNames[p % 12]); display.print((p / 12) - 1);
  // Mute marker
  if (muted[ch]){
    display.setTextSize(1);
    display.setCursor(110, 0);
    display.print("MUT");
  }
  display.drawFastHLine(0, 18, 128, SH110X_WHITE);

  // ── MIDDLE ROW: Scale + Generative Status ────────────────────
  display.setTextSize(1);
  display.setCursor(0, 23);
  display.print("SCL:");
  display.setTextSize(2);
  display.setCursor(28, 21);
  display.print(scaleNames[sm]);

  display.setTextSize(1);
  display.setCursor(96, 23);
  if (genOn){
    display.fillRect(94, 21, 32, 11, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(102, 23);
    display.print("GEN");
    display.setTextColor(SH110X_WHITE);
  } else {
    display.drawRect(94, 21, 32, 11, SH110X_WHITE);
    display.setCursor(100, 23);
    display.print("OFF");
  }
  display.drawFastHLine(0, 36, 128, SH110X_WHITE);

  // ── BOTTOM ROW: Slide% / Octave / Velocity / Gate / BPM ──────
  display.setTextSize(1);
  display.setCursor(0, 41);
  display.print("SLD:"); display.print(randomSlideProb[ch]); display.print("%");

  display.setCursor(64, 41);
  int oct = (int)octaveSpread[ch] - 2;
  display.print("OCT:");
  if (oct > 0) display.print("+");
  display.print(oct);

  display.setCursor(0, 53);
  display.print("VEL:"); display.print(channelVelocity[ch]);

  display.setCursor(48, 53);
  display.print("GT:"); display.print(noteLenNames[noteLenIdx]);

  display.setCursor(90, 53);
  display.print("BPM:"); display.print(bpm);

  display.display();
}

void SimpleSequencer::drawEuclidView(){
  // Menu 2: show euclid params and pattern for selected channel
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  // Header
  display.setCursor(0, 0);
  display.print("EUCLID  CH"); display.print(selectedChannel+1);
  display.drawFastHLine(0, 9, 128, SH110X_WHITE);
  // Params
  display.setCursor(0, 12);
  display.print("Hits:"); display.print(pulses[selectedChannel]);
  display.setCursor(48, 12);
  display.print("Offset:"); display.print(euclidOffset[selectedChannel]);
  display.setCursor(96, 12);
  display.print(euclidEnabled[selectedChannel] ? "ON" : "OFF");
  // Pattern grid: 16 steps as small squares in one row
  const uint8_t sq = 7, gap = 1, startX = 0, startY = 24;
  for (uint8_t s = 0; s < NUM_STEPS; s++){
    int x = startX + s * (sq + gap);
    bool active = euclidEnabled[selectedChannel]
                  ? euclidPattern[selectedChannel][s]
                  : steps[selectedChannel][s];
    bool isHead = isRunning && (s == currentStep);
    if (isHead){
      display.fillRect(x, startY, sq, sq, SH110X_WHITE);
      // blink centre
      uint32_t now = millis();
      if ((now / 125) % 2 == 0)
        display.fillRect(x+2, startY+2, 3, 3, SH110X_BLACK);
    } else if (active){
      display.fillRect(x, startY, sq, sq, SH110X_WHITE);
    } else {
      display.drawRect(x, startY, sq, sq, SH110X_WHITE);
    }
  }
  // Velocity row for selected channel
  display.setCursor(0, 36);
  display.print("Vel:"); display.print(channelVelocity[selectedChannel]);
  display.setCursor(48, 36);
  const char* scaleNames[] = {"OFF","LOC","DIM","ATO"};
  display.print("Scl:"); display.print(scaleNames[euclidScaleMode[selectedChannel] % 4]);
  // BPM bottom right
  display.setCursor(84, 56);
  display.print("BPM:"); display.print(bpm);
  // Running spinner
  const char spinFrames[] = {'-','\\','|','/'};
  display.setCursor(0, 56);
  display.print(isRunning ? spinFrames[(millis()/120)%4] : '.');
  display.display();
}

void SimpleSequencer::drawStepVisualiser(){
  display.clearDisplay();
  uint32_t now = millis();

  // ── TOP BAR: channel tabs ──────────────────────────────────────
  for (uint8_t c = 0; c < NUM_CHANNELS; c++){
    int bx = c * 21;
    bool isSelected = (c == selectedChannel);
    bool isMuted    = muted[c];
    if (isSelected){
      display.fillRect(bx, 0, 20, 9, SH110X_WHITE);
      display.setTextColor(SH110X_BLACK);
    } else {
      display.drawRect(bx, 0, 20, 9, SH110X_WHITE);
      display.setTextColor(SH110X_WHITE);
      if (isMuted){
        // strikethrough for muted
        display.drawLine(bx+1, 4, bx+18, 4, SH110X_WHITE);
      }
    }
    display.setTextSize(1);
    display.setCursor(bx + 3, 1);
    display.print(c + 1);
  }
  display.setTextColor(SH110X_WHITE);

  // ── STEP GRID: 16 steps in 2 rows of 8 ───────────────────────
  // Each cell is 14px wide x 16px tall with 2px gap
  const uint8_t cellW = 14, cellH = 16, gapX = 2, gapY = 3;
  const uint8_t gridX = 4, gridY = 13;

  for (uint8_t s = 0; s < NUM_STEPS; s++){
    uint8_t col = s % 8;
    uint8_t row = s / 8;
    int x = gridX + col * (cellW + gapX);
    int y = gridY + row * (cellH + gapY);

    bool active = euclidEnabled[selectedChannel]
                  ? euclidPattern[selectedChannel][s]
                  : steps[selectedChannel][s];

    bool isPlayhead = isRunning && (s == currentStep);

    if (isPlayhead){
      // Animated playhead: full bright fill + blinking inner dot
      display.fillRect(x, y, cellW, cellH, SH110X_WHITE);
      // Blink the centre pixel at 8 Hz
      if ((now / 125) % 2 == 0){
        display.fillRect(x+4, y+5, 6, 6, SH110X_BLACK);
      }
    } else if (active){
      display.fillRect(x, y, cellW, cellH, SH110X_WHITE);
      // Show fill state markers
      uint8_t fs = fillState[selectedChannel][s];
      if (fs == 1){
        // Fill-only: hollow centre
        display.fillRect(x+3, y+4, cellW-6, cellH-8, SH110X_BLACK);
      } else if (fs == 2){
        // Anti-fill: X mark
        display.drawLine(x+2, y+3, x+cellW-3, y+cellH-4, SH110X_BLACK);
        display.drawLine(x+cellW-3, y+3, x+2, y+cellH-4, SH110X_BLACK);
      }
      // Slide indicator: small triangle bottom-right
      if (stepSlide[selectedChannel][s]){
        display.fillTriangle(x+cellW-4, y+cellH-1,
                             x+cellW-1, y+cellH-4,
                             x+cellW-1, y+cellH-1, SH110X_BLACK);
      }
    } else {
      // Inactive step: outline only
      display.drawRect(x, y, cellW, cellH, SH110X_WHITE);
    }

    // Ratchet indicator: small dot top-left of cell
    if (stepRatchet[selectedChannel][s] > 0){
      display.fillRect(x+1, y+1, 2, 2,
        active ? SH110X_BLACK : SH110X_WHITE);
    }
  }

  // ── BOTTOM BAR: BPM + running state + fill indicator ─────────
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  // Animated running indicator: rotating dash at far left
  const char spinFrames[] = {'-','\\','|','/'};
  uint8_t spinFrame = (now / 120) % 4;
  display.setCursor(0, 57);
  if (isRunning){
    display.print(spinFrames[spinFrame]);
  } else {
    display.print('.');
  }

  // BPM centre
  display.setCursor(34, 57);
  display.print("BPM:");
  display.print(bpm);

  // Fill active indicator right side
  if (fillModeActive){
    // Pulsing FILL text: show/hide at 4 Hz
    if ((now / 250) % 2 == 0){
      display.setCursor(100, 57);
      display.print("FILL");
    }
  }

  // Step counter: current step / total (only when running)
  if (isRunning){
    display.setCursor(100, 57);
    display.print(currentStep + 1);
    display.print("/");
    display.print(NUM_STEPS);
  }

  display.display();
}
