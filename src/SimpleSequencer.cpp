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

// Rate multiplier slots for Pages menu Pot 3
const float SimpleSequencer::RATE_VALUES[SimpleSequencer::RATE_COUNT] = {
  0.25f, 0.5f, 1.0f, 2.0f, 4.0f
};

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

// Trigger-machine helpers defined further down; forward-declared so the pot
// handlers above their definition can size density to each machine's pool.
static uint8_t machinePoolMax(uint8_t machine);

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
    numSteps[c] = NUM_STEPS; // default to full 16-step pattern length
    channelPitch[c] = 33; // default A1 — matches Rytm MK2 default TRIG NOTE
    channelVelocity[c] = 100;
    midiChannel[c] = c; // default: CH1->MIDI ch1, CH2->ch2, ... (0-indexed = MIDI ch 1-6)
    ratchetIntervalTicks[c] = 0;
    lastNotePlaying[c] = 255;
    // Generative defaults
    randomSlideProb[c] = 0;   // 0% slide by default
    octaveSpread[c]    = 0;   // 0 = no octave spread (notes stay in root octave)
    lastScaleMode[c]   = 1;   // remember Major as last-active scale
    randomVelEnabled[c] = false; // Pot 6 press toggles per-channel random velocity
    randomGateEnabled[c] = false;// Pot 5 press toggles per-channel random gate length
    contourBias[c]      = 0;     // Function + encoder 3: melodic contour bias
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
    machineExtraCount[c] = 0;
    for (uint8_t i = 0; i < NUM_STEPS; i++) machineExtraSeq[c][i] = 0;
    // Gate length default per channel
    noteLenIdx[c] = NOTE_LEN_DEFAULT_IDX;
  }
  for (uint8_t i = 0; i < NUM_CV_OUTS; i++) cvVolts[i] = 0.0f;
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
  display.setRotation(SCREEN_ROTATION); // 0 = normal (release); override to 2 for upside-down PCB

  // second I2C bus + second OLED (overview screen)
  Wire1.begin();
  Wire1.setClock(400000);
  // probe address 0x3C on Wire1 before begin() — avoids long blocking init if absent
  Wire1.beginTransmission(0x3C);
  if (Wire1.endTransmission() == 0){
    display2.begin(0x3C);
    display2.setRotation(SCREEN_ROTATION); // matches the primary screen
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

  // NOTE: the MAX11300 (PIXI) is brought up lazily on first use (see
  // ensurePixi), not here — so a quiet/half-wired SPI bus can never stall boot.

  // Boot straight into the Notes page so there's a real menu on screen instead
  // of the placeholder default overview.
  activeMenu = 1;
}

// Bring up the MAX11300 once, the first time CV is actually needed (entering the
// Analog menu or running the 'v' self-test). Kept out of begin() so SPI never
// blocks boot.
void SimpleSequencer::ensurePixi(){
  if (pixiInit) return;
  pixiInit = true;
  Serial.println("PIXI: init...");
  pixiPresent = pixi.begin();
  if (pixiPresent){
    uint16_t id = pixi.readReg(Max11300::REG_DEVICE_ID);
    Serial.print("PIXI MAX11300 detected, dev_id=0x"); Serial.println(id, HEX);
    for (uint8_t i = 0; i < NUM_CV_OUTS; i++){
      pixi.configDac(CV_PORTS[i], Max11300::RANGE_0_TO_10);
    }
    Serial.print("Configured "); Serial.print(NUM_CV_OUTS);
    Serial.println(" CV out(s) as 0-10V DACs");
  } else {
    Serial.println("PIXI MAX11300 not detected on SPI");
  }
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
  // UI-only loop: read controls and update display. Time-critical MIDI work runs in engine timer.
  readButtons();
  readEncoders();
  unsigned long now = millis();

  // --- LIVE MODIFIERS (computed from fresh matrix state) ---
  bool fnHeld   = isFunctionHeld();
  bool fillNow  = isFillHeld();
  bool pageHeld = (MATRIX_BTN_PAGE_INDEX < MATRIX_KEYS) && matrixState[MATRIX_BTN_PAGE_INDEX];
  // Plain Fill = global fill performance (unchanged). Function+Fill = slide-all
  // and Function+Page = accent-all, both on the active channel only.
  fillModeActive = fillNow && !fnHeld;
  slideAllHold   = fillNow && fnHeld;
  accentAllHold  = pageHeld && fnHeld;
  if (fillModeActive != fillBtnLastState){
    Serial.print("FILL "); Serial.println(fillModeActive ? "ON" : "OFF");
    fillBtnLastState = fillModeActive;
  }

  // --- CLEAR: Function + Page + Fill held together for 1s clears active ch ---
  if (fnHeld && pageHeld && fillNow){
    if (clearComboStartMs == 0){
      clearComboStartMs = now;
    } else if (!clearComboFired && (now - clearComboStartMs) >= 1000){
      clearTrack(selectedChannel);
      clearComboFired = true;
      clearAnimCh = selectedChannel;
      clearAnimEndMs = now + 700;
      Serial.print("CLEAR (hold) CH"); Serial.println(selectedChannel + 1);
    }
  } else {
    clearComboStartMs = 0;
    clearComboFired = false;
  }

  // Transport (Play/Stop) is now Function + Menu 4 held together.
  bool startReading = fnHeld && matrixState[MATRIX_BTN_MENU4_INDEX];
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
              bool isActive = isStepActive(ch, playIdx(ch, localStep(ch)));
              if (isActive) triggerChannel(ch);
            }
            if (!externalMidiClockActive && !midiTimerRunning) {
              float eff = (float)bpm * rateCurrent;
              if (eff < 1.0f) eff = 1.0f;
              uint32_t interval = (uint32_t)(2500000.0f / eff);
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
    if (c == 'v' || c == 'V'){
      cvSelfTest();
    }
    if (c == 'x' || c == 'X'){
      pixiDiag();
    }
    if (c == 'y' || c == 'Y'){
      pixiPinTest();
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

  // --- Fill button: held-only modifier (plain = global fill; Function+Fill =
  // slide-all). No on-press action; handled live in loop(). ---
  if (i == MATRIX_BTN_FILL_INDEX){
    return;
  }

  // --- Page button: enter Pages menu, or cycle pages while inside it ---
  // Function + Page is reserved for transport (Start/Stop) and handled
  // elsewhere — only act on a plain tap.
  if (i == MATRIX_BTN_PAGE_INDEX){
    if (!isFunctionHeld()){
      lastPageBtnMs = millis();
      if (activeMenu != 5){
        activeMenu = 5;
        heldStep = -1; focusEncoder = 0;
        Serial.println("PAGE -> activeMenu=5 (Pages)");
      } else {
        // Already inside Pages: each tap cycles the page being edited.
        // In Global mode this advances globalPage (affects all channels);
        // in Channel mode it advances editPage[selectedChannel] only.
        if (pageEditGlobal){
          globalPage = (uint8_t)((globalPage + 1) % MAX_PAGES);
          Serial.print("PAGE tap -> globalPage="); Serial.println(globalPage + 1);
        } else {
          uint8_t ch = selectedChannel;
          uint8_t np = numPages[ch] > 0 ? numPages[ch] : 1;
          editPage[ch] = (uint8_t)((editPage[ch] + 1) % np);
          Serial.print("PAGE tap CH"); Serial.print(ch+1);
          Serial.print(" editPage="); Serial.println(editPage[ch] + 1);
        }
      }
    }
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
    activeMenu = 6;  // Analog CV outputs
    heldStep = -1; focusEncoder = 0;
    // NOTE: do NOT auto-init the PIXI here — SPI bring-up is currently hanging,
    // and that would freeze the UI on menu entry. Run it on demand via serial
    // 'x' (diag) or 'v' (self-test) until the SPI issue is resolved.
    Serial.println("MENU2 -> activeMenu=6 (Analog Outs)");
    return;
  }
  if (i == MATRIX_BTN_MENU3_INDEX){
    activeMenu = 2;  // Euclid page
    heldStep = -1; focusEncoder = 0;
    Serial.print("MENU3 -> activeMenu=2 (Euclid)"); Serial.println();
    return;
  }
  if (i == MATRIX_BTN_MENU4_INDEX){
    // Function + Menu 4 is the transport (Play/Stop) combo, handled in loop();
    // only switch menus on a plain tap.
    if (!isFunctionHeld()){
      activeMenu = 4;  // Trigger Machines page
      heldStep = -1; focusEncoder = 0;
      Serial.println("MENU4 -> activeMenu=4 (Trigger Machines)");
    }
    return;
  }

  // --- Step buttons 0-15 only ---
  if (i < NUM_STEPS){
    // Pages mode: Function + step 1..4 jumps straight to that page
    // (auto-extending numPages if needed). A plain step press falls through to
    // the normal toggle below so you can edit the pattern from the Pages menu.
    if (activeMenu == 5 && isFunctionHeld() && i < MAX_PAGES){
      uint8_t target = (uint8_t)i;
      if (target >= numPages[selectedChannel]){
        growPagesAndDuplicate(selectedChannel, (uint8_t)(target + 1));
      }
      editPage[selectedChannel] = target;
      Serial.print("CH"); Serial.print(selectedChannel+1);
      Serial.print(" goto page "); Serial.println(target+1);
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
bool SimpleSequencer::isStartHeld()   { return isFunctionHeld() && matrixState[MATRIX_BTN_MENU4_INDEX]; }
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
  if (activeMenu == 6){
    // Analog Outs: pot-button N resets that jack's DAC port (latch-up recovery
    // after a short); pot 6 button soft-resets the whole PIXI + reapplies all.
    if (pot < NUM_CV_OUTS){
      resetCvOut(pot);
    } else if (pot == 5){
      resetPixiAll();
    }
    return;
  }
  if (activeMenu == 5){
    if (pot == 0){
      // Pages mode: Pot 1 button toggles Global / Channel page-edit focus.
      pageEditGlobal = !pageEditGlobal;
      Serial.print("PAGES mode="); Serial.println(pageEditGlobal ? "GLOBAL" : "CHANNEL");
    } else if (pot == 2){
      // Pages mode: Pot 3 button resets rate back to 1x, smoothly ramped.
      rateIdx = 2; // 1.0x slot
      rateTarget = RATE_VALUES[rateIdx];
      rateRampFrom = rateCurrent;
      rateRampStartMs = millis();
      rateRamping = (fabsf(rateCurrent - rateTarget) > 0.0005f);
      Serial.println("RATE -> 1.0x (ramp)");
    } else if (pot == 5){
      // Pages mode: Pot 6 button = global reset.
      // Every channel back to 1 page / 16 steps, edit cursor and globalPage
      // back to page 1. Step contents are left intact (only the structural
      // length parameters are reset).
      for (uint8_t c = 0; c < NUM_CHANNELS; c++){
        numPages[c] = 1;
        numSteps[c] = NUM_STEPS;
        editPage[c] = 0;
      }
      globalPage = 0;
      Serial.println("PAGES global reset -> 1 page, 16 steps");
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
    } else if (pot == 2){
      // Pot 3 button: mutate — change one random active note on the edit
      // page (toggle slide, toggle accent, change length, or change pitch).
      mutatePattern(selectedChannel);
    } else if (pot == 4){
      // Pot 5 button: toggle random gate length for this channel. When on,
      // every note that uses the channel default gate gets a random length
      // across the full 1/32..1 range.
      uint8_t ch = selectedChannel;
      randomGateEnabled[ch] = !randomGateEnabled[ch];
      Serial.print("RND GATE CH"); Serial.print(ch+1);
      Serial.println(randomGateEnabled[ch] ? " ON" : " OFF");
    } else if (pot == 5){
      // Pot 6 button: toggle random velocity for this channel. When on, every
      // note that uses the channel default velocity is jittered +/-27.
      uint8_t ch = selectedChannel;
      randomVelEnabled[ch] = !randomVelEnabled[ch];
      Serial.print("RND VEL CH"); Serial.print(ch+1);
      Serial.println(randomVelEnabled[ch] ? " ON" : " OFF");
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
  static const uint8_t divTrigMachine[6] = {5, 4, 2, 3, 3, 3}; // density (pot2) slower for kick's 0..12 range
  static const uint8_t divAnalog[6]      = {2, 2, 2, 2, 2, 2}; // CV outs: uniform, ~0.05V per detent
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
  else if (activeMenu == 6) divTable = divAnalog;
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
      float eff = (float)bpm * rateCurrent;
      if (eff < 1.0f) eff = 1.0f;
      uint32_t interval = (uint32_t)(2500000.0f / eff);
      midiClockTimer.update(interval);
    }
    bpmFocusEndMs = millis() + 1500;
    Serial.print("BPM="); Serial.println(bpm);
    return;
  }

  // --- GLOBAL MODIFIER: Function + Pot 3 (encoder 3) = melody contour bias ---
  // Clockwise -> ascending bias, anti-clockwise -> descending. Applied on the
  // next pattern generation. Shown as an arrow on the Menu 1 screen 2.
  if (pot == 2 && isFunctionHeld()){
    int v = (int)contourBias[selectedChannel] + ticks * 8; // ~8% per detent
    if (v < -100) v = -100;
    if (v > 100)  v = 100;
    contourBias[selectedChannel] = (int8_t)v;
    Serial.print("CONTOUR CH"); Serial.print(selectedChannel+1);
    Serial.print("="); Serial.println(contourBias[selectedChannel]);
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

  // --- GLOBAL MODIFIER: Channel-held + Pot 2 = per-channel root NOTE ---
  // Lets the user edit the channel's note in any menu (notably the drum
  // machine view) without leaving machine controls behind.
  if (pot == 1 && heldChannel >= 0 && heldChannel < (int)NUM_CHANNELS){
    int oldRoot = (int)channelPitch[heldChannel];
    int p = oldRoot + ticks;
    p = constrain(p, 0, 127);
    int delta = p - oldRoot;
    channelPitch[heldChannel] = (uint8_t)p;
    if (delta != 0) transposeChannelNotes(heldChannel, delta);
    Serial.print("CH"); Serial.print(heldChannel+1);
    Serial.print(" NOTE="); Serial.println(channelPitch[heldChannel]);
    return;
  }

  if (activeMenu == 1){
    uint8_t ch = selectedChannel;

    // --- P-LOCKS: holding a step re-targets the pots at that single step ---
    // While a step button is held, the per-step override arrays are edited
    // instead of the channel-wide values, so you can pin a fixed note, gate,
    // slide or velocity on one step. These overrides are wiped when the
    // channel is regenerated (randomizeEuclidMelody) so the step "re-joins".
    if (heldStep >= 0 && heldStep < (int8_t)NUM_STEPS){
      uint16_t eI = editIdx(ch, (uint8_t)heldStep);
      bool edited = true;
      switch (pot){
        case 0: { // Note (pitch) for this step
          int cur = (pitch[ch][eI] == 255) ? (int)channelPitch[ch] : (int)pitch[ch][eI];
          pitch[ch][eI] = (uint8_t)constrain(cur + ticks, 0, 127);
          Serial.print("PLOCK note s"); Serial.print(heldStep+1);
          Serial.print("="); Serial.println(pitch[ch][eI]);
          break;
        }
        case 3: { // Slide on/off for this step (direction sets state)
          stepSlide[ch][eI] = (ticks > 0);
          Serial.print("PLOCK slide s"); Serial.print(heldStep+1);
          Serial.println(stepSlide[ch][eI] ? "=ON" : "=OFF");
          break;
        }
        case 4: { // Gate length for this step
          int cur = (noteLen[ch][eI] == 255) ? (int)noteLenIdx[ch] : (int)noteLen[ch][eI];
          noteLen[ch][eI] = (uint8_t)constrain(cur + ticks, 0, (int)NOTE_LEN_COUNT - 1);
          Serial.print("PLOCK gate s"); Serial.print(heldStep+1);
          Serial.print("="); Serial.println(noteLen[ch][eI]);
          break;
        }
        case 5: { // Velocity for this step
          int cur = (stepVelocity[ch][eI] == 255) ? (int)channelVelocity[ch] : (int)stepVelocity[ch][eI];
          stepVelocity[ch][eI] = (uint8_t)constrain(cur + ticks, 0, 127);
          Serial.print("PLOCK vel s"); Serial.print(heldStep+1);
          Serial.print("="); Serial.println(stepVelocity[ch][eI]);
          break;
        }
        default: edited = false; break; // scale/spread have no per-step meaning
      }
      if (edited){
        // Activate the held step so the p-lock is audible, and cancel the
        // on-release toggle so tweaking never flips the step off.
        steps[ch][eI] = true;
        if (euclidEnabled[ch]) euclidPattern[ch][eI] = true;
        pendingToggle[heldStep] = false;
      }
      return;
    }

    switch (pot){
      case 0: { // Pot 1: Root note — TRANSPOSE existing per-step notes (no regenerate)
        int oldRoot = (int)channelPitch[ch];
        int p = oldRoot + ticks;
        p = constrain(p, 0, 127);
        int delta = p - oldRoot;
        channelPitch[ch] = (uint8_t)p;
        if (delta != 0) transposeChannelNotes(ch, delta);
        Serial.print("ROOT="); Serial.println(channelPitch[ch]);
        break;
      }
      case 1: { // Pot 2: Scale selection — store only (next regenerate applies it)
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
      case 2: { // Pot 3: Octave spread 0..60 semitones — store only (next regenerate applies it)
        octaveSpread[ch] = (uint8_t)constrain(
          (int)octaveSpread[ch] + ticks, 0, 60);
        Serial.print("SPRD="); Serial.println(octaveSpread[ch]);
        break;
      }
      case 3: { // Pot 4: Random Slide probability — re-roll slides immediately
        randomSlideProb[ch] = (uint8_t)constrain(
          (int)randomSlideProb[ch] + ticks, 0, 100);
        rerollSlides(ch);
        Serial.print("SLIDE%="); Serial.println(randomSlideProb[ch]);
        break;
      }
      case 4: { // Pot 5: Gate length (per-channel)
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
      case 5: { // Pot 6: Base velocity 0..127
        channelVelocity[ch] = (uint8_t)constrain(
          (int)channelVelocity[ch] + ticks, 0, 127);
        Serial.print("VEL="); Serial.println(channelVelocity[ch]);
        break;
      }
    }
    return;
  }

  // --- Menu 5: Pages mode ---
  if (activeMenu == 5){
    uint8_t ch = selectedChannel;
    if (pot == 0){
      // Pot 1: pattern length in pages. In Global mode the dialled value is
      // broadcast to every channel so all tracks share the same length;
      // in Channel mode only the selected channel changes.
      // Use the Page button tap to navigate the edit page within numPages.
      if (pageEditGlobal){
        int v = (int)numPages[ch] + ticks;
        if (v < 1) v = 1;
        if (v > MAX_PAGES) v = MAX_PAGES;
        uint8_t target = (uint8_t)v;
        for (uint8_t c = 0; c < NUM_CHANNELS; c++){
          if (target > numPages[c]){
            growPagesAndDuplicate(c, target);
          } else if (target < numPages[c]){
            numPages[c] = target;
          }
          if (editPage[c] >= numPages[c]){
            editPage[c] = (uint8_t)(numPages[c] - 1);
          }
        }
        if (globalPage >= target) globalPage = (uint8_t)(target - 1);
        Serial.print("GLOBAL numPages="); Serial.println(target);
      } else {
        // Channel mode: Pot 1 sets the selected channel's pattern length only.
        int v = (int)numPages[ch] + ticks;
        if (v < 1) v = 1;
        if (v > MAX_PAGES) v = MAX_PAGES;
        uint8_t target = (uint8_t)v;
        if (target > numPages[ch]){
          growPagesAndDuplicate(ch, target);
        } else if (target < numPages[ch]){
          numPages[ch] = target;
        }
        if (editPage[ch] >= numPages[ch]){
          editPage[ch] = (uint8_t)(numPages[ch] - 1);
        }
        Serial.print("CH"); Serial.print(ch+1);
        Serial.print(" numPages="); Serial.println(numPages[ch]);
      }
    } else if (pot == 1){
      // Pot 2: pattern step count (1..NUM_STEPS).
      // Global mode broadcasts to all channels; Channel mode targets only
      // the active channel.
      int v = (int)numSteps[ch] + ticks;
      if (v < 1) v = 1;
      if (v > NUM_STEPS) v = NUM_STEPS;
      uint8_t target = (uint8_t)v;
      if (pageEditGlobal){
        for (uint8_t c = 0; c < NUM_CHANNELS; c++) numSteps[c] = target;
        Serial.print("GLOBAL numSteps="); Serial.println(target);
      } else {
        numSteps[ch] = target;
        Serial.print("CH"); Serial.print(ch+1);
        Serial.print(" numSteps="); Serial.println(target);
      }
    } else if (pot == 2){
      // Pot 3: rate multiplier (0.25/0.5/1/2/4x). Smoothly ramp to new target.
      int v = (int)rateIdx + ticks;
      if (v < 0) v = 0;
      if (v >= (int)RATE_COUNT) v = RATE_COUNT - 1;
      if ((uint8_t)v != rateIdx){
        rateIdx = (uint8_t)v;
        rateTarget = RATE_VALUES[rateIdx];
        rateRampFrom = rateCurrent;
        rateRampStartMs = millis();
        rateRamping = (fabsf(rateCurrent - rateTarget) > 0.0005f);
        Serial.print("RATE="); Serial.print(rateTarget, 2); Serial.println("x");
      }
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
        // Newly-selected machines start at their skeleton (density 0) with a
        // fresh accumulation, so you always dial extras up from nothing.
        trigDensity[ch] = 0;
        machineExtraCount[ch] = 0;
        if (trigMachine[ch] == TM_EUCLID) updateEuclid(ch);
        regenerateMachinePattern(ch);
        Serial.print("MACHINE CH"); Serial.print(ch+1);
        Serial.print(" = "); Serial.println(trigMachine[ch]);
        break;
      }
      case 1: { // Pot 2: density — incremental extras (0..machine pool size)
        int maxV = machinePoolMax(trigMachine[ch]);
        if (maxV < 1) maxV = 1; // EUCLID/OFF have no pool
        trigDensity[ch] = (uint8_t)constrain((int)trigDensity[ch] + ticks, 0, maxV);
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

  // --- Menu 2: Analog CV outputs — pots set each output's voltage (0..10V) ---
  if (activeMenu == 6){
    if (pot < NUM_CV_OUTS){
      setCvOut(pot, cvVolts[pot] + (float)ticks * 0.1f);
      Serial.print("CV"); Serial.print(pot + 1);
      Serial.print("="); Serial.print(cvVolts[pot], 2); Serial.println("V");
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
  data.magicNumber = 13572477; // Unique signature (v11 — per-channel numSteps)
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
    data.savedNumSteps[c]           = numSteps[c];
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

  if (data.magicNumber == 13572477) {
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
      uint8_t ns = data.savedNumSteps[c];
      numSteps[c] = (ns >= 1 && ns <= NUM_STEPS) ? ns : NUM_STEPS;
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

// All machines now share one engine: weight==100 = skeleton (always on, the
// "density 0" pattern); every other weight is the draw probability for the
// accumulating extras (density adds one weighted-random step at a time). Since
// no weight is 0, max density fills all 16 steps for every machine — so they
// converge to the same pattern at the top and only differ in how they fill in.

// KICK: 4-on-the-floor skeleton (0,4,8,12). Extras strongly favour the 8th-note
// offbeats (2,6,10,14, weight 70) so density splits 4/4 -> 8/8 first, with the
// other 1/16ths (weight 15) as occasional ~30% variation.
static const uint8_t W_KICK[16] = {
  100, 15, 70, 15, 100, 15, 70, 15, 100, 15, 70, 15, 100, 15, 70, 15
};

// HIHAT: skeleton on the offbeats (2,6,10,14). Fills the downbeats (0,4,8,12,
// weight 70) next, then the other 1/16ths (weight 40).
static const uint8_t W_HIHAT[16] = {
  70, 40, 100, 40, 70, 40, 100, 40, 70, 40, 100, 40, 70, 40, 100, 40
};

// SNARE: skeleton on the backbeats (4,12). Extras are ghost notes that cluster
// just before/after the backbeats.
static const uint8_t W_SNARE[16] = {
  15, 15, 20, 40, 100, 40, 20, 15, 15, 15, 20, 40, 100, 40, 20, 30
};

// ANTI-KICK: skeleton on the 1/8 offbeats (2,6,10,14). Other 1/16ths (40) fill
// before the downbeats (10).
static const uint8_t W_ANTIKICK[16] = {
  10, 40, 100, 40, 10, 40, 100, 40, 10, 40, 100, 40, 10, 40, 100, 40
};

// PERC: skeleton on every 1/8 (even steps). Extras fill the in-between 1/16ths.
static const uint8_t W_PERC[16] = {
  100, 50, 100, 50, 100, 50, 100, 50, 100, 50, 100, 50, 100, 50, 100, 50
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

// Number of non-skeleton steps a machine can accumulate (its max density).
static uint8_t machinePoolMax(uint8_t machine){
  const uint8_t* w = getMachineWeights(machine);
  if (!w) return 0;
  uint8_t n = 0;
  for (uint8_t s = 0; s < NUM_STEPS; s++) if (w[s] != 100) n++;
  return n;
}

void SimpleSequencer::regenerateMachinePattern(uint8_t ch){
  uint8_t m = trigMachine[ch];
  uint8_t shift = trigShift[ch] % NUM_STEPS;

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

  // Unified accumulation engine for every machine:
  //  - Lay the skeleton (weight==100 steps, shift-aware) = the density-0 sound.
  //  - Each density tick adds one new weighted-random extra to the stored
  //    sequence; lower weights come in less often, giving the machine its
  //    character. Decreasing density plays fewer entries; increasing again
  //    restores them. Dropping to 0 clears the list so it re-seeds fresh.
  for (uint8_t s = 0; s < NUM_STEPS; s++){
    uint8_t src = (s + NUM_STEPS - shift) % NUM_STEPS;
    machinePattern[ch][s] = (w[src] == 100);
    machineRatchet[ch][s] = 0;
  }

  uint8_t poolMax = machinePoolMax(m);
  uint8_t n = trigDensity[ch];
  if (n > poolMax) n = poolMax;
  if (n == 0) machineExtraCount[ch] = 0; // re-seed on the way back up

  // Grow the stored accumulation with weighted draws (without replacement)
  // over source positions, using the machine weights as probabilities.
  if (n > machineExtraCount[ch]){
    bool used[NUM_STEPS] = {false};
    for (uint8_t s = 0; s < NUM_STEPS; s++) if (w[s] == 100) used[s] = true;
    for (uint8_t i = 0; i < machineExtraCount[ch]; i++) used[machineExtraSeq[ch][i]] = true;
    while (n > machineExtraCount[ch] && machineExtraCount[ch] < NUM_STEPS){
      uint32_t total = 0;
      for (uint8_t s = 0; s < NUM_STEPS; s++) if (!used[s]) total += w[s];
      if (total == 0) break;
      uint32_t r = (uint32_t)random(0, (long)total);
      uint8_t pick = 255;
      for (uint8_t s = 0; s < NUM_STEPS; s++){
        if (used[s]) continue;
        if (r < w[s]){ pick = s; break; }
        r -= w[s];
      }
      if (pick == 255) break;
      used[pick] = true;
      machineExtraSeq[ch][machineExtraCount[ch]++] = pick;
    }
  }
  // Render the first n stored extras (shift applied).
  uint8_t render = (n < machineExtraCount[ch]) ? n : machineExtraCount[ch];
  for (uint8_t i = 0; i < render; i++){
    uint8_t visPos = (machineExtraSeq[ch][i] + shift) % NUM_STEPS;
    machinePattern[ch][visPos] = true;
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
// Operates on the current edit page only. Slides cluster: after a slide step
// the next-step probability is boosted by +50% (capped at 100%) to recreate
// the 303-style consecutive-slide phrasing.
void SimpleSequencer::rerollSlides(uint8_t ch){
  uint8_t prob = randomSlideProb[ch];
  uint16_t base = (uint16_t)editPage[ch] * NUM_STEPS;
  bool prevSlide = false;
  for (uint8_t s = 0; s < NUM_STEPS; s++){
    int eff = (int)prob + (prevSlide ? 50 : 0);
    if (eff > 100) eff = 100;
    bool sl = (random(0, eff > 0 ? 100 : 1) < eff);
    stepSlide[ch][base + s] = sl;
    prevSlide = sl;
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

void SimpleSequencer::duplicatePageContent(uint8_t ch, uint8_t srcPg, uint8_t dstPg){
  if (srcPg >= MAX_PAGES || dstPg >= MAX_PAGES || srcPg == dstPg) return;
  uint16_t sb = (uint16_t)srcPg * NUM_STEPS;
  uint16_t db = (uint16_t)dstPg * NUM_STEPS;
  for (uint8_t s = 0; s < NUM_STEPS; s++){
    steps[ch][db + s]            = steps[ch][sb + s];
    pitch[ch][db + s]            = pitch[ch][sb + s];
    noteLen[ch][db + s]          = noteLen[ch][sb + s];
    stepRatchet[ch][db + s]      = stepRatchet[ch][sb + s];
    stepVelocity[ch][db + s]     = stepVelocity[ch][sb + s];
    stepSlide[ch][db + s]        = stepSlide[ch][sb + s];
    fillState[ch][db + s]        = fillState[ch][sb + s];
    machineOverlay[ch][db + s]   = machineOverlay[ch][sb + s];
    euclidPattern[ch][db + s]    = euclidPattern[ch][sb + s];
  }
}

void SimpleSequencer::growPagesAndDuplicate(uint8_t ch, uint8_t target){
  if (target > MAX_PAGES) target = MAX_PAGES;
  if (target <= numPages[ch]) return;
  uint8_t oldN = numPages[ch];
  for (uint8_t pg = oldN; pg < target; pg++){
    duplicatePageContent(ch, 0, pg);
  }
  numPages[ch] = target;
}

void SimpleSequencer::randomizeEuclidMelody(uint8_t ch) {
  uint8_t mode = euclidScaleMode[ch];
  uint8_t totalPages = numPages[ch];
  if (totalPages < 1) totalPages = 1;

  if (mode == 0) {
    // Clear generated note overrides on every active page.
    for (uint8_t pg = 0; pg < totalPages; pg++){
      uint16_t base = (uint16_t)pg * NUM_STEPS;
      for (uint8_t s = 0; s < NUM_STEPS; s++) {
        pitch[ch][base + s]        = 255;
        stepSlide[ch][base + s]    = false;
        stepVelocity[ch][base + s] = 255;
        noteLen[ch][base + s]      = 255;
      }
    }
    return;
  }

  uint8_t root = channelPitch[ch];
  const uint8_t* scale;
  uint8_t size = getScale(mode, &scale);

  // octaveSpread acts in semitones (0..60). For each step we sample a
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

  // Probability the picked note is forced to be the root (gives the melody a
  // tonal centre regardless of spread).
  const int ROOT_BIAS_PERCENT = 35;

  // Contour bias (-100..+100): when non-zero, notes walk up/down the scale
  // instead of being sampled independently. Stronger bias -> more consistently
  // ascending (positive) or descending (negative). Root anchoring fades out as
  // the bias grows so a full sweep can run cleanly.
  int contour = contourBias[ch];

  // Fill every active page with fresh notes (each page gets independent picks).
  for (uint8_t pg = 0; pg < totalPages; pg++){
    uint16_t base = (uint16_t)pg * NUM_STEPS;
    // Slide clustering state — reset per page so a page can start fresh.
    bool prevSlide = false;
    // Contour walk position (scale-degree index into valid[]). Ascending bias
    // starts low and climbs; descending starts high and falls.
    int cur = (contour > 0) ? 0 : (contour < 0 ? (nValid - 1) : 0);
    for (uint8_t s = 0; s < NUM_STEPS; s++) {
      // Exponential bias: pick from valid[] with weight exp(-i/T) where T is
      // half the count. Falls back to uniform when spread = 0. With a 35%
      // probability we override the pick to the root (offset 0) so the
      // tonic recurs often enough to anchor the phrase.
      uint8_t pick;
      if (contour != 0 && spread != 0 && nValid > 1){
        int absC = contour < 0 ? -contour : contour;
        int rootProb = (ROOT_BIAS_PERCENT * (100 - absC)) / 100;
        if ((int)random(0, 100) < rootProb){
          pick = 0; cur = 0;
        } else {
          int pUp = 50 + contour / 2;          // +100 -> always up, -100 -> always down
          if (pUp < 0) pUp = 0;
          if (pUp > 100) pUp = 100;
          bool up = (int)random(0, 100) < pUp;
          int stepDeg = (int)random(1, 3);      // move 1..2 scale degrees
          cur += up ? stepDeg : -stepDeg;
          if (cur < 0) cur = 0;
          if (cur > nValid - 1) cur = nValid - 1;
          pick = (uint8_t)cur;
        }
      } else {
        bool forceRoot = (random(0, 100) < ROOT_BIAS_PERCENT);
        if (forceRoot || spread == 0 || nValid <= 1){
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
      }
      int note = (int)root + (int)valid[pick];
      pitch[ch][base + s] = (uint8_t)constrain(note, 0, 127);

      // Slide clustering: a slide on the previous step boosts the next-step
      // slide probability by +50% (capped at 100). Recreates 303-style
      // consecutive-slide phrasing without forcing the user to set it high.
      int slideProb = (int)randomSlideProb[ch] + (prevSlide ? 50 : 0);
      if (slideProb > 100) slideProb = 100;
      bool sl = slideProb > 0 ? (random(0, 100) < slideProb) : false;
      stepSlide[ch][base + s] = sl;
      prevSlide = sl;

      int v = (int)channelVelocity[ch] + random(-10, 10);
      stepVelocity[ch][base + s] = (uint8_t)constrain(v, 0, 127);
      noteLen[ch][base + s]      = noteLenIdx[ch];
      // Note: steps[] is intentionally NOT touched here. Whether a step fires
      // is the user's rhythm decision (manual toggle or Euclid). Generative
      // mode only paints the pitches.
    }
  }
}

// Pick one random active step on the current edit page and apply a single
// mutation: toggle slide, toggle accent, change note length, or change note
// value. Used for "subtle evolution" — keeps the pattern intact and just
// nudges one note.
void SimpleSequencer::mutatePattern(uint8_t ch){
  uint16_t base = (uint16_t)editPage[ch] * NUM_STEPS;
  uint8_t actives[NUM_STEPS];
  uint8_t nA = 0;
  for (uint8_t s = 0; s < NUM_STEPS; s++){
    // Only mutate notes that are actually being triggered by the sequencer:
    // honour machine overlays/patterns and skip anti-fill steps so we never
    // touch a step that won't sound in the current pattern.
    if (!isStepActive(ch, base + s)) continue;
    if (fillState[ch][base + s] == 2) continue; // anti-fill: never plays
    actives[nA++] = s;
  }
  if (nA == 0){
    Serial.print("MUTATE CH"); Serial.print(ch+1);
    Serial.println(" -> no active (triggered) steps");
    return;
  }
  uint8_t pickStep = actives[random(0, nA)];
  uint16_t idx = base + pickStep;
  uint8_t mutation = (uint8_t)random(0, 4);
  switch (mutation){
    case 0: { // Toggle slide
      stepSlide[ch][idx] = !stepSlide[ch][idx];
      Serial.print("MUT slide CH"); Serial.print(ch+1);
      Serial.print(" step "); Serial.print(pickStep+1);
      Serial.println(stepSlide[ch][idx] ? " ON" : " OFF");
      break;
    }
    case 1: { // Toggle accent (max vs default velocity)
      uint8_t curV = stepVelocity[ch][idx];
      if (curV == 255) curV = channelVelocity[ch];
      if (curV >= 120){
        stepVelocity[ch][idx] = channelVelocity[ch];
        Serial.print("MUT accent OFF CH");
      } else {
        stepVelocity[ch][idx] = 127;
        Serial.print("MUT accent ON CH");
      }
      Serial.print(ch+1);
      Serial.print(" step "); Serial.println(pickStep+1);
      break;
    }
    case 2: { // Change note length to a different index
      uint8_t cur = noteLen[ch][idx];
      if (cur == 255) cur = noteLenIdx[ch];
      uint8_t next = cur;
      if (NOTE_LEN_COUNT > 1){
        do { next = (uint8_t)random(0, NOTE_LEN_COUNT); } while (next == cur);
      }
      noteLen[ch][idx] = next;
      Serial.print("MUT len CH"); Serial.print(ch+1);
      Serial.print(" step "); Serial.print(pickStep+1);
      Serial.print(" -> "); Serial.println(next);
      break;
    }
    case 3: { // Change note value (re-pick from active scale)
      uint8_t mode = euclidScaleMode[ch];
      if (mode == 0) mode = lastScaleMode[ch] > 0 ? lastScaleMode[ch] : 1;
      const uint8_t* scale;
      uint8_t size = getScale(mode, &scale);
      uint8_t SPRD = octaveSpread[ch];
      if (SPRD > 60) SPRD = 60;
      if (SPRD == 0) SPRD = 12; // mutation always allows at least one octave
      uint8_t valid[64];
      uint8_t nValid = 0;
      for (int oct = 0; oct * 12 <= (int)SPRD && nValid < 64; oct++){
        for (uint8_t i = 0; i < size && nValid < 64; i++){
          int off = oct * 12 + (int)scale[i];
          if (off > (int)SPRD) break;
          valid[nValid++] = (uint8_t)off;
        }
      }
      if (nValid == 0){ valid[0] = 0; nValid = 1; }
      uint8_t pick = (uint8_t)random(0, nValid);
      int note = (int)channelPitch[ch] + (int)valid[pick];
      pitch[ch][idx] = (uint8_t)constrain(note, 0, 127);
      Serial.print("MUT note CH"); Serial.print(ch+1);
      Serial.print(" step "); Serial.print(pickStep+1);
      Serial.print(" -> "); Serial.println(pitch[ch][idx]);
      break;
    }
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

  // 0) Advance rate-ramp envelope. Smoothly interpolates rateCurrent from
  // rateRampFrom toward rateTarget over rateRampDurMs (linear).
  if (rateRamping){
    uint32_t elapsed = nowMs - rateRampStartMs;
    float prevRate = rateCurrent;
    if (elapsed >= rateRampDurMs){
      rateCurrent = rateTarget;
      rateRamping = false;
    } else {
      float t = (float)elapsed / (float)rateRampDurMs;
      rateCurrent = rateRampFrom + (rateTarget - rateRampFrom) * t;
    }
    // Update timer interval if the internal clock is driving playback and
    // rate actually changed meaningfully (avoid hammering update()).
    if (!externalMidiClockActive && midiTimerRunning && fabsf(rateCurrent - prevRate) > 0.001f){
      float eff = (float)bpm * rateCurrent;
      if (eff < 1.0f) eff = 1.0f;
      uint32_t interval = (uint32_t)(2500000.0f / eff);
      midiClockTimer.update(interval);
    }
  }

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
        bool isActive = isStepActive(ch, playIdx(ch, localStep(ch)));
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
        float eff = (float)bpm * rateCurrent;
        if (eff < 1.0f) eff = 1.0f;
        uint32_t interval = (uint32_t)(2500000.0f / eff);
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
      // NOTE: do NOT regenerate machine patterns on page wrap. Random
      // placements (kick extras, spread, ratchet rolls) are baked in when
      // the user touches a knob and stay stable until the next edit.
      // trigger channels that have the step enabled
        for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
          bool isActive = isStepActive(ch, playIdx(ch, localStep(ch)));
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
  if (vel == 255){
    vel = channelVelocity[ch];
    // Random velocity only jitters notes that follow the channel default,
    // leaving per-step (p-locked) velocities fixed.
    if (randomVelEnabled[ch]){
      int lo = (int)vel - RANDOM_VEL_RANGE;
      int hi = (int)vel + RANDOM_VEL_RANGE;
      if (lo < 0) lo = 0;
      if (hi > 127) hi = 127;
      vel = (uint8_t)random(lo, hi + 1);
    }
  }
  // Accent-all (Function + Page) forces max velocity on the active channel.
  if (accentAllHold && ch == selectedChannel) vel = 127;
  // Slide-all (Function + Fill) forces slide on the active channel.
  bool slideNow = stepSlide[ch][pIdx] || encoderSlideHold ||
                  (slideAllHold && ch == selectedChannel);

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
  prevSlide[ch] = slideNow;

  // 3. RATCHET & GATE LENGTH
  uint8_t lenIdx = noteLen[ch][pIdx];
  if (lenIdx == 255){
    lenIdx = noteLenIdx[ch];
    // Random gate only affects notes that follow the channel default, leaving
    // per-step (p-locked) gate lengths fixed. Full 1/32..1 range.
    if (randomGateEnabled[ch]) lenIdx = (uint8_t)random(0, (int)NOTE_LEN_COUNT);
  }

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

    if (slideNow) {
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
  if (activeMenu == 6){ drawAnalogView();     return; }

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

  // ── DEBUG MODE ── (disabled: FN+FILL is now the slide-all performance modifier)
  bool debugHold = false;
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

  // Playhead lights up only when the playback page matches the page the
  // user is currently editing — so editing page 2 while page 1 is playing
  // shows page 2's pattern with no white light moving across.
  uint8_t selCh   = selectedChannel;
  uint8_t playPg  = (numPages[selCh] > 0) ? (uint8_t)(globalPage % numPages[selCh]) : 0;
  bool pageMatchesPlay = (playPg == editPage[selCh]);
  uint8_t lstep = localStep(selCh);

  for (uint8_t s = 0; s < LED_COUNT && s < NUM_STEPS; s++){
    bool active = isStepActive(selCh, editIdx(selCh, s));
    bool isPlayhead = isRunning && pageMatchesPlay && (s == lstep);

    if (isPlayhead){
      ledStrip.setPixelColor(s, ledStrip.Color(255, 255, 255));
    } else if (active){
      ledStrip.setPixelColor(s, ledStrip.Color(cr, cg, cb));
    } else {
      // Dim hint so the grid is visible even at rest
      ledStrip.setPixelColor(s, ledStrip.Color(cr / 16, cg / 16, cb / 16));
    }

    // Decorations for Menu 4 overlay state and fill marks — kept steady so the
    // grid doesn't flash/animate when Fill is held.
    uint16_t eI = editIdx(selCh, s);
    if (activeMenu == 4 && trigMachine[selCh] != TM_OFF){
      uint8_t ov = machineOverlay[selCh][eI];
      if (!isPlayhead){
        if (ov == 1){
          ledStrip.setPixelColor(s, ledStrip.Color(180, 180, 180));
        } else if (ov == 2){
          ledStrip.setPixelColor(s, ledStrip.Color(40, 0, 0));
        }
      }
    } else if (fillState[selCh][eI] == 1 && !isPlayhead){
      // Fill-only step: steady dim green so you can see fills exist.
      ledStrip.setPixelColor(s, ledStrip.Color(0, 70, 0));
    }
  }

  // ── Channel LEDs (idx 23..29 on the shared chain) ───────────────
  // Selected channel: GREEN if unmuted, YELLOW if muted.
  // Every other channel: RED if unmuted, OFF if muted.
  for (uint8_t c = 0; c < NUM_CHANNELS; c++){
    uint32_t col;
    if (c == selectedChannel){
      col = muted[c] ? ledStrip.Color(255, 180, 0)   // yellow
                     : ledStrip.Color(0, 255, 0);     // green
    } else {
      col = muted[c] ? ledStrip.Color(0, 0, 0)        // off
                     : ledStrip.Color(255, 0, 0);     // red
    }
    ledStrip.setPixelColor(ledForChannel(c), col);
  }

  // ── Menu / modifier indicator LEDs (idx 16..22) ─────────────────
  // Clear the UI band, then light the active menu plus any held modifiers.
  for (uint8_t idx = LED_PAGE_INDEX; idx <= LED_FUNCTION_INDEX; idx++)
    ledStrip.setPixelColor(idx, 0);
  // Physical button -> activeMenu: Notes=MENU1(18), Step=MENU2(19),
  // Euclid=MENU3(20), TrigMachines=MENU4(21); Pages lights the Page LED (16).
  uint8_t menuLed = 255;
  switch (activeMenu){
    case 1: menuLed = LED_MENU_BASE + 0; break; // Notes
    case 6: menuLed = LED_MENU_BASE + 1; break; // Analog Outs (MENU2 button)
    case 2: menuLed = LED_MENU_BASE + 2; break; // Euclid
    case 4: menuLed = LED_MENU_BASE + 3; break; // Trigger Machines
    case 5: menuLed = LED_PAGE_INDEX;    break; // Pages
    default: break;
  }
  if (menuLed != 255) ledStrip.setPixelColor(menuLed, ledStrip.Color(120, 120, 120));
  // Modifier LEDs light while held.
  if (fillModeActive)   ledStrip.setPixelColor(LED_FILL_INDEX,     ledStrip.Color(0, 180, 40));
  if (isFunctionHeld()) ledStrip.setPixelColor(LED_FUNCTION_INDEX, ledStrip.Color(200, 120, 0));

  // When paused, slow-pulse Function + Menu 4 white to hint the play combo.
  if (!isRunning){
    float s = sinf((float)millis() * 0.004f) * 0.5f + 0.5f;
    uint8_t pw = (uint8_t)(s * 200.0f);
    ledStrip.setPixelColor(LED_FUNCTION_INDEX,  ledStrip.Color(pw, pw, pw));
    ledStrip.setPixelColor(LED_MENU_BASE + 3,   ledStrip.Color(pw, pw, pw));
  }

  // Slide-all / accent-all performance overlays take over the whole strip.
  if (accentAllHold){
    // All LEDs flash red.
    bool on = ((millis() / 120) % 2) == 0;
    uint32_t col = on ? ledStrip.Color(255, 0, 0) : 0;
    for (uint8_t i = 0; i < LED_COUNT; i++) ledStrip.setPixelColor(i, col);
  } else if (slideAllHold){
    // All LEDs ripple as a dark-purple wave.
    uint32_t t = millis();
    for (uint8_t i = 0; i < LED_COUNT; i++){
      float ph = sinf((float)i * 0.6f - (float)t * 0.012f) * 0.5f + 0.5f;
      uint8_t b = (uint8_t)(ph * 120.0f);
      ledStrip.setPixelColor(i, ledStrip.Color((uint8_t)(b * 0.55f), 0, b));
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

  // Centred boot card (size-1, 6px per char): "made by Bob and Zak" is 19
  // chars = 114px, so x=7 keeps it on-screen.
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(46, 20); display.print("seq-23");
  display.setCursor(7,  34); display.print("made by Bob and Zak");
  display.setCursor(28, 48); display.print("v. prototype");
  display.display();

  if (display2Present){
    display2.fillRect(0, 0, 128, 64, SH110X_WHITE);
    display2.setTextColor(SH110X_BLACK);
    display2.setTextSize(1);
    display2.setCursor(46, 20); display2.print("seq-23");
    display2.setCursor(7,  34); display2.print("made by Bob and Zak");
    display2.setCursor(28, 48); display2.print("v. prototype");
    display2.display();
  }
  delay(3000); // hold the boot card up for 3 seconds

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


// Serial 'v': bring-up check for the MAX11300 CV outputs. First sets each out
// to a distinct voltage (0 / 2.5 / 5 / 10V) so you can confirm which jack is
// which, then sweeps all outs together through those levels. Measure with a
// multimeter; blocking is fine for a bench test.
void SimpleSequencer::cvSelfTest(){
  ensurePixi();
  if (!pixiPresent){ Serial.println("CV self-test: PIXI not present"); return; }
  Serial.println("--- CV SELF-TEST (0-10V DACs) ---");
  const float levels[4] = {0.0f, 2.5f, 5.0f, 10.0f};

  Serial.println("Distinct: out0=0V out1=2.5V out2=5V out3=10V (extra outs=0)");
  for (uint8_t i = 0; i < NUM_CV_OUTS; i++){
    pixi.setVoltage0to10(CV_PORTS[i], (i < 4) ? levels[i] : 0.0f);
  }
  delay(3000);

  for (uint8_t L = 0; L < 4; L++){
    for (uint8_t i = 0; i < NUM_CV_OUTS; i++) pixi.setVoltage0to10(CV_PORTS[i], levels[L]);
    Serial.print("All CV outs = "); Serial.print(levels[L], 3); Serial.println(" V");
    delay(1500);
  }

  for (uint8_t i = 0; i < NUM_CV_OUTS; i++) pixi.setVoltage0to10(CV_PORTS[i], 0.0f);
  Serial.println("CV self-test done (outputs at 0V)");
}

// Serial 'x': read PIXI registers back over SPI to tell whether the bus is
// working at all. If the write/readback of port_cfg matches, SPI read+write is
// good and any "no voltage" problem is power/range/wiring on the analog side
// (e.g. AVDDIO/AVSSIO rails). If it mismatches or dev_id is 0x0000/0xFFFF, SPI
// itself isn't talking to the chip (MOSI/MISO/SCK/CS wiring or chip power).
void SimpleSequencer::pixiDiag(){
  // Full re-init every time: soft reset + reference + port config. This also
  // serves as a recovery path if a port latched up (e.g. after an output short).
  pixiInit = false;
  ensurePixi();
  Serial.println("--- PIXI DIAG ---");
  Serial.print("dev_id         = 0x"); Serial.println(pixi.readReg(Max11300::REG_DEVICE_ID), HEX);
  Serial.print("device_control = 0x"); Serial.println(pixi.readReg(Max11300::REG_DEVICE_CONTROL), HEX);
  // Fault status: interrupt flags (0x01) + DAC over-current status (0x04/0x05).
  // interrupt_flag bit 0x0020 = DACOI (a DAC hit its current limit at some point).
  Serial.print("interrupt_flag = 0x"); Serial.println(pixi.readReg(0x01), HEX);
  Serial.print("dac_oi_15_0    = 0x"); Serial.println(pixi.readReg(0x04), HEX);
  Serial.print("dac_oi_19_16   = 0x"); Serial.println(pixi.readReg(0x05), HEX);
  for (uint8_t i = 0; i < NUM_CV_OUTS; i++){
    uint8_t p = CV_PORTS[i];
    Serial.print("port_cfg[");  Serial.print(p); Serial.print("] = 0x");
    Serial.print(pixi.readReg(Max11300::REG_PORT_CFG_BASE + p), HEX);
    Serial.print("  dac_data[");  Serial.print(p); Serial.print("] = 0x");
    Serial.println(pixi.readReg(Max11300::REG_DAC_DATA_BASE + p), HEX);
  }
  // Write/readback test on the first port's config register.
  uint8_t p0 = CV_PORTS[0];
  pixi.writeReg(Max11300::REG_PORT_CFG_BASE + p0, 0x5100); // DAC, 0-10V range
  uint16_t rb = pixi.readReg(Max11300::REG_PORT_CFG_BASE + p0);
  Serial.print("port_cfg writeback 0x5100 -> 0x"); Serial.println(rb, HEX);
  Serial.println(rb == 0x5100 ? "SPI WRITE/READ OK"
                              : "SPI MISMATCH -> check MOSI/MISO/SCK/CS wiring + chip power");
  // Distinct voltages so each jack can be identified on a meter.
  pixi.configDac(p0, Max11300::RANGE_0_TO_10);
  setCvOut(0, 5.0f);
  if (NUM_CV_OUTS > 1) setCvOut(1, 2.5f);
  Serial.println("set out0=5.00V out1=2.50V (measure now)");
}

// Serial 'y': continuity/level test with just a multimeter. Drives CS/SCK/MOSI
// as plain GPIO (HIGH for 6s, then LOW for 6s) so you can probe those pins at
// the MAX11300 end and confirm the wire actually carries the level. Then reads
// MISO with a pull-up: reads 1 = line can go high (OK-ish); reads 0 = MISO is
// held low (shorted to GND, unpowered chip, or not connected to a driver).
void SimpleSequencer::pixiPinTest(){
  Serial.println("--- PIXI PIN TEST ---");
  pinMode(MAX_CS, OUTPUT);
  pinMode(MAX_SCK, OUTPUT);
  pinMode(MAX_MOSI, OUTPUT);
  Serial.println("CS(37) SCK(13) MOSI(11) = HIGH for 6s (should read ~3.3V at the chip)");
  digitalWrite(MAX_CS, HIGH); digitalWrite(MAX_SCK, HIGH); digitalWrite(MAX_MOSI, HIGH);
  delay(6000);
  Serial.println("CS(37) SCK(13) MOSI(11) = LOW for 6s (should read ~0V at the chip)");
  digitalWrite(MAX_CS, LOW); digitalWrite(MAX_SCK, LOW); digitalWrite(MAX_MOSI, LOW);
  delay(6000);

  pinMode(MAX_MISO, INPUT_PULLUP);
  delay(2);
  int misoPU = digitalRead(MAX_MISO);
  pinMode(MAX_MISO, INPUT);
  delay(2);
  int misoZ = digitalRead(MAX_MISO);
  Serial.print("MISO(12): with pull-up="); Serial.print(misoPU);
  Serial.print("  floating="); Serial.println(misoZ);
  Serial.println("  (pull-up=0 => MISO held low: chip unpowered / not driving / shorted)");

  // Restore SPI mux and allow ensurePixi to run again on the next 'x'.
  SPI.begin();
  pixiInit = false;
  pixiPresent = false;
  Serial.println("PIN TEST done. Re-run 'x' to retry SPI.");
}

// Reset ONE CV output after a latch-up (e.g. the jack got shorted): drop the
// port to high-impedance, reconfigure it as a 0-10V DAC, and rewrite its last
// voltage. Leaves every other output untouched.
void SimpleSequencer::resetCvOut(uint8_t idx){
  if (idx >= NUM_CV_OUTS) return;
  ensurePixi();
  if (!pixiPresent){ Serial.println("CV reset: PIXI not present"); return; }
  uint8_t p = CV_PORTS[idx];
  pixi.writeReg((uint8_t)(Max11300::REG_PORT_CFG_BASE + p), 0x0000); // HI-Z
  delay(2);
  pixi.configDac(p, Max11300::RANGE_0_TO_10);
  pixi.setVoltage0to10(p, cvVolts[idx]);
  Serial.print("CV"); Serial.print(idx + 1);
  Serial.print(" (P"); Serial.print(p); Serial.print(") reset, re-set to ");
  Serial.print(cvVolts[idx], 2); Serial.println("V");
}

// Full-chip recovery: soft-reset the MAX11300, reconfigure every CV port, and
// reapply the stored voltages. The big hammer when a single-port reset fails.
void SimpleSequencer::resetPixiAll(){
  pixiInit = false;
  ensurePixi();
  if (!pixiPresent){ Serial.println("PIXI reset: not present"); return; }
  for (uint8_t i = 0; i < NUM_CV_OUTS; i++){
    pixi.setVoltage0to10(CV_PORTS[i], cvVolts[i]);
  }
  Serial.println("PIXI full reset, all CV outs reapplied");
}

// Store + push a manual voltage to one CV output (0..10V). Safe to call when no
// PIXI is attached — it just keeps the stored value for the display.
void SimpleSequencer::setCvOut(uint8_t idx, float volts){
  if (idx >= NUM_CV_OUTS) return;
  if (volts < 0.0f) volts = 0.0f;
  if (volts > 10.0f) volts = 10.0f;
  cvVolts[idx] = volts;
  if (pixiPresent) pixi.setVoltage0to10(CV_PORTS[idx], volts);
}

// Menu 2 (MENU2 button): analog CV outputs. First version is a manual control
// surface — pots 1..NUM_CV_OUTS set each output's voltage (0..10V). Doubles as
// a no-serial bring-up test. Assignment modes (follow pitch/gate/etc.) TBD.
void SimpleSequencer::drawAnalogView(){
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  // Header: title + PIXI presence chip.
  display.setTextSize(1);
  display.setCursor(2, 1);
  display.print("ANALOG OUT");
  if (pixiPresent){
    display.fillRect(100, 0, 24, 9, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(106, 1); display.print("OK");
    display.setTextColor(SH110X_WHITE);
  } else {
    display.drawRect(100, 0, 24, 9, SH110X_WHITE);
    display.setCursor(104, 1); display.print("--");
  }
  display.drawFastHLine(0, 12, 128, SH110X_WHITE);

  // Up to 6 outs laid out in two rows of three to line up with the pots.
  const int colX[3] = {2, 45, 88};
  const int lblY1 = 16, valY1 = 26;
  const int lblY2 = 42, valY2 = 52;
  for (uint8_t i = 0; i < NUM_CV_OUTS && i < 6; i++){
    int col = i % 3;
    bool row2 = (i >= 3);
    int x = colX[col];
    int ly = row2 ? lblY2 : lblY1;
    int vy = row2 ? valY2 : valY1;
    // Label: CVn + the PIXI port it drives.
    display.setCursor(x, ly);
    display.print("CV"); display.print(i + 1);
    display.print(" P"); display.print(CV_PORTS[i]);
    // Value in volts (2 d.p.).
    display.setCursor(x, vy);
    display.print(cvVolts[i], 2); display.print("V");
  }

  // Footer hint: pot-button resets a stuck output (only when row 2 is free).
  if (NUM_CV_OUTS <= 3){
    display.setCursor(2, 56);
    display.print("PUSH POT=RST  P6=RST ALL");
  }

  display.display();
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
  machineExtraCount[ch] = 0;
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
  // Menu 1: Generative performance page for the selected channel.
  // Channel select/mute state now lives on the channel LEDs (see updateLEDs),
  // so the top of the screen is freed up for a bigger key/scale header and a
  // 2x3 parameter grid that lines up with the 6 physical pots.
  display.clearDisplay();
  const char* noteNames[]   = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  const char* scaleNames[]  = {"OFF","MAJOR","MINOR","PENTA","LOCR","DIM","ATONL"};
  uint8_t ch  = selectedChannel;
  uint8_t sm  = euclidScaleMode[ch];
  if (sm > 6) sm = 6;
  bool genOn = (sm != 0);

  display.setTextColor(SH110X_WHITE);

  // ── HEADER: big root note + scale + GEN/OFF chip ────────────────
  // Channel/mute state now shows on the channel LEDs, so no CH number here.
  uint8_t p = channelPitch[ch];
  display.setTextSize(2);
  display.setCursor(2, 0);
  display.print(noteNames[p % 12]); display.print((int)(p / 12) - 1);

  display.setTextSize(1);
  display.setCursor(50, 4);
  display.print(scaleNames[sm]);

  // GEN / OFF chip (top right) — stops at x=124
  if (genOn){
    display.fillRect(102, 0, 22, 11, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(105, 2);
    display.print("GEN");
    display.setTextColor(SH110X_WHITE);
  } else {
    display.drawRect(102, 0, 22, 11, SH110X_WHITE);
    display.setCursor(105, 2);
    display.print("OFF");
  }
  display.drawFastHLine(0, 16, 128, SH110X_WHITE);

  // ── 2x3 PARAM GRID (columns line up with the 6 pots) ────────────
  // Row 1 = pots 1-3 (KEY / SCALE / SPREAD), Row 2 = pots 4-6 (SLIDE / GATE / VEL).
  const int colX[3] = {2, 45, 88};
  const int lblY1 = 21, valY1 = 31; // row 1
  const int lblY2 = 44, valY2 = 54; // row 2 (54..60 fits under 64)
  display.setTextSize(1);

  // Row 1 labels
  display.setCursor(colX[0], lblY1); display.print("KEY");
  display.setCursor(colX[1], lblY1); display.print("SCALE");
  display.setCursor(colX[2], lblY1); display.print("SPRD");
  // Row 1 values
  display.setCursor(colX[0], valY1);
  display.print(noteNames[p % 12]); display.print((int)(p / 12) - 1);
  display.setCursor(colX[1], valY1); display.print(scaleNames[sm]);
  display.setCursor(colX[2], valY1); display.print(octaveSpread[ch]);

  // Row 2 labels
  display.setCursor(colX[0], lblY2); display.print("SLD");
  display.setCursor(colX[1], lblY2); display.print("GATE");
  display.setCursor(colX[2], lblY2); display.print("VEL");
  // Row 2 values
  display.setCursor(colX[0], valY2); display.print(randomSlideProb[ch]); display.print("%");
  display.setCursor(colX[1], valY2); display.print(noteLenNames[noteLenIdx[ch]]);
  if (randomGateEnabled[ch]) display.print(" R"); // random-gate indicator
  display.setCursor(colX[2], valY2); display.print(channelVelocity[ch]);
  if (randomVelEnabled[ch]) display.print("R"); // random-velocity indicator

  display.display();
}

void SimpleSequencer::drawEuclidView(){
  // Menu 3 (Euclid): formatted like the Notes page — a header (pattern grid +
  // ON/OFF chip) over a 2x3 parameter grid whose columns line up with the 6
  // pots: P1 PULSE  P2 OFST  P3 SCALE / P4 VEL  P5 GATE  P6 SLD.
  display.clearDisplay();
  uint32_t now = millis();
  display.setTextColor(SH110X_WHITE);
  const char* scaleNames[] = {"OFF","MAJ","MIN","PEN","LOC","DIM","ATO"};
  uint8_t ch = selectedChannel;
  bool en = euclidEnabled[ch];
  uint8_t sm = euclidScaleMode[ch]; if (sm > 6) sm = 6;

  // ── HEADER: 16-step pattern (left) + ON/OFF chip (right) ─────────
  const uint8_t sq = 5, gap = 1, sx = 2, sy = 1;
  uint8_t lstep = localStep(ch);
  for (uint8_t s = 0; s < NUM_STEPS; s++){
    int x = sx + s * (sq + gap);
    uint16_t eI = editIdx(ch, s);
    bool active = en ? euclidPattern[ch][eI] : steps[ch][eI];
    bool isHead = isRunning && (s == lstep);
    if (isHead){
      display.fillRect(x, sy, sq, sq, SH110X_WHITE);
      if ((now / 125) % 2 == 0) display.fillRect(x+1, sy+1, 3, 3, SH110X_BLACK);
    } else if (active){
      display.fillRect(x, sy, sq, sq, SH110X_WHITE);
    } else {
      display.drawRect(x, sy, sq, sq, SH110X_WHITE);
    }
  }
  if (en){
    display.fillRect(100, 0, 24, 9, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(105, 1); display.print("ON");
    display.setTextColor(SH110X_WHITE);
  } else {
    display.drawRect(100, 0, 24, 9, SH110X_WHITE);
    display.setCursor(103, 1); display.print("OFF");
  }
  display.drawFastHLine(0, 12, 128, SH110X_WHITE);

  // ── 2x3 PARAM GRID (columns line up with the 6 pots) ────────────
  const int colX[3] = {2, 45, 88};
  const int lblY1 = 16, valY1 = 26; // row 1 (pots 1-3)
  const int lblY2 = 42, valY2 = 52; // row 2 (pots 4-6)
  display.setTextSize(1);

  // Row 1 labels
  display.setCursor(colX[0], lblY1); display.print("PULS");
  display.setCursor(colX[1], lblY1); display.print("OFST");
  display.setCursor(colX[2], lblY1); display.print("SCALE");
  // Row 1 values
  display.setCursor(colX[0], valY1); display.print(pulses[ch]);
  display.setCursor(colX[1], valY1); display.print(euclidOffset[ch]);
  display.setCursor(colX[2], valY1); display.print(scaleNames[sm]);

  // Row 2 labels
  display.setCursor(colX[0], lblY2); display.print("VEL");
  display.setCursor(colX[1], lblY2); display.print("GATE");
  display.setCursor(colX[2], lblY2); display.print("SLD");
  // Row 2 values
  display.setCursor(colX[0], valY2); display.print(channelVelocity[ch]);
  display.setCursor(colX[1], valY2); display.print(noteLenNames[noteLenIdx[ch]]);
  display.setCursor(colX[2], valY2); display.print(randomSlideProb[ch]); display.print("%");

  display.display();
}

void SimpleSequencer::drawStepVisualiser(){
  // Original single-channel grid layout, restored.
  display.clearDisplay();
  uint32_t now = millis();

  // Channel + mute state now shown on the channel LEDs, so no tab strip here.
  display.setTextColor(SH110X_WHITE);

  // ── STEP GRID: 16 steps in 2 rows of 8 ───────────────────────
  // Each cell is 14px wide x 16px tall with 2px gap. gridX=1 to fit cleanly.
  const uint8_t cellW = 14, cellH = 16, gapX = 2, gapY = 3;
  const uint8_t gridX = 1, gridY = 6;

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

  // Channel + mute state now shown on the channel LEDs, so no tab strip here.
  display.setTextColor(SH110X_WHITE);

  // Param row: machine | density | shift  (+ kick extras when KICK is active)
  uint8_t ch = selectedChannel;
  uint8_t m = trigMachine[ch];
  if (m >= TM_COUNT) m = 0;
  uint8_t poolMax = machinePoolMax(m);
  display.setTextSize(1);
  if (m == TM_KICK){
    // Compact kick layout — fits all six in 120 px. Density shows as N/max.
    display.setCursor(2, 2);
    display.print("KCK D"); display.print(trigDensity[ch]); display.print("/"); display.print(poolMax);
    display.print(" S"); display.print(trigShift[ch]);
    display.print(" P"); display.print(kickNoteSpread[ch]);
    display.print(" R"); display.print(kickRatchetProb[ch]);
    display.print(kickExtrasAreFills[ch] ? " F" : "");
  } else {
    display.setCursor(2, 2);
    display.print("M:"); display.print(machineNames[m]);
    display.setCursor(56, 2);
    display.print("D:"); display.print(trigDensity[ch]);
    if (poolMax > 0){ display.print("/"); display.print(poolMax); }
    display.setCursor(104, 2);
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
  const uint8_t startX = 4, startY = 16;
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

  // Mode badge (GLOBAL or CHANNEL) + big page indicator
  uint8_t ch = selectedChannel;
  display.setTextSize(1);
  display.setCursor(2, 12);
  if (pageEditGlobal){
    display.print("GLOBAL");
  } else {
    display.print("CH"); display.print(ch + 1);
  }

  // Big page number — depends on which mode is active
  //   Global mode  : current playing page out of MAX_PAGES
  //   Channel mode : the channel's pattern length (Pot1) with current edit
  //                  page shown smaller above. Page button taps cycle the
  //                  edit page within numPages.
  display.setTextSize(3);
  char buf[12];
  if (pageEditGlobal){
    snprintf(buf, sizeof(buf), "P%u/%u", (unsigned)(globalPage + 1), (unsigned)MAX_PAGES);
  } else {
    // Big: pattern length. Small annotation above shows edit page.
    snprintf(buf, sizeof(buf), "%uPGS", (unsigned)numPages[ch]);
    display.setTextSize(1);
    display.setCursor(96, 12);
    display.print("EDIT P");
    display.print((int)(editPage[ch] + 1));
    display.setTextSize(3);
  }
  int tw = (int)strlen(buf) * 18;
  display.setCursor((128 - tw) / 2, 22);
  display.print(buf);

  // Footer row: STEPS  RATE
  display.setTextSize(1);
  display.setCursor(2, 50);
  display.print("STEPS "); display.print(numSteps[ch]);

  display.setCursor(64, 50);
  // Show target rate (with a subtle "..." while ramping)
  char rbuf[10];
  float r = RATE_VALUES[rateIdx];
  if (r >= 1.0f) snprintf(rbuf, sizeof(rbuf), "%.0fx", (double)r);
  else           snprintf(rbuf, sizeof(rbuf), "%.2fx", (double)r);
  display.print("RATE "); display.print(rbuf);
  if (rateRamping) display.print("..");

  // Bottom hint line — wording depends on the active scope
  display.setCursor(2, 58);
  if (pageEditGlobal){
    display.print("P1 page  P2 steps  P3 rate");
  } else {
    display.print("P1 pages P2 steps  P3 rate");
  }

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
  // Icons shrunk to a 22x22 bounding box so the layout never clips the top
  // of the screen and there is breathing room above the machine title.
  switch (machine){
    case SimpleSequencer::TM_KICK: {
      d.fillCircle(cx, cy, 11, SH110X_WHITE);
      break;
    }
    case SimpleSequencer::TM_HIHAT: {
      for (int o = -1; o <= 1; o++){
        d.drawLine(cx - 10 + o, cy - 10, cx + 10 + o, cy + 10, SH110X_WHITE);
        d.drawLine(cx + 10 + o, cy - 10, cx - 10 + o, cy + 10, SH110X_WHITE);
      }
      break;
    }
    case SimpleSequencer::TM_SNARE: {
      d.fillTriangle(cx, cy - 11, cx - 11, cy + 9, cx + 11, cy + 9, SH110X_WHITE);
      break;
    }
    case SimpleSequencer::TM_ANTIKICK: {
      d.drawCircle(cx, cy, 11, SH110X_WHITE);
      d.drawCircle(cx, cy, 10, SH110X_WHITE);
      d.fillCircle(cx, cy, 3, SH110X_WHITE);
      break;
    }
    case SimpleSequencer::TM_PERC: {
      d.fillCircle(cx - 6, cy - 6, 3, SH110X_WHITE);
      d.fillCircle(cx + 6, cy - 6, 3, SH110X_WHITE);
      d.fillCircle(cx - 6, cy + 6, 3, SH110X_WHITE);
      d.fillCircle(cx + 6, cy + 6, 3, SH110X_WHITE);
      d.fillCircle(cx, cy, 2, SH110X_WHITE);
      break;
    }
    case SimpleSequencer::TM_EUCLID: {
      d.drawCircle(cx, cy, 11, SH110X_WHITE);
      d.drawCircle(cx, cy, 7, SH110X_WHITE);
      d.drawCircle(cx, cy, 3, SH110X_WHITE);
      break;
    }
    default: { // TM_OFF — two thick horizontal lines (mute mark)
      d.fillRect(cx - 10, cy - 2, 20, 4, SH110X_WHITE);
      break;
    }
  }
}

void SimpleSequencer::drawNotesKeyboard(){
  // Secondary OLED for Menu 1: piano-roll of the selected channel's notes.
  // X = steps (only the channel's active pattern length), Y = pitch (higher =
  // nearer the top), block width = note length (clipped to the next note so
  // bars don't overlap). When running it follows the page that's PLAYING, so a
  // multi-page channel scrolls through its pages; when stopped it shows the
  // edit page. Only steps that actually trigger are drawn (isStepActive).
  display2.clearDisplay();
  display2.setTextColor(SH110X_WHITE);
  static const char* noteNames[]  = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  static const char* scaleNames[] = {"OFF","MAJOR","MINOR","PENTA","LOCR","DIM","ATONL"};
  uint8_t ch = (heldChannel >= 0) ? (uint8_t)heldChannel : selectedChannel;
  uint8_t sm = euclidScaleMode[ch]; if (sm > 6) sm = 6;

  // Page to show: the playing page while running, otherwise the edit page.
  uint8_t playPg = (numPages[ch] > 0) ? (uint8_t)(globalPage % numPages[ch]) : 0;
  uint8_t dispPg = isRunning ? playPg : editPage[ch];
  uint16_t base  = (uint16_t)dispPg * NUM_STEPS;
  uint8_t nSteps = numSteps[ch]; if (nSteps < 1) nSteps = 1; if (nSteps > NUM_STEPS) nSteps = NUM_STEPS;

  // Header: root note, scale, page (channel shown on the LEDs now).
  display2.setTextSize(1);
  uint8_t rootN = channelPitch[ch];
  display2.setCursor(2, 1);
  display2.print(noteNames[rootN % 12]); display2.print((int)(rootN / 12) - 1);
  display2.setCursor(44, 1);
  display2.print(scaleNames[sm]);
  display2.setCursor(98, 1);
  display2.print("P"); display2.print(dispPg + 1); display2.print("/"); display2.print(numPages[ch]);
  display2.drawFastHLine(0, 10, 128, SH110X_WHITE);

  const int plotTop = 13, plotBot = 62; // 1px bottom margin
  const int plotH = plotBot - plotTop;
  // Column width scales to the pattern length so all steps fill the width.
  // Reserve a strip on the right for the contour-bias arrow.
  const int x0 = 1;
  const int plotW = 116;

  // ── Contour-bias arrow (Function + encoder 3) ───────────────────
  // Grows upward for an ascending bias, downward for descending.
  {
    int cb = contourBias[ch];
    int axc = 123;
    int midY = (plotTop + plotBot) / 2;
    int maxLen = (plotH / 2) - 1;
    int len = (abs(cb) * maxLen) / 100;
    if (cb == 0){
      display2.drawFastHLine(axc - 2, midY, 5, SH110X_WHITE);
    } else if (cb > 0){
      display2.drawFastVLine(axc, midY - len, len, SH110X_WHITE);
      display2.drawLine(axc, midY - len, axc - 2, midY - len + 3, SH110X_WHITE);
      display2.drawLine(axc, midY - len, axc + 2, midY - len + 3, SH110X_WHITE);
    } else {
      display2.drawFastVLine(axc, midY, len, SH110X_WHITE);
      display2.drawLine(axc, midY + len, axc - 2, midY + len - 3, SH110X_WHITE);
      display2.drawLine(axc, midY + len, axc + 2, midY + len - 3, SH110X_WHITE);
    }
  }

  // Collect the triggered notes on the display page and find the pitch span.
  uint8_t pv[NUM_STEPS], wd[NUM_STEPS];
  bool act[NUM_STEPS];
  int loP = 127, hiP = 0; bool any = false;
  for (uint8_t s = 0; s < nSteps; s++){
    uint16_t eI = base + s;
    act[s] = isStepActive(ch, eI);
    if (!act[s]) continue;
    uint8_t pitchV = (pitch[ch][eI] == 255) ? channelPitch[ch] : pitch[ch][eI];
    uint8_t li     = (noteLen[ch][eI] == 255) ? noteLenIdx[ch] : noteLen[ch][eI];
    if (li >= NOTE_LEN_COUNT) li = noteLenIdx[ch];
    uint8_t w = noteLenTicks[li] / 6; // 6 ticks == one 1/16 step
    if (w < 1) w = 1;
    if (w > nSteps) w = nSteps;
    pv[s] = pitchV; wd[s] = w;
    if (pitchV < loP) loP = pitchV;
    if (pitchV > hiP) hiP = pitchV;
    any = true;
  }

  if (!any){
    display2.setCursor(16, 34);
    display2.print("(no active notes)");
    display2.display();
    return;
  }

  // Pad the range a touch and enforce a minimum span so single-note patterns
  // don't draw a giant block across the whole screen.
  loP -= 1; hiP += 1; if (loP < 0) loP = 0;
  int range = hiP - loP;
  if (range < 4){ int c = (hiP + loP) / 2; loP = c - 2; hiP = c + 2; if (loP < 0){ hiP -= loP; loP = 0; } range = hiP - loP; }
  if (range < 1) range = 1;

  // Helper: x pixel for the start of step s (columns scaled to nSteps).
  auto colX = [&](int s){ return x0 + (s * plotW) / nSteps; };

  // Playhead column.
  if (isRunning){
    int px = colX(localStep(ch));
    display2.drawFastVLine(px, plotTop, plotH, SH110X_WHITE);
  }

  // Draw the note blocks. Width is the note length but clipped to the next
  // active step so adjacent bars read as separate notes.
  const int bh = 3;
  for (uint8_t s = 0; s < nSteps; s++){
    if (!act[s]) continue;
    // distance to the next active step (cap the bar there)
    int span = wd[s];
    for (int d = 1; d < (int)nSteps; d++){
      if (act[(s + d) % nSteps]){ if (d < span) span = d; break; }
    }
    int x = colX(s) + 1;
    int xEnd = colX(s + span); // start of the step after this note ends
    int w = xEnd - x - 1; if (w < 2) w = 2; if (x + w > x0 + plotW) w = x0 + plotW - x;
    int y = plotBot - (int)((long)(pv[s] - loP) * (plotH - bh) / range);
    if (y < plotTop) y = plotTop;
    if (y > plotBot - bh) y = plotBot - bh;
    display2.fillRect(x, y, w, bh, SH110X_WHITE);
  }

  display2.display();
}

void SimpleSequencer::drawOverview(){
  display2.clearDisplay();
  display2.setTextColor(SH110X_WHITE);
  uint32_t now = millis();
  uint8_t ch = (heldChannel >= 0) ? (uint8_t)heldChannel : selectedChannel;

  // ── Menu 2: Analog CV outputs — level bars (0..10V) ─────────────
  if (activeMenu == 6){
    display2.setTextSize(1);
    display2.setCursor(2, 1);
    display2.print("ANALOG OUT");
    display2.setCursor(96, 1);
    display2.print(pixiPresent ? "OK" : "--");
    display2.drawFastHLine(0, 11, 128, SH110X_WHITE);

    const int n = NUM_CV_OUTS;
    const int top = 14, bot = 56, h = bot - top;
    int slot = 128 / (n > 0 ? n : 1);
    int bw = slot - 8; if (bw < 6) bw = 6;
    for (int i = 0; i < n; i++){
      int x = i * slot + (slot - bw) / 2;
      display2.drawRect(x, top, bw, h, SH110X_WHITE);
      int fh = (int)(cvVolts[i] / 10.0f * (h - 2) + 0.5f);
      if (fh > h - 2) fh = h - 2;
      if (fh > 0) display2.fillRect(x + 1, bot - 1 - fh, bw - 2, fh, SH110X_WHITE);
      display2.setCursor(x, bot + 2);
      display2.print(i + 1);
    }
    display2.display();
    return;
  }

  // ── Menu 4: Trigger Machines — stripped-down focus view ─────────
  if (activeMenu == 4){
    uint8_t m = trigMachine[ch];
    if (m >= TM_COUNT) m = 0;
    static const char* names[TM_COUNT] = {
      "OFF", "KICK", "HIHAT", "SNARE", "ANTIKICK", "PERC", "EUCLID"
    };
    // Layout (KICK shown to scale, 64x64):
    //   y0..23   icon (centred at 64,12, radius 11 — small enough not to
    //            overlap the title below).
    //   y26..40  machine name in size-2 text, centred.
    //   y43..51  KICK live-params row (Spread / Ratchet / Fill chip).
    //   y54..62  density fill bar (Digitone-style segments).
    // Non-KICK layouts skip the kick row and let the density bar sit at the
    // same bottom slot.

    drawMachineIcon(display2, 64, 12, m);

    const char* nm = names[m];
    int textW = (int)strlen(nm) * 12;
    int tx = (128 - textW) / 2; if (tx < 0) tx = 0;
    display2.setTextSize(2);
    display2.setCursor(tx, 26);
    display2.print(nm);

    // Density fill bar — one segment per slot in the machine's extra pool, so
    // the bar fills one notch per added trigger.
    const int barX = 8, barY = 54, barW = 112, barH = 8;
    display2.drawRect(barX, barY, barW, barH, SH110X_WHITE);
    int dens = trigDensity[ch];
    int maxD = machinePoolMax(m); if (maxD < 1) maxD = 1;
    int segs = maxD;
    int fillW = ((dens * (barW - 2)) + maxD/2) / maxD;
    if (fillW > barW - 2) fillW = barW - 2;
    if (fillW > 0){
      display2.fillRect(barX + 1, barY + 1, fillW, barH - 2, SH110X_WHITE);
      for (int i = 1; i < segs; i++){
        int sx = barX + 1 + (i * (barW - 2)) / segs;
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

    // When Euclid is off the dotted ring already reads as "off", so skip the
    // redundant OFF label and the empty pulses bar. Only show the status line
    // and the fill bar while it's enabled.
    if (enabled){
      // "P 6/16  O 2" status line
      display2.setTextColor(SH110X_WHITE);
      char buf[16];
      snprintf(buf, sizeof(buf), "P %u/16  O %u",
               (unsigned)pulses[ch], (unsigned)euclidOffset[ch]);
      int tw = (int)strlen(buf) * 6;
      display2.setTextSize(1);
      display2.setCursor((128 - tw) / 2, 46);
      display2.print(buf);

      // Pulses fill bar at the bottom (Digitone-style 16 segments)
      const int barX = 4, barY = 56, barW = 120, barH = 7;
      display2.drawRect(barX, barY, barW, barH, SH110X_WHITE);
      int p = pulses[ch];
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
    }

    display2.display();
    return;
  }

  // ── Menu 1: Notes — piano-roll of the channel's notes on the edit page ──
  if (activeMenu == 1){
    drawNotesKeyboard();
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

  // ── Menu 5: Pages mode — per-channel page summary + global page ─
  if (activeMenu == 5){
    display2.setTextColor(SH110X_WHITE);
    display2.setTextSize(1);

    // Top: GLOBAL page indicator (big-ish on the left, current edit mode on right)
    display2.setCursor(2, 1);
    display2.print("GLOBAL P");
    display2.print((int)(globalPage + 1));
    display2.print("/");
    display2.print((int)MAX_PAGES);
    // Right edge: which scope Pot1 is editing right now
    display2.setCursor(86, 1);
    display2.print(pageEditGlobal ? "[GLB]" : "[CH ]");
    if (!pageEditGlobal){
      display2.setCursor(110, 1);
      display2.print((int)(selectedChannel + 1));
    }
    display2.drawFastHLine(2, 10, 124, SH110X_WHITE);

    // 7-row table: channel | edit page | total pages | bar showing pages
    const int rowH = 7;
    const int gridY = 13;
    for (uint8_t c = 0; c < NUM_CHANNELS; c++){
      int y = gridY + c * rowH;
      bool isSel = (c == selectedChannel);
      if (isSel){
        display2.fillRect(0, y - 1, 128, rowH, SH110X_WHITE);
        display2.setTextColor(SH110X_BLACK);
      } else {
        display2.setTextColor(SH110X_WHITE);
      }
      // Channel label
      display2.setCursor(2, y);
      display2.print("CH"); display2.print((int)(c + 1));
      // Mute marker
      if (muted[c]){
        display2.setCursor(20, y);
        display2.print("M");
      }
      // Edit page / total
      display2.setCursor(30, y);
      display2.print("P"); display2.print((int)(editPage[c] + 1));
      display2.print("/"); display2.print((int)numPages[c]);
      // Number of steps
      display2.setCursor(64, y);
      display2.print("S"); display2.print((int)numSteps[c]);
      // Playhead page indicator (which page this channel is currently on)
      uint8_t playPg = numPages[c] > 0 ? (uint8_t)(globalPage % numPages[c]) : 0;
      display2.setCursor(92, y);
      display2.print("> P"); display2.print((int)(playPg + 1));
    }
    display2.setTextColor(SH110X_WHITE);
    display2.display();
    return;
  }

  // (Legacy multi-page grid view kept disabled by the early return above)
  if (false){
    display2.setTextColor(SH110X_WHITE);
    display2.setTextSize(1);

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
