#include "SimpleSequencer.h"
#include <IntervalTimer.h>

// Background Hardware Timer for flawless MIDI clock
static IntervalTimer midiClockTimer;
static volatile bool midiTimerRunning = false;

void sendClockISR() {
  // ISR must be as tiny as possible
  Serial8.write(0xF8);
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



// Note length factors (fraction of a quarter note) and printable names
static const float noteLenFactors[] = { 4.0f, 2.0f, 1.0f, 0.5f, 0.25f };
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
  for (uint8_t c=0;c<NUM_CHANNELS;c++){
    pulses[c]=4;
    retrig[c]=1;
    euclidEnabled[c]=false;
    noteOffTime[c]=0;
    for(uint8_t s=0;s<NUM_STEPS;s++){
      steps[c][s]=false;
      euclidPattern[c][s]=false;
      // Default pitch set to MIDI 36 (Elektron default drum pitch center)
      pitch[c][s] = 36;
      noteLen[c][s] = 3; // Default every step to 1/8th note length
    }
    lastNotePlaying[c] = 255;
  }
  lastMidiClockMicros = 0;
  noteLenIdx = 3; // default to 1/8
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
  uint8_t status = 0x80 | (channel & 0x0F);
  midiSendByte(status);
  midiSendByte(note & 0x7F);
  midiSendByte(vel & 0x7F);
}

void SimpleSequencer::setupPins(){
  // buttons use interrupt-driven handlers
  for (uint8_t i=0;i<NUM_STEPS;i++){
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  }
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
        // pressed: toggle running and send MIDI start/stop
        isRunning = !isRunning;
        if (isRunning){
          // Starting: go to start of sequence and trigger step 0 immediately
          Serial.println("MIDI START");
          midiSendByte(0xFA); // MIDI Start
          // reset MIDI clock timer so clocks start aligned
          // send an immediate MIDI Clock tick so receivers start in sync
          midiSendByte(0xF8);
          // start the hardware timer for internal MIDI clock if we're not driven externally
          if (!externalMidiClockActive && !midiTimerRunning) {
            uint32_t interval = (60000000UL / bpm) / 24;
            midiClockTimer.begin(sendClockISR, interval);
            midiTimerRunning = true;
          }
          // reset position to start
          currentStep = 0;
          lastStepMillis = now;
          // trigger any enabled steps at step 0 immediately
          for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
            if (steps[ch][currentStep]) triggerChannel(ch);
          }
        } else {
          // Stopping: send Stop and silence any playing notes, reset to start
          Serial.println("MIDI STOP");
          midiSendByte(0xFC); // MIDI Stop
          // send Note Off for any scheduled notes to avoid hanging notes
          for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
            if (noteOffTime[ch]){
                if (lastNotePlaying[ch] < 128) midiSendNoteOff(ch, lastNotePlaying[ch], 0);
                noteOffTime[ch] = 0;
              }
          }
          // reset clock timer while stopped and ensure hardware timer is stopped
          if (midiTimerRunning) { midiClockTimer.end(); midiTimerRunning = false; }
          // reset position to start so next Start begins at step 0
          currentStep = 0;
          lastStepMillis = now;
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
  // process any received MIDI bytes from hardware Serial8 (MIDI)
  while (Serial8.available() > 0) {
    uint8_t b = Serial8.read();

    // 1. Handle Clock Ticks (0xF8)
    if (b == 0xF8) {
      externalMidiClockActive = true;
      lastExternalClockMillis = now;

      // If we were running an internal hardware timer, stop it because external clock is authoritative
      if (midiTimerRunning) {
        midiClockTimer.end();
        midiTimerRunning = false;
      }

      // Sequencer Advancement (6 ticks = 16th note)
      midiStepTickCounter++;
      if (midiStepTickCounter >= 6) {
        midiStepTickCounter = 0;
        midiHandleClockTick();
      }

      // Store the absolute timestamp of this tick in the rolling window
      uint32_t currentMicros = micros();
      tickTimestamps[tickIndex] = currentMicros;
      if (validTicks < BPM_TICK_WINDOW) {
        validTicks++;
      } else {
        // oldest index is one past current in the ring
        uint8_t oldestIndex = (tickIndex + 1) % BPM_TICK_WINDOW;
        uint32_t elapsedMicros = currentMicros - tickTimestamps[oldestIndex];
        if (elapsedMicros > 0) {
          // 48 ticks = 2 quarter notes -> BPM = (2 beats / elapsed_seconds) * 60
          // simplified: BPM = 120,000,000 / elapsedMicros
          float calculatedBpm = 120000000.0f / (float)elapsedMicros;
          // lighter smoothing for faster reaction to tempo changes
          smoothedBpm = (smoothedBpm * 0.40f) + (calculatedBpm * 0.60f);
          bpm = (uint32_t)(smoothedBpm + 0.5f);
        }
      }
      tickIndex = (tickIndex + 1) % BPM_TICK_WINDOW;
    }

    // 2. Handle Start (0xFA)
    else if (b == 0xFA) {
      Serial.println("Ext MIDI Start");
      externalMidiClockActive = true;
      lastExternalClockMillis = now;

      // Reset all counters so the math starts fresh
      midiStepTickCounter = 0;
      validTicks = 0;
      tickIndex = 0;

      // stop internal timer if it was running
      if (midiTimerRunning) { midiClockTimer.end(); midiTimerRunning = false; }

      midiHandleStart();
    }

    // 3. Handle Continue (0xFB)
    else if (b == 0xFB) {
      Serial.println("Ext MIDI Continue");
      externalMidiClockActive = true;
      lastExternalClockMillis = now;
      if (midiTimerRunning) { midiClockTimer.end(); midiTimerRunning = false; }
      midiHandleContinue();
    }

    // 4. Handle Stop (0xFC)
    else if (b == 0xFC) {
      Serial.println("Ext MIDI Stop");
      // external Stop -> stop internal timer too
      if (midiTimerRunning) { midiClockTimer.end(); midiTimerRunning = false; }
      midiHandleStop();
    }
    else {
      if (b != 0xF8) {
        Serial.print("MIDI RX byte: 0x"); Serial.println(b, HEX);
      }
    }
  }
  // MIDI clock generation is handled by a background IntervalTimer when running
  // Nothing to do here in the foreground loop for internal clocking.

  // if external MIDI clock was active but we haven't seen ticks for a while, fall back to internal clock
  if (externalMidiClockActive){
    if (millis() - lastExternalClockMillis > 2000){
      externalMidiClockActive = false;
      midiStepTickCounter = 0;
      // reset timestamp window
      validTicks = 0;
      tickIndex = 0;
      Serial.println("External MIDI clock lost; reverting to internal clock");
      // start internal hardware timer if sequencer is running
      if (isRunning && !midiTimerRunning){
        uint32_t interval = (60000000UL / bpm) / 24;
        midiClockTimer.begin(sendClockISR, interval);
        midiTimerRunning = true;
      }
    }
  }

  // advance internal step clock only when NOT driven by external MIDI clock
  if (!externalMidiClockActive) stepClock();
  // handle scheduled MIDI note-offs
  // note-off scheduling
  for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
    if (noteOffTime[ch] && now >= noteOffTime[ch]){
      if (lastNotePlaying[ch] < 128) midiSendNoteOff(ch, lastNotePlaying[ch], 0);
      noteOffTime[ch] = 0;
    }
  }
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
    unsigned long now = millis();
    bool reading = (digitalRead(BUTTON_PINS[i]) == LOW); // pressed = LOW
    if (reading != lastReading[i]){
      lastDebounceTime[i] = now;
    }
    if ((now - lastDebounceTime[i]) > debounceMs){
      if (reading != buttonState[i]){
        buttonState[i] = reading;
        if (buttonState[i]){ // pressed
          // toggle step for selected channel; do NOT trigger immediately
          steps[selectedChannel][i] = !steps[selectedChannel][i];
          Serial.print("Ch"); Serial.print(selectedChannel+1);
          Serial.print(" Step "); Serial.print(i);
          Serial.print(" = "); Serial.println(steps[selectedChannel][i]);
          // note playback will occur when the sequencer reaches this step
          // record which step is held for parameter-lock editing
          heldStep = i;
        } else {
          // released: if this was the held step, clear heldStep
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
  const int8_t encTable[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static bool encInitialized = false;
  // initialize previous states for all encoders on first call
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
      // apply every quadrature transition immediately (but encoder1 uses half-step)
      Serial.print("Enc"); Serial.print(e+1); Serial.print(" delta="); Serial.println(delta);
      // debug raw for encoder 1
      if (e==0){
        Serial.print("Enc1 raw A="); Serial.print(a); Serial.print(" B="); Serial.println(b);
      }
      static int8_t encAcc1 = 0;
      int steps = 0;
      if (e == 0){
        // encoder 1: half the step size (accumulate two transitions per BPM step)
        encAcc1 += delta;
        if (encAcc1 >= 2) { steps = encAcc1 / 2; encAcc1 = encAcc1 % 2; }
        else if (encAcc1 <= -2) { steps = encAcc1 / 2; encAcc1 = encAcc1 % 2; }
      } else {
        steps = delta; // other encoders: one per transition
      }
      // apply steps
      if (e == 0){ // encoder 1: BPM
        int newBpm = (int)bpm + steps;
        if (newBpm < 20) newBpm = 20;
        if (newBpm > 300) newBpm = 300;
        bpm = newBpm;
        Serial.print("BPM="); Serial.println(bpm);
        // update hardware timer interval immediately if using internal clock
        if (isRunning && !externalMidiClockActive && midiTimerRunning){
          uint32_t interval = (60000000UL / bpm) / 24;
          midiClockTimer.update(interval);
        }
      } else if (e == 1){ // encoder 2: channel select OR per-step pitch when holding a step
        if (heldStep >= 0){
          int note = (int)pitch[selectedChannel][heldStep] + steps;
          if (note < 0) note = 0;
          if (note > 127) note = 127;
          pitch[selectedChannel][heldStep] = (uint8_t)note;
          Serial.print("Ch"); Serial.print(selectedChannel+1); Serial.print(" Step"); Serial.print(heldStep); Serial.print(" pitch="); Serial.println(pitch[selectedChannel][heldStep]);
        } else {
          int ch = (int)selectedChannel + steps;
          while (ch < 0) ch += NUM_CHANNELS;
          selectedChannel = ch % NUM_CHANNELS;
          Serial.print("Channel="); Serial.println(selectedChannel+1);
        }
      } else if (e == 2){ // encoder 3: note length
        if (heldStep >= 0){
          int idxn = (int)noteLen[selectedChannel][heldStep] + steps;
          int maxIdx = (int)(sizeof(noteLenFactors)/sizeof(noteLenFactors[0])) - 1;
          if (idxn < 0) idxn = 0;
          if (idxn > maxIdx) idxn = maxIdx;
          noteLen[selectedChannel][heldStep] = (uint8_t)idxn;
          Serial.print("Ch"); Serial.print(selectedChannel+1); Serial.print(" Step"); Serial.print(heldStep); Serial.print(" len="); Serial.println(noteLenNames[noteLen[selectedChannel][heldStep]]);
        } else {
          int idxn = (int)noteLenIdx + steps;
          int maxIdx = (int)(sizeof(noteLenFactors)/sizeof(noteLenFactors[0])) - 1;
          if (idxn < 0) idxn = 0;
          if (idxn > maxIdx) idxn = maxIdx;
          noteLenIdx = (uint8_t)idxn;
          Serial.print("NoteLen="); Serial.println(noteLenNames[noteLenIdx]);
        }
      } else if (e == 3){ // encoder 4: euclid pulses when enabled
        if (euclidEnabled[selectedChannel]){
          int p = (int)pulses[selectedChannel] + steps;
          if (p < 0) p = 0;
          if (p > NUM_STEPS) p = NUM_STEPS;
          pulses[selectedChannel] = p;
          updateEuclid(selectedChannel);
          Serial.print("Ch"); Serial.print(selectedChannel+1); Serial.print(" pulses="); Serial.println(pulses[selectedChannel]);
        }
      }
    }
    lastState[e] = st;
    // encoder switch
    bool sw = (digitalRead(ENC_SW[e]) == LOW);
    unsigned long now = millis();
    if (sw != lastSwState[e]){
      lastSwDebounce[e] = now;
    }
    if ((now - lastSwDebounce[e]) > debounceMs){
      if (sw != lastSwState[e]){
        lastSwState[e] = sw;
        if (sw){
          // pressed
          if (e == 3){
            // toggle euclid for selected channel
            euclidEnabled[selectedChannel] = !euclidEnabled[selectedChannel];
            updateEuclid(selectedChannel);
            Serial.print("Ch"); Serial.print(selectedChannel+1); Serial.print(" euclid="); Serial.println(euclidEnabled[selectedChannel]);
          }
        }
      }
    }
  }
}



void SimpleSequencer::updateEuclid(uint8_t ch){
  uint8_t k = pulses[ch];
  uint8_t n = NUM_STEPS;
  if (k == 0){
    for (uint8_t i=0;i<n;i++) euclidPattern[ch][i]=false;
    return;
  }
  if (k >= n){
    for (uint8_t i=0;i<n;i++) euclidPattern[ch][i]=true;
    return;
  }
  for (uint8_t j=0;j<n;j++){
    int x = (j * k) / n;
    int y = ((j+1) * k) / n;
    euclidPattern[ch][j] = (y > x);
  }
}

void SimpleSequencer::stepClock(){
  if (!isRunning) return; // don't advance when stopped
  // ms per step: quarter-note divided by 4 (16 steps = 4 beats)
  // compute ms per step using musical division relative to quarter note
  float quarterMs = 60000.0f / (float)bpm;
  float msPerStep = quarterMs * getDivisionFactor(stepDivision);
  uint32_t now = millis();
  if (now - lastStepMillis >= (uint32_t)msPerStep){
    // advance step
    currentStep = (currentStep + 1) % NUM_STEPS;
    lastStepMillis = now;
    // trigger channels that have the step enabled
    for(uint8_t ch=0;ch<NUM_CHANNELS;ch++){
      if (steps[ch][currentStep]){
        triggerChannel(ch);
      }
    }
  }
}

void SimpleSequencer::triggerChannel(uint8_t ch){
  // interrupt any currently-playing note on this channel, then send new Note On
  uint32_t now = millis();
  // if a previous note was playing on this channel, turn it off
  if (noteOffTime[ch] && now < noteOffTime[ch]){
    if (lastNotePlaying[ch] < 128) midiSendNoteOff(ch, lastNotePlaying[ch], 0);
    noteOffTime[ch] = 0;
  }
  // send MIDI Note On for this channel using per-step pitch mapping
  uint8_t note = constrain(pitch[ch][currentStep], 0, 127);
  uint8_t vel = 100;
  // Send NoteOn on the specific channel (track channels 0..3 map to MIDI 1..4)
  midiSendNoteOn(ch, note, vel);
  // remember which note we just played for proper NoteOff later
  lastNotePlaying[ch] = note;
  // debug: report fired note to serial monitor
  Serial.print("FIRING NOTE! MIDI Ch: "); Serial.print(ch + 1);
  Serial.print(" | Note: "); Serial.println(note);
  // compute note length using selected noteLenIdx (fractions of a quarter)
  float quarterMs = 60000.0f / (float)bpm;
  // use per-step note length index when available
  uint8_t lenIdx = noteLen[ch][currentStep];
  float noteLenF = quarterMs * noteLenFactors[lenIdx];
  uint32_t noteLen = (uint32_t)(noteLenF + 0.5f);
  noteOffTime[ch] = now + noteLen;
}

// CV/Gate functions removed; using MIDI out only

void SimpleSequencer::drawDisplay(){
  display.clearDisplay();

  // 1. TOP INFO BAR
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("BPM:"); display.print(bpm);
  display.setCursor(56, 0);
  display.print("CH:"); display.print(selectedChannel + 1);
  display.setCursor(96, 0);
  display.print("L:"); display.print(noteLenNames[noteLenIdx]);

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

    // Draw filled box if step is active
    if (steps[selectedChannel][i]) {
      display.fillRect(x, y, stepW, stepH, SH110X_WHITE);
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
    // Black out a space at the bottom for the edit menu
    display.fillRect(0, 50, 128, 14, SH110X_BLACK);
    display.drawLine(0, 49, 128, 49, SH110X_WHITE); // Separator

    uint8_t p = pitch[selectedChannel][heldStep];
    uint8_t lenIdx = noteLen[selectedChannel][heldStep];
    const char* noteNames[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

    // Display the specific step data
    display.setCursor(2, 54);
    display.print("STEP "); display.print(heldStep + 1);
    display.print(" | ");
    display.print(noteNames[p % 12]); display.print((p / 12) - 1);
    display.print(" | L:"); display.print(noteLenNames[lenIdx]);
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

// MIDI input handlers
void SimpleSequencer::midiHandleClockTick(){
  if (!isRunning) return;
  // advance step and trigger channels
  currentStep = (currentStep + 1) % NUM_STEPS;
  lastStepMillis = millis();
  for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
    if (steps[ch][currentStep]) triggerChannel(ch);
  }
}

void SimpleSequencer::midiHandleStart(){
  isRunning = true;
  currentStep = 0;
  lastStepMillis = millis();
  for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
    if (steps[ch][currentStep]) triggerChannel(ch);
  }
}

void SimpleSequencer::midiHandleContinue(){
  // Resume without resetting position
  isRunning = true;
  lastStepMillis = millis();
}

void SimpleSequencer::midiHandleStop(){
  isRunning = false;
  // silence notes
  for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
    if (noteOffTime[ch]){
      if (lastNotePlaying[ch] < 128) midiSendNoteOff(ch, lastNotePlaying[ch], 0);
      noteOffTime[ch] = 0;
    }
  }
  // when external Stop received, consider external clock inactive
  externalMidiClockActive = false;
}

void SimpleSequencer::midiHandleReset(){
  isRunning = false;
  currentStep = 0;
  lastStepMillis = millis();
  externalMidiClockActive = false;
}

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
