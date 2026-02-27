#include "SimpleSequencer.h"
#include <IntervalTimer.h>

SimpleSequencer::SimpleSequencer()
  : bpm(200), lastStepMillis(0), currentStep(0), selectedChannel(0)
{
  for (uint8_t c=0;c<NUM_CHANNELS;c++){
    pulses[c]=4;
    retrig[c]=1;
    pitch[c]=48; // base pitch default to C3 (MIDI 48)
    euclidEnabled[c]=false;
    noteOffTime[c]=0;
    for(uint8_t s=0;s<NUM_STEPS;s++){
      steps[c][s]=false;
      euclidPattern[c][s]=false;
    }
  }
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
  // quick I2C scan to help debug wiring/address
  Serial.println("Scanning I2C bus...");
  bool any = false;
  static int8_t encAcc[4] = {0,0,0,0};
  const int encThreshold = 4; // four transitions = one detent (half again)
  for (uint8_t e=0;e<4;e++){
    uint8_t a = digitalRead(ENC_A[e])==HIGH ? 1:0;
    uint8_t b = digitalRead(ENC_B[e])==HIGH ? 1:0;
    uint8_t st = (a<<1) | b;
    uint8_t idx = (lastState[e] << 2) | st;
    int8_t delta = encTable[idx & 0x0F];
    if (delta != 0){
      // accumulate transitions and apply when threshold reached
      encAcc[e] += delta;
      // debug raw for encoder 1
      if (e==0){
        Serial.print("Enc1 raw A="); Serial.print(a); Serial.print(" B="); Serial.print(b);
        Serial.print(" acc="); Serial.println(encAcc[e]);
      }
      int steps = 0;
      while (encAcc[e] >= encThreshold){ encAcc[e] -= encThreshold; steps++; }
      while (encAcc[e] <= -encThreshold){ encAcc[e] += encThreshold; steps--; }
      if (steps != 0){
        Serial.print("Enc"); Serial.print(e+1); Serial.print(" step="); Serial.println(steps);
        if (e == 0){ // encoder 1: BPM
          int newBpm = (int)bpm + steps;
          if (newBpm < 20) newBpm = 20;
          if (newBpm > 300) newBpm = 300;
          bpm = newBpm;
          Serial.print("BPM="); Serial.println(bpm);
        } else if (e == 1){ // encoder 2: channel select
          int ch = (int)selectedChannel + steps;
          while (ch < 0) ch += NUM_CHANNELS;
          selectedChannel = ch % NUM_CHANNELS;
          Serial.print("Channel="); Serial.println(selectedChannel+1);
        } else if (e == 2){ // encoder 3: note length
          int idxn = (int)noteLenIdx + steps;
          if (idxn < 0) idxn = 0;
          if (idxn > (int)(sizeof(noteLenFactors)/sizeof(noteLenFactors[0]))-1) idxn = (int)(sizeof(noteLenFactors)/sizeof(noteLenFactors[0]))-1;
          noteLenIdx = (uint8_t)idxn;
          Serial.print("NoteLen="); Serial.println(noteLenNames[noteLenIdx]);
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
    }
    lastState[e] = st;
static volatile uint8_t midiCurByte = 0;
static volatile int midiBitIndex = 0; // 0=start,1..8 data,9 stop, >9 finished
static volatile bool midiSending = false;
static const uint32_t midiBitPeriodUs = 32; // ~1/31250 s

// MIDI clock timing (24 PPQN)
static uint32_t lastMidiClockMillis = 0;

static void midiTimerISR(){
  // called every midiBitPeriodUs microseconds
  if (midiBitIndex == 0){
    // start bit (LOW)
    digitalWriteFast(MIDI_TX_PIN, LOW);
    midiBitIndex++;
    return;
  }
  if (midiBitIndex >=1 && midiBitIndex <=8){
    uint8_t bit = (midiCurByte >> (midiBitIndex-1)) & 0x1;
    digitalWriteFast(MIDI_TX_PIN, bit ? HIGH : LOW);
    midiBitIndex++;
    return;
  }
  if (midiBitIndex == 9){
    // stop bit
    digitalWriteFast(MIDI_TX_PIN, HIGH);
    midiBitIndex++;
    return;
  }
  // finished sending a byte, load next if available
  if (midiHead != midiTail){
    midiCurByte = midiBuf[midiTail];
    midiTail = (midiTail + 1) & 127;
    midiBitIndex = 0;
    return; 
  }
  // no more data, stop timer
  midiTimer.end();
  midiSending = false;
}

void SimpleSequencer::midiSendByte(uint8_t b){
  uint8_t next = (midiHead + 1) & 127;
  // simple blocking if buffer full
  while (next == midiTail) { ; }
  midiBuf[midiHead] = b;
  midiHead = next;
  if (!midiSending){
    // start sending first byte
    midiSending = true;
    // load first
    midiCurByte = midiBuf[midiTail];
    midiTail = (midiTail + 1) & 127;
    midiBitIndex = 0;
    digitalWriteFast(MIDI_TX_PIN, HIGH);
    midiTimer.begin(midiTimerISR, midiBitPeriodUs);
  }
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
          Serial.println("MIDI START");
          midiSendByte(0xFA); // MIDI Start
          // reset MIDI clock timer so clocks start aligned
          lastMidiClockMillis = now;
          // send an immediate MIDI Clock tick so receivers start in sync
          midiSendByte(0xF8);
        } else {
          Serial.println("MIDI STOP");
          midiSendByte(0xFC); // MIDI Stop
          // reset clock timer while stopped
          lastMidiClockMillis = now;
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
  }
  // send MIDI Clock (0xF8) at 24 PPQN while running
  if (isRunning){
    float quarterMs = 60000.0f / (float)bpm;
    uint32_t midiClockInterval = (uint32_t)(quarterMs / 24.0f + 0.5f);
    if (now - lastMidiClockMillis >= midiClockInterval){
      midiSendByte(0xF8);
      lastMidiClockMillis += midiClockInterval;
      // guard against large drift
      if (now - lastMidiClockMillis > midiClockInterval) lastMidiClockMillis = now;
    }
  } else {
    // keep timer reference fresh while stopped
    lastMidiClockMillis = now;
  }

  stepClock();
  // handle scheduled MIDI note-offs
  // note-off scheduling
  for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
    if (noteOffTime[ch] && now >= noteOffTime[ch]){
      uint8_t note = constrain(pitch[ch], 0, 127);
      midiSendNoteOff(ch, note, 0);
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
      } else if (e == 1){ // encoder 2: channel select
        int ch = (int)selectedChannel + steps;
        while (ch < 0) ch += NUM_CHANNELS;
        selectedChannel = ch % NUM_CHANNELS;
        Serial.print("Channel="); Serial.println(selectedChannel+1);
      } else if (e == 2){ // encoder 3: note length
        int idxn = (int)noteLenIdx + steps;
        if (idxn < 0) idxn = 0;
        if (idxn > (int)(sizeof(noteLenFactors)/sizeof(noteLenFactors[0]))-1) idxn = (int)(sizeof(noteLenFactors)/sizeof(noteLenFactors[0]))-1;
        noteLenIdx = (uint8_t)idxn;
        Serial.print("NoteLen="); Serial.println(noteLenNames[noteLenIdx]);
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
  uint8_t curNote = constrain(pitch[ch], 0, 127);
  if (noteOffTime[ch] && now < noteOffTime[ch]){
    // send Note Off for the previous note immediately
    midiSendNoteOff(ch, curNote, 0);
    noteOffTime[ch] = 0;
  }
  // send MIDI Note On for this channel using pitch mapping
  uint8_t note = curNote;
  uint8_t vel = 100;
  midiSendNoteOn(ch, note, vel);
  // compute note length using selected noteLenIdx (fractions of a quarter)
  float quarterMs = 60000.0f / (float)bpm;
  float noteLenF = quarterMs * noteLenFactors[noteLenIdx];
  uint32_t noteLen = (uint32_t)(noteLenF + 0.5f);
  noteOffTime[ch] = now + noteLen;
}

// CV/Gate functions removed; using MIDI out only

void SimpleSequencer::drawDisplay(){
  display.clearDisplay();
  // Compact UI: small BPM + single compact row per channel
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.print("BPM:"); display.print(bpm);

  const int stepW = 4; // compact width
  const int stepH = 6;
  const int spacing = 1;
  const int x0 = 2;
  const int rowY0 = 12;
  for (uint8_t ch=0; ch<NUM_CHANNELS; ch++){
    int y = rowY0 + ch * (stepH + 6);
    // small channel label
    display.setCursor(0, y-8);
    display.print("C"); display.print(ch+1);
    for (uint8_t s=0;s<NUM_STEPS;s++){
      int x = x0 + 12 + s * (stepW + spacing);
      if (steps[ch][s]) display.fillRect(x, y, stepW, stepH, SH110X_WHITE);
      else display.drawRect(x, y, stepW, stepH, SH110X_WHITE);
      if (s == currentStep) {
        display.drawFastVLine(x + stepW/2, y-4, 3, SH110X_WHITE);
      }
    }
    if (ch == selectedChannel){
      display.drawRect(8, y-10, 120, stepH+12, SH110X_WHITE);
    }
  }

  // small bottom info
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print("BPM:"); display.print(bpm);
  display.setCursor(48,56);
  display.print("Len:"); display.print(noteLenNames[noteLenIdx]);
  display.setCursor(92,56);
  display.print("Ch"); display.print(selectedChannel+1);
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
