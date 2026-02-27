#ifndef SIMPLESEQUENCER_H
#define SIMPLESEQUENCER_H

#include <Arduino.h>
#include "SeqConfig.h"
// OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

class SimpleSequencer {
  public:
    // step division relative to quarter note (musical denominations)
    enum Division { DIV_WHOLE = 0, DIV_HALF, DIV_QUARTER, DIV_EIGHTH, DIV_SIXTEENTH };
    SimpleSequencer();
    void begin();
    void loop();
    void runSwitchTest(uint32_t ms);
    // MIDI output
    void midiSendByte(uint8_t b);
    void midiSendNoteOn(uint8_t channel, uint8_t note, uint8_t vel);
    void midiSendNoteOff(uint8_t channel, uint8_t note, uint8_t vel);
    // ISR access
    static SimpleSequencer* instancePtr;
    void handleButtonIRQ(uint8_t idx);

  private:
    bool steps[NUM_CHANNELS][NUM_STEPS];
    bool euclidPattern[NUM_CHANNELS][NUM_STEPS];
    uint8_t pulses[NUM_CHANNELS];
    uint8_t retrig[NUM_CHANNELS];
    int16_t pitch[NUM_CHANNELS];
    bool euclidEnabled[NUM_CHANNELS];
    uint8_t noteLenIdx; // index into note length table (0..4)

    // runtime
    uint32_t bpm;
    uint32_t lastStepMillis;
    uint16_t currentStep;
    uint8_t selectedChannel;
    Division stepDivision = DIV_SIXTEENTH; // default to 1/16 (16 steps per 4/4 bar)
    // MIDI note-off scheduling (non-blocking)
    uint32_t noteOffTime[NUM_CHANNELS];
    const uint16_t noteMs = 200; // note length in ms (longer sustain)
    // display (use concrete SH1106G implementation)
    Adafruit_SH1106G display{128, 64, &Wire};
    uint32_t lastDisplayMillis;
    const uint32_t displayRefreshMs = 16; // display refresh interval in ms (~60Hz)
    void drawDisplay();
    void displayTest();

    // button debounce parameters (Arduino example)
    const unsigned long debounceMs = 10;
    // run state + start/stop button debounce state
    bool isRunning = false;
    bool startLastReading = false;
    bool startState = false;
    unsigned long startLastDebounceTime = 0;
    // debug LED for ISR activity
    volatile bool debugLedFlag;
    uint32_t debugLedOffTime;

    void setupPins();
    void readButtons();
    void updateEuclid(uint8_t ch);
    void readEncoders();
    void stepClock();
    void triggerChannel(uint8_t ch);
};

#endif
