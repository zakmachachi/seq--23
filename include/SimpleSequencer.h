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
    void runEncoderSwitchTest(uint32_t ms);
    void printEncoderRaw();
    void runMidiPinMonitor(uint32_t ms);
    // MIDI input handlers (called from MIDI RX processor)
    void midiHandleStart();
    void midiHandleStop();
    void midiHandleClockTick();
    void midiHandleContinue();
    void midiHandleReset();
    // MIDI output
    void midiSendByte(uint8_t b);
    void midiSendNoteOn(uint8_t channel, uint8_t note, uint8_t vel);
    void midiSendNoteOff(uint8_t channel, uint8_t note, uint8_t vel);
    // ISR access
    static SimpleSequencer* instancePtr;
    void handleButtonIRQ(uint8_t idx);
    // Engine moved to a 1ms hardware timer: runs MIDI processing and step advancement
    void runEngine();
    void internalClockTick();

  private:
    bool steps[NUM_CHANNELS][NUM_STEPS];
    bool pendingToggle[NUM_STEPS]; // tracks pending toggle state for each step (p-lock override)
    bool euclidPattern[NUM_CHANNELS][NUM_STEPS];
    uint8_t pulses[NUM_CHANNELS];
    uint8_t retrig[NUM_CHANNELS];
    // --- UPDATED: Per-Step Parameter Arrays ---
    uint8_t pitch[NUM_CHANNELS][NUM_STEPS];  // per-step pitch (MIDI note)
    uint8_t noteLen[NUM_CHANNELS][NUM_STEPS]; // per-step length index into noteLenTicks
    uint8_t stepRatchet[NUM_CHANNELS][NUM_STEPS]; // per-step ratchet count (0 = off)
    int8_t heldStep = -1; // Tracks which button is currently held down (-1 means none)
    bool euclidEnabled[NUM_CHANNELS];
    
    // --- MUTE & MODIFIER STATE ---
    bool muted[NUM_CHANNELS]; 
    bool startStopModifierFlag = false; 
    uint8_t noteLenIdx; // global default length index when no step is held
    // --- CHANNEL DEFAULT PITCHES ---
    uint8_t channelPitch[NUM_CHANNELS]; // per-channel base pitch (used when per-step pitch == 255)
    uint8_t lastNotePlaying[NUM_CHANNELS]; // last note sent per channel (for proper NoteOff)

    // runtime
    uint32_t bpm;
    uint32_t lastStepMillis;
    uint16_t currentStep;
    uint8_t selectedChannel;
    // high-resolution MIDI clock reference moved to file-scope static variable
    Division stepDivision = DIV_SIXTEENTH; // default to 1/16 (16 steps per 4/4 bar)
    // --- TICK-BASED NOTE LENGTH ENGINE ---
    uint32_t noteOffTick[NUM_CHANNELS];
    uint32_t absoluteTickCounter = 0;
    // --- FILL / PERFORMANCE MODES ---
    bool fillModeActive = false; // live hold modifier (CHANNEL_BTN_PIN)
    bool fillStep[NUM_CHANNELS][NUM_STEPS]; // per-step Fill memory

    // --- RATCHET ENGINE ---
    uint32_t activeRatchetInterval[NUM_CHANNELS];
    uint32_t activeRatchetNext[NUM_CHANNELS];
    uint32_t activeRatchetEnd[NUM_CHANNELS];
    uint32_t ratchetNoteOffMs[NUM_CHANNELS];
    uint8_t activeRatchetPitch[NUM_CHANNELS];
    // display (use concrete SH1106G implementation)
    Adafruit_SH1106G display{128, 64, &Wire};
    uint32_t lastDisplayMillis;
    const uint32_t displayRefreshMs = 16; // display refresh interval in ms (~60Hz)
    void drawDisplay();
    void displayTest();

    // button debounce parameters (Arduino example)
    const unsigned long debounceMs = 10;
    // --- MODIFIER PINS ---
    const uint8_t CHANNEL_BTN_PIN = 28; // channel modifier (hold + Steps 1-4 to select channel)
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
    void triggerChannel(uint8_t ch);
};

#endif
