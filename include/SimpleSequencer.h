#ifndef SIMPLESEQUENCER_H
#define SIMPLESEQUENCER_H

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include "SeqConfig.h"
#include <EEPROM.h>
// OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

class SimpleSequencer {
  public:
    // step division relative to quarter note (musical denominations)
    enum Division { DIV_WHOLE = 0, DIV_HALF, DIV_QUARTER, DIV_EIGHTH, DIV_SIXTEENTH };
    enum TrigMachine : uint8_t {
      TM_OFF = 0, TM_KICK, TM_HIHAT, TM_SNARE, TM_ANTIKICK, TM_PERC, TM_EUCLID, TM_COUNT
    };
    SimpleSequencer();
    void begin();
    void loop();
    void runSwitchTest(uint32_t ms);
    void runEncoderSwitchTest(uint32_t ms);
    void printEncoderRaw();
    void runMidiPinMonitor(uint32_t ms);
    // MIDI input handlers (moved into `runEngine()` to avoid concurrent Serial reads)
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
    uint8_t euclidOffset[NUM_CHANNELS];
    uint8_t retrig[NUM_CHANNELS];
    // --- UPDATED: Per-Step Parameter Arrays ---
    uint8_t pitch[NUM_CHANNELS][NUM_STEPS];  // per-step pitch (MIDI note)
    uint8_t noteLen[NUM_CHANNELS][NUM_STEPS]; // per-step length index into noteLenTicks
    uint8_t stepRatchet[NUM_CHANNELS][NUM_STEPS]; // per-step ratchet count (0 = off)
    // --- ACCENT / SLIDE (TB-303 style) ---
    uint8_t stepVelocity[NUM_CHANNELS][NUM_STEPS]; // 255 = use channel default
    bool stepSlide[NUM_CHANNELS][NUM_STEPS];
    int8_t heldStep = -1; // Tracks which button is currently held down (-1 means none)
    bool euclidEnabled[NUM_CHANNELS];
    
    // --- MUTE & MODIFIER STATE ---
    bool muted[NUM_CHANNELS]; 
    bool startStopModifierFlag = false; 
    uint8_t noteLenIdx; // global default length index when no step is held
    // --- CHANNEL DEFAULT PITCHES ---
    uint8_t channelPitch[NUM_CHANNELS]; // per-channel base pitch (used when per-step pitch == 255)
    uint8_t lastNotePlaying[NUM_CHANNELS]; // last note sent per channel (for proper NoteOff)
    uint8_t channelVelocity[NUM_CHANNELS]; // default velocity per channel (0-127)
    uint8_t midiChannel[NUM_CHANNELS];     // per-channel MIDI Out channel (0..15 = MIDI ch 1..16)
    // --- GENERATIVE PARAMETERS (Menu 1: Notes Page) ---
    uint8_t randomSlideProb[NUM_CHANNELS]; // Probability 0-100 a generated step has Slide
    uint8_t octaveSpread[NUM_CHANNELS];    // 0..60 = max semitones above root for random per-step picks
    uint8_t lastScaleMode[NUM_CHANNELS];   // remembered scale to restore on Pot 1 toggle
    void rerollSlides(uint8_t ch);
    void transposeChannelNotes(uint8_t ch, int semitones);

    // --- TRIGGER MACHINES (Menu 4) ---
    uint8_t trigMachine[NUM_CHANNELS];           // active machine type per channel
    uint8_t trigDensity[NUM_CHANNELS];           // 0..100 density / threshold
    uint8_t trigShift[NUM_CHANNELS];             // 0..15 step shift
    uint8_t machineOverlay[NUM_CHANNELS][NUM_STEPS]; // 0=auto, 1=force-on, 2=force-off
    bool machinePattern[NUM_CHANNELS][NUM_STEPS];    // cached pattern from generator
    void regenerateMachinePattern(uint8_t ch);
    bool isStepActive(uint8_t ch, uint8_t step);
    void drawTrigMachineView();

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
    // per-step Fill memory: 0 = normal, 1 = fill (plays only when Fill held), 2 = anti-fill (never plays)
    uint8_t fillState[NUM_CHANNELS][NUM_STEPS];
    uint8_t euclidScaleMode[NUM_CHANNELS];
    // UI focus helpers
    uint8_t focusEncoder = 0;        // 0 = none, 1-4 = encoder focused
    uint32_t lastEncoderMoveTime = 0;
    const uint32_t focusTimeout = 1500; // ms to keep focus visible

    // --- RATCHET / RETRIG ENGINE ---
    uint8_t ratchetIntervalTicks[NUM_CHANNELS];
    uint32_t ratchetNextTick[NUM_CHANNELS];
    uint32_t ratchetEndTick[NUM_CHANNELS];
    uint8_t ratchetPitch[NUM_CHANNELS];
    // UI menu state
    bool menuMode = false;
    uint8_t activeMenu = 0;
    // Transport play/stop OLED splash
    uint32_t transportAnimEndMs = 0;
    bool transportAnimIsPlay = false;
    // Fill-mark feedback splash
    uint32_t fillAnimEndMs = 0;
    uint8_t fillAnimStep = 0;
    bool fillAnimSet = false;
    // Mute toggle feedback splash
    uint32_t muteAnimEndMs = 0;
    uint8_t muteAnimCh = 0;
    bool muteAnimMuted = false;
    // Function + Pot1 BPM editing splash
    uint32_t bpmFocusEndMs = 0;
    // Clear-track feedback splash
    uint32_t clearAnimEndMs = 0;
    uint8_t clearAnimCh = 0;
    // Channel-button held: when >= 0, OLED shows that channel's params (mute, MIDI out)
    int8_t heldChannel = -1;
    bool fillBtnLastState = false; // for transition logging
    // display (use concrete SH1106G implementation)
    Adafruit_SH1106G display{128, 64, &Wire};
    // --- HARDWARE LED GRID ---
    Adafruit_NeoPixel ledStrip;
    void updateLEDs();
    uint32_t lastDisplayMillis;
    const uint32_t displayRefreshMs = 16; // display refresh interval in ms (~60Hz)
    void drawDisplay();
    void drawDebugGrid();
    void drawNotesView();
    void drawEuclidView();
    void drawStepVisualiser();
    void bootAnimation();

    // button debounce parameters (Arduino example)
    const unsigned long debounceMs = 10;
    // --- Matrix keyboard state (5x6) ---
    // Use global MATRIX_ROWS / MATRIX_COLS from SeqConfig.h
    uint8_t matrixScanCol = 0; // which column to scan next
    uint8_t matrixRawState[MATRIX_KEYS]; // raw last reading (0/1)
    bool matrixState[MATRIX_KEYS]; // debounced stable state (true = pressed)
    unsigned long matrixLastDebounce[MATRIX_KEYS];

    // --- MODIFIER ACCESSORS ---
    bool isFunctionHeld();
    bool isFillHeld();
    // keep these as aliases so runEngine() compiles unchanged:
    bool isStartHeld();
    bool isChannelHeld();
    // run state + start/stop button debounce state
    bool isRunning = false;
    bool startLastReading = false;
    bool startState = false;
    unsigned long startLastDebounceTime = 0;
    // debug LED for ISR activity
    volatile bool debugLedFlag;
    uint32_t debugLedOffTime;
    // Encoder 3 (index 2) held slide modifier
    bool encoderSlideHold = false;

    void setupPins();
    void readButtons();
    void scanMatrixStep();
    void onKeyPress(uint8_t row, uint8_t col);
    void onKeyRelease(uint8_t row, uint8_t col);
    void updateEuclid(uint8_t ch);
    void randomizeEuclidMelody(uint8_t ch);
    void readEncoders();
    void shiftEuclidNotes(uint8_t ch, int steps);
    void triggerChannel(uint8_t ch);
    void clearTrack(uint8_t ch);
    // --- CONTEXTUAL POT INPUT HANDLERS ---
    void onPotButtonPress(uint8_t pot);
    void handlePotRotation(uint8_t pot, int ticks);
    // --- EEPROM SAVE SYSTEM ---
    struct SaveData {
      uint32_t magicNumber;
      uint32_t savedBpm;
      uint8_t savedNoteLenIdx;
      uint8_t savedChannelPitch[NUM_CHANNELS];
      bool savedMuted[NUM_CHANNELS];
      bool savedEuclidEnabled[NUM_CHANNELS];
      uint8_t savedPulses[NUM_CHANNELS];
      uint8_t savedEuclidOffset[NUM_CHANNELS];
      uint8_t savedEuclidScaleMode[NUM_CHANNELS];
      bool savedSteps[NUM_CHANNELS][NUM_STEPS];
      uint8_t savedPitch[NUM_CHANNELS][NUM_STEPS];
      uint8_t savedNoteLen[NUM_CHANNELS][NUM_STEPS];
      uint8_t savedFillStep[NUM_CHANNELS][NUM_STEPS];
      uint8_t savedStepRatchet[NUM_CHANNELS][NUM_STEPS];
      // Persisted Accent/Slide
      uint8_t savedStepVelocity[NUM_CHANNELS][NUM_STEPS];
      uint8_t savedStepSlide[NUM_CHANNELS][NUM_STEPS];
      uint8_t savedChannelVelocity[NUM_CHANNELS];
      // Persisted generative parameters
      uint8_t savedRandomSlideProb[NUM_CHANNELS];
      uint8_t savedOctaveSpread[NUM_CHANNELS];
      // Persisted per-channel MIDI Out channel
      uint8_t savedMidiChannel[NUM_CHANNELS];
      // Persisted trigger-machine state (v6)
      uint8_t savedTrigMachine[NUM_CHANNELS];
      uint8_t savedTrigDensity[NUM_CHANNELS];
      uint8_t savedTrigShift[NUM_CHANNELS];
      uint8_t savedMachineOverlay[NUM_CHANNELS][NUM_STEPS];
    };
    void saveState();
    void loadState();
};

#endif
