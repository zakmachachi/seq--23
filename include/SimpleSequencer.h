#ifndef SIMPLESEQUENCER_H
#define SIMPLESEQUENCER_H

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include "SeqConfig.h"
#include "Max11300.h"
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
    bool steps[NUM_CHANNELS][TOTAL_STEPS];
    bool pendingToggle[NUM_STEPS]; // tracks pending toggle state for each step (p-lock override)
    bool euclidPattern[NUM_CHANNELS][TOTAL_STEPS];
    uint8_t pulses[NUM_CHANNELS];
    uint8_t euclidOffset[NUM_CHANNELS];
    uint8_t retrig[NUM_CHANNELS];
    // --- Per-step parameter arrays (now per-page across all NUM_STEPS*MAX_PAGES slots) ---
    uint8_t pitch[NUM_CHANNELS][TOTAL_STEPS];
    uint8_t noteLen[NUM_CHANNELS][TOTAL_STEPS];
    uint8_t stepRatchet[NUM_CHANNELS][TOTAL_STEPS];
    uint8_t stepVelocity[NUM_CHANNELS][TOTAL_STEPS];
    bool stepSlide[NUM_CHANNELS][TOTAL_STEPS];
    // --- Pages mode (Digitakt-style 1..MAX_PAGES pages per track) ---
    uint8_t numPages[NUM_CHANNELS]; // 1..MAX_PAGES per channel
    uint8_t editPage[NUM_CHANNELS]; // 0..(numPages-1) which page the user is editing
    uint8_t globalPage;             // 0..(MAX_PAGES-1) — wraps every full 16-step bar
    uint8_t numSteps[NUM_CHANNELS]; // 1..NUM_STEPS — per-channel pattern length
    bool pageEditGlobal = true;     // Pages menu: Pot1 edits globalPage (true) or editPage[ch] (false)
    // --- Per-channel rate multiplier (Pages menu Pot3) ---
    // Rate index: 0=0.25x 1=0.5x 2=1x 3=2x 4=4x. Default 2.
    static const uint8_t RATE_COUNT = 5;
    static const float RATE_VALUES[RATE_COUNT];
    uint8_t rateIdx = 2;            // target rate slot
    float rateCurrent = 1.0f;       // smoothed actual rate currently applied
    float rateTarget  = 1.0f;       // where rateCurrent is heading
    float rateRampFrom = 1.0f;      // value at ramp start
    uint32_t rateRampStartMs = 0;   // when ramp began
    uint32_t rateRampDurMs   = 500; // ramp duration
    bool rateRamping = false;       // ramp in progress?
    // --- Page button tap state (entering / cycling pages) ---
    uint32_t lastPageBtnMs = 0;     // for diagnostic/feedback
    inline uint16_t editIdx(uint8_t ch, uint8_t s) const {
      return (uint16_t)editPage[ch] * (uint16_t)NUM_STEPS + s;
    }
    inline uint16_t playIdx(uint8_t ch, uint8_t s) const {
      uint8_t pg = numPages[ch] > 0 ? (globalPage % numPages[ch]) : 0;
      return (uint16_t)pg * (uint16_t)NUM_STEPS + s;
    }
    // Local channel step taking numSteps into account.
    inline uint8_t localStep(uint8_t ch) const {
      uint8_t n = numSteps[ch];
      if (n == 0) n = 1;
      return (uint8_t)(currentStep % n);
    }
    int8_t heldStep = -1; // Tracks which button is currently held down (-1 means none)
    bool euclidEnabled[NUM_CHANNELS];
    
    // --- MUTE & MODIFIER STATE ---
    bool muted[NUM_CHANNELS]; 
    bool startStopModifierFlag = false; 
    uint8_t noteLenIdx[NUM_CHANNELS]; // per-channel default length index when no step is held
    // --- CHANNEL DEFAULT PITCHES ---
    uint8_t channelPitch[NUM_CHANNELS]; // per-channel base pitch (used when per-step pitch == 255)
    uint8_t lastNotePlaying[NUM_CHANNELS]; // last note sent per channel (for proper NoteOff)
    uint8_t channelVelocity[NUM_CHANNELS]; // default velocity per channel (0-127)
    uint8_t midiChannel[NUM_CHANNELS];     // per-channel MIDI Out channel (0..15 = MIDI ch 1..16)
    // --- GENERATIVE PARAMETERS (Menu 1: Notes Page) ---
    uint8_t randomSlideProb[NUM_CHANNELS]; // Probability 0-100 a generated step has Slide
    uint8_t octaveSpread[NUM_CHANNELS];    // 0..60 = max semitones above root for random per-step picks
    uint8_t lastScaleMode[NUM_CHANNELS];   // remembered scale to restore on Pot 1 toggle
    // Pot 6 press toggles per-channel random velocity: each triggered note that
    // uses the channel default velocity is jittered by +/- RANDOM_VEL_RANGE.
    bool randomVelEnabled[NUM_CHANNELS];
    static const uint8_t RANDOM_VEL_RANGE = 27;
    // Pot 5 (encoder 5) press toggles per-channel random gate length: each note
    // that uses the channel default gate gets a random length across the full
    // 1/32..1 range.
    bool randomGateEnabled[NUM_CHANNELS];
    // Function + encoder 3 sets a -100..+100 melodic contour bias per channel:
    // positive favours ascending motion when a pattern is generated, negative
    // descending, 0 = unbiased. Shown as an arrow on the Menu 1 screen 2.
    int8_t contourBias[NUM_CHANNELS];
    void rerollSlides(uint8_t ch);
    void transposeChannelNotes(uint8_t ch, int semitones);
    // Mutate one random active step on the edit page: toggle slide, toggle
    // accent, change note length, or change note value. Used as a "subtle
    // evolution" alternative to a full regenerate.
    void mutatePattern(uint8_t ch);
    // Copy one full 16-step page worth of per-step state (steps, pitches,
    // velocities, slides, ratchets, gates, fill marks, machine overlays)
    // from srcPg to dstPg on channel ch. Used to seed newly-allocated pages
    // with the contents of page 0 so growing pattern length doesn't reveal
    // empty pages.
    void duplicatePageContent(uint8_t ch, uint8_t srcPg, uint8_t dstPg);
    // Grow numPages[ch] to at least 'target', duplicating page 0 into each
    // newly-allocated slot. No-op if target <= current numPages.
    void growPagesAndDuplicate(uint8_t ch, uint8_t target);

    // --- TRIGGER MACHINES (Menu 4) ---
    uint8_t trigMachine[NUM_CHANNELS];           // active machine type per channel
    uint8_t trigDensity[NUM_CHANNELS];           // 0..100 density / threshold
    uint8_t trigShift[NUM_CHANNELS];             // 0..15 step shift
    uint8_t machineOverlay[NUM_CHANNELS][TOTAL_STEPS]; // 0=auto, 1=force-on, 2=force-off (per-page)
    bool machinePattern[NUM_CHANNELS][NUM_STEPS];      // cached pattern for the current play page
    uint8_t machineRatchet[NUM_CHANNELS][NUM_STEPS];   // cached ratchet for the current play page
    // Kick-specific live-performance params (per channel)
    uint8_t kickNoteSpread[NUM_CHANNELS];     // 0..5 semitones added to non-base kicks
    uint8_t kickRatchetProb[NUM_CHANNELS];    // 0..100 % chance an extra step is a ratchet
    uint8_t kickExtrasAreFills[NUM_CHANNELS]; // 0=always play, 1=non-base kicks fire only when Fill held
    // Accumulating ordered list of extra (non-skeleton) source positions per
    // channel. Density adds one new weighted-random spot at a time and the
    // previously-placed ones stay put; dropping density to 0 clears the list so
    // it re-seeds fresh on the way back up. Used by every machine, not just kick.
    uint8_t machineExtraSeq[NUM_CHANNELS][NUM_STEPS];
    uint8_t machineExtraCount[NUM_CHANNELS];
    void regenerateMachinePattern(uint8_t ch);
    bool isStepActive(uint8_t ch, uint16_t absStep);
    void drawTrigMachineView();
    void drawPagesView();

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
    uint8_t fillState[NUM_CHANNELS][TOTAL_STEPS];
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
    // Last-rotated pot tracking (for screen 2 focused-parameter view on Menu 1)
    int8_t lastTouchedPot = -1;
    uint32_t lastPotTouchMs = 0;
    const uint32_t potFocusTimeout = 2500; // ms before focus view fades back to default
    // display (use concrete SH1106G implementation)
    Adafruit_SH1106G display{128, 64, &Wire};
    // secondary OLED on Wire1 (Teensy 4.1: SDA1=17, SCL1=16) — global overview screen
    Adafruit_SH1106G display2{128, 64, &Wire1};
    bool display2Present = false;
    // --- ANALOG CV OUTPUTS (MAX11300 PIXI over SPI) ---
    Max11300 pixi{MAX_CS, MAX11300_SPI_HZ};
    bool pixiPresent = false;
    void cvSelfTest(); // serial 'v': step the CV outs through 0/2.5/5/10V
    // Manual per-output voltage (0..10V), set from the Analog Outs menu (Menu 2).
    float cvVolts[NUM_CV_OUTS];
    void setCvOut(uint8_t idx, float volts); // clamp, store, write to the PIXI

    // --- HARDWARE LED GRID ---
    Adafruit_NeoPixel ledStrip;
    void updateLEDs();
    uint32_t lastDisplayMillis;
    const uint32_t displayRefreshMs = 20; // display refresh interval in ms (~50Hz)
    void drawDisplay();
    void drawOverview(); // secondary OLED: global state dashboard
    void drawDebugGrid();
    void drawNotesView();
    // Secondary OLED piano-roll: the selected channel's notes for the current
    // edit page drawn as blocks (height = pitch, width = note length).
    void drawNotesKeyboard();
    void drawEuclidView();
    void drawAnalogView(); // Menu 2: analog CV outputs
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

    // --- LIVE PERFORMANCE MODIFIERS (held combos) ---
    bool slideAllHold = false;   // Function + Fill: slide every note on the active channel
    bool accentAllHold = false;  // Function + Page: accent every note on the active channel
    uint32_t clearComboStartMs = 0; // Function + Page + Fill held: clear after 1s
    bool clearComboFired = false;

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
      uint8_t savedNoteLenIdx[NUM_CHANNELS];
      uint8_t savedChannelPitch[NUM_CHANNELS];
      bool savedMuted[NUM_CHANNELS];
      bool savedEuclidEnabled[NUM_CHANNELS];
      uint8_t savedPulses[NUM_CHANNELS];
      uint8_t savedEuclidOffset[NUM_CHANNELS];
      uint8_t savedEuclidScaleMode[NUM_CHANNELS];
      bool savedSteps[NUM_CHANNELS][TOTAL_STEPS];
      uint8_t savedPitch[NUM_CHANNELS][TOTAL_STEPS];
      uint8_t savedNoteLen[NUM_CHANNELS][TOTAL_STEPS];
      uint8_t savedFillStep[NUM_CHANNELS][TOTAL_STEPS];
      uint8_t savedStepRatchet[NUM_CHANNELS][TOTAL_STEPS];
      // Persisted Accent/Slide
      uint8_t savedStepVelocity[NUM_CHANNELS][TOTAL_STEPS];
      uint8_t savedStepSlide[NUM_CHANNELS][TOTAL_STEPS];
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
      uint8_t savedMachineOverlay[NUM_CHANNELS][TOTAL_STEPS];
      // Kick-specific live params (v8)
      uint8_t savedKickNoteSpread[NUM_CHANNELS];
      uint8_t savedKickRatchetProb[NUM_CHANNELS];
      uint8_t savedKickExtrasAreFills[NUM_CHANNELS];
      // Pages mode state (v10)
      uint8_t savedNumPages[NUM_CHANNELS];
      // Per-channel pattern length (v11)
      uint8_t savedNumSteps[NUM_CHANNELS];
    };
    void saveState();
    void loadState();
};

#endif
