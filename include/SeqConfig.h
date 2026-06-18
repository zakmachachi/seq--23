#ifndef SEQCONFIG_H
#define SEQCONFIG_H

#include <Arduino.h>


// --- Button matrix (5 rows x 6 columns) ------------------------
// Hardware: 5 row pins (inputs with internal pull-ups), 6 column pins (outputs)
// Buttons connect ROW to COL when pressed. Columns are driven HIGH idle, pulled LOW to scan.
// Scan convention below: set all COLs HIGH, pull one COL LOW, wait, read ROW inputs.
static const uint8_t MATRIX_ROW_PINS[5] = {6, 7, 8, 9, 10};   // INPUTS (use internal pull-ups, read LOW when pressed) - connect to rows of button matrix
static const uint8_t MATRIX_COL_PINS[6] = {5,4, 3, 2, 1, 0};    // outputs (drive HIGH idle, LOW to scan)
// Column idle/active states
// Column idle/active states: columns idle LOW, driven HIGH to scan (matches teensi.ino)
static const uint8_t MATRIX_COL_IDLE = LOW;
static const uint8_t MATRIX_COL_ACTIVE = HIGH;

// Matrix dimensions (global)
static const uint8_t MATRIX_ROWS = 5;
static const uint8_t MATRIX_COLS = 6;
static const uint8_t MATRIX_KEYS = MATRIX_ROWS * MATRIX_COLS;

// Matrix special buttons (indexes are row-major: idx = row * MATRIX_COLS + col)
static const uint8_t MATRIX_BTN_FUNCTION_INDEX = 22;  // Hardware: physical function/modifier key (matrix idx 22)
static const uint8_t MATRIX_BTN_PAGE_INDEX     = 16;  // Page (transport modifier) — verified KEY 16
static const uint8_t MATRIX_BTN_FILL_INDEX     = 17;  // Fill (performance) — verified KEY 17
static const uint8_t MATRIX_BTN_MENU1_INDEX    = 18;  // Menu: Notes
static const uint8_t MATRIX_BTN_MENU2_INDEX    = 19;  // Menu: Euclid
static const uint8_t MATRIX_BTN_MENU3_INDEX    = 20;  // Menu: Step Visualizer
static const uint8_t MATRIX_BTN_MENU4_INDEX    = 21;  // Menu: Trigger Machines (button right of MENU3)
// Channel select buttons (pressing alone = select, Function held = mute/unmute)
// PCB v2: 7 consecutive channel buttons (idx 23..29 in reverse).
static const uint8_t MATRIX_BTN_CH[7] = {29, 28, 27, 26, 25, 24, 23}; // Ch1-7 → matrix idx 29..23
// Transport: Function + Page held together = Start/Stop toggle
static const uint8_t MATRIX_BTN_START_INDEX   = MATRIX_BTN_FUNCTION_INDEX; // kept for runEngine compat
static const uint8_t MATRIX_BTN_CHANNEL_INDEX = MATRIX_BTN_PAGE_INDEX;     // kept for runEngine compat

// Analog outs (for CV) #TODO: update these for new PCB when needed; not currently used but will be in future updates
static const uint8_t MAX_MOSI = 11; // SPI MOSI pin for DAC (not used at the moment but will be in future updates)
static const uint8_t MAX_MISO = 12; // SPI MISO pin for DAC (not used at the moment but will be in future updates)
static const uint8_t MAX_INTB = 28; // DAC interrupt pin (not used at the moment but will be in future updates)
static const uint8_t MAX_CNVTB = 32 ; // DAC convert pin (not used at the moment but will be in future updates)
static const uint8_t MAX_CS = 37; // DAC chip select pin (not used at the moment but will be in future updates)
static const uint8_t MAX_SCK = 13; // SPI clock pin for DAC (not used at the moment but will be in future updates)

// --- Potentiometer mappings (each pot has two analog inputs + a push button) ---
// Format per pot: PotX: PinA, PinB, Button
static const uint8_t POT_A_PINS[6] = {38, 40, 14, 22, 24, 26};
static const uint8_t POT_B_PINS[6] = {39, 41, 15, 23, 25, 27};
static const uint8_t POT_BTN_PINS[6] = {36, 35, 34, 33, 30, 31};

// --- OLED displays ---
// Primary OLED to use (SH110x) — SDA/SCL for Wire() on this display
static const uint8_t OLED1_SDA_PIN = 19; // user-specified
static const uint8_t OLED1_SCL_PIN = 18; // user-specified
// Secondary OLED (not used for now)
static const uint8_t OLED2_SDA_PIN = 17;
static const uint8_t OLED2_SCL_PIN = 16;

// --- Previous encoder/button/LED mappings removed in favour of matrix/pots ---
// NOTE: LEDs are currently disabled in firmware during button/pot bring-up.

// Sequencer parameters
static const uint8_t NUM_CHANNELS = 7;
static const uint8_t NUM_STEPS = 16;
// Up to 4 pages per channel (Digitakt-style). Per-step arrays grow to
// NUM_STEPS * MAX_PAGES = 64 cells.
static const uint8_t MAX_PAGES = 4;
static const uint16_t TOTAL_STEPS = (uint16_t)NUM_STEPS * (uint16_t)MAX_PAGES;

// MIDI TX/RX: keep defaults unless your PCB remapped MIDI
// MIDI TX/RX: updated for new PCB
static const uint8_t MIDI_TX_PIN = 20; // MIDI OUT (connect to DIN of MIDI OUT opto/driver)
static const uint8_t MIDI_RX_PIN = 21; // MIDI IN (connect from MIDI IN opto)

// MIDI serial interface (Serial5 on Teensy 4.1 uses pins 20/21)
#define MIDI_SERIAL Serial5

// --- WS2812 LEDs (single chain wired in series) ---
// Physical wiring (hardware docs use 1-based "LED numbers"; firmware uses
// 0-based indices, shown below):
//   LED 1..16   step buttons          -> idx 0..15
//   LED 17      Page button           -> idx 16
//   LED 18      Fill button           -> idx 17
//   LED 19..22  Menu 1..4 buttons     -> idx 18..21
//   LED 23      Function button       -> idx 22
//   LED 24..30  Channel LEDs (ch7..ch1)-> idx 23..29 (ch1=29 ... ch7=23)
static const uint8_t LED_PIN = 29;
static const uint8_t LED_STEP_COUNT = NUM_STEPS; // first 16 LEDs = step buttons
static const uint8_t LED_COUNT = 30;             // full chain length
static const uint8_t LED_BRIGHTNESS = 80;        // 0..255
// UI / channel LED indices on the shared chain (0-based)
static const uint8_t LED_PAGE_INDEX     = 16;
static const uint8_t LED_FILL_INDEX     = 17;
static const uint8_t LED_MENU_BASE      = 18; // menu1..menu4 = idx 18..21
static const uint8_t LED_FUNCTION_INDEX = 22;
// Channel LED for a 0-based channel: ch1->29, ch2->28, ... ch7->23.
static inline uint8_t ledForChannel(uint8_t ch){ return (uint8_t)(29 - ch); }

#endif
