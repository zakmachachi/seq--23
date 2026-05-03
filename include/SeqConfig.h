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
static const uint8_t MATRIX_COL_IDLE = HIGH;
static const uint8_t MATRIX_COL_ACTIVE = LOW;

// Matrix dimensions (global)
static const uint8_t MATRIX_ROWS = 5;
static const uint8_t MATRIX_COLS = 6;
static const uint8_t MATRIX_KEYS = MATRIX_ROWS * MATRIX_COLS;

// Matrix special buttons (indexes are row-major: idx = row * MATRIX_COLS + col)
// Set START (play) to matrix element 24 for now.
static const uint8_t MATRIX_BTN_START_INDEX = 24;
// Channel modifier (fill) — mapped to a matrix element; set to 25 by default (adjust if different)
static const uint8_t MATRIX_BTN_CHANNEL_INDEX = 25;

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
// NOTE: some of these new pin assignments overlap with earlier defaults (e.g. LED_DATA_PIN
// previously used pin 16). Update wiring or LED pin if needed. Keep MIDI pins as before unless
// your PCB remapped them.

// Sequencer parameters
static const uint8_t NUM_CHANNELS = 4;
static const uint8_t NUM_STEPS = 16;

// MIDI TX/RX: keep defaults unless your PCB remapped MIDI
// MIDI TX/RX: updated for new PCB
static const uint8_t MIDI_TX_PIN = 20; // MIDI OUT (connect to DIN of MIDI OUT opto/driver)
static const uint8_t MIDI_RX_PIN = 21; // MIDI IN (connect from MIDI IN opto)

// LED data pin for chained per-step LEDs
static const uint8_t LED_DATA_PIN = 29;

#endif
