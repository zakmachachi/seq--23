#ifndef SEQCONFIG_H
#define SEQCONFIG_H

#include <Arduino.h>


// --- Button matrix (5 rows x 6 columns) ------------------------
// Hardware: 5 row pins (inputs with internal pull-ups), 6 column pins (outputs)
// Buttons connect ROW to COL when pressed. Columns are driven HIGH idle, pulled LOW to scan.
// Scan convention below: set all COLs HIGH, pull one COL LOW, wait, read ROW inputs.
static const uint8_t MATRIX_COL_PINS[6] = {7, 6, 5, 4, 3, 2};   // outputs (drive HIGH idle, LOW to scan)
static const uint8_t MATRIX_ROW_PINS[5] = {12, 11, 10, 9, 8};    // inputs (use INPUT_PULLUP)
// Column idle/active states
static const uint8_t MATRIX_COL_IDLE = HIGH;
static const uint8_t MATRIX_COL_ACTIVE = LOW;

// --- Potentiometer mappings (each pot has two analog inputs + a push button) ---
// Format per pot: PotX: PinA, PinB, Button
static const uint8_t POT_A_PINS[6] = {30, 32, 35, 44, 16, 18};
static const uint8_t POT_B_PINS[6] = {31, 33, 36, 45, 17, 19};
static const uint8_t POT_BTN_PINS[6] = {28, 27, 26, 25, 22, 23};

// --- OLED displays ---
// Primary OLED to use (SH110x) — SDA/SCL for Wire() on this display
static const uint8_t OLED1_SDA_PIN = 40; // user-specified
static const uint8_t OLED1_SCL_PIN = 41; // user-specified
// Secondary OLED (not used for now)
static const uint8_t OLED2_SDA_PIN = 15;
static const uint8_t OLED2_SCL_PIN = 14;

// --- Previous encoder/button/LED mappings removed in favour of matrix/pots ---
// NOTE: some of these new pin assignments overlap with earlier defaults (e.g. LED_DATA_PIN
// previously used pin 16). Update wiring or LED pin if needed. Keep MIDI pins as before unless
// your PCB remapped them.

// Sequencer parameters
static const uint8_t NUM_CHANNELS = 4;
static const uint8_t NUM_STEPS = 16;

// MIDI TX/RX: keep defaults unless your PCB remapped MIDI
// MIDI TX/RX: updated for new PCB
static const uint8_t MIDI_TX_PIN = 42; // MIDI OUT (connect to DIN of MIDI OUT opto/driver)
static const uint8_t MIDI_RX_PIN = 43; // MIDI IN (connect from MIDI IN opto)

// Start/Stop button (if a dedicated button still wired separately)
static const uint8_t START_STOP_PIN = 27;

// LED data pin for chained per-step LEDs (single DIN chain). If you moved Pot5 to pin 16,
// change this to a free GPIO to avoid conflict.
static const uint8_t LED_DATA_PIN = 29;

#endif
