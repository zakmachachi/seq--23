#ifndef SEQCONFIG_H
#define SEQCONFIG_H

#include <Arduino.h>

// Button pins (16 buttons). User mapping: 0-12, 24-26
static const uint8_t BUTTON_PINS[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,24,25,26};

// CV output pins (analogWrite)
// (CV/Gate pins removed - using MIDI out only)

// Encoder pins - sensible defaults; edit if you wired differently
// Each encoder: A, B, Switch
static const uint8_t ENC_A[4] = {38,40,15,21};
static const uint8_t ENC_B[4] = {37,41,14,22};
static const int8_t ENC_SW[4] = {36,39,13,20};

// Sequencer parameters
static const uint8_t NUM_CHANNELS = 4;
static const uint8_t NUM_STEPS = 16;

// MIDI TX pin (connect to DIN pin of MIDI OUT optocoupler circuit)
static const uint8_t MIDI_TX_PIN = 35;
// MIDI RX pin (DIN input from MIDI IN optocoupler)
static const uint8_t MIDI_RX_PIN = 34;
// Start/Stop button pin
static const uint8_t START_STOP_PIN = 27;

// LED data pin for chained per-step LEDs (single DIN chain)
static const uint8_t LED_DATA_PIN = 16;

#endif
