#pragma once
#include <stdint.h>
namespace softsynth {
constexpr uint8_t CHANNEL=13; // wire channel 14; kick is wire channel 15
constexpr uint8_t ENGINE=119, SLIDE=29, HOOVER_BASE=70, PERIOD=76, BAND_MIX=77, LEVEL=78;
constexpr uint8_t FX_BASE=80; // BPF, tube send, HPF, delay, reverb, tracking erosion
constexpr uint8_t SKITTER=87;
constexpr const char* SKITTER_TIMING[4]={"16TH 2B","16TH 4B","8TH 2B","8TH 4B"};
constexpr uint8_t DIST_MODEL=86; // 0=tube, 127=Mackie
constexpr uint8_t HOOVER_DEFAULTS[6]={88,94,100,74,80,85};
constexpr uint8_t ACID_DEFAULTS[6]={62,76,92,48,88,0};
// CC119: Acid=0, Screech=64, Hoover=127 (existing selections preserved).
constexpr uint8_t SCREECH_BASE=90, SCREECH_LEVEL=96, SCREECH_STYLE=97;
constexpr const char* SCREECH_STYLES[6]={"SQUEAL","ERROR","CHATTER","LASER","VOWEL","STATIC"};
constexpr uint8_t SCREECH_DEFAULTS[6]={70,32,85,72,38,64};
}
