#pragma once
#include <stdint.h>
namespace softsynth {
// Teensy row 5 is the editable synth voice on MIDI 12; the kick follower also
// uses this voice on its offbeats. Other synth lanes use MIDI 13 and 14
// (zero-based 12/13); the kick remains on MIDI channel 15.
constexpr uint8_t CHANNEL=11;
constexpr uint8_t CHANNEL_B=12;
constexpr uint8_t CHANNEL_C=13;
constexpr uint8_t CHANNEL_COUNT=3;
constexpr uint8_t USER_CHANNEL_FIRST=CHANNEL_B;
constexpr uint8_t USER_CHANNEL_LAST=CHANNEL_C;
inline int8_t VoiceIndex(uint8_t channel){
    return channel==CHANNEL?0:channel==CHANNEL_B?1:channel==CHANNEL_C?2:-1;
}
constexpr uint8_t KICK_CHANNEL=14;
// Kick Seed next-hit packet. CC 110..114 stage one complete kick preset;
// CC 115 commits it atomically for the next MIDI kick trigger.
constexpr uint8_t KICK_NEXT_PITCH=110, KICK_NEXT_DECAY=111,
                  KICK_NEXT_SHAPE=112, KICK_NEXT_SWEEP=113,
                  KICK_NEXT_TAIL=114, KICK_NEXT_COMMIT=115;
constexpr uint8_t ACID_DELAY_RATE=108;
constexpr uint8_t ACID_BPF_PERIOD=109;
constexpr uint8_t ACID_COMP_AMOUNT=116, ACID_COMP_ENABLE=117, ACID_OUTPUT_LEVEL=118;
constexpr uint8_t ACID_DUB_ECHO=111;
constexpr uint8_t KICK_PUMP_AMOUNT=35; // shared KICK FX-menu PUMP macro
constexpr uint8_t KICK_FX_ROUTES=92;   // bit 0 enables the internal pump route
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
