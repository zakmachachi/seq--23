#pragma once
#include <stdint.h>

// Shared SEQ23/Seed protocol. MIDI channel remains the kick channel (15).
namespace heldfx {
enum Cc : uint8_t { SELECT=80, CHARACTER=81, MOTION=82, SIZE=83, MIX=84, KEEPALIVE=85, LAYERS=86 };
constexpr uint8_t KICK_TEMP_MUTE=60, KICK_PERFORMANCE_HPF=62, KICK_TEMP_DECAY=63, KICK_TEMP_LEVEL=66, KICK_DUB_ECHO=65;
constexpr uint8_t KICK_GROUND_MACHINE=70;
// Acid Seed 3 internal held dub tail: shortens the voice decay and opens its
// own feedback echo, independent of the Kick Seed's CC65 effect.
constexpr uint8_t ACID_DUB_ECHO=111;
enum Mode : uint8_t { HPF_REPEAT, TOY, RADIO, MEMORY, METAL, BINARY, DECAY, HAZARD, MODE_COUNT };
enum Layer : uint8_t { BPF_HELD=1, BPF_RETRIGGER=2, RATCHET=4 };
constexpr uint32_t LEASE_MS=900, HEARTBEAT_MS=200;
constexpr uint8_t DEFAULT_CHARACTER=127, DEFAULT_MOTION=88, DEFAULT_SIZE=64, DEFAULT_MIX=112;
inline const char* Name(unsigned mode){
    static const char* const names[]={"BPF/RND","TOY","RADIO","MEMORY","METAL","BINARY","DECAY","HAZARD"};
    return names[mode%MODE_COUNT];
}
inline float Smooth(float x){return x*x*(3.f-2.f*x);}
// Builds rise over four beats then hold; fast gestures repeat once per beat.
// High-level CCs scale these performances without adding a controller menu.
inline float Gesture(float beats,bool fast){
    if(!fast)return .18f+.82f*Smooth(beats>=4.f?1.f:beats*.25f);
    float phase=beats-int(beats);
    float triangle=phase<.5f?phase*2.f:(1.f-phase)*2.f;
    return .30f+.70f*Smooth(triangle);
}
}
