#pragma once
#include <cmath>
#include <cstdint>
// Gate for the external input before it reaches the shared mono mix. An
// unplugged codec input still carries board noise and digital crosstalk;
// with the gate closed the input contributes exact silence. Line-level
// material opens it within 1 ms and passes at unity.
// Zero defaults: starts closed, and costs no flash for initial values.
struct InputGate {
    float envelope=0, gain=0;
    uint32_t hold=0;
    bool open=false;
    void Reset(){*this=InputGate();}
    float Process(float input){
        constexpr float kOpen=.0039810717f;   // -48 dBFS
        constexpr float kClose=.0015848932f;  // -56 dBFS
        constexpr uint32_t kHoldSamples=4800; // 100 ms
        float level=std::fabs(input);
        envelope=level>envelope?level:envelope*.99979169f; // 100 ms release
        if(envelope>kOpen)open=true;
        if(open){
            if(envelope>kClose)hold=kHoldSamples;
            else if(hold)--hold;
            else open=false;
        }
        // 1 ms attack, 50 ms release; settles to exact zero when closed.
        gain+=((open?1.f:0.f)-gain)*(open?.02061492f:.00041658f);
        if(!open && gain<1e-6f)gain=0;
        return input*gain;
    }
};
