#pragma once
#include <cmath>
#include <cstdint>
// Gate for the external input before it reaches the shared mono mix. An
// unplugged codec input still picks up board noise and digital crosstalk.
// A peak gate opening at -48 dBFS chattered on it, turning a constant whine
// into faint periodic beeps, so that pickup reaches roughly that level and
// is bursty. The detector averages the rectified input
// over ~10 ms so short bursts cannot open it, and the thresholds sit well
// above that pickup: a playing line-level source opens it within ~1 ms of
// reaching -36 dBFS average and passes at unity.
// Zero defaults: starts closed, and costs no flash for initial values.
struct InputGate {
    float level=0, gain=0;
    uint32_t hold=0;
    bool open=false;
    void Reset(){*this=InputGate();}
    float Process(float input){
        constexpr float kOpen=.015848932f;    // -36 dBFS average rectified
        constexpr float kClose=.0050118723f;  // -46 dBFS
        constexpr uint32_t kHoldSamples=12000; // 250 ms
        level+=(std::fabs(input)-level)*.0020811719f; // 10 ms
        if(level>kOpen)open=true;
        if(open){
            if(level>kClose)hold=kHoldSamples;
            else if(hold)--hold;
            else open=false;
        }
        // 1 ms attack, 50 ms release; settles to exact zero when closed.
        gain+=((open?1.f:0.f)-gain)*(open?.02061492f:.00041658f);
        if(!open && gain<1e-6f)gain=0;
        return input*gain;
    }
};
