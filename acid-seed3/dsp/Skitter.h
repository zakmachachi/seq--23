#pragma once
#include "Blocks.h"
namespace synth {
// Clock-locked amplitude gestures. No oscillator retriggers or post-FX gating.
struct Skitter {
    uint8_t mode=0; // 0 bypass; 1/2 sixteenths at 2/4 bars; 3/4 eighths
    float wet=0,amplitude=1;
    static float Triangle(float phase){return 1.f-fabsf(2.f*Wrap(phase)-1.f);}
    static float Hold(float cyclePhase){
        // A triangle phase-modulates another triangle; both return exactly
        // to their initial phase every complete 2/4-bar cycle.
        float outer=Triangle(cyclePhase);
        return .07f+.80f*Triangle(2.f*cyclePhase+.35f*outer);
    }
    void Set(uint8_t value){mode=value<=4?value:0;}
    float Process(float input,const Clock& clock,uint32_t now){
        bool enabled=mode && clock.running && clock.haveTick;
        float target=1;
        if(enabled){
            unsigned step=mode>=3?12:6;
            unsigned cycle=(mode==2||mode==4)?384:192;
            // First clock after Start is the downbeat, not tick one late.
            unsigned tick=((clock.ticks+767)%768)%cycle;
            float fraction=fminf(1.f,float(uint32_t(now-clock.last))/clock.periodSamples);
            unsigned start=tick-tick%step;
            float p=(float(tick%step)+fraction)/step;
            if(p>=1.f){p=0;start=(start+step)%cycle;}
            // Latch width from this pulse's start: changing LFO hold duration
            // during its release could otherwise re-open the same pulse.
            float hold=Hold(float(start)/cycle);
            float stepSamples=float(clock.periodSamples*step);
            float a=Clamp(96.f/stepSamples,.005f,.12f);
            float d=Clamp(288.f/stepSamples,.015f,.2f);
            float end=a+hold*(1.f-a-d);
            target=p<a?Smooth(p/a):p<end?1.f:p<end+d?1.f-Smooth((p-end)/d):0.f;
        }
        amplitude+=.020619f*(target-amplitude); // 1 ms edge/jitter guard
        wet+=.00104112f*((enabled?1.f:0.f)-wet); // 20 ms bypass fade
        if(!enabled&&wet<.000001f){wet=0;amplitude=1;return input;}
        return input*(1.f+wet*(amplitude-1.f));
    }
};
}
