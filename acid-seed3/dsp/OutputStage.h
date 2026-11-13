#pragma once
#include "Blocks.h"
namespace synth {
// Level-matched tanh compressor. This runs per voice part before the shared
// output limiter, so enabling it cannot bypass the DAC peak safety stage.
struct TanhCompressor {
    float amount=0.f,amountTarget=0.f,wet=0.f,wetTarget=0.f;
    float inputLevel=0.f,outputLevel=0.f,makeup=1.f;
    bool enabled=false;
    void SetAmount(uint8_t cc){amountTarget=cc/127.f;}
    void SetEnabled(bool on){enabled=on;wetTarget=on?1.f:0.f;}
    float Process(float x){
        amount+=(amountTarget-amount)*.001f;
        wet+=(wetTarget-wet)*.002f;
        if(!enabled && wet<.001f)return x;
        float absIn=fabsf(x);
        inputLevel+=absIn>inputLevel?.0069f*(absIn-inputLevel):.0002083f*(absIn-inputLevel);
        float drive=1.f+8.f*amount;
        float shaped=tanhf(x*drive);
        float absOut=fabsf(shaped);
        outputLevel+=absOut>outputLevel?.0069f*(absOut-outputLevel):.0002083f*(absOut-outputLevel);
        float targetMakeup=outputLevel>1e-5f?Clamp(inputLevel/outputLevel,.25f,4.f):1.f;
        // Slow makeup motion avoids gain modulation when the kick pump and
        // compressor detector react to the same transient.
        makeup+=(targetMakeup-makeup)*.0005f;
        return x+(shaped*makeup-x)*wet;
    }
};

// 1.33 ms lookahead catches accidental effect/voice peaks at unity output gain.
// Peak gain reduction only; no AGC boosting quiet tails or noise.
struct OutputStage {
    float delay[64]={},peak=0,gain=1;unsigned pos=0,hold=0,safetyClamps=0;
    float Tick(float x){
        float magnitude=fabsf(x);
        if(magnitude>=peak){peak=magnitude;hold=64;}
        else if(hold)--hold;
        else peak*=.9997917f; // approximately 100 ms peak release
        float target=peak>.94f?.94f/peak:1.f;
        gain+=(target<gain?.125f:.0004166f)*(target-gain);
        float y=delay[pos]*gain;delay[pos]=x;pos=(pos+1)&63;
        if(fabsf(y)>.98f){++safetyClamps;y=Clamp(y,-.98f,.98f);}
        return y;
    }
};

// MIDI-kick-triggered amplitude ducking for the separate Acid Seed output.
// Amount and internal-route are mirrored from the KICK FX page; the envelope
// is post-FX so the kick and synth can leave separate DACs without competing
// for the same Seed's output headroom.
struct KickOutputPump {
    float amount=0.f,amountTarget=0.f,route=0.f,routeTarget=0.f;
    float duck=0.f,duckStart=0.f;
    uint32_t age=0,attackSamples=144,releaseSamples=24000;
    bool active=false;
    void SetAmount(uint8_t cc){amountTarget=cc/127.f;}
    void SetInternalRoute(bool enabled){routeTarget=enabled?1.f:0.f;}
    void Trigger(const Clock& clock){
        if(routeTarget<.5f || amountTarget<.005f)return;
        duckStart=duck;age=0;attackSamples=144;
        const uint32_t quarter=clock.haveTick?clock.periodSamples*24u:24000u;
        releaseSamples=static_cast<uint32_t>(Clamp(.72f*quarter,4800.f,43200.f));
        active=true;
    }
    float Process(float input){
        amount+=.00208117f*(amountTarget-amount); // ~10 ms control smoothing
        route+=.00208117f*(routeTarget-route);
        float envelope=0.f;
        if(active){
            ++age;
            if(age<=attackSamples){
                float t=Clamp(float(age)/float(attackSamples),0.f,1.f);
                envelope=duckStart+(1.f-duckStart)*Smooth(t);
            }else if(age<=attackSamples+releaseSamples){
                float t=Clamp(float(age-attackSamples)/float(releaseSamples),0.f,1.f);
                envelope=1.f-Smooth(t);
            }else{active=false;duck=0.f;}
            if(active)duck=envelope;
        }
        const float gain=1.f-.78f*amount*route*duck;
        return input*gain;
    }
};
}
