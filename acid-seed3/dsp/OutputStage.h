#pragma once
#include "Blocks.h"
namespace synth {
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
}
