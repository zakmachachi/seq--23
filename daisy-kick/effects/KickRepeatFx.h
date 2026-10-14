#pragma once
#include "Blocks.h"

namespace heldfx {
// Capture the first eighth of the new kick, then repeat that phrase at unity
// rate. Fixed storage, no resampling, feedback or callback-time allocation.
struct KickRepeatFx {
    static constexpr unsigned N=32768;
    float data[N]={};
    unsigned write=0,valid=0,end=0,length=0,pos=0,recordLeft=0;
    float startBlend=0;
    void Reset(){write=valid=end=length=pos=recordLeft=0;}
    void Capture(float x){data[write]=x;write=(write+1)&(N-1);if(valid<N)++valid;}
    void Start(float quarter){
        length=unsigned(Clamp(quarter*SR*.5f,192.f,float(N-128)));
        recordLeft=length+96;pos=0;startBlend=0;
    }
    float Process(float live){
        if(recordLeft){Capture(live);if(--recordLeft==0)end=write;return live;}
        if(!length || valid<length)return live;
        unsigned base=(end-length-96)&(N-1);
        float x=data[(base+96+pos)&(N-1)];
        // Overlap the end with the 96-sample lead-in. At wrap the last
        // lead-in sample continues into base+96, without shortening the beat.
        if(pos>=length-96){unsigned fade=pos-(length-96);
            float t=Smooth(float(fade)/96.f);
            float head=data[(base+fade)&(N-1)];x+=(head-x)*t;}
        startBlend=fminf(1.f,startBlend+1.f/384.f);
        x=live+(x-live)*Smooth(startBlend);
        if(++pos>=length)pos=0;
        return x;
    }
};
}
