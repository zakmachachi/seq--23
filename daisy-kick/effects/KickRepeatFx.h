#pragma once
#include "Blocks.h"

namespace heldfx {
// Fixed storage, no feedback, allocation, buffer clear or transcendental
// functions in the callback. Capture a kick fragment, then loop at unity rate.
struct KickRepeatFx {
    static constexpr unsigned N=32768;
    float data[N]={};
    unsigned write=0,valid=0,end=0,length=0,pos=0,changes=0;
    int lastStep=-1;
    Random rng;
    float g=.01f,targetG=.01f,s1=0,s2=0,cutoff=0;
    bool sixteenth=false;
    float lastInput=0,bridge=0;
    unsigned bridgeLeft=0;
    void Reset(){write=valid=changes=0;lastStep=-1;s1=s2=0;rng.Reset();}
    void Capture(float x){data[write]=x;write=(write+1)&(N-1);if(valid<N)++valid;}
    void Choose(float quarter){
        cutoff=100.f+1700.f*rng.Uniform()*rng.Uniform();
        float z=PI*cutoff/SR;
        targetG=z+z*z*z/3.f; // tan approximation: <0.004% error here
        sixteenth=rng.Uniform()<.5f;
        length=unsigned(Clamp(quarter*SR*(sixteenth?.25f:.5f),96.f,float(N-128)));
        pos=0;++changes;
    }
    void Start(float quarter){end=write;lastStep=0;Choose(quarter);s1=s2=0;g=targetG;bridgeLeft=0;lastInput=0;}
    float Process(float live,float quarter,float beats,bool fast){
        int step=int(beats*4.f);
        bool changed=fast && step!=lastStep;
        if(changed){lastStep=step;Choose(quarter);}
        float x=live;
        if(valid>=length+96){
            unsigned base=(end-length)&(N-1);
            x=data[(base+pos)&(N-1)];
            // Crossfade the preceding contiguous tail into the loop's start.
            if(pos<96){float t=Smooth(float(pos)/96.f);float tail=data[(end+pos-96)&(N-1)];x=tail+(x-tail)*t;}
            if(++pos>=length)pos=0;
        }
        if(changed){bridge=lastInput-x;bridgeLeft=96;}
        if(bridgeLeft){x+=bridge*(float(bridgeLeft)/96.f);--bridgeLeft;}
        lastInput=x;
        g+=(targetG-g)*.002f;
        constexpr float k=.8f; // Q=1.25: a modest, bounded DJ resonance
        float a=1.f/(1.f+g*(g+k));
        float v1=a*(s1+g*(x-s2)),v2=s2+g*v1;
        s1=2.f*v1-s1;s2=2.f*v2-s2;
        return .8f*(x-k*v1-v2);
    }
};
}
