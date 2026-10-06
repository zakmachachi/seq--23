#pragma once
#include <cmath>

// Two complementary Hann-windowed delay heads. ~43 ms grains at 48 kHz.
// Time-domain FX: not a formant-preserving vocal transposer. Zero is exact dry.
struct ExternalPitchFx {
    float buffer[4096]={};
    unsigned write=0;
    float phase=0,ratio=0,wet=0,lp1=0,lp2=0;
    void Reset(){for(float& x:buffer)x=0;write=0;phase=wet=lp1=lp2=0;ratio=1;}
    float Read(float delay) const {
        int whole=int(delay);float f=delay-whole;
        unsigned i=(write-whole)&4095;
        return buffer[i]+(buffer[(i-1)&4095]-buffer[i])*f;
    }
    float Process(float input,float targetRatio){
        ratio+=(targetRatio-ratio)*.0006942034f;
        float targetWet=targetRatio==1.f?0.f:1.f;
        wet+=(targetWet-wet)*.0004165799f;
        if(!targetWet && wet<1e-7f)wet=0;
        // Limit ultrasonic folding on upward shifts; only the wet signal is filtered.
        lp1+=(input-lp1)*.65f;lp2+=(lp1-lp2)*.65f;
        buffer[write]=lp2;
        phase+=(1.f-ratio)/2048.f;
        if(phase<0)phase+=1.f;else if(phase>=1)phase-=1.f;
        float other=phase+.5f;if(other>=1)other-=1;
        float weight=.5f-.5f*cosf(6.28318530718f*phase);
        float shifted=Read(16.f+2048.f*phase)*weight+Read(16.f+2048.f*other)*(1.f-weight);
        write=(write+1)&4095;
        return input+(shifted-input)*wet;
    }
};
