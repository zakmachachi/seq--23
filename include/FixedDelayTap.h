#pragma once
#include <cmath>
#include <stdint.h>

// Tempo changes crossfade two stationary read heads. No moving-tap Doppler
// pitch bend; latest requested time is picked up after an in-flight fade.
struct FixedDelayTap {
    float current=0,next=0;
    unsigned fade=0;
    static constexpr unsigned FADE_SAMPLES=1440; // 30 ms at 48 kHz
    void Reset(){current=next=0;fade=0;}
    static float Read(const float* buffer,unsigned size,unsigned write,
                      unsigned valid,float delay){
        unsigned whole=static_cast<unsigned>(delay);
        if(valid<whole)return 0;
        float frac=delay-whole;
        unsigned a=(write+size-whole)%size,b=(a+size-1)%size;
        float older=valid>whole?buffer[b]:0;
        return buffer[a]+frac*(older-buffer[a]);
    }
    float Process(const float* buffer,unsigned size,unsigned write,
                  unsigned valid,float requested){
        requested=fmaxf(2.f,fminf(float(size-2),requested));
        if(current==0)current=next=requested;
        // Ignore sub-millisecond clock jitter instead of constantly crossfading.
        if(!fade&&fabsf(requested-current)>48.f){next=requested;fade=FADE_SAMPLES;}
        float a=Read(buffer,size,write,valid,current);
        if(!fade)return a;
        float x=1.f-float(fade)/FADE_SAMPLES;x=x*x*(3.f-2.f*x);
        float y=a+(Read(buffer,size,write,valid,next)-a)*x;
        if(--fade==0)current=next;
        return y;
    }
};
