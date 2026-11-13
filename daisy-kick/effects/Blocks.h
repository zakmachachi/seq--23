#pragma once
#include <cmath>
#include <stdint.h>
namespace heldfx {
constexpr float PI=3.14159265358979323846f, SR=48000.f;
inline float Clamp(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
inline float Unit(float x){return Clamp(x,0.f,1.f);}
inline float Sat(float x){return x/(1.f+fabsf(x));}
inline float Wrap(float x){return x-floorf(x);}
struct Random {
    uint32_t state=0;
    void Reset(){state=0x19490723u;}
    uint32_t Next(){state^=state<<13;state^=state>>17;state^=state<<5;return state;}
    float Uniform(){return (Next()>>8)*(1.f/16777216.f);}
    float Bipolar(){return Uniform()*2.f-1.f;}
};
struct Band {
    float g=0,k=0,a=0,s1=0,s2=0;
    void Reset(){s1=s2=0;}
    void Tune(float hz,float q){g=tanf(PI*Clamp(hz,40.f,9000.f)/SR);k=1.f/Clamp(q,.5f,240.f);a=1.f/(1.f+g*(g+k));}
    float Process(float x){float v1=a*(s1+g*(x-s2)),v2=s2+g*v1;s1=2*v1-s1;s2=2*v2-s2;return k*v1;}
};
struct Analysis {
    float fast=0,slow=0,low=0,bright=0;uint32_t since=0,quiet=0;
    bool transient=false;
    void Reset(){fast=slow=low=bright=0;since=48000;quiet=0;transient=false;}
    void Process(float x){
        float level=fabsf(x);fast+=(level-fast)*(level>fast?.025f:.0007f);slow+=(level-slow)*.00035f;
        low+=(x-low)*.06f;bright+=(fabsf(x-low)-bright)*.001f;
        if(since<480000)++since;
        transient=since>2400 && fast>slow*1.75f+.004f && level>.012f;
        if(transient)since=0;
        quiet=level<1e-6f?quiet+unsigned(quiet<480000):0;
    }
    float Protect() const {return 1.f-Unit((float(since)-144.f)/576.f);} // intact first 3 ms; fade to effect over 12 ms
};
struct History {
    static constexpr unsigned N=65536;
    float data[N]={};unsigned write=0;
    void Reset(){for(float& x:data)x=0;write=0;}
    void Push(float x){data[write]=x;write=(write+1)&(N-1);}
    float At(float index) const {
        int whole=int(floorf(index));float f=index-whole;unsigned i=unsigned(whole)&(N-1);
        return data[i]+(data[(i+1)&(N-1)]-data[i])*f;
    }
    float Delay(float delay) const {return At(float(write)-1.f-Clamp(delay,1.f,float(N-2)));}
};
inline float Quantize(float x,float bits){
    bits=Clamp(bits,3.f,16.f);unsigned b=unsigned(bits);float f=bits-b,levels=float(1u<<(b-1));
    x=Clamp(x,-.9999f,.9999f);
    float coarse=roundf(x*levels)/levels,fine=roundf(x*(levels*2.f))/(levels*2.f);
    return coarse+(fine-coarse)*f;
}
struct Macro {float character=0,motion=0,size=0,mix=0,quarter=0;};
}
