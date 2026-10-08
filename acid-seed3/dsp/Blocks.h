#pragma once
#include <cmath>
#include <stdint.h>
namespace synth {
constexpr float SR=48000.f, PI=3.14159265358979323846f;
inline float Clamp(float x,float a,float b){return x<a?a:x>b?b:x;}
inline float Smooth(float x){return x*x*(3.f-2.f*x);}
inline float Sat(float x){x=Clamp(x,-3.f,3.f);return x*(27.f+x*x)/(27.f+9.f*x*x);}
inline float Wrap(float x){return x-floorf(x);}
inline float Tri(float p){return 1.f-4.f*fabsf(p-.5f);}
inline float Blep(float p,float dt){
    if(p<dt){float x=p/dt;return x+x-x*x-1.f;}
    if(p>1.f-dt){float x=(p-1.f)/dt;return x*x+x+x+1.f;}
    return 0;
}
struct SVF {
    float g=.1f,k=1.f,a=1.f,s1=0,s2=0;
    void Tune(float hz,float q,float sr=SR){g=tanf(PI*Clamp(hz,25.f,sr*.2f)/sr);k=1.f/Clamp(q,.5f,4.f);a=1.f/(1.f+g*(g+k));}
    float Process(float x,bool band=false){float v1=a*(s1+g*(x-s2)),v2=s2+g*v1;s1=2*v1-s1;s2=2*v2-s2;return band?k*v1:v2;}
};
struct DC {
    float x1=0,y1=0;
    float Process(float x){float y=x-x1+.9974f*y1;x1=x;y1=y;return y;}
};
// Real T/L state machine: three timed levels, sustain L3, release T4.
// Retriggers start at the current value; note-off jumps to T4 from any stage.
struct TLEnvelope {
    float t[4]={.004f,.15f,.3f,.05f},l[3]={1,.8f,.65f};
    float value=0,from=0,progress=0;unsigned stage=5;bool junoCurve=false;
    void On(){stage=0;progress=0;from=value;}
    void Off(){stage=4;progress=0;from=value;}
    float Tick(){
        if(stage==5)return 0;
        if(stage==3)return value;
        unsigned idx=stage==4?3:stage;
        progress=fminf(1.f,progress+1.f/(SR*fmaxf(.001f,t[idx])));
        float dest=stage==4?0.f:l[stage];
        float curve=Smooth(progress);
        if(junoCurve){
            curve=progress;
            // Linear T1/T2; a bounded exponential-like approach for T3/release.
            if(stage==2 || stage==4){float r=1-progress;curve=1-r*r*r*r;}
        }
        value=from+(dest-from)*curve;
        if(progress>=1.f){from=value;progress=0;if(stage==4){stage=5;value=0;}else ++stage;}
        return value;
    }
};
struct Clock {
    unsigned ticks=0;uint32_t last=0,periodSamples=1000;bool running=false,haveTick=false;float heldFraction=0;
    void Realtime(uint8_t b,uint32_t now){
        if(b==0xfa){ticks=0;last=now;haveTick=false;running=true;heldFraction=0;}
        if(b==0xfc){if(running&&haveTick)heldFraction=fminf(1.f,float(uint32_t(now-last))/periodSamples);running=false;}
        if(b==0xfb){running=true;last=now;haveTick=false;}
        if(b==0xf8 && running){
            uint32_t gap=now-last;
            if(haveTick && gap>=200 && gap<=6000)periodSamples=(periodSamples*3+gap)/4;
            last=now;haveTick=true;heldFraction=0;ticks=(ticks+1)%768;
        }
    }
    float Phase(unsigned bars,uint32_t now)const{
        unsigned length=bars==8?768:384;
        float fraction=running&&haveTick?fminf(1.f,float(uint32_t(now-last))/periodSamples):heldFraction;
        return Wrap((float(ticks%length)+fraction)/length);
    }
};
}
