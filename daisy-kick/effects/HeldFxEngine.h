#pragma once
#include "IndustrialFx.h"
#include "HazardFx.h"
#include "KickRepeatFx.h"

namespace heldfx {
// MIDI/control writer and audio reader exchange aligned scalar mailboxes only.
// Existing USART implementation is untouched. SELECT carries absolute held state;
// keepalives renew a lease so a missing release/disconnected controller fades dry.
struct Control {
    volatile uint32_t request=0,lastRx=0;
    volatile uint8_t character=DEFAULT_CHARACTER,motion=DEFAULT_MOTION,size=DEFAULT_SIZE,mix=DEFAULT_MIX;
    bool Handle(uint8_t cc,uint8_t value,uint32_t now){
        switch(cc){
        case SELECT:lastRx=now;request=((request+256u)&~255u)|(value<=16?value:0);return true;
        case KEEPALIVE:lastRx=now;return true;
        case CHARACTER:character=value;return true;
        case MOTION:motion=value;return true;
        case SIZE:size=value;return true;
        case MIX:mix=value;return true;
        default:return false;
        }
    }
};
struct Engine {
    History history;
    IndustrialFx industrial;
    MemoryFx memory;
    HazardFx hazard;
    KickRepeatFx repeat;
    Analysis analysis;
    Macro macro;
    uint32_t lastRequest=0,pendingSamples=0,controlCounter=0;
    uint8_t pending=0,active=0,next=0;
    bool fadingOut=false,wasRunning=false;
    float gain=0,beats=0,quarter=0,dcIn=0,dcOut=0;
    float targetCharacter=0,targetMotion=0,targetSize=0,targetMix=0;
    uint32_t faults=0;
    bool suspended=false;
    bool releasePending=false;
    uint32_t controlRequest=0;
    Random bpfRng;
    float kickBpfHz[3]={330.f,700.f,1450.f};
    bool kickBpfSet=false;
    float emergencyOffset=0,lastDelta=0;
    bool emergencyKick=false;
    void Suspend(){
        if(!active || suspended)return;
        suspended=true;emergencyOffset=lastDelta;emergencyKick=active==1 || active==9;
        pending=next=active=0;gain=0;fadingOut=false;++faults;
    }

    void Reset(){
        history.Reset();industrial.Reset(0);memory.Reset();hazard.Reset();analysis.Reset();
        repeat.Reset();suspended=false;releasePending=false;emergencyOffset=lastDelta=0;emergencyKick=false;
        controlRequest=0;bpfRng.Reset();kickBpfSet=false;
        macro={};lastRequest=pendingSamples=controlCounter=0;pending=active=next=0;
        fadingOut=wasRunning=false;gain=beats=dcIn=dcOut=0;quarter=.5f;
        targetCharacter=targetMotion=targetSize=targetMix=0;faults=0;
    }
    void BeginBlock(const Control& control,uint32_t now,bool running,bool kick,bool quarterEdge,float quarterMs,unsigned frames){
        controlRequest=uint8_t(control.request);
        quarter=Clamp(quarterMs*.001f,.12f,1.2f);
        uint32_t command=control.request;
        bool expired=uint32_t(now-control.lastRx)>LEASE_MS;
        if(wasRunning && !running){pending=next=0;fadingOut=true;lastRequest=command;}
        wasRunning=running;
        // Re-anchor the gesture to actual MIDI quarter edges; smoothing absorbs
        // small timing corrections without read-head or parameter discontinuities.
        if(quarterEdge && active)beats=floorf(beats+.5f);
        if(command!=lastRequest){
            bool quantizedRelease=(uint8_t(command)==0 && uint8_t(lastRequest)==9);
            lastRequest=command;pending=uint8_t(command);pendingSamples=0;next=0;
            releasePending=false;
            if(!pending)suspended=false;
            if(!pending){
                if(quantizedRelease)releasePending=true;
                else {next=0;fadingOut=true;}
            }
        }
        if(expired){pending=next=0;fadingOut=true;releasePending=false;controlRequest=0;kickBpfSet=false;}
        if(suspended){pending=next=0;return;}
        if(pending){
            pendingSamples+=frames;
            // Causal quantization: next kick OR quarter; no clock means a
            // one-quarter bounded fallback instead of an indefinitely stuck pad.
            bool fastStart=pending==9;
            if(!running || kick || (!fastStart && quarterEdge) || (!fastStart && pendingSamples>=unsigned(quarter*SR)+frames)){
                next=pending;pending=0;fadingOut=true;
            }
        }
        if(releasePending && (!running || kick || quarterEdge)){
            releasePending=false;next=0;fadingOut=true;kickBpfSet=false;
        }
        targetCharacter=control.character/127.f;targetMotion=control.motion/127.f;
        targetSize=control.size/127.f;targetMix=control.mix/127.f;
    }
    bool KickBpfForKick(bool kick){
        if(!kick || suspended)return false;
        uint8_t command=uint8_t(controlRequest);
        if(command!=1 && command!=9){kickBpfSet=false;return false;}
        if(command==9 || !kickBpfSet){
            float centre=120.f+2680.f*bpfRng.Uniform();
            kickBpfHz[0]=Clamp(centre*.5f,85.f,3200.f);
            kickBpfHz[1]=Clamp(centre,85.f,3200.f);
            kickBpfHz[2]=Clamp(centre*2.f,85.f,3200.f);
            kickBpfSet=true;
        }
        return true;
    }
    bool KickBpfEngaged() const { return controlRequest==1 || controlRequest==9 || releasePending; }
    void Activate(){
        active=next;next=0;beats=0;controlCounter=0;dcIn=dcOut=0;macro.character=0;
        if(active){
            unsigned mode=(active-1)%MODE_COUNT;
            if(mode==HPF_REPEAT)repeat.Start(quarter);
            else if(mode==MEMORY)memory.Reset();else if(mode==HAZARD)hazard.Reset();else industrial.Reset(mode);
        }
        fadingOut=false;
    }
    float Process(float input,float* kick=nullptr){
        // No effect feedback ever reaches the capture/input analysis stage.
        if(!std::isfinite(input))input=0;
        if(emergencyOffset!=0.f){
            float offset=emergencyOffset;emergencyOffset*=.99f;
            if(fabsf(emergencyOffset)<1e-7f)emergencyOffset=0;
            if(emergencyKick && kick)*kick+=offset;else input+=offset;
            return input;
        }
        if(suspended)return input;
        history.Push(input);analysis.Process(input);
        if(active!=1 && active!=9)repeat.Capture(kick?*kick:input);
        if(fadingOut){gain=fmaxf(0.f,gain-1.f/384.f);if(gain==0.f)Activate();}
        else if(active)gain=fminf(1.f,gain+1.f/576.f);
        if(!active)return input;
        beats+=1.f/(quarter*SR);if(beats>64.f)beats-=32.f;
        unsigned mode=(active-1)%MODE_COUNT;
        float external=input;
        if(mode==HPF_REPEAT && kick)input=*kick;
        bool fast=active>MODE_COUNT;
        float character=Gesture(beats,fast)*targetCharacter;
        // 25 ms smoothing of all externally controllable continuous parameters.
        macro.character+=(character-macro.character)*.000833f;
        macro.motion+=(targetMotion-macro.motion)*.000833f;
        macro.size+=(targetSize-macro.size)*.000833f;
        macro.mix+=(targetMix-macro.mix)*.000833f;macro.quarter=quarter;
        if(controlCounter++%192==0){
            if(mode==HAZARD)hazard.Tune(macro,analysis);
            else if(mode!=HPF_REPEAT)industrial.Tune(mode,macro,beats);
        }
        // Pads 1/9 are implemented at the generated-kick BPF bank in the
        // parent callback. The held engine must leave the external lane dry.
        float wet=mode==HPF_REPEAT?input:mode==MEMORY?memory.Process(input,macro,beats,history):mode==HAZARD?hazard.Process(input,macro,analysis,beats):industrial.Process(mode,input,macro,analysis,beats);
        if(!std::isfinite(wet) || fabsf(wet)>32.f){
            Suspend();return external;
        }
        float ac=wet-dcIn+.9987f*dcOut;dcIn=wet;dcOut=ac;
        // Transparent safety knee on the effect return only, no makeup gain.
        float magnitude=fabsf(ac);
        if(magnitude>.72f)ac=copysignf(.72f+.23f*(magnitude-.72f)/(.23f+magnitude-.72f),ac);
        float mix=gain*macro.mix;
        if(mode==TOY || mode==MEMORY || mode==BINARY || mode==DECAY)
            mix*=1.f-analysis.Protect();
        lastDelta=(ac-input)*mix;
        if(mode==HPF_REPEAT)return external;
        return input+lastDelta;
    }
};
}
