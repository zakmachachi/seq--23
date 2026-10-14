#pragma once
#include "IndustrialFx.h"
#include "HazardFx.h"
#include "KickRepeatFx.h"

namespace heldfx {
// MIDI/control writer and audio reader exchange aligned scalar mailboxes only.
// Existing USART implementation is untouched. SELECT carries absolute held state;
// keepalives renew a lease so a missing release/disconnected controller fades dry.
struct Control {
    volatile uint32_t request=0,lastRx=0,layerRequest=0;
    volatile uint8_t character=DEFAULT_CHARACTER,motion=DEFAULT_MOTION,size=DEFAULT_SIZE,mix=DEFAULT_MIX;
    bool Handle(uint8_t cc,uint8_t value,uint32_t now){
        switch(cc){
        case SELECT:lastRx=now;request=((request+256u)&~255u)|(value<=16?value:0);return true;
        case KEEPALIVE:lastRx=now;return true;
        case LAYERS:lastRx=now;layerRequest=((layerRequest+256u)&~255u)|(value&7u);return true;
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
    uint8_t layers=0;
    uint32_t blockedLayers=~0u,blockedSelect=~0u,lastLayerRequest=0;
    bool loopActive=false,ratchetActive=false,ratchetTrigger=false;
    float loopGain=0,ratchetPhase=0,ratchetNext=0,ratchetError=0;
    void UpdateLayers(const Control& control,bool expired,bool stop,bool running,
                      bool kick,bool edge,unsigned frames){
        if(stop || expired){blockedLayers=control.layerRequest;blockedSelect=control.request;}
        if(control.layerRequest!=lastLayerRequest){
            lastLayerRequest=control.layerRequest;
            if(uint8_t(lastLayerRequest)==0 && uint8_t(control.request)==0)suspended=false;
        }
        layers=expired || suspended || control.layerRequest==blockedLayers?0:uint8_t(control.layerRequest);
        if(suspended || control.request==blockedSelect)controlRequest=0;
        const bool loopWanted=(layers&BPF_LOOP) || controlRequest==9;
        if(loopWanted!=loopActive && (expired || stop || !running || kick || edge)){
            loopActive=loopWanted && !expired && !stop;
            if(loopActive)repeat.Start(quarter);
        }
        const bool ratchetWanted=(layers&RATCHET)!=0;
        ratchetTrigger=false;
        if(!ratchetWanted){ratchetActive=false;ratchetPhase=ratchetNext=ratchetError=0;}
        else if(!ratchetActive){
            if(kick || edge || !running){
                ratchetActive=true;ratchetPhase=ratchetError=0;ratchetNext=1;ratchetTrigger=true;
            }
        }else{
            // One bar: quarter, eighth, sixteenth, thirty-second. Independent
            // of the selected external effect; normal coincident hits merge.
            if(edge){ratchetPhase=floorf(ratchetPhase+.5f);ratchetError=0;}
            if(ratchetPhase>=4.f){ratchetPhase-=4.f;ratchetNext-=4.f;}
            if(ratchetPhase+float(frames)*.5f/(quarter*SR)>=ratchetNext){
                ratchetTrigger=true;
                float interval=ratchetNext<1.f?1.f:ratchetNext<2.f?.5f:ratchetNext<3.f?.25f:.125f;
                ratchetNext+=interval;
            }
        }
        if(ratchetActive){
            // Compensated float accumulation avoids drift at small audio blocks.
            float step=float(frames)/(quarter*SR)-ratchetError;
            float nextPhase=ratchetPhase+step;
            ratchetError=(nextPhase-ratchetPhase)-step;ratchetPhase=nextPhase;
        }
    }
    float emergencyOffset=0,lastDelta=0;
    bool emergencyKick=false;
    void Suspend(){
        if((!active && !loopActive && !ratchetActive && !layers) || suspended)return;
        suspended=true;emergencyOffset=lastDelta;emergencyKick=active==1 || active==9;
        pending=next=active=0;gain=0;fadingOut=false;
        loopActive=ratchetActive=ratchetTrigger=false;layers=0;++faults;
    }

    void Reset(){
        history.Reset();industrial.Reset(0);memory.Reset();hazard.Reset();analysis.Reset();
        repeat.Reset();suspended=false;releasePending=false;emergencyOffset=lastDelta=0;emergencyKick=false;
        controlRequest=0;bpfRng.Reset();kickBpfSet=false;
        layers=0;lastLayerRequest=0;blockedLayers=blockedSelect=~0u;loopActive=ratchetActive=ratchetTrigger=false;
        loopGain=ratchetPhase=ratchetNext=ratchetError=0;
        macro={};lastRequest=pendingSamples=controlCounter=0;pending=active=next=0;
        fadingOut=wasRunning=false;gain=beats=dcIn=dcOut=0;quarter=.5f;
        targetCharacter=targetMotion=targetSize=targetMix=0;faults=0;
    }
    void BeginBlock(const Control& control,uint32_t now,bool running,bool kick,bool quarterEdge,float quarterMs,unsigned frames){
        controlRequest=uint8_t(control.request);
        quarter=Clamp(quarterMs*.001f,.12f,1.2f);
        uint32_t command=control.request;
        bool expired=uint32_t(now-control.lastRx)>LEASE_MS;
        const bool stopped=wasRunning && !running;
        if(stopped){pending=next=0;fadingOut=true;lastRequest=command;}
        if(expired)controlRequest=0;
        UpdateLayers(control,expired,stopped,running,kick,quarterEdge,frames);
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
        uint8_t command=(loopActive || (layers&BPF_LOOP))?9:(layers&BPF_HELD)?1:uint8_t(controlRequest);
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
    bool KickBpfEngaged() const { return loopActive || (layers&(BPF_HELD|BPF_LOOP)) || controlRequest==1 || controlRequest==9 || releasePending; }
    void Activate(){
        active=next;next=0;beats=0;controlCounter=0;dcIn=dcOut=0;macro.character=0;
        if(active){
            unsigned mode=(active-1)%MODE_COUNT;
            if(mode==MEMORY)memory.Reset();else if(mode==HAZARD)hazard.Reset();else industrial.Reset(mode);
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
        loopGain=Clamp(loopGain+(loopActive?1.f:-1.f)/384.f,0.f,1.f);
        if(kick && (loopActive || loopGain>0.f)){
            float looped=repeat.Process(*kick);
            *kick+=(looped-*kick)*loopGain;
        }
        if(fadingOut){gain=fmaxf(0.f,gain-1.f/384.f);if(gain==0.f)Activate();}
        else if(active)gain=fminf(1.f,gain+1.f/576.f);
        if(!active)return input;
        beats+=1.f/(quarter*SR);if(beats>64.f)beats-=32.f;
        unsigned mode=(active-1)%MODE_COUNT;
        float external=input;
        if(mode==HPF_REPEAT && kick)input=*kick;
        bool fast=active>MODE_COUNT;
        float character=Gesture(beats,fast)*targetCharacter;
        if(mode==RADIO)character*=.80f;
        if(mode==BINARY)character*=.85f;
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
    // The raw kick path uses only the clocked kick looper. This keeps its
    // capture/crossfade alive without running the unrelated external-pad DSP.
    float ProcessKick(float kick){
        if(suspended)return kick;
        loopGain=Clamp(loopGain+(loopActive?1.f:-1.f)/384.f,0.f,1.f);
        if(loopActive || loopGain>0.f){
            float looped=repeat.Process(kick);
            kick+=(looped-kick)*loopGain;
        }
        return kick;
    }
};
}
