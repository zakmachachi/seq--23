#pragma once
#include "Blocks.h"
#include "../../include/FixedDelayTap.h"
namespace synth {
struct SynthFx {
    float target[6]={},amount[6]={};
    SVF bpf,hpf,delaySendHp,delayReturnHp,reverbSendHp,reverbReturnHp;
    float bpfLp=0,tubeDc=0,mackieDcX[3]={},mackieDcY[3]={};
    bool mackieTarget=false;float mackieMix=0;
    float delay[65536]={};unsigned delayWrite=0,delayValid=0;
    FixedDelayTap delayTap;float delayLp=0;
    float revA[1553]={},revB[1867]={};unsigned revAPos=0,revBPos=0;
    float erosion[128]={};unsigned erosionWrite=0;uint32_t rng=0x48a71f29;float erosionPhase=0;
    SynthFx(){
        delaySendHp.Tune(160.f,.707f);delayReturnHp.Tune(180.f,.707f);
        reverbSendHp.Tune(250.f,.707f);reverbReturnHp.Tune(180.f,.707f);
    }
    void CC(uint8_t cc,uint8_t value){if(cc>=softsynth::FX_BASE&&cc<softsynth::FX_BASE+6)target[cc-softsynth::FX_BASE]=value/127.f;}
    void SetMackie(bool enabled){mackieTarget=enabled;}
    float Mackie(float input,float drive){
        float y=input;
        for(unsigned i=0;i<3;++i){
            float shaped=y+.020f*y*fabsf(y),rail=shaped>=0.f?1.f:.955f;
            float u=shaped/rail;y=rail*u/sqrtf(1.f+u*u);
            float hp=y-mackieDcX[i]+.9987f*mackieDcY[i];
            mackieDcX[i]=y;mackieDcY[i]=hp;y=hp*(1.f+.8f*drive);
        }
        return y/(1.f+1.8f*drive);
    }
    float Process(float input,float trackingHz,const Clock& clock){
        for(unsigned i=0;i<6;++i)amount[i]+=.0007f*(target[i]-amount[i]);
        mackieMix+=.0007f*((mackieTarget?1.f:0.f)-mackieMix);
        // Resonant parallel band-pass: centre sweeps 120 Hz..5.4 kHz.
        float x=amount[0],fc=120.f*powf(45.f,x);bpf.Tune(fc,.75f+4.5f*x*x);
        float band=bpf.Process(input,true);float y=input+x*.75f*band;
        // Parallel tube-style branch with even asymmetry and level compensation.
        x=amount[1];float driven=y*(1+7*x*x)+.18f*x;float tube=Sat(driven)-.12f*x;
        tubeDc+=.0015f*(tube-tubeDc);tube-=tubeDc;
        float character=tube+(Mackie(y,x)-tube)*mackieMix;
        y=y*(1-.55f*x)+character*(.55f*x)/(1+1.4f*x);
        // Performance HPF. Resonance rises moderately without self oscillation.
        x=amount[2];hpf.Tune(28.f*powf(85.f,x),.707f+1.25f*x*x);
        float low=hpf.Process(y);y+=(y-low-y)*Smooth(x); // exact dry at zero, HP at one
        // Match the kick external delay: dotted quarter, 1.5 beats, capped
        // at 840 ms. Two stationary taps crossfade without resampling pitch.
        x=amount[3];float desiredDelay=clock.haveTick?Clamp(clock.periodSamples*36.f,960.f,40320.f):24000.f; // 180 BPM fallback, same as kick
        float echo=delayTap.Process(delay,65536,delayWrite,delayValid,desiredDelay);
        float delaySend=y-delaySendHp.Process(y);
        float delayReturn=echo-delayReturnHp.Process(echo);
        delayLp+=.692136f*(delayReturn-delayLp);delayReturn=delayLp;
        delay[delayWrite]=Clamp(delaySend+delayReturn*(.18f+.55f*x),-8.f,8.f);delayWrite=(delayWrite+1)&65535;
        if(delayValid<65536)++delayValid;
        y+=delayReturn*(.65f*Smooth(x));
        // Compact dual-comb reverb; dark feedback, with dry retained.
        x=amount[4];float ra=revA[revAPos],rb=revB[revBPos];
        float dark=.5f*(ra+rb);bpfLp+=.08f*(dark-bpfLp);
        float reverbSend=y-reverbSendHp.Process(y);
        float reverbReturn=dark-reverbReturnHp.Process(dark);
        revA[revAPos]=Clamp(reverbSend+bpfLp*(.58f+.30f*x),-1.2f,1.2f);
        revB[revBPos]=Clamp(reverbSend+ra*(.51f+.32f*x),-1.2f,1.2f);
        if(++revAPos==1553)revAPos=0;
        if(++revBPos==1867)revBPos=0;
        y+=reverbReturn*(.52f*Smooth(x));
        // Erosion tracks the played oscillator: centre follows its 4th harmonic.
        x=amount[5];rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;
        float noise=float(rng>>8)*(2.f/16777215.f)-1.f;
        float tracked=Clamp(trackingHz*4.f,90.f,9000.f);
        erosionPhase=Wrap(erosionPhase+tracked/SR);
        float modulation=(1-x)*sinf(2*PI*erosionPhase)+x*noise;
        float d=1.f+62.f*x*x*(.5f+.5f*modulation);
        erosion[erosionWrite]=y;int whole=int(d);float frac=d-whole;
        unsigned i=(erosionWrite+128-whole)&127,j=(i+127)&127;
        float crushed=erosion[i]+(erosion[j]-erosion[i])*frac;erosionWrite=(erosionWrite+1)&127;
        return y+(crushed-y)*Smooth(x);
    }
};
}
