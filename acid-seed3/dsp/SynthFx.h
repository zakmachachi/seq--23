#pragma once
#include "Blocks.h"
#include "../../include/FixedDelayTap.h"
namespace synth {
struct SynthFx {
    float target[6]={},amount[6]={};
    float bpfMotion=0.f,bpfWet=0.f,bpfCentre=440.f;
    uint8_t delayRate=64;
    uint8_t bpfPeriod=0;
    SVF bpf,bpfUp,bpfDown,hpf,delaySendHp,delayReturnHp,reverbSendHp,reverbReturnHp;
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
    void SetBpfPeriod(uint8_t value){bpfPeriod=value>=8?8:value>=4?4:value>=2?2:value?1:0;}
    void SetDelayRate(uint8_t value){delayRate=value;}
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
    float Process(float input,float trackingHz,const Clock& clock,uint32_t now,bool acidMode){
        for(unsigned i=0;i<6;++i)amount[i]+=.0007f*(target[i]-amount[i]);
        mackieMix+=.0007f*((mackieTarget?1.f:0.f)-mackieMix);
        // Hoover-style dual BPF: opposing filter centres travel around the
        // note's fourth harmonic. Depth controls both sweep span and wet mix.
        float x=amount[0],y=input;
        if(acidMode){
            float phase=bpfPeriod?clock.Phase(bpfPeriod,now):0.f;
            float motion=bpfPeriod?(1.f-4.f*fabsf(phase-.5f))*2.5f*x*x:0.f;
            bpfMotion+=.001f*(motion-bpfMotion);
            bpfWet+=.001f*((bpfPeriod?x:0.f)-bpfWet);
            float centreTarget=Clamp(trackingHz*4.f,220.f,4200.f);
            bpfCentre+=.001f*(centreTarget-bpfCentre);
            float q=.75f+4.75f*x*x;
            bpfUp.Tune(Clamp(bpfCentre*exp2f(bpfMotion),30.f,9500.f),q);
            bpfDown.Tune(Clamp(bpfCentre*exp2f(-bpfMotion),30.f,9500.f),q);
            float band=.5f*(bpfUp.Process(input,true)+bpfDown.Process(input,true));
            // At full depth the resonant mid peak adds 4.62x the dry signal,
            // giving approximately +15 dB at the moving filter centres.
            // The final output limiter catches accumulated peaks downstream.
            const float peakBoost=4.6234f*x;
            y=input+bpfWet*peakBoost*band;
        }else{
            float fc=120.f*powf(45.f,x);bpf.Tune(fc,.75f+4.5f*x*x);
            y=input+x*.75f*bpf.Process(input,true);
        }
        // Parallel tube-style branch with even asymmetry and level compensation.
        x=amount[1];float driven=y*(1+7*x*x)+.18f*x;float tube=Sat(driven)-.12f*x;
        tubeDc+=.0015f*(tube-tubeDc);tube-=tubeDc;
        float character=tube+(Mackie(y,x)-tube)*mackieMix;
        y=y*(1-.55f*x)+character*(.55f*x)/(1+1.4f*x);
        // Performance HPF. Resonance rises moderately without self oscillation.
        x=amount[2];hpf.Tune(28.f*powf(85.f,x),.707f+1.25f*x*x);
        float low=hpf.Process(y);y+=(y-low-y)*Smooth(x); // exact dry at zero, HP at one
        // Clock division is selected by the back-menu macro. Two stationary
        // taps crossfade to the new period without resampling pitch.
        static const float beats[7]={.25f,.5f,1.f,1.5f,2.f,3.f,4.f};
        unsigned rate=(unsigned(delayRate)*6u+63u)/127u;if(rate>6u)rate=6u;
        x=amount[3];float desiredDelay=clock.haveTick
            ?Clamp(clock.periodSamples*24.f*beats[rate],960.f,65000.f)
            :Clamp(24000.f*(beats[rate]/1.5f),960.f,65000.f);
        float echo=delayTap.Process(delay,65536,delayWrite,delayValid,desiredDelay);
        float delaySend=y-delaySendHp.Process(y);
        float delayReturn=echo-delayReturnHp.Process(echo);
        // Darken each repeat before it re-enters the feedback buffer. This
        // keeps bright nonlinear Acid attacks from building a scratchy top.
        delayLp+=.55f*(delayReturn-delayLp);delayReturn=delayLp;
        delay[delayWrite]=Clamp(delaySend+delayReturn*(.12f+.42f*x),-8.f,8.f);delayWrite=(delayWrite+1)&65535;
        if(delayValid<65536)++delayValid;
        y+=delayReturn*(.50f*Smooth(x));
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
