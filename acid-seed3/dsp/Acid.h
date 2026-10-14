#pragma once
#include "Blocks.h"
namespace synth {
struct AcidVoice {
    float phase=0,frequency=110,target=110,glide=0,pitchBendSemitones=0;
    float cutoff=720,resonance=.55f,envMod=.72f,decay=.24f,accent=.75f,wave=.0f,drive=.18f,master=.75f;
    int currentNote=-1;
    float pulse=.5f,env=0,amp=0,filterEnv=0,gate=0,last=0,stage[4]={},decayEnv=0;
    bool filterAttack=false;
    bool decayAttack=false;
    float accentState=0,cutoffState=720,resState=.55f,envModState=.72f;
    float waveState=0,driveState=.18f,masterState=.75f;
    bool modded=false;float modeMix=0,oldStage[4]={},oldLast=0;
    float oldDc=0;
    bool slideNext=false,accentNext=false;
    bool finite(float x){return std::isfinite(x);}
    void Reset(){*this=AcidVoice();}
    void SetCC(uint8_t cc,uint8_t v){
        float x=v/127.f;
        switch(cc){
        case 20: cutoff=55.f*powf(7200.f/55.f,x); break;
        case 21: resonance=.02f+.94f*x; break;
        case 22: envMod=x; break;
        case 23: decay=.07f*powf(1.5f/.07f,x); break;
        case 24: accent=x; break;
        case 25: wave=x; break;
        case 26: drive=2.5f*x*x; break;
        case 27: glide=.008f*powf(.280f/.008f,x); break;
        case 28: master=.15f+.85f*x; break;
        case 29: slideNext=v>=64; break;
        case 30: modded=v>=64; break;
        default: break;
        }
    }
    void NoteOn(uint8_t note,uint8_t velocity){
        currentNote=note;target=440.f*powf(2.f,(int(note)-69+pitchBendSemitones)/12.f);
        bool slide=slideNext;slideNext=false;
        accentNext=velocity>=96;
        // The 303 filter envelope has a fast attack, not a one-sample jump.
        // A jump here was the main source of the sharp Acid click, especially
        // after the delayed return was enabled.
        // Retrigger from the current levels: resetting either envelope while
        // the VCA is open produces a discontinuity on overlapping notes.
        if(!slide){filterAttack=true;decayAttack=true;}
        gate=1; if(!slide)frequency=target;
    }
    void SetPitchBend(uint16_t value){
        const float normalized=(float(value)-8192.f)/8192.f;
        pitchBendSemitones=Clamp(normalized*2.f,-2.f,2.f);
        if(currentNote>=0)
            target=440.f*powf(2.f,(float(currentNote)-69.f+pitchBendSemitones)/12.f);
    }
    void NoteOff(uint8_t note){if(currentNote==note){gate=0;currentNote=-1;}}
    float Tick(){
        // MIDI controls are targets. Slew the audible parameters rather than
        // stepping filter coefficients, waveform or output level at CC edges.
        constexpr float controlSlew=.0026f;
        cutoffState+=controlSlew*(cutoff-cutoffState);
        resState+=controlSlew*(resonance-resState);
        envModState+=controlSlew*(envMod-envModState);
        waveState+=controlSlew*(wave-waveState);
        driveState+=controlSlew*(drive-driveState);
        masterState+=controlSlew*(master-masterState);
        accentState+=.015f*((accentNext?1.f:0.f)-accentState);
        // Portamento is an exponential control glide; phase is intentionally
        // free-running across ordinary notes and only frequency changes.
        float glideTime=glide>0?glide:.008f;
        frequency += (target-frequency)*(1.f-expf(-1.f/(SR*glideTime)));
        float envRate=gate ? (1.f-expf(-1.f/(SR*.0007f))) : (1.f-expf(-1.f/(SR*.012f)));
        env += ((gate?1.f:0.f)-env)*envRate;
        float decayRate=1.f-expf(-1.f/(SR*decay));
        if(filterAttack){
            filterEnv += (1.f-filterEnv)*(1.f-expf(-1.f/(SR*.0009f)));
            if(filterEnv>.999f){filterEnv=1;filterAttack=false;}
        }else filterEnv *= (1.f-decayRate);
        if(decayAttack){
            decayEnv+=(1.f-decayEnv)*.029326f; // 0.7 ms attack
            if(decayEnv>.999f){decayEnv=1;decayAttack=false;}
        }else decayEnv*=1.f-decayRate;
        float p=phase;phase+=frequency/SR;if(phase>=1.f)phase-=floorf(phase);
        float dt=frequency/SR;float saw=2.f*p-1.f-Blep(p,dt);float square=(p<pulse?1.f:-1.f)+Blep(p,dt)-Blep(Wrap(p-pulse),dt);
        float osc=saw*(1.f-waveState)+square*waveState;
        float a=accentState;
        float v=osc*(.72f+.28f*a)*(1.f+.2f*driveState);
        float fc=Clamp(cutoffState*(1.f+envModState*(.15f+.85f*filterEnv)*(1.f+.8f*a)),35.f,9000.f);
        float g=1.f-expf(-(2*PI)*fc/SR);
        // Bound the resonant loop just below unity. The previous >1.9x
        // feedback at full resonance/accent could settle into a sample-rate
        // limit cycle, especially while the envelope opened the filter.
        float feedback=last*(resState*(.98f+.03f*a));
        float input=tanhf((v-feedback)*(1.f+3.f*driveState));
        // Saturate the stage input, not its stored integrator value. Repeated
        // gain on the state created an unintended positive-feedback latch,
        // strongest at low cutoff, followed by abrupt polarity switches.
        for(int i=0;i<4;++i){stage[i]+=g*(tanhf(input)-stage[i]);input=stage[i];}
        last=input;
        // Original nonlinear state-feedback ladder: deliberately available
        // for its aggressive latch/break-up character. Run both filters so
        // mode changes can crossfade without resetting their memories.
        float oldInput=tanhf((v-oldLast*(resState*(.98f+.03f*a)))*(1.f+3.f*driveState));
        for(int i=0;i<4;++i){
            oldStage[i]+=g*(oldInput-oldStage[i]);
            oldStage[i]=tanhf(oldStage[i]*(1.f+.18f*resState));
            oldInput=oldStage[i];
        }
        oldLast=oldInput;
        float oldOutput=tanhf(oldInput*(1.f+1.8f*driveState));
        oldDc+=.0019616f*(oldOutput-oldDc); // 15 Hz DC removal before VCA
        modeMix+=.00208f*((modded?1.f:0.f)-modeMix);
        float vca=env*(.22f+.78f*decayEnv)*(1.f+.55f*a*accent);
        amp+=.020619f*(vca-amp); // 1 ms VCA smoothing, including accent depth
        float cleanOutput=tanhf(input*(1.f+1.8f*driveState));
        float out=(cleanOutput+(oldOutput-oldDc-cleanOutput)*modeMix)*amp*masterState*.72f;
        if(!finite(out)){Reset();return 0;}
        return out;
    }
};

}
