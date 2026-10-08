#include "../dsp/Engine.h"
#include <algorithm>
#include <cassert>
#include <cstdio>
using namespace synth;
int main(){
    // A small perturbation of an idle low-cutoff filter must decay, not latch.
    AcidVoice idle;idle.cutoff=idle.cutoffState=55;idle.resonance=idle.resState=.96f;
    idle.frequency=idle.target=0;idle.phase=.5f;idle.stage[0]=.001f;
    for(unsigned i=0;i<48000;++i)idle.Tick();
    float residual=0;for(float x:idle.stage)residual=std::max(residual,fabsf(x));
    assert(residual<.00001f);
    float eventStep=0,peak=0;
    for(unsigned corner=0;corner<32;++corner){
        AcidVoice v;v.SetCC(20,corner&1?127:0);v.SetCC(21,corner&2?127:0);
        v.SetCC(25,corner&4?127:0);v.SetCC(26,corner&8?127:0);v.SetCC(23,corner&16?127:0);
        v.NoteOn(45,80);
        for(unsigned i=0;i<48000;++i){
            if(i && i%3000==0){
                AcidVoice uninterrupted=v;
                v.NoteOn(45,(i/3000)%2?127:80);
                float changed=v.Tick(),reference=uninterrupted.Tick();
                eventStep=std::max(eventStep,fabsf(changed-reference));
                assert(std::isfinite(changed));
            }else{
                float x=v.Tick();assert(std::isfinite(x));peak=std::max(peak,fabsf(x));
            }
        }
        v.NoteOff(45);for(unsigned i=0;i<48000;++i)v.Tick();assert(fabsf(v.Tick())<.00001f);
    }
    assert(eventStep<.01f);assert(peak<.98f);
    // Fresh zero-amount effects must be an exact dry path.
    SynthFx fx;Clock clock;
    for(unsigned i=0;i<48000;++i){float x=.7f*sinf(i*.073f);assert(fx.Process(x,110,clock)==x);}
    // CC edges on a sounding voice should not jump straight to new values.
    AcidVoice v;v.NoteOn(45,100);for(unsigned i=0;i<6000;++i)v.Tick();
    AcidVoice reference=v;
    for(unsigned cc=20;cc<=28;++cc)v.SetCC(cc,127);
    float ccStep=fabsf(v.Tick()-reference.Tick());assert(ccStep<.01f);
    printf("Acid: idle residual %.8f, retrigger deviation %.6f, CC deviation %.6f, peak %.4f; 32 corners and zero-FX dry path passed\n",residual,eventStep,ccStep,peak);
}
