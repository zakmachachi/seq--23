#pragma once
#include "Acid.h"
#include "Hoover.h"
#include "Screech.h"
#include "OutputStage.h"
#include "SynthFx.h"
#include "Skitter.h"
namespace synth {
struct Engine {
    AcidVoice acid;Hoover hoover;Screech screech;Clock clock;uint32_t samples=0;
    OutputStage output;
    SynthFx fx;Skitter skitter;
    uint8_t engine=0,status=0,data=0,count=0;float gain=0;
    Engine(){for(unsigned i=0;i<6;++i)acid.SetCC(20+i,softsynth::ACID_DEFAULTS[i]);acid.SetCC(26,24);acid.SetCC(27,45);acid.SetCC(28,96);}
    void Panic(){acid.gate=0;acid.currentNote=-1;acid.slideNext=false;hoover.Panic();screech.Panic();}
    void Discontinuity(){status=count=0;Panic();}
    void Midi(uint8_t b){
        if(b>=0xf8){clock.Realtime(b,samples);if(b==0xfc||b==0xff)Panic();return;}
        if(b&128){status=b<0xf0?b:0;count=0;return;}
        if(!status)return;
        uint8_t type=status&0xf0;
        if(type==0xc0||type==0xd0)return;
        if(count++==0){data=b;return;}count=0;
        if((status&15)!=softsynth::CHANNEL)return;
        if(type==0xb0){
            if(data==softsynth::ENGINE){uint8_t next=b==64?2:(b>=64?1:0);if(next!=engine){Panic();engine=next;gain=0;}return;}
            if(data==120||data==123){Panic();return;}
            if(data==softsynth::SLIDE){acid.SetCC(data,b);hoover.CC(data,b);}
            else if(data>=90&&data<=97)screech.CC(data,b);
            else if(data>=70&&data<=78)hoover.CC(data,b);
            else if(data==softsynth::SKITTER)skitter.Set(b);
            else if(data==softsynth::DIST_MODEL)fx.SetMackie(b>=64);
            else if(data>=softsynth::FX_BASE&&data<softsynth::FX_BASE+6)fx.CC(data,b);
            else acid.SetCC(data,b);
        }else if(type==0x90&&b){if(engine==2)screech.On(data,b);else if(engine==1)hoover.On(data,b);else acid.NoteOn(data,b);}
        else if(type==0x80||type==0x90){if(engine==2)screech.Off(data);else if(engine==1)hoover.Off(data);else acid.NoteOff(data);}
    }
    void Tick(float& l,float& r){
        if(engine==2)l=r=screech.Tick();else if(engine==1)hoover.Tick(clock,samples,l,r);else l=r=acid.Tick();
        gain+=.002f*(1-gain);l*=gain;r*=gain;++samples;
    }
    void TickOutput(float& output1,float& output2){
        float l,r;Tick(l,r);
        // Preserve both ensemble/filter branches on the connected mono jack.
        float noteHz=engine==2?screech.frequency:engine==1?hoover.frequency:acid.frequency;
        // Unity voice gain preserves headroom; limiter is a peak safety net only.
        float dry=skitter.Process(.5f*(l+r),clock,samples-1);
        output1=output.Tick(fx.Process(dry,noteHz,clock));output2=0;
    }
};
}
