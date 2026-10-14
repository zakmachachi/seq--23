#pragma once
#include "Acid.h"
#include "Hoover.h"
#include "Screech.h"
#include "OutputStage.h"
#include "SynthFx.h"
#include "Skitter.h"
namespace synth {
struct Engine {
    struct Part {
        AcidVoice acid;
        Hoover hoover;
        Screech screech;
        uint8_t engine=0,fxValue[6]={},skitterMode=0;
        bool mackie=false;
        int8_t note=-1;
        void Panic(){acid.gate=0;acid.currentNote=-1;acid.slideNext=false;hoover.Panic();screech.Panic();note=-1;}
        void Off(uint8_t n){
            if(note!=n)return;
            if(engine==2)screech.Off(n);else if(engine==1)hoover.Off(n);else acid.NoteOff(n);
            note=-1;
        }
        void On(uint8_t n,uint8_t velocity){
            note=n;
            if(engine==2)screech.On(n,velocity);else if(engine==1)hoover.On(n,velocity);else acid.NoteOn(n,velocity);
        }
        float Tick(const Clock& clock,uint32_t samples,float& l,float& r){
            if(engine==2){l=r=screech.Tick();return screech.frequency;}
            if(engine==1){hoover.Tick(clock,samples,l,r);return hoover.frequency;}
            l=r=acid.Tick();return acid.frequency;
        }
    };

    Part parts[softsynth::CHANNEL_COUNT];
    // Keep the first lane's direct voice members available to host-side tools.
    AcidVoice& acid;Hoover& hoover;Screech& screech;
    Clock clock;uint32_t samples=0;
    OutputStage output;KickOutputPump kickPump;SynthFx fx;Skitter skitter;
    uint8_t engine=0,status=0,data=0,count=0,activePart=0;
    uint8_t pendingPart=255,pendingNote=0,pendingVelocity=0;
    uint16_t handoffOut=0,handoffIn=0;
    static constexpr uint16_t HANDOFF_SAMPLES=96; // 2 ms at 48 kHz
    float gain=0;

    Engine():acid(parts[0].acid),hoover(parts[0].hoover),screech(parts[0].screech){
        for(unsigned p=0;p<softsynth::CHANNEL_COUNT;++p){
            for(unsigned i=0;i<6;++i)parts[p].acid.SetCC(20+i,softsynth::ACID_DEFAULTS[i]);
            parts[p].acid.SetCC(26,24);parts[p].acid.SetCC(27,45);parts[p].acid.SetCC(28,96);
        }
    }
    void LoadFx(uint8_t p){
        for(unsigned i=0;i<6;++i)fx.CC(softsynth::FX_BASE+i,parts[p].fxValue[i]);
        fx.SetMackie(parts[p].mackie);skitter.Set(parts[p].skitterMode);
    }
    void Panic(){
        for(unsigned i=0;i<softsynth::CHANNEL_COUNT;++i)parts[i].Panic();
        pendingPart=255;handoffOut=handoffIn=0;
    }
    void Discontinuity(){status=count=0;Panic();}
    int8_t PartIndex(uint8_t channel)const{return softsynth::VoiceIndex(channel);}
    void Midi(uint8_t b){
        if(b>=0xf8){clock.Realtime(b,samples);if(b==0xfc||b==0xff)Panic();return;}
        if(b&128){status=b<0xf0?b:0;count=0;return;}
        if(!status)return;
        uint8_t type=status&0xf0;
        if(type==0xc0||type==0xd0)return;
        if(count++==0){data=b;return;}count=0;
        const uint8_t channel=status&15;
        if(type==0xb0){
            if(channel==softsynth::KICK_CHANNEL){
                if(data==softsynth::KICK_PUMP_AMOUNT)kickPump.SetAmount(b);
                else if(data==softsynth::KICK_FX_ROUTES)kickPump.SetInternalRoute((b&1)!=0);
                return;
            }
            const int8_t index=PartIndex(channel);if(index<0)return;
            Part& part=parts[index];
            if(data==softsynth::ENGINE){
                uint8_t next=b==64?2:(b>=64?1:0);
                if(next!=part.engine){part.Panic();part.engine=next;if(index==activePart)engine=next;}
                return;
            }
            if(data==120||data==123){part.Panic();return;}
            if(data==softsynth::SLIDE){part.acid.SetCC(data,b);part.hoover.CC(data,b);}
            else if(data>=90&&data<=97)part.screech.CC(data,b);
            else if(data>=70&&data<=78)part.hoover.CC(data,b);
            else if(data==softsynth::SKITTER){part.skitterMode=b<=4?b:0;if(index==activePart)skitter.Set(part.skitterMode);}
            else if(data==softsynth::DIST_MODEL){part.mackie=b>=64;if(index==activePart)fx.SetMackie(part.mackie);}
            else if(data>=softsynth::FX_BASE&&data<softsynth::FX_BASE+6){
                part.fxValue[data-softsynth::FX_BASE]=b;if(index==activePart)fx.CC(data,b);
            }
            else part.acid.SetCC(data,b);
        }else if(type==0x90&&b){
            if(channel==softsynth::KICK_CHANNEL){kickPump.Trigger(clock);return;}
            const int8_t index=PartIndex(channel);if(index<0)return;
            if(pendingPart<softsynth::CHANNEL_COUNT){
                if(index==0&&activePart==0){
                    handoffOut=handoffIn=0;
                    parts[0].On(data,b);return;
                }
                // MIDI 12 wins a simultaneous handoff; lower lanes cannot
                // replace it while it is waiting to enter. If the primary
                // lane is currently sounding, retain one lower-priority note
                // for its note-off edge instead of silently dropping it.
                if(pendingPart==0 && index!=0)return;
                if(index==pendingPart){pendingNote=data;pendingVelocity=b;return;}
                if(index==0 || (pendingPart==2 && index==1)){
                    pendingPart=(uint8_t)index;pendingNote=data;pendingVelocity=b;return;
                }
                return;
            }
            if(index!=activePart){
                Part& current=parts[activePart];
                // MIDI 12 is the priority lane for Teensy row 5 and its
                // kick-follow fallback. It may pre-empt lower-priority lanes.
                if(activePart==0 && current.note>=0 && index!=0){
                    pendingPart=(uint8_t)index;pendingNote=data;pendingVelocity=b;
                    return;
                }
                // User lanes do not steal one another mid-note. Their next
                // note is accepted once the current gate has ended.
                if(index!=0 && current.note>=0)return;
                if(current.note>=0)current.Off((uint8_t)current.note);
                pendingPart=(uint8_t)index;pendingNote=data;pendingVelocity=b;
                handoffOut=HANDOFF_SAMPLES;handoffIn=0;
                return;
            }
            parts[activePart].On(data,b);
        }else if(type==0xE0){
            const int8_t index=PartIndex(channel);
            if(index==0)parts[0].acid.SetPitchBend(uint16_t(data)|(uint16_t(b)<<7));
        }else if(type==0x80||type==0x90){
            const int8_t index=PartIndex(channel);
            if(index>=0){
                if(pendingPart==(uint8_t)index&&pendingNote==data){pendingPart=255;handoffOut=0;}
                else {
                    const bool activeGate=index==activePart&&parts[index].note==(int8_t)data;
                    parts[index].Off(data);
                    // Do not interrupt MIDI 12's note. Once its gate closes,
                    // hand the mono voice to the queued user lane smoothly.
                    if(activeGate&&pendingPart<softsynth::CHANNEL_COUNT){
                        handoffOut=HANDOFF_SAMPLES;handoffIn=0;
                    }
                }
            }
        }
    }
    float Tick(){
        float l=0,r=0;Part& current=parts[activePart];
        current.Tick(clock,samples,l,r);float mono=.5f*(l+r);
        if(handoffOut){
            mono*=float(handoffOut)/float(HANDOFF_SAMPLES);
            if(--handoffOut==0&&pendingPart<softsynth::CHANNEL_COUNT){
                activePart=pendingPart;pendingPart=255;engine=parts[activePart].engine;
                LoadFx(activePart);parts[activePart].On(pendingNote,pendingVelocity);
                handoffIn=HANDOFF_SAMPLES;
            }
        }else if(handoffIn){
            mono*=float(HANDOFF_SAMPLES-handoffIn)/float(HANDOFF_SAMPLES);
            --handoffIn;
        }
        gain+=.002f*(1-gain);++samples;
        return mono*gain;
    }
    void TickOutput(float& output1,float& output2){
        float dry=Tick();
        const Part& part=parts[activePart];
        float noteHz=engine==2?part.screech.frequency:engine==1?part.hoover.frequency:part.acid.frequency;
        dry=skitter.Process(dry,clock,samples-1);
        output1=kickPump.Process(output.Tick(fx.Process(dry,noteHz,clock)));output2=0;
    }
};
}
