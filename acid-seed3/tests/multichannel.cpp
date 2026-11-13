#include "../dsp/Engine.h"
#include <cassert>
#include <cmath>
using namespace synth;
static void Message(Engine& e,uint8_t status,uint8_t a,uint8_t b){e.Midi(status);e.Midi(a);e.Midi(b);}
static void CC(Engine& e,uint8_t channel,uint8_t cc,uint8_t value){Message(e,0xB0|channel,cc,value);}
static void On(Engine& e,uint8_t channel,uint8_t note,uint8_t vel){Message(e,0x90|channel,note,vel);}
static void Off(Engine& e,uint8_t channel,uint8_t note){Message(e,0x80|channel,note,0);}
static void AdvanceHandoff(Engine& e){for(unsigned i=0;i<Engine::HANDOFF_SAMPLES;++i){float l,r;e.TickOutput(l,r);}}
int main(){
    Engine e;
    CC(e,softsynth::CHANNEL,20,12);
    CC(e,softsynth::CHANNEL_B,20,112);
    assert(e.parts[0].acid.cutoff!=e.parts[1].acid.cutoff);
    CC(e,softsynth::CHANNEL,119,127);
    CC(e,softsynth::CHANNEL_B,119,127);
    CC(e,softsynth::CHANNEL_C,119,64);
    assert(e.parts[0].engine==1&&e.parts[1].engine==1&&e.parts[2].engine==2);
    CC(e,softsynth::CHANNEL_B,119,64);
    assert(e.parts[1].engine==2); // MIDI channel 13 can select SCREECH.
    CC(e,softsynth::CHANNEL_B,119,127);
    CC(e,softsynth::CHANNEL,119,0); // editable row 5 can return to Acid
    assert(e.parts[0].engine==0);
    // MIDI-12 voice accepts fine pitch bend (±2 semitones).
    Message(e,0xE0|softsynth::CHANNEL,0,96); // +1 semitone
    On(e,softsynth::CHANNEL,60,88);
    assert(fabsf(e.parts[0].acid.target-440.f*powf(2.f,(-8.f)/12.f))<.01f);
    Off(e,softsynth::CHANNEL,60);
    CC(e,softsynth::CHANNEL,80,20);
    CC(e,softsynth::CHANNEL_B,80,100);
    assert(e.parts[0].fxValue[0]==20&&e.parts[1].fxValue[0]==100);
    On(e,softsynth::CHANNEL,60,100);
    assert(e.activePart==0&&e.parts[0].note==60);
    On(e,softsynth::CHANNEL_B,48,110);
    assert(e.activePart==0&&e.parts[1].note==-1&&e.pendingPart==1); // queued, cannot steal an open gate
    On(e,softsynth::CHANNEL,61,96); // row 5 keeps priority over queued MIDI 13
    assert(e.activePart==0&&e.parts[0].note==61&&e.pendingPart==1);
    Off(e,softsynth::CHANNEL,60);
    assert(e.parts[0].note==61);
    Off(e,softsynth::CHANNEL,61);
    AdvanceHandoff(e);
    assert(e.activePart==1&&e.parts[1].note==48);
    assert(fabsf(e.fx.target[0]-100.f/127.f)<1e-6f);
    Off(e,softsynth::CHANNEL,60); // A's late note-off cannot choke B.
    assert(e.parts[1].note==48);
    On(e,softsynth::CHANNEL_C,45,100); // lower-priority lanes cannot steal
    assert(e.activePart==1&&e.parts[2].note==-1&&e.pendingPart==255);
    Off(e,softsynth::CHANNEL_B,48);
    assert(e.parts[1].note==-1);
    On(e,softsynth::CHANNEL,62,90);
    assert(e.pendingPart==0&&e.activePart==1);
    AdvanceHandoff(e);
    assert(e.activePart==0&&fabsf(e.fx.target[0]-20.f/127.f)<1e-6f);
    On(e,softsynth::CHANNEL_C,45,100); // MIDI-12 lane keeps priority until note-off
    assert(e.activePart==0&&e.parts[2].note==-1);
    Off(e,softsynth::CHANNEL,62);
    On(e,softsynth::CHANNEL_C,45,100);
    assert(e.pendingPart==2);
    AdvanceHandoff(e);
    assert(e.activePart==2&&e.engine==2);
    float peak=0,step=0,previous=0;
    for(unsigned i=0;i<12000;++i){float l,r;e.TickOutput(l,r);assert(std::isfinite(l)&&r==0);peak=fmaxf(peak,fabsf(l));step=fmaxf(step,fabsf(l-previous));previous=l;}
    assert(peak<.98f&&step<.25f&&e.output.safetyClamps==0);
    // MIDI channel 15 remains a kick-sidechain input, not a synth part.
    On(e,softsynth::KICK_CHANNEL,36,100);
    assert(e.activePart==2);
}
