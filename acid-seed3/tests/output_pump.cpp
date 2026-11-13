#include "../dsp/Engine.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include "../../include/SoftSynthProtocol.h"
using namespace synth;

static void Message(Engine& e,uint8_t status,uint8_t a,uint8_t b){e.Midi(status);e.Midi(a);e.Midi(b);}

int main(){
    Engine dry,pumped;
    // Both engines play the same Acid note. The Kick Seed's PUMP amount and
    // EXT+INT route arrive on MIDI channel 15; a kick note is only a sidechain.
    Message(dry,0x9b,45,100);Message(pumped,0x9b,45,100);
    Message(dry,0xbe,softsynth::KICK_PUMP_AMOUNT,127);
    Message(pumped,0xbe,softsynth::KICK_PUMP_AMOUNT,127);
    Message(dry,0xbe,softsynth::KICK_FX_ROUTES,0);
    Message(pumped,0xbe,softsynth::KICK_FX_ROUTES,1);
    Message(dry,0x9e,36,110);Message(pumped,0x9e,36,110);
    assert(dry.acid.currentNote==45 && pumped.acid.currentNote==45);
    assert(!dry.kickPump.active && pumped.kickPump.active);

    double dryEnergy=0,pumpedEnergy=0;float maxStep=0,previous=0;
    for(unsigned i=0;i<24000;++i){
        float dl,dr,pl,pr;dry.TickOutput(dl,dr);pumped.TickOutput(pl,pr);
        assert(dr==0.f && pr==0.f);
        assert(std::isfinite(dl)&&std::isfinite(pl));
        if(i>4000){dryEnergy+=double(dl)*dl;pumpedEnergy+=double(pl)*pl;}
        maxStep=fmaxf(maxStep,fabsf(pl-previous));previous=pl;
    }
    assert(pumpedEnergy<dryEnergy*.75); // independent output is ducked
    assert(maxStep<.2f); // attack/retrigger envelope is click-safe

    // Turning the internal route off restores unity with a smoothed release.
    Message(pumped,0xbe,softsynth::KICK_FX_ROUTES,0);
    float before=pumped.kickPump.route;
    float l,r;for(unsigned i=0;i<1000;++i)pumped.TickOutput(l,r);
    assert(pumped.kickPump.route<before && pumped.kickPump.route>0.f);
    printf("Acid output pump: kick-channel sidechain, FX-route isolation, smooth duck/release passed (energy ratio %.3f, max step %.4f)\n",
           sqrt(pumpedEnergy/dryEnergy),maxStep);
}
