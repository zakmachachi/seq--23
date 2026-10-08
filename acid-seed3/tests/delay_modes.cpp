#include "../dsp/Engine.h"
#include <cassert>
#include <cstdio>
#include <algorithm>
using namespace synth;
int main(){
    static float buffer[65536]={};FixedDelayTap tap;float prev=0,maxDelta=0;unsigned write=0;
    for(unsigned i=0;i<120000;++i){
        float requested=i<70000?3000.f:17001.f;
        float y=tap.Process(buffer,65536,write,std::min(i,65536u),requested);
        buffer[write]=.5f*sinf(2*PI*997*i/SR);write=(write+1)&65535;
        if(i>69000&&i<73000)maxDelta=std::max(maxDelta,fabsf(y-prev));prev=y;
        assert(std::isfinite(y));
    }
    assert(maxDelta<.07f);assert(tap.current==17001 && tap.fade==0);
    SynthFx fx;Clock clock;clock.haveTick=true;clock.periodSamples=1000;
    fx.target[3]=fx.amount[3]=1;float onset=0;
    for(unsigned i=0;i<37000;++i){float y=fx.Process(i==0?1.f:0.f,110,clock);if(i>0&&i<36000)assert(fabsf(y)<1e-8f);if(i==36000)onset=y;}
    assert(onset>.1f);
    Engine e;e.acid.SetCC(30,127);e.acid.NoteOn(45,127);float peak=0,transition=0,old=0;
    for(unsigned i=0;i<144000;++i){
        if(i==48000)e.acid.SetCC(30,0);if(i==96000)e.acid.SetCC(30,127);
        if(i%6000==0)e.acid.NoteOn(45+(i/6000)%12, i%12000?80:127);
        float l,r;e.TickOutput(l,r);assert(std::isfinite(l)&&fabsf(l)<=.98f);peak=std::max(peak,fabsf(l));
        if(i==48000||i==96000)transition=std::max(transition,fabsf(l-old));old=l;
    }
    assert(transition<.1f);assert(e.output.safetyClamps==0);
    printf("Delay: 36000-sample dotted-quarter onset %.4f, tempo-change max step %.6f; Acid mode switch step %.6f, output peak %.4f, no safety clamps\n",onset,maxDelta,transition,peak);
}
