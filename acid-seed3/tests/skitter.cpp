#include "../dsp/Engine.h"
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <vector>
using namespace synth;
int main(){
    float maxDelta=0;
    for(unsigned mode=1;mode<=4;++mode){
        Clock c;Skitter a,b;a.Set(mode);b.Set(mode);c.Realtime(0xfa,0);
        float previous=1,minimum=1,maximum=0;
        std::vector<float> cycle;
        unsigned cycleSamples=(mode==2||mode==4?384:192)*800;
        for(unsigned i=0;i<cycleSamples*3;++i){
            if(i%800==0)c.Realtime(0xf8,i);
            float x=a.Process(1,c,i),y=b.Process(1,c,i);assert(x==y);
            assert(x>=0&&x<=1);maxDelta=std::max(maxDelta,fabsf(x-previous));previous=x;
            if(i>=cycleSamples&&i<2*cycleSamples){cycle.push_back(x);minimum=std::min(minimum,x);maximum=std::max(maximum,x);}
            if(i>=2*cycleSamples)assert(fabsf(x-cycle[i-2*cycleSamples])<.00001f);
        }
        assert(minimum<.01f&&maximum>.98f);
        a.Set(0);for(unsigned i=0;i<20000;++i)a.Process(.5f,c,cycleSamples*3+i);
        assert(a.Process(.5f,c,cycleSamples*3+20000)==.5f);
    }
    assert(maxDelta<.025f);
    // A full modulation cycle closes exactly, and differs from plain triangle.
    assert(Skitter::Hold(0)==Skitter::Hold(1));assert(Skitter::Hold(.25f)>.5f);
    Engine e;e.Midi(0xbd);e.Midi(softsynth::SKITTER);e.Midi(2);assert(e.skitter.mode==2);
    for(unsigned selection: {0u,127u,64u}){
        e.Midi(0xbd);e.Midi(119);e.Midi(selection);assert(e.skitter.mode==2);
    }
    // FX remain downstream: a stored echo survives a zero dry input.
    SynthFx fx;Clock c;c.haveTick=true;c.periodSamples=800;fx.target[3]=fx.amount[3]=1;
    float echo=0;for(unsigned i=0;i<=28800;++i)echo=fx.Process(i==0?1.f:0.f,110,c);
    assert(echo>.1f);
    printf("Skitter: all four timings repeat across cycles, bounded gains, max adjacent gain change %.6f, exact bypass, shared routing and downstream delay passed\n",maxDelta);
}
