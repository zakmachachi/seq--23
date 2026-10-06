#pragma once
#include <cmath>
#include <cstdint>
// Short-delay modulation inspired by Erosion's published principle, not a
// circuit/algorithm clone. No noise is added directly to the audio signal.
struct ErosionFx {
    float buffer[64]={};
    unsigned write=0, control=0;
    uint32_t rng=0x71e20a53u;
    float amount=0, frequency=1000, phase=0, g=.06554346f;
    float ic1=0,ic2=0;
    static float Frequency(float position){return 80.f*std::pow(150.f,position);}
    void Reset(){*this=ErosionFx();}
    float Process(float input,float target,float frequency_position){
        amount+=(target-amount)*.0006942034f; // 30 ms, including bypass
        if(target==0 && amount<1e-7f)amount=0;
        if(++control>=48){
            control=0;
            frequency+=(Frequency(frequency_position)-frequency)*.0327839f;
            g=std::tan(3.14159265359f*frequency/48000.f);
        }
        rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;
        float noise=float(rng>>8)*(2.f/16777215.f)-1.f;
        // Topology-preserving state-variable bandpass, bandwidth increases
        // with macro. Noise filter only: no changing audio-filter poles.
        float k=.4f+1.6f*amount, a=1.f/(1.f+g*(g+k));
        float v1=a*(ic1+g*(noise-ic2));
        float v2=ic2+g*v1;
        ic1=2*v1-ic1;ic2=2*v2-ic2;
        float band=v1/(.5f+std::fabs(v1)); // bounded, no modulation excursions
        float sine=std::sin(6.28318530718f*phase);
        phase+=frequency/48000.f;if(phase>=1)phase-=1;
        float mod=sine+(band-sine)*amount;
        // Delay starts at zero and reaches 0..48 samples. No parallel delayed
        // dry copy and no feedback; linear interpolation cannot exceed the
        // peak of its input samples. Silence stays silent.
        float delay=24.f*amount*amount*(1.f+mod);
        buffer[write]=input;
        int whole=int(delay);float fraction=delay-whole;
        unsigned i=(write+64-unsigned(whole))&63,j=(i+63)&63;
        float output=buffer[i]+(buffer[j]-buffer[i])*fraction;
        write=(write+1)&63;
        return output;
    }
};
