// Same oscillator/effects code as firmware. Isolated note for reference comparison.
#ifdef STUDY_OLD_HEADER
#include STUDY_OLD_HEADER
#else
#include "dsp/Hoover.h"
#endif
#include <cstdio>
#include <cassert>
#include <cmath>
static void U16(FILE* f,unsigned x){fputc(x&255,f);fputc((x>>8)&255,f);}
static void U32(FILE* f,unsigned x){U16(f,x);U16(f,x>>16);}
int main(int argc,char** argv){
    assert(argc==3);
    synth::Hoover voice;synth::Clock clock;
    const uint8_t params[6]={88,94,100,0,0,25};
    for(unsigned i=0;i<6;++i)voice.CC(70+i,params[i]);
    float l,r;for(unsigned i=0;i<9600;++i)voice.Tick(clock,i,l,r);
    FILE* f=fopen(argv[1],"wb");FILE* csv=fopen(argv[2],"w");assert(f&&csv);
    constexpr unsigned frames=100800;
    fwrite("RIFF",1,4,f);U32(f,36+frames*4);fwrite("WAVEfmt ",1,8,f);
    U32(f,16);U16(f,1);U16(f,2);U32(f,48000);U32(f,192000);U16(f,4);U16(f,16);
    fwrite("data",1,4,f);U32(f,frames*4);
    fprintf(csv,"seconds,dco_hz,amp,pwm_offset,delay0_samples\n");
    voice.On(50,100); // D3, matching the reference's roughly 147 Hz harmonic spacing.
    for(unsigned i=0;i<frames;++i){
        if(i==88320)voice.Off(50); // reference's release starts at about 1.84 s
        voice.Tick(clock,9600+i,l,r);
        assert(std::isfinite(l)&&std::isfinite(r)&&fabsf(l)<1&&fabsf(r)<1);
        U16(f,unsigned(int16_t(std::round(l*32767))));U16(f,unsigned(int16_t(std::round(r*32767))));
        if(i%48==0)fprintf(csv,"%.6f,%.5f,%.5f,%.5f,%.5f\n",i/48000.f,voice.step*48000.f*8.f,voice.ampValue,voice.width,voice.tap[0]);
    }
    fclose(f);fclose(csv);
}
