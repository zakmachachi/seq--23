#include "../dsp/Engine.h"
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <vector>
using namespace synth;
void CC(Engine& e,unsigned c,unsigned v){e.Midi(0xbd);e.Midi(c);e.Midi(v);}
void On(Engine& e,unsigned n,unsigned v){e.Midi(0x9d);e.Midi(n);e.Midi(v);}
void Off(Engine& e,unsigned n){e.Midi(0x8d);e.Midi(n);e.Midi(0);}
void U16(FILE* f,unsigned v){fputc(v&255,f);fputc(v>>8&255,f);}
void U32(FILE* f,unsigned v){U16(f,v);U16(f,v>>16);}
int main(int argc,char** argv){
    float peak=0,maxStep=0;
    for(unsigned style=0;style<6;++style)for(unsigned c=0;c<64;++c){
        Screech s;s.CC(97,style);
        for(unsigned i=0;i<6;++i)s.CC(90+i,(c&(1<<i))?127:0);
        s.On(45,110);std::vector<float> first;
        for(unsigned i=0;i<144000;++i){float y=s.Tick();assert(std::isfinite(y));peak=std::max(peak,fabsf(y));if(i<120000)first.push_back(y);}
        s.On(45,110);
        for(unsigned i=0;i<120000;++i)assert(first[i]==s.Tick());
        s.On(127,127);float previous=0;
        for(unsigned i=0;i<24000;++i){if(i==4000)s.Off(127);float y=s.Tick();assert(std::isfinite(y));maxStep=std::max(maxStep,fabsf(y-previous));previous=y;}
        assert(!s.shot[s.current].active);
    }
    Engine e;CC(e,119,64);assert(e.engine==2);
    e.Midi(0x90);e.Midi(48);e.Midi(127);assert(e.screech.note==-1);
    On(e,48,100);Off(e,47);assert(e.screech.note==48);Off(e,48);assert(e.screech.note==-1);
    CC(e,119,127);assert(e.engine==1);CC(e,119,0);assert(e.engine==0);
    printf("6 styles x 64 corners: repeat shots bit-identical, finite, note-off release passed; dry peak %.4f, high-note max step %.4f\n",peak,maxStep);
    if(argc<2)return 0;
    // Six four-second scenes. FX are zero; output level is not normalized.
    const uint8_t preset[6][6]={{72,15,65,64,0,40},{80,112,91,83,0,55},{65,64,115,92,12,70},{76,35,98,76,100,68},{50,96,78,40,127,90},{104,8,102,88,78,74}};
    const unsigned frames=24*48000;
    FILE* f=fopen(argv[1],"wb");assert(f);fwrite("RIFF",1,4,f);U32(f,36+frames*4);fwrite("WAVEfmt ",1,8,f);U32(f,16);U16(f,1);U16(f,2);U32(f,48000);U32(f,192000);U16(f,4);U16(f,16);fwrite("data",1,4,f);U32(f,frames*4);
    CC(e,119,64);float demoPeak=0;
    for(unsigned i=0;i<frames;++i){
        if(i%192000==0){CC(e,97,i/192000);for(unsigned j=0;j<6;++j)CC(e,90+j,preset[i/192000][j]);}
        if(i%48000==0)On(e,45+(i/48000%4)*5,112);
        float l,r;e.TickOutput(l,r);assert(std::isfinite(l)&&fabsf(l)<=.98f&&r==0);demoPeak=std::max(demoPeak,fabsf(l));
        int16_t v=int16_t(l*32767);U16(f,uint16_t(v));U16(f,uint16_t(v));
    }
    fclose(f);assert(e.output.safetyClamps==0);printf("Demo peak %.4f; limiter safety clamps %u\n",demoPeak,e.output.safetyClamps);
}
