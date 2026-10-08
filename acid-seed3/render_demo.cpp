#include "dsp/Engine.h"
#include <cstdio>
#include <cassert>
#include <algorithm>
#include <vector>
using namespace synth;
void CC(Engine& e,int cc,int v){e.Midi(0xbd);e.Midi(cc);e.Midi(v);}
void On(Engine& e,int note,int velocity){e.Midi(0x9d);e.Midi(note);e.Midi(velocity);}
void Off(Engine& e,int note){e.Midi(0x8d);e.Midi(note);e.Midi(0);}
void U16(FILE* f,unsigned x){fputc(x&255,f);fputc((x>>8)&255,f);}
void U32(FILE* f,unsigned x){U16(f,x);U16(f,x>>16);}
void Wav(const char* path,const std::vector<float>& a){
    FILE* f=fopen(path,"wb");assert(f);fwrite("RIFF",1,4,f);U32(f,36+a.size()*2);
    fwrite("WAVEfmt ",1,8,f);U32(f,16);U16(f,1);U16(f,2);U32(f,48000);U32(f,192000);U16(f,4);U16(f,16);
    fwrite("data",1,4,f);U32(f,a.size()*2);for(float x:a)U16(f,unsigned(int16_t(std::round(x*32767))));fclose(f);
}
void Check(){
    Engine outputs;CC(outputs,119,127);On(outputs,50,100);
    for(unsigned i=0;i<48000;++i){float o1,o2;outputs.TickOutput(o1,o2);assert(std::isfinite(o1)&&fabsf(o1)<=.98f&&o2==0);}
    assert(outputs.output.safetyClamps==0);
    Engine e;e.Midi(0x90);e.Midi(60);e.Midi(100);assert(e.acid.currentNote==-1);
    On(e,48,100);CC(e,29,127);On(e,55,100);Off(e,48);assert(e.acid.gate==1);Off(e,55);assert(e.acid.gate==0);
    Clock c;c.Realtime(0xfa,0);for(unsigned i=0;i<384;++i)c.Realtime(0xf8,i*1000);
    assert(c.Phase(4,383000)==0);assert(c.Phase(8,383000)==.5f);
    float freeze=c.Phase(8,400000);assert(c.Phase(8,900000)==freeze);
    c.Realtime(0xfc,900000);float stopped=c.Phase(8,900000);assert(stopped==freeze);assert(c.Phase(8,990000)==stopped);
    c.Realtime(0xfb,990000);assert(c.running);c.Realtime(0xfa,1000000);assert(c.ticks==0);
    float maxPeak=0;
    for(unsigned corner=0;corner<64;++corner){
        Engine v;CC(v,119,127);for(unsigned i=0;i<6;++i)CC(v,70+i,(corner&(1<<i))?127:0);
        On(v,36+corner%36,100);float l,r;
        for(unsigned s=0;s<72000;++s){if(s==48000)Off(v,36+corner%36);v.TickOutput(l,r);assert(std::isfinite(l)&&std::isfinite(r)&&r==0);maxPeak=std::max(maxPeak,std::max(fabsf(l),fabsf(r)));assert(fabsf(l)<1 && fabsf(r)<1);}
        assert(fabsf(l)<.001f&&fabsf(r)<.001f);
        assert(v.output.safetyClamps==0);
    }
    Engine body;CC(body,119,127);for(unsigned s=0;s<48000;++s){float l,r;body.TickOutput(l,r);}On(body,48,110);
    double onset=0,sustain=0;unsigned onsetN=0,sustainN=0;float firstMs=0,lastSample=0,onsetDelta=0;
    for(unsigned s=0;s<33600;++s){
        float l,r;body.TickOutput(l,r);
        if(s>=480&&s<2400){onset+=l*l;++onsetN;}
        if(s>=12000&&s<28800){sustain+=l*l;++sustainN;}
        if(s>=64&&s<112){firstMs=std::max(firstMs,fabsf(l));onsetDelta=std::max(onsetDelta,fabsf(l-lastSample));}
        lastSample=l;
    }
    float bodyDb=10.f*log10f(float((sustain/sustainN)/(onset/onsetN)));
    assert(bodyDb>-6.f&&firstMs<.02f&&onsetDelta<.01f);
    Engine delayed;CC(delayed,83,127);On(delayed,48,100);float previous=0,delayJump=0;
    for(unsigned s=0;s<96000;++s){
        if(s>=24000&&s%1000==0)delayed.Midi(0xf8); // clock arrives after fallback delay
        if(s>0&&s%12000==0){Off(delayed,48);On(delayed,48,100);}
        float l,r;delayed.TickOutput(l,r);delayJump=std::max(delayJump,fabsf(l-previous));previous=l;
    }
    assert(delayJump<.3f);
    printf("routing, slide note-off, clock and 64 Hoover corners passed; peak %.5f; default body/onset %.2f dB; first-ms peak %.5f, max delta %.5f; delayed Acid max delta %.5f\n",maxPeak,bodyDb,firstMs,onsetDelta,delayJump);
}
void Render(bool hoover){
    Engine e;CC(e,119,hoover?127:0);e.Midi(0xfa);
    // Four 4-bar passages, 150 BPM. Last half demonstrates an 8-bar motion cycle.
    constexpr unsigned beat=19200,step=beat/2,bar=beat*4,frames=bar*16+48000;
    const int notes[16]={45,48,51,54,57,54,51,48,43,46,49,52,55,52,49,46};
    std::vector<float> audio;audio.reserve(frames*2);float peak=0;double l2=0,r2=0,m2=0;
    int previous=-1;
    for(unsigned s=0;s<frames;++s){
        if(s<bar*16 && s%(beat/24)==0)e.Midi(0xf8);
        if(s<bar*16 && s%(bar*4)==0){
            unsigned section=s/(bar*4);
            if(hoover){
                const uint8_t presets[4][6]={{48,62,0,0,0,25},{82,92,95,55,64,62},{112,106,117,88,112,108},{120,118,127,100,127,127}};
                for(unsigned i=0;i<6;++i)CC(e,70+i,presets[section][i]);CC(e,76,section>=2?127:64);
            }else{CC(e,20,35+section*22);CC(e,21,45+section*23);CC(e,22,95);CC(e,26,section*32);}
        }
        if(s<bar*16 && s%step==0){
            unsigned index=(s/step)%16;int note=notes[index];bool slide=index%4==1;
            CC(e,29,slide?127:0);if(!slide&&previous>=0)Off(e,previous);
            On(e,note,index%4==0?120:80);if(slide&&previous>=0&&previous!=note)Off(e,previous);previous=note;
        }
        // Every fourth note is a longer tie; others show articulated short release.
        if(s<bar*16 && s%step==step*3/4 && (s/step)%4!=0)Off(e,previous);
        if(s==bar*16){e.Midi(0xfc);Off(e,previous);}
        float l,r;e.TickOutput(l,r);r=l; // Monitor Out 1 in both ears in the demo.
        assert(std::isfinite(l)&&std::isfinite(r));
        peak=std::max(peak,std::max(fabsf(l),fabsf(r)));assert(peak<1);
        audio.push_back(l);audio.push_back(r);l2+=l*l;r2+=r*r;m2+=(l+r)*(l+r)*.25;
    }
    const char* path=hoover?"hoover_demo.wav":"acid_demo.wav";Wav(path,audio);
    assert(e.output.safetyClamps==0);
    printf("%s: %.2fs, peak %.2f dBFS, stereo RMS %.2f dBFS, mono RMS %.2f dBFS (no normalization)\n",path,frames/SR,20*log10(peak),10*log10((l2+r2)/(2*frames)),10*log10(m2/frames));
}
void MovementDemo(){
    Engine e;CC(e,119,127);e.Midi(0xfa);
    // Long articulations expose the contour before the faster performance example.
    const unsigned beat=19200,bar=beat*4,frames=bar*8+48000;
    std::vector<float> audio;audio.reserve(frames*2);int previous=-1;
    for(unsigned s=0;s<frames;++s){
        if(s%(beat/24)==0)e.Midi(0xf8);
        if(s<bar*8 && s%(bar*2)==0){
            unsigned section=s/(bar*2);
            const uint8_t params[4][6]={{88,94,100,0,0,25},{112,105,116,0,0,65},{120,118,123,80,108,105},{127,127,127,103,127,127}};
            for(unsigned i=0;i<6;++i)CC(e,70+i,params[section][i]);
            CC(e,76,section<2?64:127);
        }
        unsigned spacing=s<bar*4?bar:beat;
        if(s<bar*8 && s%spacing==0){
            if(previous>=0)Off(e,previous);
            const int notes[8]={50,53,56,59,50,56,53,47};previous=notes[(s/spacing)%8];
            On(e,previous,110);
        }
        if(s<bar*8 && s%spacing==spacing*7/8)Off(e,previous);
        if(s==bar*8)e.Midi(0xfc);
        float l,r;e.TickOutput(l,r);r=l; // Monitor Out 1 in both ears in the demo.
        assert(std::isfinite(l)&&std::isfinite(r)&&fabsf(l)<1&&fabsf(r)<1);
        audio.push_back(l);audio.push_back(r);
    }
    Wav("hoover_movement_demo.wav",audio);
    printf("hoover_movement_demo.wav: %.2fs, held notes then short aggressive phrases\n",frames/SR);
}
int main(){Check();Render(true);Render(false);MovementDemo();}
