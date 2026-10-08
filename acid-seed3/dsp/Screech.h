#pragma once
#include "Blocks.h"
#include "../../include/SoftSynthProtocol.h"
namespace synth {
// Two resettable shots allow a short overlap instead of cutting off the old
// waveform on retrigger. Only the retiring shot runs during the 4 ms crossfade.
struct ScreechShot {
    float p[6]={},carrier=0,modulator=0,submod=0,frequency=110;
    float age=0,length=.4f,envelope=0,velocity=1,gate=1,release=1;
    float glitch=0,glitchTarget=0,mask=1,maskTarget=1,lastStep=-1;
    float sampleHz=110,hzState=110,ratio=2,index=1,fc=1000,drive=1;
    uint32_t rng=0x5a17c9e3;unsigned control=0,style=0;bool active=false;
    SVF formant1,formant2,decimator,decimator2;DC dc;
    void Start(uint8_t note,uint8_t vel,const float* params,unsigned family=0){
        *this=ScreechShot();style=family;for(unsigned i=0;i<6;++i)p[i]=params[i];
        frequency=440.f*powf(2.f,(int(note)-69)/12.f);
        float bipolar=p[1]*127.f-64.f;
        bipolar/=bipolar<0?64.f:63.f;
        hzState=Clamp(frequency*powf(2.f,-bipolar*18.f/12.f),25,5000);
        length=.075f*powf(32.f,p[0]);velocity=.5f+.5f*vel/127.f;
        decimator.Tune(13000,.707f,SR*2);decimator2.Tune(13000,.707f,SR*2);active=true;
    }
    void Off(){gate=0;}
    float Random(){rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;return float(rng>>8)/16777216.f;}
    float Tick(){
        if(!active)return 0;
        const float t=Clamp(age/length,0,1);
        if((control++&15u)==0){
            // Low sweep falls; high sweep rises. Centre is a stationary note.
            float sweep=p[1]*127.f-64.f;
            sweep=36.f*sweep/(sweep<0?64.f:63.f);
            float trajectory=style==3?1.f-expf(-7.f*t):Smooth(t);
            if(style==1)trajectory=floorf(trajectory*(4.f+12.f*p[4]))/(4.f+12.f*p[4]);
            float step=floorf(age*(6.f+58.f*p[4]));
            if(step!=lastStep){
                lastStep=step;glitchTarget=(Random()*2-1)*p[4]*p[4]*18.f;
                maskTarget=Random()<p[4]*.42f?.12f:1.f;
            }
            if(style==2){float cell=Wrap(age*(8.f+56.f*p[4]));maskTarget=cell<.45f?1.f:.08f;}
            sampleHz=Clamp(frequency*powf(2.f,(sweep*(trajectory-.5f)+glitch)/12.f),25,5000);
            // Dense moving FM sidebands, with fixed seeded interval changes.
            ratio=1.f+5.f*p[2]*p[2];
            if(style==1)ratio=1.f+floorf(p[2]*7.f)*.5f;
            index=(.15f+6.f*p[2]*p[2])*(.55f+.45f*sinf(PI*t));
            fc=Clamp(280.f*powf(18.f,p[3])*(.7f+.6f*trajectory),180,7800);
            if(style==4)fc=Clamp(fc*powf(2.f,sinf(2*PI*t)*1.3f),180,7800);
            formant1.Tune(fc,.8f+2.7f*p[3],SR*2);
            formant2.Tune(Clamp(fc*(1.8f-.5f*p[3]),250,11000),1.4f,SR*2);
            drive=1.f+9.f*p[5]*p[5];
        }
        // Smooth stochastic edges; randomness never steps the output directly.
        glitch+=.006f*(glitchTarget-glitch);mask+=.006f*(maskTarget-mask);
        hzState+=.015f*(sampleHz-hzState);
        float y=0;
        for(unsigned os=0;os<2;++os){
            float hz=hzState,modHz=fminf(hz*ratio,12500.f);
            carrier=Wrap(carrier+hz/(SR*2));modulator=Wrap(modulator+modHz/(SR*2));
            submod=Wrap(submod+hz*.5f/(SR*2));
            // Reduce index as pitch rises to limit high-frequency sidebands.
            float safeIndex=fminf(index,18000.f/fmaxf(modHz,1.f));
            float m=sinf(2*PI*modulator+.5f*p[2]*sinf(2*PI*submod));
            float raw=sinf(2*PI*carrier+safeIndex*m);
            if(style==3)raw=sinf(2*PI*carrier+.35f*safeIndex*m);
            if(style==4){float dt=hz/(SR*2);raw=.65f*(2*carrier-1-Blep(carrier,dt))+.35f*raw;}
            if(style==5)raw=.25f*raw+.75f*(2.f*Random()-1.f);
            float vowel=.65f*formant1.Process(raw,true)+.35f*formant2.Process(raw,true);
            float shaped=raw*(.65f-.35f*p[3])+vowel*(.5f+.8f*p[3]);
            shaped=Sat(shaped*drive)/sqrtf(drive);
            y=decimator2.Process(decimator.Process(shaped));
        }
        float attack=Smooth(Clamp(age/.004f,0,1));
        float tail=Smooth(Clamp((length-age)/.018f,0,1));
        if(!gate)release*=.9982654f; // 12 ms exponential note-off, then retire
        envelope=attack*tail*release*expf(-t*(.6f+1.2f*(1-p[0])));
        y=dc.Process(y)*envelope*mask*velocity*.85f;
        age+=1.f/SR;
        if(age>=length||release<.0001f){active=false;return 0;}
        return y;
    }
};
struct Screech {
    float target[6]={},level=.75f,levelState=.75f,frequency=110;
    ScreechShot shot[2];unsigned current=0,fade=0,style=0;int note=-1;
    Screech(){for(unsigned i=0;i<6;++i)target[i]=softsynth::SCREECH_DEFAULTS[i]/127.f;}
    void CC(uint8_t cc,uint8_t value){
        if(cc>=90&&cc<=95)target[cc-90]=value/127.f;
        if(cc==96)level=value/127.f;
        if(cc==97)style=value%6;
    }
    void On(uint8_t n,uint8_t vel){
        current^=1;shot[current].Start(n,vel,target,style);fade=192;note=n;
        frequency=shot[current].frequency;
    }
    void Off(uint8_t n){if(note==n){shot[current].Off();note=-1;}}
    void Panic(){shot[0].Off();shot[1].Off();note=-1;}
    float Tick(){
        float y=shot[current].Tick();
        if(fade){float x=fade/192.f;y+=shot[current^1].Tick()*Smooth(x);--fade;}
        levelState+=.002f*(level-levelState);
        return y*levelState;
    }
};
}
