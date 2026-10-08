#pragma once
#include "Blocks.h"
#include "../../include/SoftSynthProtocol.h"
namespace synth {
struct Hoover {
    float target[6]={},p[6]={},phase=0,pwmPhase=0,frequency=110,noteHz=110;
    float delay[4096]={},chorusPhase[3]={0,.31f,.67f},tap[6]={},width=.5f,step=0;
    float bandMix=-1,level=.75f,motion=0,ampValue=0,drive=1;
    float widthTarget=.5f,stepTarget=0,mixSmooth=0,levelSmooth=.75f,velocity=1,velocitySmooth=1;
    unsigned write=0,control=0,period=4;int note=-1;bool slide=false;
    TLEnvelope amp,pitch;SVF core,decimator,body,band[2];DC dc[2];
    Hoover(){for(unsigned i=0;i<6;++i)p[i]=target[i]=softsynth::HOOVER_DEFAULTS[i]/127.f;decimator.Tune(15000,.707f,SR*2);body.Tune(700,.707f,SR*2);}
    void CC(uint8_t cc,uint8_t v){
        if(cc>=70&&cc<=75){target[cc-70]=v/127.f;if(cc==73)bandMix=-1;}
        if(cc==76)period=v<32?0:v<96?4:8;
        if(cc==77)bandMix=v/127.f*.75f;
        if(cc==78)level=v/127.f;
        if(cc==29)slide=v>=64;
    }
    void On(uint8_t n,uint8_t v){note=n;velocity=.6f+.4f*sqrtf(v/127.f);noteHz=440.f*exp2f((int(n)-69)/12.f);if(!slide){amp.On();pitch.On();frequency=noteHz;}slide=false;}
    void Off(uint8_t n){if(note==n){amp.Off();pitch.Off();note=-1;}}
    void Panic(){amp.Off();pitch.Off();note=-1;slide=false;}
    float Read(float samples)const{
        float pos=Wrap((float(write)-samples)/4096.f)*4096.f;
        int i=int(pos);float f=pos-i;
        float a=delay[(i-1)&4095],b=delay[i&4095],c=delay[(i+1)&4095],d=delay[(i+2)&4095];
        return b+.5f*f*(c-a+f*(2*a-5*b+4*c-d+f*(3*(b-c)+d-a)));
    }
    void Tick(const Clock& clock,uint32_t now,float& left,float& right){
        if(control++%32==0){
            for(unsigned i=0;i<6;++i)p[i]+=.0645f*(target[i]-p[i]);
            float h=p[0],s=p[1],c=p[2],b=p[3],m=p[4],z=p[5]*p[5];
            amp.t[0]=.008f-.0065f*h;amp.t[1]=.09f+.22f*h;amp.t[2]=.15f+.5f*h;amp.t[3]=.12f-.096f*h;
            amp.l[0]=1;amp.l[1]=.60f+.35f*h;amp.l[2]=.38f+.48f*h;
            for(unsigned i=0;i<4;++i)pitch.t[i]=amp.t[i];
            pitch.l[0]=4+11*h;pitch.l[1]=-2-5*h;pitch.l[2]=0;
            pwmPhase=Wrap(pwmPhase+32.f/SR*(.35f+5.65f*s*s));
            widthTarget=Clamp(.4f+.1f*s+(.04f+.34f*s)*Tri(pwmPhase),.08f,.92f);
            frequency+=(noteHz-frequency)*.025f;
            stepTarget=Clamp(frequency*exp2f(pitch.value/12.f),20.f,7000.f)/(SR*8.f);
            drive=1+6*z;core.Tune(3800+6200*z,.707f,SR*2);
            static const float bases[6]={9.7f,14.3f,19.1f,12.1f,17.7f,23.9f};
            for(unsigned i=0;i<3;++i)chorusPhase[i]=Wrap(chorusPhase[i]+32.f/SR*(.18f+.37f*c)*(1.f+.13f*i));
            for(unsigned i=0;i<6;++i){float desired=48.f*(bases[i]+(.15f+2.25f*c)*sinf(2*PI*(chorusPhase[i%3]+(i>=3?.37f:0.f))));tap[i]+=.04f*(desired-tap[i]);}
            float dest=period?Tri(clock.Phase(period,now))*2.5f*m*m:0;
            motion+=.0645f*(dest-motion);
            float centre=220.f*powf(4200.f/220.f,b);
            band[0].Tune(centre*exp2f(motion),.7f+3.1f*b*b);
            band[1].Tune(centre*exp2f(-motion),.7f+3.1f*b*b);
        }
        width+=.03f*(widthTarget-width);step+=.03f*(stepTarget-step);
        velocitySmooth+=.004f*(velocity-velocitySmooth);
        ampValue=amp.Tick()*velocitySmooth;pitch.Tick();float out=0;
        for(unsigned os=0;os<2;++os){
            phase=Wrap(phase+step);float ph=Wrap(phase*4),dt=step*4;
            float pulse=(ph<width?1.f:-1.f)+Blep(ph,dt)-Blep(Wrap(ph-width),dt);
            float saw=(ph<width?2*ph/width-1:-1)+(1-width)-Blep(Wrap(ph-width),dt);
            float sub=(phase<.5f?1.f:-1.f)+Blep(phase,step)-Blep(Wrap(phase-.5f),step);
            float x=.46f*saw+.34f*pulse+(.32f-.14f*p[5]*p[5])*sub;
            float low=body.Process(x);x+=.6f*p[5]*p[5]*(x-low);
            x=core.Process(Sat(x*drive))/sqrtf(drive);
            x=(1-p[5]*p[5])*x+p[5]*p[5]*Sat(x*(1+3*p[5]))*.65f;
            out=decimator.Process(x)*ampValue;
        }
        delay[write]=out;
        float wet[2]={};float extra=p[2]*p[2],norm=1+2*extra;
        for(unsigned i=0;i<6;++i)wet[i/3]+=Read(fmaxf(4.f,tap[i]))*(i%3==0?1.f:extra)/norm;
        write=(write+1)&4095;
        float blend=.68f*sqrtf(p[2]),mix=bandMix<0?.58f*Smooth(p[3]):bandMix;
        mixSmooth+=.002f*(mix-mixSmooth);levelSmooth+=.002f*(level-levelSmooth);
        float* result[2]={&left,&right};
        for(unsigned i=0;i<2;++i){float x=out*(1-blend)+wet[i]*blend;x=(x+mixSmooth*band[i].Process(x,true))/(1+.5f*mixSmooth);*result[i]=dc[i].Process(x)*levelSmooth*.65f;}
    }
};
}
