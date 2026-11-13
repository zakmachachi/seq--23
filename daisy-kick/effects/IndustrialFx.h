#pragma once
#include "Blocks.h"
#include "../../include/HeldFxProtocol.h"

namespace heldfx {
// One preallocated state per mode; only the selected mode is processed.
// Control-rate Tune is shared across all effects (250 Hz), never per sample.
struct IndustrialFx {
    Band bands[6];
    Random rng;
    float phase=0,lp=0,hp=0,held=0,clock=0,rate=0,bits=0,drift=0;
    float colour=0,drop=0,dropTarget=0,tailGain=1;
    uint32_t event=0;
    void Reset(unsigned mode){
        for(auto& b:bands)b.Reset();
        rng.Reset();
        phase=lp=hp=held=clock=drift=colour=drop=dropTarget=0;
        rate=48000;bits=16;event=0;tailGain=1;
        (void)mode;
    }
    void Tune(unsigned mode,const Macro& m,float beats){
        float c=m.character,w=sinf(2.f*PI*beats*.25f),motion=m.motion;
        drift+=(rng.Bipolar()-drift)*.035f;
        colour=.1f+.9f*c;
        switch(mode){
        case TOY:
            rate=22000.f*powf(650.f/22000.f,c)*(1.f+motion*(.09f*w+.045f*drift));
            bits=15.f-11.f*c;break;
        case RADIO:
            bands[0].Tune(1000.f+1500.f*c+250.f*motion*w,.6f+2.f*c);
            bits=12.f-7.f*c;break;
        case METAL:{
            const float ratios[]={1.f,1.413f,2.071f,2.713f,3.937f,5.183f};
            float root=700.f*powf(.19f,m.size);
            for(int i=0;i<6;++i)bands[i].Tune(root*ratios[i]*(1.f+.012f*motion*w*(i%2?1:-1)),12.f+210.f*c*c);
            break;}
        default:break;
        }
    }
    float Speaker(float x,float c){
        hp+=(x-hp)*(.014f+.032f*c);float ac=x-hp;
        lp+=(ac-lp)*(.6f-.43f*c);return lp;
    }
    float Process(unsigned mode,float x,const Macro& m,const Analysis& input,float beats){
        float c=m.character,wet=x;
        switch(mode){
        case TOY:{
            clock+=rate/SR;
            if(clock>=1.f){
                clock-=1.f;float y=x;
                if(c>.65f && rng.Uniform()<.035f*(c-.65f)*m.motion)y=rng.Uniform()<.4f?0:held;
                else y=Quantize(y+.014f*c*(x*x),bits);
                held=y;clock+=rng.Bipolar()*.10f*m.motion;
            }
            wet=.35f*x+Speaker(held,c)*.90f;break;}
        case RADIO:{
            float radio=bands[0].Process(x)*Clamp(.14f/(input.fast+.035f),.7f,3.5f);
            if(++event>=unsigned(m.quarter*SR*.0625f)){
                event=0;dropTarget=(c>.55f && rng.Uniform()<.3f*c*m.motion)?1.f:0.f;
            }
            drop+=(dropTarget-drop)*.006f;
            float gate=input.fast/(input.fast+.015f);
            phase=Wrap(phase+(31.f+150.f*c)/SR);
            wet=Quantize(1.25f*Sat(radio*(1.5f+2*c)),bits)*(1.f-.88f*drop);
            wet+=gate*c*m.motion*(rng.Bipolar()*.024f+sinf(phase*2*PI)*.012f);
            break;}
        case METAL:{
            float metal=0,excite=Sat(x*(1.f+3.f*c));
            for(int i=0;i<6;++i)metal+=bands[i].Process(excite);
            wet=.35f*x+1.6f*metal;break;}
        case BINARY:{
            // Signed PCM16 converted through an explicitly unsigned word; no
            // shifts of negative values or implementation-defined signed casts.
            int pcm=int(Clamp(x,-1.f,1.f)*32767.f);uint32_t word=uint32_t(pcm)&65535u;
            unsigned depth=unsigned(2.f+11.f*c),mask=(1u<<depth)-1u;
            uint32_t corrupt=word&~mask;
            unsigned step=unsigned(beats*(4.f+12.f*m.motion));
            if(c>.32f)corrupt^=((step&1)?0x1555u:0x0AAAu)&mask;
            if(c>.72f){unsigned bit=unsigned(9.f+3.f*m.motion);unsigned a=(corrupt>>bit)&1u,b=(corrupt>>(bit-2))&1u;corrupt^=(a^b)*((1u<<bit)|(1u<<(bit-2)));}
            int signedPcm=(corrupt&32768u)?int(corrupt)-65536:int(corrupt);
            float gated=float(signedPcm)/32768.f * input.fast/(input.fast+.0015f);
            float mix=.15f+.60f*Smooth(c);
            wet=(x+(gated-x)*mix)*(.98f-.16f*c*c);break;}
        case DECAY:{
            if(event++%32==0){
                float t=float(input.since)/SR;
                float degrade=c*(1.f-expf(-t/(.012f+.10f*m.size)));
                rate=48000.f*powf(650.f/48000.f,degrade);bits=16.f-12.f*degrade;
                tailGain=1.f-.70f*degrade;
            }
            clock+=rate/SR;if(clock>=1.f){clock-=1.f;held=Quantize(x,bits);}
            wet=held*tailGain;break;}
        default:break;
        }
        return wet;
    }
};

struct MemoryFx {
    struct Head {float base=0,pos=0,length=0,rate=0;};
    Head current,previous;Random rng;float cross=0;
    int lastStep=0;bool initialized=false;
    void Reset(){rng.Reset();cross=1;lastStep=-1;initialized=false;}
    float Read(Head& h,const History& history){
        float value=history.At(h.base+h.pos);
        h.pos+=h.rate;if(h.pos>=h.length)h.pos-=h.length;if(h.pos<0)h.pos+=h.length;
        // Each fragment has a short edge window; wrapping never splices full-level samples.
        float edge=fminf(h.pos,h.length-h.pos)/fminf(96.f,h.length*.25f);
        return value*Smooth(Unit(edge));
    }
    float Process(float /*input*/,const Macro& m,float beats,const History& history){
        int step=int(beats*4.f);
        if((!initialized || step!=lastStep) && cross>=1.f){
            lastStep=step;
            if(!initialized || rng.Uniform()<.20f+.80f*m.character){
                previous=current;
                // Whole clock divisions: repeats lock to sixteenths/eighths,
                // and recalled material comes from the same rhythmic grid.
                float grid=m.quarter*SR*.25f;
                float length=Clamp(grid*(m.character>.70f?1.f:2.f),96.f,28800.f);
                unsigned maxBack=unsigned((History::N-length-128.f)/grid);
                unsigned back=unsigned(rng.Uniform()*float(maxBack+1)*m.character);
                float displacement=32.f+float(back)*grid;
                current={float(history.write)-length-displacement,0,length,
                    m.character>.65f && rng.Uniform()<.45f?-1.f:1.f};
                if(current.rate<0)current.pos=length-1;
                if(!initialized)previous=current;
                cross=0;initialized=true;
            }
        }
        float a=Read(previous,history),b=Read(current,history);
        cross=fminf(1.f,cross+1.f/384.f);
        return a+(b-a)*Smooth(cross);
    }
};
}
