// Raw probes of production EQ, glue and character; no parallel DSP model.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <complex>
#define main firmware_main
#include FIRMWARE
#undef main
namespace daisy { uint32_t System::now_ms=0; }
static GPIO_TypeDef gp{};static USART_TypeDef ua{};
GPIO_TypeDef* GPIOC=&gp;USART_TypeDef* USART3=&ua;
int main(int argc,char**argv){
 if(argc<3)return 2;
 const char* mode=argv[1];FILE*f=fopen(argv[2],"wb");if(!f)return 3;
 if(!strcmp(mode,"eq")){
#ifndef OLD_MODEL
  for(int cc=0;cc<128;++cc){
   float hz=MacroBpfFrequencyHz(cc/127.f);
   for(float rate:{48000.f,192000.f}){
    Biquad eq;eq.SetPeak(hz,15.f,rate);
    for(float mult:{.25f,.5f,1.f,2.f,4.f}){
     auto z=std::polar(1.,-2.*M_PI*hz*mult/rate);
     auto h=(double(eq.b0)+double(eq.b1)*z+double(eq.b2)*z*z)/(1.+double(eq.a1)*z+double(eq.a2)*z*z);
     fprintf(f,"%d,%g,%g,%g,%g\n",cc,hz,rate,mult,20*log10(abs(h)));
    }
   }
  }
#endif
 } else if(!strcmp(mode,"alias")){
  int count=argc>3?atoi(argv[3]):0;double hz=argc>4?atof(argv[4]):3001.;
  macro_bpf_layer_count_latched=count;param_bpf_gain=1;param_mackie_gain=1;
  macro_bpf_target_hz[0]=330;macro_bpf_target_hz[1]=700;macro_bpf_target_hz[2]=1450;
  macro_bpf_bank.Reset();MacroMackieProcessor m;m.Reset();
  for(int i=0;i<96000;++i){float x=.5*sin(2*M_PI*hz*i/48000.);
#ifdef OLD_MODEL
   x=macro_bpf_bank.ProcessDriveFeed(x);
#endif
   float y=m.Process(x*6.f,1.f);if(i>=48000)fwrite(&y,sizeof(y),1,f);
  }
 } else if(!strcmp(mode,"pitch")){
#ifndef OLD_MODEL
  for(int vel:{0,32,63,64,65,96,127})for(int rate:{0,64,127}){
   KickVoice v;v.Trigger(73.41619f,.5f,.55f,.55f,vel,0,MacroDecaySeconds(.75f),0,rate,64);
   for(int i=0;i<72000;++i){KickVoiceOut o;v.Process(o);
    if(i%48==0)fprintf(f,"%d,%d,%g,%g\n",vel,rate,i/48.f,v.instantaneous_hz);
   }
  }
#endif
 } else if(!strcmp(mode,"glue")){
  MixGlue g;
  for(int i=0;i<96000;++i){
   float x=i<12000?0.f:.8f*sinf(TWO_PI*36.708095f*i/SAMPLE_RATE);
   float out[3]={x,g.Process(x),g.gain};fwrite(out,sizeof(float),3,f);
  }
 }
 fclose(f);
}
