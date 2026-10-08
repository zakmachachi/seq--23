#include <cassert>
#include <cstdio>
#include <algorithm>
#define main firmware_main
#include "../midi_oled_monitor.cpp"
#undef main
namespace daisy { uint32_t System::now_ms=0; }
static GPIO_TypeDef gp{};static USART_TypeDef ua{};
GPIO_TypeDef* GPIOC=&gp;USART_TypeDef* USART3=&ua;
void CC(unsigned cc,unsigned v,unsigned channel=14){ProcessMidiByte(0xb0|channel);ProcessMidiByte(cc);ProcessMidiByte(v);}
float Lane(float x){
 x=added_performance_fx.ProcessExternal(x);
 x=external_pitch_fx.Process(x,external_pitch_ratio);
 x=external_reverb.Process(x,param_reverb_amount);
 x=external_bitcrusher.Process(x,bitcrush_target);
 x=external_erosion_fx.Process(x,erosion_amount,erosion_frequency);
 return held_fx_engine.Process(x);
}
int main(){
 try{firmware_main();}catch(const daisy::HostAudioStarted&){}
 // Synth CC traffic must not engage kick-channel effects.
 for(unsigned cc=0;cc<120;++cc)CC(cc,127,13);
 assert(bitcrush_target==0&&erosion_amount==0&&external_pitch_ratio==1&&held_fx_control.request%256==0);
 float worst=0;
 for(float level: {0.00001f,.005f,.1f,.5f})for(bool triggers: {false,true}){
  ResetAudioDspState();param_line_gain=param_line_gain_target=0;
  Biquad a,b;a.Reset();b.Reset();a.SetLowpass(18000,.70710678f);b.SetLowpass(18000,.70710678f);
  for(unsigned n=0;n<48000;n+=2){
   float in1[2],in2[2]={},l[2],r[2],reference[2];const float* in[]={in1,in2};float* out[]={l,r};
   for(int i=0;i<2;++i){in1[i]=level*sinf(2*PI*997*(n+i)/48000.f);reference[i]=b.Process(a.Process(in1[i]*EXTERNAL_RETURN_GAIN*EXTERNAL_OUTPUT_LINEAR_GAIN*MIX_OUTPUT_TRIM*OUTPUT_MAKEUP_GAIN));}
   if(triggers&&n%12000==0){ProcessMidiByte(0x9e);ProcessMidiByte(38);ProcessMidiByte(64);}
   AudioCallback(in,out,2);
   for(int i=0;i<2;++i){assert(std::isfinite(l[i])&&l[i]==r[i]);worst=std::max(worst,fabsf(l[i]-reference[i]));assert(fabsf(l[i]-reference[i])<.00001f);}
  }
 }
 // Exercise each EXT effect, then explicitly zero it with no kick/clock.
 for(unsigned cc: {30u,31u,32u,33u,34u,35u,36u,37u,38u,93u}){
  ResetAudioDspState();CC(cc,100);added_performance_fx.ServiceExternalQuantizedCommands();
  for(unsigned i=0;i<96000;++i){float x=.1f*sinf(i*.13f);assert(std::isfinite(Lane(x)));}
  CC(cc,cc==93?64:0);added_performance_fx.ServiceExternalQuantizedCommands();
  float error=0;
  for(unsigned i=0;i<96000;++i){float x=.1f*sinf(i*.13f),y=Lane(x);assert(std::isfinite(y));if(i>95000)error=std::max(error,fabsf(y-x));}
  printf("CC%u bypass error %.9f\n",cc,error);assert(error<.00001f);
 }
 // Layer all external effects, then release all without a kick trigger.
 ResetAudioDspState();
 for(unsigned cc: {30u,31u,32u,33u,34u,35u,36u,37u,38u,93u})CC(cc,127);
 added_performance_fx.ServiceExternalQuantizedCommands();
 for(unsigned i=0;i<144000;++i){float y=Lane(.25f*sinf(i*.079f)+.1f*sinf(i*.213f));assert(std::isfinite(y)&&fabsf(y)<10.f);}
 for(unsigned cc: {30u,31u,32u,33u,34u,35u,36u,37u,38u,93u})CC(cc,cc==93?64:0);
 added_performance_fx.ServiceExternalQuantizedCommands();
 for(unsigned i=0;i<144000;++i){float x=.1f*sinf(i*.13f),y=Lane(x);assert(std::isfinite(y));if(i>143000)assert(fabsf(y-x)<.00001f);}
 // SELECT includes a generation counter; pad identity uses only its low byte.
 heldfx::Control c;heldfx::Engine& h=held_fx_engine;h.Reset();
 c.Handle(heldfx::SELECT,9,0);h.BeginBlock(c,0,true,true,false,400,16);
 for(unsigned i=0;i<600;++i)h.Process(.1f);assert(h.KickBpfEngaged());
 c.Handle(heldfx::SELECT,0,10);h.BeginBlock(c,10,true,false,false,400,16);assert(h.releasePending);
 h.BeginBlock(c,11,true,false,true,400,16);assert(!h.releasePending);
 for(unsigned i=0;i<600;++i)h.Process(.1f);assert(!h.active&&!h.KickBpfEngaged());
 c.Handle(heldfx::SELECT,1,20);h.BeginBlock(c,20,true,true,false,400,16);assert(h.KickBpfEngaged());
 h.BeginBlock(c,1000,true,false,false,400,16);assert(!h.KickBpfEngaged());
 printf("PASS external path: quiet/loud, kick-independent bypass max error %.9f, synth MIDI isolation, FX off, held generation/lease release\n",worst);
}
