// Production MIDI parser: route mask and bar-scheduled per-effect resets.
#include <cassert>
#include <cmath>
#include <cstdio>
#define main firmware_main
#include "../../daisy-kick/midi_oled_monitor.cpp"
#undef main
namespace daisy { uint32_t System::now_ms=0; }
static GPIO_TypeDef gp{};static USART_TypeDef ua{};
GPIO_TypeDef* GPIOC=&gp;USART_TypeDef* USART3=&ua;
static void CC(int cc,int value){ProcessMidiByte(0xbe);ProcessMidiByte(cc);ProcessMidiByte(value);}
static void Clock(int count){while(count--)ProcessMidiByte(0xf8);}
int main(){
 try{firmware_main();}catch(const daisy::HostAudioStarted&){}
 for(int mask=0;mask<128;++mask){CC(92,mask);assert(fx_internal_routes==((mask&31)|((mask&16)?2:0)));}
 ProcessMidiByte(0xfa);CC(36,110);CC(38,100);CC(32,80);
 CC(90,64);CC(91,2); // reverb and erosion, leaving delay alone
 Clock(95);assert(param_reverb_amount>0 && erosion_amount>0 && fx_bar_reset_mask==320);
 Clock(1);assert(param_reverb_amount==0 && erosion_amount==0 && macro_fx_value_delay>0 && fx_bar_reset_mask==0);
 // Reset phase is transport Start, not a trigger or free-running audio clock.
 CC(36,127);Clock(20);CC(90,64);CC(91,0);ProcessMidiByte(0xfa);
 Clock(95);assert(param_reverb_amount>0);Clock(1);assert(param_reverb_amount==0);
 // Cancellation and stopped transport: clock forwarding cannot commit a bar.
 CC(36,127);CC(90,64);CC(90,0);Clock(96);assert(param_reverb_amount>0);
 CC(90,64);ProcessMidiByte(0xfc);Clock(96);assert(param_reverb_amount>0);
 CC(90,0);CC(36,0);assert(param_reverb_amount==0);
 // Whole reset mask, both bytes, and no collateral change to synthesis.
 const uint8_t cc[]={30,31,32,33,34,35,36,37,38};
 for(auto c:cc)CC(c,100);float shape=macro_kick_shape;
 ProcessMidiByte(0xfa);CC(90,127);CC(91,3);Clock(96);
 assert(macro_fx_value_stutter==0 && macro_fx_value_looper==0 && macro_fx_value_delay==0);
 assert(macro_fx_value_hpf==0 && macro_fx_value_lpf==0 && macro_fx_value_pump==0);
 assert(param_reverb_amount==0 && bitcrush_target==0 && erosion_amount==0 && macro_kick_shape==shape);
 // Reverb histories are separate, including delay/filter state (no kick spill into EXT).
 kick_reverb.Reset();external_reverb.Reset(true);
 for(int n=0;n<96000;++n){
   kick_reverb.Process(n==5000?.5f:0.f,1.f);
   assert(external_reverb.Process(0.f,1.f)==0.f);
 }
 // FX-only route changes settle to exact bypass; finite during rapid toggles.
 float previous=0,maxJump=0;
 for(int n=0;n<48000;++n){
   float input=.1f*sinf(n*TWO_PI*300.f/48000.f);
   float target=(n/2400)%2?1.f:0.f;
   float y=external_erosion_fx.Process(external_bitcrusher.Process(external_reverb.Process(input,target),target),target,.5f);
   assert(std::isfinite(y)&&fabsf(y)<1.f);maxJump=fmaxf(maxJump,fabsf(y-previous));previous=y;
 }
 printf("PASS: MIDI bar masks, cancel/start/stop, isolated tanks, finite switched FX (max sample step %.4f)\n",maxJump);
}
