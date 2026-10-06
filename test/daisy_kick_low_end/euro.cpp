#include <cassert>
#include <cstdio>
#define main firmware_main
#include "../../daisy-kick/midi_oled_monitor.cpp"
#undef main
namespace daisy { uint32_t System::now_ms=0; }
static GPIO_TypeDef gp{};static USART_TypeDef ua{};
GPIO_TypeDef* GPIOC=&gp;USART_TypeDef* USART3=&ua;
static void Tick(){float in0[16]={},out0[16],out1[16];const float* in[2]={in0,in0};float* out[2]={out0,out1};hw.callback(in,out,16);}
static void Note(int status,int vel){ProcessMidiByte(status);ProcessMidiByte(38);ProcessMidiByte(vel);}
int main(){
 try{firmware_main();}catch(const daisy::HostAudioStarted&){}
 assert(euro_kick_gpio.pin.index==2 && euro_clock_gpio.pin.index==3);
 assert(!euro_kick_gpio.Read()&&!euro_clock_gpio.Read());
 Note(0x9e,64);Tick();assert(euro_kick_gpio.Read());
 for(int i=1;i<15;++i){Tick();assert(euro_kick_gpio.Read());}
 Tick();assert(!euro_kick_gpio.Read()); // exactly 5 ms
 for(int i=0;i<10;++i)Tick();
 unsigned kicks=euro_kick_gpio.rises;
 Note(0x9e,0);Note(0x8e,64);Note(0x90,64);Tick();assert(euro_kick_gpio.rises==kicks);
 // Running status plus interleaved clock, including while transport stopped.
 ProcessMidiByte(0xfc);ProcessMidiByte(0x9e);ProcessMidiByte(38);ProcessMidiByte(0xf8);ProcessMidiByte(64);
 Tick();assert(euro_kick_gpio.Read()&&euro_clock_gpio.Read());
 Tick();Tick();assert(euro_clock_gpio.Read());Tick();assert(!euro_clock_gpio.Read());
 // Distinct queued pulses, with at least 1 ms low between kick pulses.
 ResetEuroOutputs();kicks=euro_kick_gpio.rises;unsigned clocks=euro_clock_gpio.rises;
 for(int i=0;i<100;++i){Note(0x9e,64);ProcessMidiByte(0xf8);}
 unsigned low=48,high=0;bool last=false;
 for(int i=0;i<2000;++i){Tick();bool now=euro_kick_gpio.Read();
  if(now&&!last){assert(low>=48);low=0;}if(!now&&last){assert(high==240);high=0;}
  if(now)high+=16;else low+=16;last=now;}
 assert(euro_kick_gpio.rises-kicks==100);assert(euro_clock_gpio.rises-clocks==100);
 assert(!euro_kick_gpio.Read()&&!euro_clock_gpio.Read());
 assert(euro_kick_pulse.dropped==0 && euro_clock_pulse.dropped==0);
 for(int i=0;i<300;++i)euro_kick_pulse.Request();
 assert(euro_kick_pulse.Pending()==256 && euro_kick_pulse.dropped==44);
 ResetEuroOutputs();assert(euro_kick_pulse.Pending()==0&&!euro_kick_gpio.Read());
 puts("PASS: pin mapping, 5ms trigger / 1ms clock, channel/velocity filtering, running status, stopped clock forwarding, 100 queued edges, minimum gaps, bounded overload and reset");
}
