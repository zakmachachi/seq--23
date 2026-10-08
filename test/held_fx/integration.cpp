#include <cassert>
#include <cstdio>
#define main firmware_main
#include "../../daisy-kick/midi_oled_monitor.cpp"
#undef main
namespace daisy { uint32_t System::now_ms=0; }
static GPIO_TypeDef gp{};static USART_TypeDef ua{};
GPIO_TypeDef* GPIOC=&gp;USART_TypeDef* USART3=&ua;
static void Tick(){float input[16]={},a[16],b[16];const float* in[]={input,input};float* out[]={a,b};AudioCallback(in,out,16);for(int i=0;i<16;++i)assert(std::isfinite(a[i])&&a[i]==b[i]);}
static void CC(uint8_t cc,uint8_t v,uint8_t channel=14){ProcessMidiByte(0xb0|channel);ProcessMidiByte(cc);ProcessMidiByte(v);}
int main(){
 try{firmware_main();}catch(const daisy::HostAudioStarted&){}
 uint32_t initial=held_fx_control.request;CC(heldfx::SELECT,1,0);assert(held_fx_control.request==initial);
 ProcessMidiByte(0xfa);CC(heldfx::SELECT,1);Tick();assert(held_fx_engine.pending==1&&!held_fx_engine.active);
 for(int i=0;i<24;++i)ProcessMidiByte(0xf8);Tick();assert(held_fx_engine.active==1);
 CC(heldfx::SELECT,0);for(int i=0;i<40;++i)Tick();assert(!held_fx_engine.active);
 CC(heldfx::SELECT,7);ProcessMidiByte(0x9e);ProcessMidiByte(38);ProcessMidiByte(64);Tick();assert(held_fx_engine.active==7);
 // Existing clock/trigger GPIO forwarding and kick generation still run.
 assert(euro_kick_gpio.Read());assert(kick_voice.active);
 ProcessMidiByte(0xfc);for(int i=0;i<40;++i)Tick();assert(!held_fx_engine.active);
 CC(heldfx::SELECT,16);Tick();assert(held_fx_engine.active==16);
 daisy::System::now_ms=heldfx::LEASE_MS+1;for(int i=0;i<40;++i)Tick();assert(!held_fx_engine.active);
 assert(midi_data_count<=1&&held_fx_engine.faults==0);
 puts("PASS: production MIDI channel, next-clock/kick start, stop/lease release, GPIO triggers and mono callback integration");
}
