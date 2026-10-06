// Production parser/queue and callback under MIDI backlogs and changing FX.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include "../../include/KickShapeModel.h"
#define main firmware_main
#include "../../daisy-kick/midi_oled_monitor.cpp"
#undef main
namespace daisy { uint32_t System::now_ms=0; }
static GPIO_TypeDef gp{};static USART_TypeDef ua{};
GPIO_TypeDef* GPIOC=&gp;USART_TypeDef* USART3=&ua;
int main(){
 {MidiRxQueue q;uint16_t b;
  for(int pass=0;pass<100;++pass){
   for(int i=0;i<1023;++i)assert(q.Push(i&127));
   assert(!q.Push(99));
   for(int i=0;i<1023;++i){assert(q.Pop(b));assert((b&255)==(i&127));assert(bool(b&MidiRxQueue::GAP)==(pass>0&&i==0));}
   assert(!q.Pop(b));
  }assert(q.overflow==100);
 }
 // Continuous ignored channel messages previously wrote beyond midi_data[2].
 for(int status=0x80;status<0xf0;++status){
  ProcessMidiByte(status);
  for(int i=0;i<4096;++i){ProcessMidiByte(0xf8);ProcessMidiByte(i&127);assert(midi_data_count<=1);}
 }
 try{firmware_main();}catch(const daisy::HostAudioStarted&){}
 kick_trigger_pending=false;midi_running_status=0;midi_data_count=0;
 float input[16]={},left[16]={},right[16]={};const float* in[2]={input,input};float* out[2]={left,right};
 auto block=[&](){hw.callback(in,out,16);for(int i=0;i<16;++i){assert(std::isfinite(left[i]));assert(fabsf(left[i])<.986f);assert(left[i]==right[i]);}assert(!audio_panic_pending);};
 // Backlog retains every hit AND its paired full-resolution tail pitch.
 for(int i=0;i<100;++i){for(int b:{0xbe,77,i,0x9e,38,0xf8,64})assert(midi_rx.Push(b));}
 int count=0;
 while(midi_rx.read!=midi_rx.write){ServiceMidi();if(kick_trigger_pending){block();assert(kick_voice.tail_semitones==TailPitchSemitones(count));++count;}else block();}
 assert(count==100);
 // Loss marker clears a partial message, then fresh status resynchronizes.
 midi_rx.Push(0x9e);midi_rx.Push(38);ServiceMidi();
 midi_rx.Discontinuity();midi_rx.Push(100);ServiceMidi();assert(!kick_trigger_pending && midi_data_count==0);
 for(int b:{0x9e,38,64})midi_rx.Push(b);ServiceMidi();assert(kick_trigger_pending);block();
 // Pitch trajectory and hold do not depend on DECAY, across all CC extremes.
 for(int shape:{0,64,127})for(int sweep:{0,64,127})for(int vel:{0,64,127})for(int rate:{0,64,127}){
  KickVoice a,b;a.Trigger(73.41619f,shape/127.f,.5f,sweep/127.f,vel,0,.045f,0,rate,64);
  b.Trigger(73.41619f,shape/127.f,.5f,sweep/127.f,vel,0,2.4f,0,rate,64);
  assert(a.body_hold_samples==b.body_hold_samples);
  for(int i=0;i<16000 && a.active;++i){KickVoiceOut x,y;a.Process(x);b.Process(y);assert(a.instantaneous_hz==b.instantaneous_hz);assert(a.phase==b.phase);}
 }
 for(int v=0;v<128;++v)assert(fabsf(KickSweepTimeMs(v/127.f)-kickdaisy::sweepMs(0,v))<.001f);
 // No parameter-change impulse at fixed input, even at deepest quantization.
 WetBitcrusher crusher;float previous=.371f,max_step=0.f;
 for(int direction=0;direction<2;++direction){bitcrush_target=direction?0.f:1.f;
  for(int i=0;i<24000;++i){float y=crusher.Process(.371f);max_step=fmaxf(max_step,fabsf(y-previous));previous=y;assert(std::isfinite(y));}
 }assert(max_step<.002f);assert(crusher.Process(0.f)==0.f);
 // Static BPF incurs no recurring transcendental coefficient work.
 macro_bpf_bank.Reset();auto updates=macro_bpf_bank.coefficient_updates;
 for(int i=0;i<6000;++i)macro_bpf_bank.Update();
 assert(macro_bpf_bank.coefficient_updates==updates);
 auto cc=[](int key,int value){ProcessMidiByte(0xbe);ProcessMidiByte(key);ProcessMidiByte(value);};
 cc(48,127);cc(54,127);cc(56,127);cc(47,127);cc(40,127);cc(59,127);
 // 30 seconds of wet model/boost/frequency/bit-depth automation and retriggers.
 unsigned random=1234567;float peak=0.f;
 for(int n=0;n<90000;++n){
  random=random*1664525u+1013904223u;
  if(n%48==0){cc(44,(random>>24)&127);cc(45,(random>>16)&127);cc(46,(random>>8)&127);cc(37,random&127);cc(56,(random>>17)&127);}
  if(n%3000==0){cc(50,(n/3000)%2?127:0);cc(49,127);}
  if(n%150==0){cc(77,(random>>21)&127);cc(79,(random>>14)&127);ProcessMidiByte(0x9e);ProcessMidiByte(38);ProcessMidiByte(64);}
  block();for(float x:left)peak=fmaxf(peak,fabsf(x));
 }
 printf("MIDI: 458752 ignored/running-status bytes safe; 100 queued hits intact; overflow resynchronizes.\n");
 printf("Timing independent of DECAY; static BPF cache stable; crush control max step %.7f; 30 s FX stress peak %.6f, no nonfinite/panic.\n",max_step,peak);
}
