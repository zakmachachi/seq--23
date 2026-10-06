#include <Arduino.h>
#include <Adafruit_SH110X.h>
#include <cassert>
#include <cstdio>
#include <deque>
#include <utility>
#define private public
#include "KickPerformance.h"
#include "KickShapeModel.h"
#undef private
using K=KickPerformance;
static std::deque<long> draws;
long random(long lo,long hi){if(draws.empty())return lo+std::rand()%(hi-lo);long n=draws.front();draws.pop_front();return lo+n%(hi-lo);}
struct Harness {
 K k; std::vector<std::pair<int,int>> midi; bool ready=true;
 static bool send(void* ctx,uint8_t cc,uint8_t v){auto&h=*static_cast<Harness*>(ctx);if(!h.ready)return false;h.midi.emplace_back(cc,v);return true;}
 Harness(){k.begin(send,this);k.setActive(true);midi.clear();}
 void click(int b,uint32_t t=100){k.buttonEdge(b,true,t);k.buttonEdge(b,false,t+50);}
 void select(K::Parameter p){
   bool fx=p<=K::REVERB||p==K::BITCRUSH||p==K::EROSION||p==K::EROSION_FREQ||p==K::PITCH;
   k.setFxMenu(fx);k.setFunctionHeld(p==K::EROSION_FREQ);
   if(p==K::DELAY||p==K::LOOP||p==K::STUT||p==K::PITCH)k.fx_.repeat=p==K::DELAY?0:p==K::LOOP?1:p==K::STUT?2:3;
   if(p==K::HPF||p==K::LPF)k.fx_.filter=p==K::LPF;
   if(p>=K::BPF1&&p<=K::BPF3)k.state_.editedBpfLayer=p-K::BPF1;
   if(p==K::MACKIE||p==K::TUBE)k.state_.selectedCharacterModel=p==K::TUBE;
 }
 int knob(K::Parameter p){select(p);for(int i=0;i<6;++i)if(k.assignment(i)==p)return i;assert(false);return 0;}
 float anglePosition[6] = {};
 static float angle(float position){
   float wrapped=fmodf(position,128.f);if(wrapped<0)wrapped+=128.f;
   return 3.14159265358979323846f-wrapped*(6.28318530717958647692f/128.f);
 }
 void seed(int knob,float pos){anglePosition[knob]=pos;k.physical_[knob].initialized=false;k.sampleAngle(knob,angle(pos));}
 void turn(int knob,float amount){
   k.sampleAngle(knob,angle(anglePosition[knob])); // Seed after reassignment.
   anglePosition[knob]+=amount;k.sampleAngle(knob,angle(anglePosition[knob]));
 }
 void move(int knob,int value){k.adjust(knob,value-k.position(k.assignment(knob)));}
 void set(int knob,int value){move(knob,value);}
 bool sent(int cc,int v){for(auto&m:midi)if(m==std::make_pair(cc,v))return true;return false;}
};

int main(int argc,char**){
 // Retry a full startup snapshot, including routing and reset-mask CCs.
 {Harness h;h.k=K{};h.ready=false;h.k.begin(Harness::send,&h);assert(h.midi.empty());
 h.ready=true;h.k.service(1);assert(h.midi.size()==27);assert(h.sent(92,14)&&h.sent(90,0)&&h.sent(91,0));
 auto n=h.midi.size();h.k.begin(Harness::send,&h);h.k.setActive(true);h.k.setActive(false);h.k.setActive(true);h.k.service(200);assert(h.midi.size()==n);}
 // Dedicated slots: no duplicate assignments, route click only on release.
 {Harness h;h.k.setFxMenu(true);const K::Parameter expected[]={K::DELAY,K::HPF,K::PUMP,K::REVERB,K::BITCRUSH,K::EROSION};
 for(int k=0;k<6;++k)assert(h.k.assignment(k)==expected[k]);
 h.k.buttonEdge(3,true,100);assert(h.k.fxState().routes==14);h.k.buttonEdge(3,false,150);assert(h.k.fxState().routes==30&&h.sent(92,30));
 Adafruit_SH1106G overview,focus;h.k.drawOverview(overview);h.k.drawFocus(focus,120);
 assert(overview.has("I")&&focus.has("INT"));
 h.click(3);assert(h.k.fxState().routes==12&&h.sent(92,12));
 h.click(3);assert(h.k.fxState().routes==14&&h.sent(92,14));
 h.set(0,100);h.click(0);assert(h.k.assignment(0)==K::LOOP&&h.k.parameterValue(K::DELAY)==100);
 h.set(0,120);h.click(0);assert(h.k.assignment(0)==K::STUT&&h.k.state_.looper.on);
 h.click(0);assert(h.k.assignment(0)==K::PITCH&&h.k.parameterValue(K::PITCH)==64);
 h.set(0,127);assert(h.sent(93,127));h.k.buttonEdge(0,true,1000);h.k.buttonEdge(0,false,2000);
 assert(h.k.parameterValue(K::PITCH)==64&&h.sent(93,64));
 h.click(0);assert(h.k.assignment(0)==K::DELAY&&h.k.parameterValue(K::DELAY)==100);
 h.set(1,80);h.click(1);assert(h.k.assignment(1)==K::LPF&&h.k.parameterValue(K::HPF)==80);
 h.click(1);assert(h.k.assignment(1)==K::HPF);}
 // A hold arms exactly one per-effect reset; release cannot also toggle route.
 {Harness h;h.k.setFxMenu(true);h.k.transport(true,4);h.set(3,111);h.midi.clear();
 h.k.buttonEdge(3,true,1000);h.k.service(1999);assert(!h.k.resetPending(K::REVERB));h.k.service(2000);
 assert(h.k.resetPending(K::REVERB)&&h.k.parameterValue(K::REVERB)==111&&h.sent(90,64));
 assert(h.k.takeResetParameters()==(1ul<<K::REVERB));
 h.k.setParameterValue(K::REVERB,127);assert(h.k.parameterValue(K::REVERB)==111); // lane cannot undo pending reset
 h.k.buttonEdge(3,false,2100);assert(h.k.fxState().routes==14);
 h.k.transport(true,4);assert(h.k.parameterValue(K::REVERB)==111);
 h.midi.clear();h.k.transport(true,5);assert(h.k.parameterValue(K::REVERB)==0&&!h.k.resetPending(K::REVERB)&&h.midi.empty());h.k.service(2200);assert(h.sent(36,0)&&h.sent(90,0));
 // Release can be the first scan past one second; stopped transport is immediate.
 h.k.transport(false,5);h.set(4,100);h.k.buttonEdge(4,true,3000);h.k.buttonEdge(4,false,4000);
 assert(h.k.parameterValue(K::BITCRUSH)==0&&h.sent(37,0)&&h.k.fxState().routes==14);}
 // Multiple pending FX, cancel by moving just one, and retain holds across page exit.
 {Harness h;h.k.setFxMenu(true);h.k.transport(true,8);
 for(int k=0;k<6;++k){h.set(k,90);h.k.buttonEdge(k,true,1000+k*1100);h.k.buttonEdge(k,false,2000+k*1100);}
 assert(h.k.resetMask_==492);h.k.adjust(3,1);assert(!h.k.resetPending(K::REVERB));
 h.k.setFxMenu(false);h.k.transport(true,9);assert(h.k.parameterValue(K::REVERB)==91);
 for(auto p:{K::DELAY,K::HPF,K::PUMP,K::BITCRUSH,K::EROSION})assert(h.k.parameterValue(p)==0);
 h.k.setFxMenu(true);h.k.transport(true,10);h.set(2,87);h.k.requestReset(2);h.k.transport(false,10);h.k.service(10000);assert(h.sent(35,0));}
 // Congested UART at the bar edge: an unsent arm cannot resurrect the effect.
 {Harness h;h.k.setFxMenu(true);h.k.transport(true,20);h.set(3,90);h.midi.clear();h.ready=false;
 h.k.requestReset(3);h.k.service(2000);assert(h.midi.empty());h.k.transport(true,21);
 h.ready=true;h.k.service(2100);assert(h.sent(90,0)&&h.sent(91,0)&&h.sent(36,0)&&!h.sent(90,64));}
 // Function changes only Erosion frequency; no carried ADC movement or BPF mutation.
 {Harness h;h.k.setFxMenu(true);h.set(5,80);h.seed(5,25);h.k.setFunctionHeld(true);h.turn(5,4);
 assert(h.k.erosionFrequency_>64&&h.k.erosion_==80&&h.k.state_.bpfFrequencyValue[0]==35);
 assert(h.k.focusedParameter()==K::EROSION_FREQ);auto f=h.k.erosionFrequency_;h.k.setFunctionHeld(false);h.turn(5,4);
 assert(h.k.erosion_>80&&h.k.erosionFrequency_==f);
 h.k.setFunctionHeld(true);h.k.buttonEdge(5,true,1000);h.k.buttonEdge(5,false,2000);
 assert(h.k.erosion_==0&&h.k.erosionFrequency_==f);}
 // All virtual controls: relative limits, immediate reversal, no duplicate MIDI.
 {Harness h;for(int n=0;n<K::PARAM_COUNT;++n){auto p=(K::Parameter)n;int k=h.knob(p);
 for(int i=0;i<50;++i)h.k.adjust(k,20);assert(h.k.position(p)==127);
 h.midi.clear();h.k.adjust(k,20);assert(h.midi.empty());h.k.adjust(k,-3);assert(h.k.position(p)==124);
 for(int i=0;i<50;++i)h.k.adjust(k,-20);assert(h.k.position(p)==0);
 h.midi.clear();h.k.adjust(k,-20);assert(h.midi.empty());h.k.adjust(k,3);assert(h.k.position(p)==3);}}
 // ADC angle seam, saturation, jitter and leaving page without changing stored values.
 {Harness h;int k=h.knob(K::DELAY);h.set(k,64);h.seed(k,126);h.turn(k,5);assert(h.k.state_.delay>=68&&h.k.state_.delay<=69);
 h.turn(k,-5);assert(h.k.state_.delay>=63&&h.k.state_.delay<=65);
 for(int i=0;i<1000;++i)h.turn(k,4);h.midi.clear();for(int i=0;i<1000;++i)h.turn(k,4);assert(h.midi.empty());
 h.turn(k,.75f);h.turn(k,-3);assert(h.k.state_.delay<127);
 auto amount=h.k.state_.delay;h.k.setActive(false);h.k.sampleAngle(k,Harness::angle(35));h.k.setActive(true);h.seed(k,35);assert(h.k.state_.delay==amount);
 h.midi.clear();for(int i=0;i<1000;++i)h.k.sampleAngle(k,Harness::angle(i%2?35:36));assert(h.midi.empty());}
 // Repeat-rate law: all weighted starts and both directions, exact CC table.
 {const int bins[]={0,55,80,92,98},stut[]={16,48,80,112},loopCC[]={13,38,63,88,114};
 for(int loop=0;loop<2;++loop)for(int start=0;start<(loop?5:1);++start){Harness h;K::RepeatState r;if(loop)draws={bins[start]};h.k.updateRepeat(r,loop,4);h.k.flushMidi();assert(h.sent(loop?31:30,loop?loopCC[start]:stut[start]));
 for(int v=6;v<=127;++v)h.k.updateRepeat(r,loop,v);assert(r.division==(start<=3?(loop?4:3):0));
 for(int v=126;v>=4;--v)h.k.updateRepeat(r,loop,v);assert(r.division==start);h.k.updateRepeat(r,loop,3);assert(!r.on);}}
 // Sound controls remain independent, and K1 is now a navigation hint.
 {Harness h;h.set(4,102);h.click(4);h.set(4,32);h.click(4);assert(h.k.state_.mackieAmount==102&&h.k.state_.tubeAmount==32);
 h.set(3,37);h.click(3);h.click(3);h.set(3,71);h.click(3);h.set(3,99);for(int i=0;i<12;++i)h.click(3);
 assert(h.k.state_.bpfFrequencyValue[0]==37&&h.k.state_.bpfFrequencyValue[1]==71&&h.k.state_.bpfFrequencyValue[2]==99);
 h.click(1);assert(h.k.state_.reverseEnabled);h.click(2);assert(h.k.state_.tailDelayEnabled);assert(h.k.assignment(0)==K::PARAM_COUNT);}
 // Save/restore routing, validate corrupted state, retain inactive effect amounts.
 {Harness h;h.k.setFxMenu(true);h.click(0);h.click(1);h.click(3);auto saved=h.k.fxState();h.k.restoreFx({255,255,255});assert(h.k.fxState().repeat==0&&h.k.fxState().filter==0&&h.k.fxState().routes==31);
 h.k.restoreFx(saved);assert(h.k.assignment(0)==K::LOOP&&h.k.assignment(1)==K::LPF&&h.k.fxState().routes==30);}
 // Every value fits both OLEDs; five-second idle transition and physical focus.
 {Harness h;Adafruit_SH1106G a,b;for(int n=0;n<K::PARAM_COUNT;++n)for(int v=0;v<128;++v){auto p=(K::Parameter)n;int k=h.knob(p);h.k.position(p)=v;h.k.focus_.knob=k;h.k.drawOverview(a);h.k.drawFocus(b,300);}
 h.k.setFxMenu(true);h.k.service(10000);h.k.adjust(5,-3);h.k.render(a,&b,200,10000);
 assert(!h.k.idleOverview(14999)&&h.k.idleOverview(15000));h.k.render(a,&b,200,15000);assert(b.has("E:EXT I:INT +:BOTH"));
 if(argc>1){
   h.k.restoreFx({0,0,14});const int amounts[]={89,32,70,92,50,84};
   for(int i=0;i<6;++i)h.k.position(h.k.fxAssignment(i))=amounts[i];
   h.k.erosionFrequency_=75;h.k.focus_.knob=5;
   h.k.drawFxOverview(a);a.save("fx-overview.svg");h.k.drawFxFocus(b,true);b.save("fx-idle.svg");
   h.k.setFunctionHeld(true);h.k.drawFxFocus(b,false);b.save("fx-erosion-frequency.svg");
 }
 h.k.setActive(false);int frames=a.frames;h.k.render(a,&b,200,16000);assert(a.frames==frames);}
 for(int cc=0;cc<128;++cc)assert(fabsf(K::frequency(K::BPF1,cc)-kickdaisy::bpfHz(cc))<.001f);
 puts("PASS: six-slot FX, routing, bar resets/cancel/stop, FN frequency, recall, repeat rates, relative ADC, all OLED values and five-second overview");
}
