#include <Arduino.h>
#include <Adafruit_SH110X.h>
#include <cassert>
#include <cstdio>
#include <deque>
#include <utility>
#define private public
#include "KickPerformance.h"
#undef private
using K=KickPerformance;
static std::deque<long> draws;
long random(long lo,long hi){if(draws.empty())return lo+std::rand()%(hi-lo);long n=draws.front();draws.pop_front();return lo+n%(hi-lo);}
struct Harness {
 K k; std::vector<std::pair<int,int>> midi; bool ready=true;
 static bool send(void* ctx,uint8_t cc,uint8_t v){auto&h=*static_cast<Harness*>(ctx);if(!h.ready)return false;h.midi.emplace_back(cc,v);return true;}
 Harness(){k.begin(send,this);k.setActive(true);midi.clear();}
 void click(int b,uint32_t t=100){k.buttonEdge(b,true,t);k.buttonEdge(b,false,t+50);}
 void page(int p){while(k.state_.selectedFx!=p)click(0);}
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
int main(int argc, char** argv){
 (void)argv;
 // Initialization, backpressure and no repeated sync on page entry/render.
 {Harness h;h.k=K{};h.ready=false;h.k.begin(Harness::send,&h);assert(h.midi.empty());h.ready=true;h.k.service(1);
 const int cc[]={30,31,32,33,34,35,36,40,41,42,43,44,45,46,47,48,49,50,51,52,60,61};
 const int val[]={0,0,0,0,0,0,0,64,0,0,0,35,65,95,0,0,0,0,64,0,64,0};assert(h.midi.size()==22);
 for(int i=0;i<22;++i)assert(h.midi[i]==std::make_pair(cc[i],val[i]));
 h.k.begin(Harness::send,&h);h.k.setActive(true);h.k.setActive(false);h.k.setActive(true);h.k.service(200);assert(h.midi.size()==22);}
 // v1.3.0: Function + K3 edits the tail fine controls, Function + B3 switches
 // between them, and neither touches the TAIL amount or the tail enable.
 {Harness h;h.set(2,40);h.k.takeUserEditedParameter();h.midi.clear();h.k.setFunctionHeld(true);
 h.k.adjust(2,5);assert(h.k.fine_.tailOffset==69&&h.sent(60,69)&&h.k.state_.tailDelayAmount==40&&h.k.fineFocused());
 assert(h.k.takeUserEditedParameter()==K::PARAM_COUNT); // fine edits are never lane-recorded
 h.midi.clear();h.click(2);assert(h.k.fine_.attackMode&&!h.k.state_.tailDelayEnabled&&h.midi.empty());
 h.k.adjust(2,20);assert(h.k.fine_.tailAttack==20&&h.sent(61,20)&&h.k.fine_.tailOffset==69);
 h.k.adjust(2,-100);assert(h.k.fine_.tailAttack==0); // clamps at the limit, no wrap
 Adafruit_SH1106G a,b;h.k.drawOverview(a);h.k.drawFocus(b,120);assert(b.has("TAIL ATTACK"));
 h.k.setFunctionHeld(false);assert(!h.k.fineFocused());
 h.midi.clear();h.k.adjust(2,3);assert(h.k.state_.tailDelayAmount==43&&h.sent(42,43)&&h.k.fine_.tailAttack==0);
 h.click(2);assert(h.k.state_.tailDelayEnabled);
 // Sticky: restoring the provisional snapshot leaves the fine state alone.
 K::ControllerState snap;h.k.saveTo(snap);h.k.setFunctionHeld(true);h.click(2);h.k.adjust(2,-9);
 h.k.restoreFrom(snap);assert(h.k.fine_.tailOffset==60&&h.sent(60,60));
 K::FineState f;f.tailOffset=200;f.tailAttack=90;h.k.restoreFine(f);assert(h.k.fine_.tailOffset==64&&h.k.fine_.tailAttack==90);}
 // Relative FX edits resume stored values immediately; page selection sends no amount.
 {Harness h;h.page(K::DELAY);h.set(0,114);h.midi.clear();h.page(K::HPF);
 assert(h.k.state_.hpf==0&&h.midi.empty());h.k.adjust(0,3);assert(h.k.state_.hpf==3&&h.sent(33,3));
 h.set(0,89);h.page(K::DELAY);assert(h.k.state_.delay==114);h.k.adjust(0,-3);assert(h.k.state_.delay==111);
 h.midi.clear();h.page(K::HPF);assert(h.k.state_.hpf==89&&h.midi.empty());h.k.adjust(0,-2);assert(h.k.state_.hpf==87&&h.sent(33,87));}
 // C: all weighted bins, previous-start avoidance, both rate directions and OFF.
 {int loopBins[]={0,55,80,92,98};
 for(int i=0;i<5;++i){draws={loopBins[i]};assert(K::randomStart(-1)==i);draws={loopBins[i],loopBins[i]};assert(K::randomStart(i)!=i);}
 int histogram[5]={};std::srand(1);for(int i=0;i<10000;++i)++histogram[K::randomStart(-1)];assert(histogram[0]+histogram[1]>7000);
 // Stutter opens at the slowest division every time, with no draw consumed.
 Harness h;h.set(0,4);assert(h.k.state_.stutter.division==0&&h.k.state_.stutter.direction==1);h.move(0,127);assert(h.k.state_.stutter.division==3);h.move(0,4);assert(h.k.state_.stutter.division==0);h.move(0,0);assert(!h.k.state_.stutter.on&&h.sent(30,0));
 h.move(0,4);assert(h.k.state_.stutter.division==0&&h.k.state_.stutter.direction==1);assert(draws.empty());
 // Boundary jitter must not alternate canonical rate values.
 K::RepeatState r;h.k.updateRepeat(r,false,4);h.k.updateRepeat(r,false,27);int rate=r.division;for(int i=0;i<50;++i)h.k.updateRepeat(r,false,i%2?26:27);assert(r.division==rate);}
 // Exhaustive repeat destinations: canonical CCs and travel direction for each start.
 {const int stutCC[]={16,48,80,112},loopCC[]={13,38,63,88,114};
 const int loopBins[]={0,55,80,92,98};
 for(int loop=0;loop<2;++loop)for(int start=0;start<(loop?5:1);++start){
  Harness h;K::RepeatState r;if(loop)draws={loopBins[start]};
  h.k.updateRepeat(r,loop,4);h.k.flushMidi();assert(h.sent(loop?31:30,loop?loopCC[start]:stutCC[start]));
  int prev=start;for(int v=6;v<=127;++v){h.k.updateRepeat(r,loop,v);assert(r.direction>0?r.division>=prev:r.division<=prev);prev=r.division;}
  assert(r.division==(start<=3?(loop?4:3):0));
  for(int v=126;v>=4;--v)h.k.updateRepeat(r,loop,v);assert(r.division==start);
  h.k.updateRepeat(r,loop,3);assert(!r.on);h.k.flushMidi();assert(h.sent(loop?31:30,0));
 }}
 // Re-entering the mode ignores physical movement while away and resumes saved value.
 {Harness h;h.seed(1,111);h.turn(1,4);assert(h.k.state_.decay>64);auto value=h.k.state_.decay;
 h.k.setActive(false);h.k.sampleAngle(1,Harness::angle(35));h.k.setActive(true);h.seed(1,35);h.midi.clear();
 assert(h.k.state_.decay==value&&h.midi.empty());h.turn(1,-4);assert(h.k.state_.decay<value&&h.sent(40,h.k.state_.decay));}
 // D: repeat session survives changing FX page, return retains its relative value.
 {Harness h;h.set(0,4);auto before=h.k.state_.stutter;h.page(K::HPF);h.midi.clear();h.page(K::STUT);assert(h.k.state_.stutter.on&&h.k.state_.stutter.division==before.division&&h.k.state_.stutter.activationPosition==before.activationPosition&&h.midi.empty());}
 // E: separate character memories; model buttons send only absolute CC50.
 {Harness h;h.set(4,102);h.click(4);h.set(4,32);h.midi.clear();h.click(4);assert(h.k.state_.mackieAmount==102&&h.k.state_.tubeAmount==32&&h.midi.size()==1&&h.sent(50,0));h.click(4);assert(h.k.state_.tubeAmount==32&&h.sent(50,127));}
 // F: all BPF memories persist across repeated count cycles, per-layer edits.
 {Harness h;h.set(3,37);h.click(3);h.click(3);h.set(3,71);h.click(3);h.set(3,99);h.midi.clear();for(int i=0;i<12;++i)h.click(3);assert(h.k.state_.bpfFrequencyValue[0]==37&&h.k.state_.bpfFrequencyValue[1]==71&&h.k.state_.bpfFrequencyValue[2]==99);for(auto&m:h.midi)assert(m.first==47);}
 // G / H: absolute toggles, no held repeats, sticky focus/dirty-only max25FPS.
 {Harness h;Adafruit_SH1106G a,b;h.click(1);assert(h.k.state_.reverseEnabled&&h.sent(41,127));h.k.render(a,&b,120,100);assert(a.frames==1&&b.has("DECAY"));
 for(int t=101;t<10000;++t){h.k.service(t);h.k.render(a,&b,120,t);}assert(a.frames==1&&h.k.focus_.knob==1&&h.midi.size()==1);
 h.click(1,10010);assert(!h.k.state_.reverseEnabled&&h.sent(41,0));h.k.render(a,&b,120,10020);assert(a.frames==2);
 h.click(5,10021);h.k.render(a,&b,120,10025);assert(a.frames==2);h.k.render(a,&b,120,10060);assert(a.frames==3&&b.has("SHAPE"));
 auto count=h.midi.size();h.k.dirty_=false;
 for(int knob=0;knob<6;++knob)h.seed(knob,64);
 for(int i=0;i<1000;++i)for(int knob=0;knob<6;++knob)h.k.sampleAngle(knob,Harness::angle(i%2?64:65));
 assert(h.k.focus_.knob==5&&h.midi.size()==count&&!h.k.dirty_);
 h.seed(0,0);
 for(int i=0;i<1000;++i)h.k.sampleAngle(0,Harness::angle(i%2?.4f:-.4f));
 assert(h.k.focus_.knob==5&&h.midi.size()==count&&!h.k.dirty_);}
 // Real sampled angles: arbitrary starting positions, both seams, no pickup,
 // repeated over-travel, no MIDI at saturation, and immediate reversal.
 {Harness h;h.page(K::DELAY);h.k.state_.delay=64;h.seed(0,126);h.midi.clear();
 h.turn(0,5);assert(h.k.state_.delay>=68&&h.k.state_.delay<=69); // crosses clockwise seam
 h.turn(0,-5);assert(h.k.state_.delay>=63&&h.k.state_.delay<=65); // crosses back
 for(int i=0;i<1000;++i)h.turn(0,4);assert(h.k.state_.delay==127);
 h.midi.clear();for(int i=0;i<1000;++i)h.turn(0,4);assert(h.k.state_.delay==127&&h.midi.empty());
 h.turn(0,.75f);h.turn(0,-3);assert(h.k.state_.delay<127&&h.sent(32,h.k.state_.delay));
 for(int i=0;i<1000;++i)h.turn(0,-4);assert(h.k.state_.delay==0);
 h.midi.clear();for(int i=0;i<1000;++i)h.turn(0,-4);assert(h.k.state_.delay==0&&h.midi.empty());
 h.turn(0,-.75f);h.turn(0,3);assert(h.k.state_.delay>0&&h.sent(32,h.k.state_.delay));
 // Page change discards residual movement and binds to the current angle.
 h.turn(0,.5f);h.page(K::HPF);h.k.state_.hpf=90;h.midi.clear();h.k.sampleAngle(0,Harness::angle(40));
 assert(h.k.state_.hpf==90&&h.midi.empty());h.seed(0,40);h.turn(0,-4);assert(h.k.state_.hpf<90);
 // Movement while B1 is held cannot leak into the newly selected FX.
 h.k.buttonEdge(0,true,100);h.turn(0,20);h.k.buttonEdge(0,false,200);h.midi.clear();
 h.k.sampleAngle(0,Harness::angle(h.anglePosition[0]));assert(h.midi.empty());}
 // Every virtual parameter clamps, emits no duplicate at the limit, and reverses.
 {Harness h;for(int p=0;p<K::PARAM_COUNT;++p){
 int knob=p<=K::REVERB?0:p==K::DECAY?1:p==K::TAIL?2:p<=K::BPF3?3:p<=K::TUBE?4:5;
 h.k.state_.selectedFx=p<=K::REVERB?p:0;h.k.state_.editedBpfLayer=p>=K::BPF1&&p<=K::BPF3?p-K::BPF1:0;h.k.state_.selectedCharacterModel=p==K::TUBE;
 for(int i=0;i<50;++i)h.k.adjust(knob,20);assert(h.k.position((K::Parameter)p)==127);
 h.midi.clear();h.k.adjust(knob,20);assert(h.midi.empty());h.k.adjust(knob,-3);assert(h.k.position((K::Parameter)p)==124);
 for(int i=0;i<50;++i)h.k.adjust(knob,-20);assert(h.k.position((K::Parameter)p)==0);
 h.midi.clear();h.k.adjust(knob,-20);assert(h.midi.empty());h.k.adjust(knob,3);assert(h.k.position((K::Parameter)p)==3);
 }}
 // I: single reset at threshold, correct six CCs, page/pump/every other value preserved.
 {const int resetCC[]={30,31,32,33,34,36};
 Harness h;h.page(K::PUMP);h.set(0,89);h.set(1,83);h.click(1);h.click(2);h.click(3);h.click(4);h.click(5);h.k.state_.delay=55;h.k.state_.hpf=66;h.k.state_.lpf=77;h.k.state_.reverb=99;h.midi.clear();h.k.buttonEdge(0,true,1000);h.k.service(1499);assert(h.midi.empty());h.k.service(1500);assert(h.midi.size()==6);h.k.service(1900);h.k.buttonEdge(0,false,2000);assert(h.midi.size()==6&&h.k.state_.selectedFx==K::PUMP&&h.k.state_.pumpAmount==89&&h.k.state_.reverb==0&&h.k.state_.decay==83&&h.k.state_.reverseEnabled&&h.k.state_.tailDelayEnabled&&h.k.state_.bpfLayerCount==1&&h.k.state_.selectedCharacterModel&&h.k.state_.pumpEnabled);for(int i=0;i<6;++i)assert(h.midi[i]==std::make_pair(resetCC[i],0));assert(h.k.focus_.resetOverlay);h.k.service(2200);assert(!h.k.focus_.resetOverlay&&h.k.focus_.knob==0);
 // The release itself may be first service past threshold.
 h.midi.clear();h.k.buttonEdge(0,true,3000);h.k.buttonEdge(0,false,3500);assert(h.midi.size()==6&&h.k.state_.selectedFx==K::PUMP);}
 // J: enable/disable preserves amount.
 {Harness h;h.page(K::PUMP);h.set(0,89);h.midi.clear();for(int i=0;i<3;++i)h.click(5);assert(h.k.state_.pumpAmount==89&&h.k.state_.pumpEnabled&&h.midi.size()==3);for(auto&m:h.midi)assert(m.first==52);}
 // Every parameter at every possible value fits on the panels; draws never send MIDI.
 {Harness h;Adafruit_SH1106G a,b;for(int p=0;p<K::PARAM_COUNT;++p)for(int v=0;v<128;++v){int knob=p<=K::REVERB?0:p==K::DECAY?1:p==K::TAIL?2:p<=K::BPF3?3:p<=K::TUBE?4:5;h.k.state_.selectedFx=p<=K::REVERB?p:0;h.k.state_.editedBpfLayer=p>=K::BPF1&&p<=K::BPF3?p-K::BPF1:0;h.k.state_.selectedCharacterModel=p==K::TUBE;h.k.position((K::Parameter)p)=v;h.k.focus_.knob=knob;h.k.drawOverview(a);h.k.drawFocus(b,300);assert(!b.has("PICK"));}assert(h.midi.empty());
 h.k.state_.selectedFx=K::STUT;h.k.state_.stutter.on=true;h.k.state_.stutter.division=3;h.k.state_.stutter.randomStart=0;h.k.state_.stutter.direction=-1;h.k.focus_.knob=0;h.k.drawOverview(a);h.k.drawFocus(b,120);if(argc>1){a.save("overview.svg");b.save("stutter.svg");}
 h.k.focus_.knob=3;h.k.state_.bpfLayerCount=2;h.k.state_.editedBpfLayer=1;h.k.drawFocus(b,120);if(argc>1)b.save("bpf.svg");
 h.k.focus_.knob=5;h.k.drawFocus(b,120);if(argc>1)b.save("shape.svg");}
 puts("PASS: relative control, clamping/reversal, angle seams, recall, repeat rates, resets, MIDI, ADC noise and displays");
}
