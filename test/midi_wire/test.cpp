#include "../../include/MidiWirePolicy.h"
#include "../../daisy-kick/midi_rx_queue.h"
#include <cassert>
#include <vector>
#include <cstdio>
struct Port {int free=127;std::vector<unsigned> out;int availableForWrite(){return free;}void write(unsigned b){assert(free>0);--free;out.push_back(b);}};
int main(){
 unsigned accepted=0;for(unsigned b=0;b<256;++b)accepted+=midiwire::IsSync(b);assert(accepted==4);
 Port p;assert(midiwire::Message(p,0xbe,36,127,true));assert(midiwire::Message(p,0xbe,37,127,true));assert(!midiwire::Message(p,0xbe,38,127,true));
 // Seven channels: engine + slide + previous off + new on = 84 bytes.
 for(unsigned ch=0;ch<7;++ch){assert(midiwire::Message(p,0xbd,119,127,false));assert(midiwire::Message(p,0xbd,29,0,false));assert(midiwire::Message(p,0x90|ch,48,0,false));assert(midiwire::Message(p,0x90|ch,48,100,false));}
 assert(p.free>=3);
 for(int free=0;free<6;++free){Port q;q.free=free;assert(!midiwire::Message(q,0x90,48,100,false));assert(q.out.empty());}
 // A lost RX byte invalidates the stream exactly where data resumes.
 MidiRxQueue rx;for(unsigned i=0;i<1023;++i)assert(rx.Push(i&127));assert(!rx.Push(42));
 uint16_t b;assert(rx.Pop(b));assert(rx.Push(99));for(unsigned i=1;i<1023;++i){assert(rx.Pop(b));assert(!(b&MidiRxQueue::GAP));}assert(rx.Pop(b)&&b==(99|MidiRxQueue::GAP));assert(!rx.Pop(b));
 puts("PASS: only four sync bytes accepted; CC backpressure reserves layered notes and realtime; no partial packets; RX overflow recovery marker preserved");
}
