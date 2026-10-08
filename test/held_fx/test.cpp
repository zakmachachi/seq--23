#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>
#include <utility>
#include <chrono>
#include "../../include/HeldFxButtons.h"
#include "../../daisy-kick/effects/HeldFxEngine.h"
using namespace heldfx;
static Engine engine;
struct Sender {
    bool ready=true;std::vector<std::pair<int,int>> sent;
    static bool Send(void* p,uint8_t cc,uint8_t value){auto& s=*static_cast<Sender*>(p);if(!s.ready)return false;s.sent.emplace_back(cc,value);return true;}
};
static void Block(Control& c,uint32_t sample,bool running=true,bool kick=false,bool quarter=false){
    engine.BeginBlock(c,sample/48,running,kick,quarter,300.f,16);
}
int main(){
    // Button ownership and retry are independent of UI pages or DSP.
    Sender sender;HeldFxButtons buttons;buttons.Begin(Sender::Send,&sender);buttons.Scope(1);buttons.Service(0);
    assert(sender.sent.back()==std::make_pair(int(SELECT),0));
    assert(buttons.Press(0));buttons.Service(1);assert(buttons.Selected()==1);
    buttons.Press(15);buttons.Service(2);assert(buttons.Selected()==16);
    buttons.Release(15);buttons.Service(3);assert(buttons.Selected()==1);
    buttons.Release(0);buttons.Service(4);assert(buttons.Selected()==0);
    buttons.Press(3);buttons.Scope(0);buttons.Service(5);
    assert(buttons.Selected()==0 && buttons.Release(3)); // swallowed on a different channel
    assert(!buttons.Press(0));buttons.Scope(2);
    sender.ready=false;buttons.Press(5);buttons.Service(6);buttons.Release(5);
    sender.ready=true;buttons.Service(7);assert(sender.sent.back()==std::make_pair(int(SELECT),0));
    for(int i=0;i<16;++i)buttons.Press(i);
    assert(buttons.Selected()==16&&buttons.Held()==65535);
    for(int i=15;i>=0;--i){buttons.Release(i);assert(buttons.Selected()==i);}
    buttons.Press(2);buttons.Service(10);auto count=sender.sent.size();buttons.Service(209);assert(sender.sent.size()==count);
    buttons.Service(210);assert(sender.sent.back()==std::make_pair(int(KEEPALIVE),127));
    buttons.Clear();assert(buttons.Release(2)&&buttons.Selected()==0);

    // No start before a quantization edge; cancel before edge never sounds.
    Control c;engine.Reset();c.Handle(SELECT,1,0);Block(c,0);
    for(int i=0;i<16;++i)assert(engine.Process(.1f)==.1f);
    assert(!engine.active&&engine.pending==1);
    c.Handle(SELECT,0,1);Block(c,48,true,true,true);engine.Process(.1f);assert(!engine.active);
    c.Handle(SELECT,5,2);Block(c,96,true,true);engine.Process(.1f);assert(engine.active==5);
    for(int i=0;i<1000;++i)engine.Process(.1f);
    c.Handle(SELECT,6,3);Block(c,144,true,false,true);
    for(int i=0;i<600;++i)engine.Process(.1f);
    assert(engine.active==6&&engine.faults==0);
    c.Handle(SELECT,0,20);Block(c,960);
    for(int i=0;i<400;++i)engine.Process(.1f);assert(engine.Process(.1f)==.1f);
    Block(c,1000,false);c.Handle(SELECT,8,21);Block(c,1008,false);engine.Process(0);assert(engine.active==8);
    Block(c,48000,false);for(int i=0;i<500;++i)engine.Process(0);assert(!engine.active); // lease expiry
    c.Handle(SELECT,1,1100);Block(c,52800,true,true);engine.Process(0);
    Block(c,52816,false);for(int i=0;i<500;++i)engine.Process(0);assert(!engine.active); // Stop
    engine.Reset();c.Handle(SELECT,1,1200);
    for(unsigned s=57600;s<57600+14500;s+=16){Block(c,s);for(int j=0;j<16;++j)engine.Process(0);}
    assert(engine.active==1); // missing clock fallback bounded to a quarter
    c.Handle(SELECT,127,1500);Block(c,72000,false);for(int i=0;i<500;++i)engine.Process(0);assert(!engine.active);

    // Every pad, endpoints of every macro, silence, transients and long held tails.
    Random source;source.Reset();
    for(unsigned pad=1;pad<=16;++pad){
        engine.Reset();c.Handle(SELECT,pad,0);double error=0,energy=0;float peak=0;
        auto begin=std::chrono::steady_clock::now();
        for(unsigned n=0;n<192000;n+=16){
            if(n%9600==0)c.Handle(KEEPALIVE,127,n/48);
            if(n%48000==0){unsigned section=n/48000;
                c.Handle(CHARACTER,section==0?0:127,n/48);
                c.Handle(MOTION,section&1?127:0,n/48);
                c.Handle(SIZE,section&1?0:127,n/48);
                c.Handle(MIX,127,n/48);
            }
            Block(c,n,true,n==0,n%14400==0);
            for(unsigned j=0;j<16;++j){
                unsigned t=n+j;float envelope=expf(-float(t%14400)/2500.f);
                float x=t>=144000?0.f:envelope*(.18f*sinf(t*.05f)+.08f*source.Bipolar());
                float y=engine.Process(x);
                assert(std::isfinite(y)&&fabsf(y)<=.951f);
                peak=fmaxf(peak,fabsf(y));error+=(y-x)*(y-x);energy+=y*y;
            }
        }
        assert(engine.faults==0 && (pad==1 || pad==9 || error>.01) && energy>.01);
        c.Handle(SELECT,0,4000);Block(c,192000);
        for(int i=0;i<600;++i)engine.Process(.123f);
        assert(engine.Process(.123f)==.123f);
        auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
        printf("pad %2u %-6s peak %.4f delta %.2f host %.3fs / 4s audio\n",pad,Name((pad-1)%8),peak,error,elapsed);
    }
    // DECAY's leading transient is exactly the dry signal, then its tail degrades.
    engine.Reset();c.Handle(SELECT,7,0);c.Handle(CHARACTER,127,0);c.Handle(MIX,127,0);
    for(unsigned n=0;n<96000;n+=16){if(n%9600==0)c.Handle(KEEPALIVE,127,n/48);Block(c,n,false);for(int j=0;j<16;++j)engine.Process(0);}
    float input=.4f,first=engine.Process(input);assert(engine.analysis.transient&&first==input);
    for(int i=0;i<120;++i)assert(engine.Process(input)==input);
    float last=0;for(int i=0;i<2000;++i)last=engine.Process(.1f*sinf(i*.15f));assert(std::isfinite(last));
    // MIX=0, independent of mode or held status, is exact bypass after smoothing.
    c.Handle(MIX,0,2000);for(int i=0;i<48000;++i){if(i%16==0)Block(c,96000+i,false);engine.Process(.2f);}
    assert(fabsf(engine.Process(.2f)-.2f)<1e-6f);
    printf("PASS: held priority, retry/lease, quantization, cancellation, all 16 gestures, macro endpoints, exact release and DECAY transient; engine %zu bytes\n",sizeof(engine));
}
