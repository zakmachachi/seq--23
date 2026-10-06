// Production effect: continuity, persistent tails, clocked taps and bypass.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#define main firmware_main
#include "../../daisy-kick/midi_oled_monitor.cpp"
#undef main
namespace daisy { uint32_t System::now_ms=0; }
static GPIO_TypeDef gp{};static USART_TypeDef ua{};
GPIO_TypeDef* GPIOC=&gp;USART_TypeDef* USART3=&ua;
static double energy(KickSidechainReverb& r,int n,float amount)
{double e=0;for(int i=0;i<n;++i){float y=r.Process(0,amount);assert(std::isfinite(y));e+=y*y;}return e;}
int main()
{
    KickSidechainReverb r;perf_quarter_note_ms=400;r.Reset();
    // Regression: float wrapping used to round a tiny negative position
    // to SIZE, reading beyond the delay buffer at near-integer clock times.
    r.write=4800;reverb_delay_buffer[0]=.25f;
    reverb_delay_buffer[REVERB_DELAY_SIZE-1]=.75f;
    float near_integer=4800.00048828125f;
    assert(fabsf(r.ReadTap(near_integer)-(.25f+.5f*(near_integer-4800.f)))<1e-7f);
    r.Reset();
    for(int i=0;i<10000;++i){float x=.1f*sinf(i*.1f);assert(r.Process(x,0)==x);}
    r.Reset();
    for(int i=0;i<48000;++i)r.Process(0,1);
    r.Process(.5f,1);
    energy(r,12000,1);
    float stored=r.comb0[(r.ci0+10)%r.C0];
    r.Trigger();assert(r.comb0[(r.ci0+10)%r.C0]==stored);
    double late=energy(r,48000,1);assert(late>1e-7);
    for(int i=0;i<9600;++i)r.Process(.3f,1);
    float ducked=r.duck_gain;assert(ducked<.2f);
    energy(r,48000,1);assert(r.duck_gain>.98f);
    assert(fabsf(r.tap-9600.f)<1); // eighth note at 150 BPM
    perf_quarter_note_ms=600;
    energy(r,9600,1);assert(fabsf(r.tap-14400.f)<1);
    float peak=0,max_step=0,last=0;
    for(int n=0;n<48000*30;++n)
    {
        if(n%9600==0)r.Trigger();
        if(n%24000==0)perf_quarter_note_ms=(n%48000==0)?120.f:1200.f;
        float x=.15f*sinf(TWO_PI*500.f*n/48000.f);
        float y=r.Process(x,(n/12000)%2?1.f:0.f);
        assert(std::isfinite(y));peak=fmaxf(peak,fabsf(y));
        max_step=fmaxf(max_step,fabsf(y-last));last=y;
    }
    assert(peak<.5f);assert(max_step<.04f);
    energy(r,48000,0);assert(r.Process(.1f,0)==.1f);
    // Actual feedback filters must reject sub-bass and high fizz, while
    // keeping the useful midrange. Measure settled sine RMS, no copied model.
    for(float hz : {30.f, 1000.f, 10000.f})
    {
        r.Reset();double input=0,output=0;
        for(int n=0;n<96000;++n)
        {
            float x=sinf(TWO_PI*hz*n/48000.f);
            float y=reverb_echo_lp.Process(reverb_echo_hp.Process(x));
            if(n>=48000){input+=x*x;output+=y*y;}
        }
        double db=10*log10(output/input);
        if(hz==30.f)assert(db < -30);
        if(hz==1000.f)assert(db > -1 && db < .1);
        if(hz==10000.f)assert(db < -20);
        printf("Echo feedback filter %.0f Hz: %.2f dB\n",hz,db);
    }
    printf("Reverb: persistent tail energy %.8g; duck %.4f; clock 200/300ms correct; 30s automation peak %.5f, max step %.5f; dry bypass exact\n",late,ducked,peak,max_step);
}
