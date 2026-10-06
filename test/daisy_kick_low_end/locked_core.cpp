// Exercise the production voice and crossover, not a duplicate DSP model.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <complex>
#include "../../include/KickShapeModel.h"
#define main firmware_main
#include "../../daisy-kick/midi_oled_monitor.cpp"
#undef main
namespace daisy { uint32_t System::now_ms = 0; }
static GPIO_TypeDef gpio_storage{};
static USART_TypeDef uart_storage{};
GPIO_TypeDef* GPIOC = &gpio_storage;
USART_TypeDef* USART3 = &uart_storage;
int main()
{
    float worst = 0;
    int cases = 0;
    for(float hz : {30.f, 49.f, 55.f, 73.41619f, 120.f})
    for(float shape : {0.f, 1.f/127.f, .25f, .5f, 1.f})
    for(int velocity : {0, 64, 127})
    for(int curve : {0, 64, 127})
    for(float sweep : {0.f, .55f, 1.f})
    {
        KickVoice v;
        v.Trigger(hz, shape, .55f, sweep, velocity, 0, .045f, 0, curve, 64);
        assert(v.phase == 0 && v.body_phase == 0);
        for(int i=0; i<24000; ++i)
        {
            KickVoiceOut o;
            v.Process(o);
            float error = v.body_phase - v.phase;
            error -= roundf(error);
            worst = fmaxf(worst, fabsf(error));
            assert(fabsf(error) < 1e-7f);
            assert(std::isfinite(o.punch));
            if(i < (int)v.body_hold_samples) assert(v.body_env == 1.f);
        }
        ++cases;
    }
    // SHAPE zero is exactly independent of sweep depth/time/curve.
    KickVoice a, b;
    a.Trigger(73.41619f, 0, 0, 0, 64, 0, .3f, 0, 0, 64);
    b.Trigger(73.41619f, 0, 1, 1, 64, 0, .3f, 0, 127, 64);
    for(int i=0; i<24000; ++i)
    {
        KickVoiceOut x,y; a.Process(x); b.Process(y);
        assert(x.punch == y.punch);
        if(i == 0) assert(x.punch == 0);
    }
    // All 128 CC values are monotonic; exact neutral and endpoints.
    assert(TailPitchSemitones(0)==-12.f && TailPitchSemitones(64)==0.f && TailPitchSemitones(127)==12.f);
    for(int cc=1;cc<128;++cc) assert(TailPitchSemitones(cc)>TailPitchSemitones(cc-1));
    for(int cc : {0,64,127}) {
        KickVoice v;v.Trigger(73.41619f,0,.55f,.55f,cc,0,1.f,0,0,64);
        for(int i=0;i<48000;++i){KickVoiceOut o;v.Process(o);}
        assert(fabsf(v.instantaneous_hz / 73.41619f-powf(2.f,TailPitchSemitones(cc)/12.f))<1e-5f);
    }
    // One complete resettable LFO cycle returns to the selected note.
    KickVoice v;v.Trigger(73.41619f,0,.55f,.55f,127,0,1.f,0,127,64);
    for(int i=0;i<6000;++i){KickVoiceOut o;v.Process(o);assert(v.instantaneous_hz>=73.416f && v.instantaneous_hz<=146.833f);}
    // Impulse round-trip has exactly the delay compensated on the clean bus.
    MackieOversampling os;float peak=0;int peak_at=-1;
    for(int i=0;i<100;++i){os.Push(i==0 ? 1.f:0.f);float y=0;
        for(int k=0;k<4;++k){float z=os.Downsample(os.Upsample(k),k==0);if(k==0)y=z;}
        if(fabsf(y)>peak){peak=fabsf(y);peak_at=i;}}
    assert(peak_at==24);
    for(int cc=0;cc<128;++cc){
        assert(fabsf(MacroBpfFrequencyHz(cc/127.f)-kickdaisy::bpfHz(cc))<.001f);
        assert(TailPitchSemitones(cc)==kickdaisy::tailPitchSemitones(cc));
        assert(TailModHz(cc)==kickdaisy::tailModHz(cc));
    }
    for(int i=0;i<3;++i)assert(fabsf(macro_bpf_target_hz[i]-kickdaisy::bpfHz(35+30*i))<.001f);
    try{firmware_main();}catch(const daisy::HostAudioStarted&){}
    auto midi=[](int status,int key,int value){ProcessMidiByte(status);ProcessMidiByte(key);ProcessMidiByte(value);};
    float input[8]={},left[8]={},right[8]={};const float* in[2]={input,input};float* out[2]={left,right};
    midi(0xbe,77,0);midi(0x9e,38,1);hw.callback(in,out,8);
    assert(kick_voice.tail_semitones==-12.f && !kick_tail_pitch_pending);
    midi(0x9e,38,64);hw.callback(in,out,8);assert(kick_voice.tail_semitones==0.f);
    midi(0xbe,77,127);midi(0x9e,38,64);hw.callback(in,out,8);assert(kick_voice.tail_semitones==12.f);
    double worst_db = 0;
    for(double hz : {30., 36.708, 55., 90., 180., 360., 1000.})
    {
        KickCrossover lp, hp; lp.Reset(false); hp.Reset(true);
        std::complex<double> l(0),h(0),ref(0);
        for(int i=0;i<96000;++i)
        {
            double phase = 2. * 3.141592653589793 * hz * i / 48000.;
            float x = sin(phase), lo=lp.Process(x),hi=hp.Process(x);
            if(i>=48000)
            {
                // Hann projection avoids leakage bias between test frequencies.
                double w=.5-.5*cos(2.*3.141592653589793*(i-48000)/47999.);
                auto p=std::polar(w,-phase);
                l+=double(lo)*p; h+=double(hi)*p; ref+=double(x)*p;
            }
        }
        double db=20*log10(abs(l+h)/abs(ref));
        worst_db=fmax(worst_db,fabs(db));
        assert(fabs(db)<.02);
        assert((l*std::conj(h)).real()>0); // same phase, not opposite polarity
    }
    MixGlue glue;
    assert(glue.Process(.8f)==.8f); // onset is not instantaneously limited
    for(int i=0;i<48000;++i) glue.Process(.8f);
    assert(glue.gain>=.7943f && glue.gain<1.f);
    printf("%d phase configurations: max body phase error %.9g cycles; crossover sum error %.5f dB; SHAPE0 invariance and glue onset passed\n",cases,worst,worst_db);
}
