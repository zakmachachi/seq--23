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
    int cases = 0;
    for(float hz : {30.f, 49.f, 55.f, 73.41619f, 120.f})
    for(float shape : {0.f, 1.f/127.f, .25f, .5f, 1.f})
    for(int velocity : {0, 64, 127})
    for(int curve : {0, 64, 127})
    for(float sweep : {0.f, .55f, 1.f})
    {
        KickVoice v;
        v.Trigger(hz, shape, sweep, .045f, velocity, curve, 0.f);
        assert(v.phase == .5f);
        for(int i=0; i<24000; ++i)
        {
            KickVoiceOut o;
            v.Process(o);
            assert(std::isfinite(o.punch));
            if(shape>0.f && i < (int)v.body_hold_samples) assert(v.body_env == 1.f);
        }
        ++cases;
    }
    // SHAPE zero is exactly independent of sweep depth/time/curve.
    KickVoice a, b;
    a.Trigger(73.41619f, 0, 0, .3f, 64, 0, 0.f);
    b.Trigger(73.41619f, 0, 1, .3f, 64, 0, 0.f);
    for(int i=0; i<24000; ++i)
    {
        KickVoiceOut x,y; a.Process(x); b.Process(y);
        assert(x.punch == y.punch);
        if(i == 0) assert(fabsf(x.punch) < 1e-6f);
    }
    // All 128 CC values are monotonic; exact neutral and endpoints.
    assert(TailPitchSemitones(0)==-12.f && TailPitchSemitones(64)==0.f && TailPitchSemitones(127)==12.f);
    for(int cc=1;cc<128;++cc) assert(TailPitchSemitones(cc)>TailPitchSemitones(cc-1));
    for(int cc : {0,64,127}) {
        KickVoice v;v.Trigger(73.41619f,0,.55f,.3f,cc,0,0.f);
        for(int i=0;i<48000;++i){KickVoiceOut o;v.Process(o);}
        assert(fabsf(v.instantaneous_hz / 73.41619f-1.f)<1e-5f); // TMOD zero = no pitch movement
    }
    // A same-phase hit blended over 5 ms stays within the normal per-sample
    // slope of the oscillator, even when the prior body is mid-cycle.
    KickVoice reference;reference.Trigger(73.41619f,.5f,.5f,.3f,64,0,0.f);
    float prior=0.f, reference_step=0.f;
    for(int i=0;i<1200;++i){KickVoiceOut o;reference.Process(o);reference_step=fmaxf(reference_step,fabsf(o.punch-prior));prior=o.punch;}
    KickVoice old_voice=reference,new_voice;
    new_voice.Trigger(73.41619f,.5f,.5f,.3f,64,0,0.f);
    float previous_sample=prior, retrigger_step=0.f;
    for(int i=0;i<RAW_909_RETRIGGER_SAMPLES;++i){
        KickVoiceOut a,b;old_voice.Process(a);new_voice.Process(b);
        float t=SmoothstepAdded(float(i)/float(RAW_909_RETRIGGER_SAMPLES));
        float mixed=a.punch*(1.f-t)+b.punch*t;
        retrigger_step=fmaxf(retrigger_step,fabsf(mixed-previous_sample));previous_sample=mixed;
    }
    assert(retrigger_step < reference_step*1.2f+1e-4f);
    // TMOD starts from the same point every trigger and moves up gently from
    // static pitch at zero to deterministic triangle movement at higher rates.
    KickVoice lfoA,lfoB;
    lfoA.Trigger(73.41619f,.5f,.55f,.3f,127,127,0.f);
    lfoB.Trigger(73.41619f,.5f,.55f,.3f,127,127,0.f);
    float low_lfo=1000.f,high_lfo=0.f;
    for(int i=0;i<12000;++i){
        KickVoiceOut x,y;lfoA.Process(x);lfoB.Process(y);
        assert(x.punch==y.punch);
        if(i>6000){low_lfo=fminf(low_lfo,lfoA.instantaneous_hz);high_lfo=fmaxf(high_lfo,lfoA.instantaneous_hz);}
    }
    assert(high_lfo-low_lfo>20.f);
    assert(TailModHz(0)==0.f && fabsf(TailModHz(1)-.02f)<1e-6f && fabsf(TailModHz(127)-16.f)<1e-4f);
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
    midi(0x9e,38,64);midi(0x9e,38,64);
    assert(note_gate && kick_note_holds[38]==2);
    midi(0x8e,38,0);assert(note_gate && kick_note_holds[38]==1);
    midi(0x8e,38,0);assert(!note_gate && kick_note_holds[38]==0);
    midi(0xbe,63,127);assert(kick_temp_decay_active);
    midi(0xbe,63,0);assert(!kick_temp_decay_active);
    kick_trigger_pending=false;
    float input[8]={},left[8]={},right[8]={};const float* in[2]={input,input};float* out[2]={left,right};
    midi(0xbe,77,0);midi(0x9e,38,1);hw.callback(in,out,8);
    assert(kick_voice.tail_pitch_semitones==-12.f && !kick_tail_pitch_pending);
    midi(0x9e,38,64);hw.callback(in,out,8);assert(kick_voice.tail_pitch_semitones==0.f);
    midi(0xbe,77,127);midi(0x9e,38,64);hw.callback(in,out,8);assert(kick_voice.tail_pitch_semitones==12.f);
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
    assert(fabsf(OutputGainFromCC(64)-1.f)<1e-6f);
    assert(fabsf(OutputGainFromCC(0)-0.17782794f)<1e-5f);
    assert(fabsf(OutputGainFromCC(127)-5.623413f)<1e-4f);
    // SHAPE zero carries phase/envelope through notes. Maximum DECAY holds
    // while the MIDI gate is active, then releases after note-off.
    macro_decay=1.f;note_gate=true;
    KickVoice bass;bass.Trigger(55.f,0.f,.5f,.04f,127,127,1.f);
    float old=0.f,max_step=0.f;
    for(int i=0;i<12000;++i){KickVoiceOut o;bass.Process(o);assert(bass.body_env==1.f);assert(bass.instantaneous_hz==55.f);max_step=fmaxf(max_step,fabsf(o.punch-old));old=o.punch;}
    float phase_before=bass.phase;
    bass.Trigger(65.4064f,0.f,1.f,.04f,0,127,1.f);
    assert(bass.phase==phase_before && bass.sustained && bass.tail_pitch_semitones==0.f);
    for(int i=0;i<48000;++i){KickVoiceOut o;bass.Process(o);assert(bass.body_env==1.f);assert(bass.instantaneous_hz>=55.f&&bass.instantaneous_hz<=65.407f);max_step=fmaxf(max_step,fabsf(o.punch-old));old=o.punch;}
    assert(fabsf(bass.instantaneous_hz-65.4064f)<.01f && max_step<.02f);
    note_gate=false;
    for(int i=0;i<10000 && bass.active;++i){KickVoiceOut o;bass.Process(o);assert(std::isfinite(o.punch));}
    assert(!bass.active);
    note_gate=false;macro_decay=.5f;
    printf("%d deterministic voice configurations; crossover sum error %.5f dB; SHAPE0 invariance, gain map, and glue onset passed\n",cases,worst_db);
}
