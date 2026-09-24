/*
 * Host test for the Daisy kick voice (daisy-kick/midi_oled_monitor.cpp).
 *
 * run.sh cuts the KickVoice block out of the firmware source verbatim and
 * includes it here with stubs for the few firmware helpers it calls, so this
 * exercises the real code. Checks: punch landmarks, velocity mapping, phase
 * reset determinism, and that no retrigger, delayed sub onset or envelope end
 * produces a sample step larger than a smooth sine could.
 *
 * Pass "wav" to also write audition renders into the current directory.
 */
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
using namespace std;
static constexpr float SAMPLE_RATE = 48000.0f;
static constexpr float TWO_PI = 6.28318530718f;
static constexpr float PI = 3.14159265359f;
static inline float ClampAdded(float x, float lo, float hi){ return x<lo?lo:(x>hi?hi:x); }
static inline float Clamp01Added(float x){ return ClampAdded(x,0.f,1.f); }
static inline float SmoothstepAdded(float x){ x=Clamp01Added(x); return x*x*(3.f-2.f*x); }
static float perf_quarter_note_ms = 60000.0f/180.0f;
static float MacroTailDelayMs(float x){ x=Clamp01Added(x); return perf_quarter_note_ms*0.5f*powf(x,1.7f); }
static float MacroDecaySeconds(float x){ x=Clamp01Added(x); if(x>=0.99f) return 1000000.0f; return 0.035f*powf(171.428571f,x); }
static bool tail_delay_enabled=false; static float macro_tail_delay=0; static float macro_kick_shape=0.5f;
static float macro_decay=0.5f; static float kick_frequency=55.f; 
#include "voice_extract.inc"

struct Render { vector<float> y; float worst_ratio=0; int worst_n=-1; };

// One slot's per-sample slope bound: amplitude upper bound * omega + envelope-slope allowance.
static float SlotBound(const KickVoice& v, float gp, float gs){
    if(!v.active) return 0.f;
    float f=v.BaseFrequency()*(1.f+v.sweep_depth*v.sweep); if(f>0.45f*SAMPLE_RATE) f=0.45f*SAMPLE_RATE;
    float A=KICK_PUNCH_LEVEL*gp+KICK_SUB_LEVEL*gs;
    return A*TWO_PI*f/SAMPLE_RATE;
}
// allowance for raised-cosine onsets / fades: max slope of A*(1-cos)/2 over N samples = A*pi/(2N)
static float EnvAllow(float gp,float gs){
    float ap=KICK_PUNCH_LEVEL*gp*PI/(2.f*PUNCH_ATTACK_MS*48.f);
    float as=KICK_SUB_LEVEL*gs*PI/(2.f*SUB_ATTACK_MS*48.f);
    float af=(KICK_PUNCH_LEVEL*gp+KICK_SUB_LEVEL*gs)*PI/(2.f*RETRIGGER_FADE_MS*48.f);
    return ap+as+af;
}

// hits: sample index -> velocity. hard_reset=true emulates the click the fade prevents.
static Render Run(const vector<pair<int,int>>& hits, int len, float gp, float gs, bool hard_reset=false){
    KickVoice v, fv[KICK_FADING_SLOTS]; Render r; size_t h=0; float prev=0;
    for(int n=0;n<len;n++){
        while(h<hits.size() && hits[h].first==n){
            if(v.active && !hard_reset){ int sl=0; for(int i=0;i<KICK_FADING_SLOTS;i++){ if(!fv[i].active){sl=i;break;} if(fv[i].fade_age>fv[sl].fade_age) sl=i; } fv[sl]=v; fv[sl].BeginFadeOut(); }
            float d = (tail_delay_enabled && macro_tail_delay>0.005f)? MacroTailDelayMs(macro_tail_delay):0.f;
            v.Trigger(kick_frequency, macro_kick_shape, (uint8_t)hits[h].second, d, MacroDecaySeconds(macro_decay));
            h++;
        }
        float bound = SlotBound(v,gp,gs)+EnvAllow(gp,gs)+1e-5f; int nf=0; for(auto& q:fv){ bound+=SlotBound(q,gp,gs); nf+=q.active; } bound += nf*EnvAllow(gp,gs);
        float p=0,s=0,fp=0,fs=0;
        v.Process(gp,gs,p,s);
        for(auto& q:fv) if(q.active){ float a=0,b=0; q.ProcessFading(gp,gs,a,b); fp+=a; fs+=b; }
        float y=p+s+fp+fs;
        float ratio=fabsf(y-prev)/bound;
        if(ratio>r.worst_ratio){ r.worst_ratio=ratio; r.worst_n=n; }
        prev=y; r.y.push_back(y);
    }
    // release to silence must also be smooth
    return r;
}

static void WriteWav(const char* path, const vector<float>& y){
    FILE* f=fopen(path,"wb"); int n=y.size(); int bytes=n*2;
    auto w32=[&](uint32_t x){fwrite(&x,4,1,f);}; auto w16=[&](uint16_t x){fwrite(&x,2,1,f);};
    fwrite("RIFF",1,4,f); w32(36+bytes); fwrite("WAVEfmt ",1,8,f); w32(16); w16(1); w16(1); w32(48000); w32(96000); w16(2); w16(16);
    fwrite("data",1,4,f); w32(bytes); for(float s:y){ int v=(int)lrintf(ClampAdded(s,-1,1)*32767); w16((uint16_t)(int16_t)v);} fclose(f);
}

int main(int argc, char** argv){
    bool wav = argc > 1 && strcmp(argv[1], "wav") == 0;
    int fails=0;
    auto expect=[&](bool c,const char* m){ printf("%s %s\n", c?"PASS":"FAIL", m); if(!c) fails++; };

    // Punch landmarks
    printf("start ratio p=0 %.3f  p=0.5 %.3f  p=1 %.3f\n", PunchStartRatio(0), PunchStartRatio(0.5f), PunchStartRatio(1));
    printf("sweep ms   p=0 %.1f  p=0.5 %.1f  p=1 %.1f\n", PunchSweepMs(0), PunchSweepMs(0.5f), PunchSweepMs(1));
    expect(fabsf(PunchStartRatio(0)-1)<1e-6f, "punch 0 = no sweep");
    expect(fabsf(PunchStartRatio(0.5f)-3.8f)<0.05f, "punch 64 ~ 3.8x");
    expect(fabsf(PunchStartRatio(1)-30)<1e-3f, "punch 127 = 30x");
    expect(fabsf(VelocityToSubMoveSemitones(64))<1e-6f && fabsf(VelocityToSubMoveSemitones(1)+12)<1e-6f && fabsf(VelocityToSubMoveSemitones(127)-12)<1e-6f, "velocity 1/64/127 = -12/0/+12 st");

    const int SR=48000;
    float shapes[]={0.f,0.5f,1.f}; int vels[]={1,64,127}; float delays[]={0.f,0.6f,1.f}; float decays[]={0.2f,0.5f,0.99f};
    float worst=0; char worst_desc[160]="";
    for(float sh:shapes) for(int ve:vels) for(float dl:delays) for(float dc:decays){
        macro_kick_shape=sh; macro_decay=dc; tail_delay_enabled=dl>0; macro_tail_delay=dl;
        // single hit, then ratchets at awkward offsets (mid-punch, mid-gap, mid-sub, near-simultaneous)
        vector<pair<int,int>> hits={{100,ve},{100+SR/3,ve},{100+SR/3+1234,ve},{100+SR/3+1234+97,ve},{100+SR/3+9000,ve},{100+SR/3+9000+2000,ve}};
        Render r=Run(hits, SR*2, 1.0f, 0.95f);
        if(r.worst_ratio>worst){ worst=r.worst_ratio; snprintf(worst_desc,sizeof worst_desc,"shape %.2f vel %d delay %.2f decay %.2f at n=%d",sh,ve,dl,dc,r.worst_n);}
    }
    printf("worst |dy|/bound over 81 configs x 6 hits = %.3f (%s)\n", worst, worst_desc);
    expect(worst<=1.0f, "no sample step exceeds the smooth-sine slope bound (retriggers, delayed sub onset, envelope ends)");

    { macro_kick_shape=1; macro_decay=0.99f; tail_delay_enabled=false;
      vector<pair<int,int>> hs; for(int i=0;i<6;i++) hs.push_back({3000+i*31,127}); Render r=Run(hs, SR/2, 1.0f, 0.95f);
      printf("MIDI-rate burst worst ratio %.3f\n", r.worst_ratio); expect(r.worst_ratio<=1.0f,"6 hits at max MIDI rate stay smooth"); }
    // Max gains too
    macro_kick_shape=1; macro_decay=0.99f; tail_delay_enabled=true; macro_tail_delay=1;
    { Render r=Run({{10,127},{5000,1},{5050,127}}, SR, 2.0f, 1.6f); printf("max-gain worst ratio %.3f\n", r.worst_ratio); expect(r.worst_ratio<=1.0f,"max CC57/58 gains stay smooth"); }

    // Control: a hard phase reset must be detected by the same metric
    macro_kick_shape=0; macro_decay=0.5f; tail_delay_enabled=false;
    { Render r=Run({{0,64},{2000+217,64}}, SR/2, 1.0f, 0.95f, true); printf("control hard-reset ratio %.2f\n", r.worst_ratio); expect(r.worst_ratio>1.5f,"metric catches a hard phase reset"); }

    // Determinism: a hit from silence equals a hit after unrelated history, once the 3 ms fade is over
    macro_kick_shape=0.7f; macro_decay=0.6f; tail_delay_enabled=true; macro_tail_delay=0.4f;
    Render a=Run({{0,90}}, SR, 1, 0.95f);
    Render b=Run({{0,20},{7777,90}}, 7777+SR, 1, 0.95f);
    float maxdiff=0; for(int n=int(RETRIGGER_FADE_MS*48)+1;n<SR;n++) maxdiff=fmaxf(maxdiff,fabsf(a.y[n]-b.y[7777+n]));
    printf("determinism max diff after fade = %g\n", maxdiff);
    expect(maxdiff==0.0f, "retriggered hit is bit-identical to a fresh hit after the fade");
    Render c=Run({{0,90}}, SR, 1, 0.95f); bool same=memcmp(a.y.data(),c.y.data(),SR*4)==0;
    expect(same, "two fresh hits are bit-identical (phase reset)");
    expect(a.y[0]==0.0f, "each hit starts at exactly zero");

    // Audition renders
    if(wav){
    auto aud=[&](const char* name,float sh,int ve,float dl,float dc){ macro_kick_shape=sh; macro_decay=dc; tail_delay_enabled=dl>0; macro_tail_delay=dl;
        vector<pair<int,int>> hs; for(int i=0;i<8;i++) hs.push_back({i*SR/3,ve}); Render r=Run(hs, SR*3, 1, 0.95f); WriteWav(name,r.y); };
    aud("kick_punch000_flat.wav",0,64,0,0.5f); aud("kick_punch064_flat.wav",0.5f,64,0,0.5f); aud("kick_punch127_flat.wav",1,64,0,0.5f);
    aud("kick_punch064_down.wav",0.5f,1,0,0.6f); aud("kick_punch064_up.wav",0.5f,127,0,0.6f); aud("kick_punch064_taildelay.wav",0.5f,64,0.8f,0.5f);
    aud("kick_ratchet_longdecay.wav",0.5f,64,0,0.9f);
    }
    printf("%s\n", fails? "SOME CHECKS FAILED":"ALL CHECKS PASSED"); return fails;
}
