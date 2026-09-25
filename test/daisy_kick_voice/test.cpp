/*
 * Host test for the Daisy kick voice (daisy-kick/midi_oled_monitor.cpp).
 *
 * run.sh cuts the KickVoice block out of the firmware source verbatim and
 * includes it here with stubs for the few firmware helpers it calls, so this
 * exercises the real code.
 *
 * With the punch off (SHAPE 0, PUNCH 0) the click measure is spectral: a
 * 55 Hz sub has no business above 1 kHz, so the peak of a 4th-order 1 kHz
 * high-pass of the output, relative to the sub's own peak, is how loud any
 * click is. Every scenario must stay below CLICK_LIMIT_DB, and a control
 * proves the measure catches a real click.
 *
 * With the punch on, energy above 1 kHz is the punch itself, so the check is
 * instead that no sample steps further than a smooth sine at the voice's
 * own frequency and level could, plus determinism.
 *
 * Pass "wav" to also write audition renders into the current directory.
 */
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <vector>
using namespace std;

static constexpr float SAMPLE_RATE = 48000.0f;
static constexpr float TWO_PI = 6.28318530718f;
static constexpr float PI = 3.14159265359f;
static inline float ClampAdded(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float Clamp01Added(float x) { return ClampAdded(x, 0.0f, 1.0f); }
static inline float SmoothstepAdded(float x) { x = Clamp01Added(x); return x * x * (3.0f - 2.0f * x); }
static float perf_quarter_note_ms = 60000.0f / 180.0f;
static float MacroTailDelayMs(float x) { x = Clamp01Added(x); return perf_quarter_note_ms * 0.5f * powf(x, 1.7f); }
static float MacroDecaySeconds(float x) { x = Clamp01Added(x); if(x >= 0.99f) return 1000000.0f; return 0.035f * powf(171.428571f, x); }
static bool tail_delay_enabled = false;
static float macro_tail_delay = 0.0f;
static float macro_decay = 0.34f;
static float macro_kick_shape = 0.0f;
/* v1.3.0 per-hit controls, as the firmware's CC60..63 leave them. */
static float test_tail_attack_ms = 6.0f;
static float test_sweep_time_scale = 1.0f;
static float test_tail_mod = 0.0f;
static float kick_frequency = 55.0f;
#include "voice_extract.inc"

static constexpr double CLICK_LIMIT_DB = -55.0;

struct Biquad
{
    double b0, b1, b2, a1, a2, z1 = 0, z2 = 0;
    Biquad(double f, double q)
    {
        double w = 2 * M_PI * f / 48000, c = cos(w), al = sin(w) / (2 * q), a0 = 1 + al;
        b0 = (1 + c) / 2 / a0; b1 = -(1 + c) / a0; b2 = b0; a1 = -2 * c / a0; a2 = (1 - al) / a0;
    }
    double Process(double x) { double y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y; }
};

/*
 * Peak above 1 kHz in dB, relative to the signal peak or a full-level sub,
 * whichever is louder. Relative to the peak alone, a render that is nearly
 * silent (DECAY at minimum under the old anatomy) turns its own rounding
 * into a "click".
 */
static double ClickDb(const vector<float>& y, double fc = 1000.0)
{
    Biquad a(fc, 0.54119610), b(fc, 1.30656296);
    double peak = KICK_SUB_LEVEL * 0.95, hf = 0;
    /* Renders may end mid-note; only the render itself is analysed. */
    for(float s : y) { peak = fmax(peak, fabs(s)); hf = fmax(hf, fabs(b.Process(a.Process(s)))); }
    return 20 * log10(fmax(hf, 1e-12) / peak);
}

struct Render
{
    vector<float> y;
    float worst_step_ratio = 0.0f;  /* largest |dy| over the smooth-sine bound */
    int worst_step_at = -1;
};

/*
 * One voice's per-sample slope bound: each path's level times angular
 * frequency, the punch scaled by its drive (SoftClip's slope is at most the
 * drive), plus the fastest raised-cosine onset or smoothstep fade it can run.
 */
static float SlopeBound(const KickVoice& v, float pg, float sg)
{
    if(!v.active) return 0.0f;
    float f = v.BaseFrequency(v.SubAge()) * (1.0f + v.sweep_depth * v.sweep);
    /* A handoff glides from the old base pitch, plus its <= ~9 Hz correction. */
    if(v.handoff) f += fabsf(v.handoff_drift) * SAMPLE_RATE + 10.0f;
    float w = TWO_PI * fminf(f, 0.45f * SAMPLE_RATE) / SAMPLE_RATE;
    float punch = v.punch_level_scale * pg;
    float sub = KICK_SUB_LEVEL * sg + (v.handoff ? v.handoff_level : 0.0f);
    float drive = KICK_PUNCH_OLD_DRIVE ? fmaxf(1.0f, v.punch_drive) : 1.0f;
    return punch * drive * w + sub * w +
           punch * (v.punch_sharp_attack ? 6.9078f : PI / 2.0f) / v.punch_attack_samples +
           KICK_SUB_LEVEL * sg * PI / (2.0f * MsToSamples(SUB_ATTACK_MS)) +
           (punch + sub) * 1.5f / MsToSamples(RETRIGGER_HANDOFF_MS) +
           (punch + sub) * 1.6f / (v.anatomy_end - v.anatomy_start);
}

/* The firmware's TriggerKickVoice() and per-sample loop. */
static Render RunFull(const vector<pair<int, int>>& hits, int length, float pg, float sg)
{
    KickVoice voice, fading[KICK_FADING_SLOTS];
    auto free_slot = [&]() -> KickVoice&
    {
        int slot = 0;
        for(int i = 0; i < KICK_FADING_SLOTS; i++)
        {
            if(!fading[i].active) return fading[i];
            if(fading[i].fade_age > fading[slot].fade_age) slot = i;
        }
        return fading[slot];
    };
    Render r;
    size_t h = 0;
    float prev = 0.0f;
    for(int n = 0; n < length; n++)
    {
        while(h < hits.size() && hits[h].first == n)
        {
            float delay = (tail_delay_enabled && macro_tail_delay > 0.005f) ? MacroTailDelayMs(macro_tail_delay) : 0.0f;
            bool sounding = voice.Sounding();
            if(delay > 0.0f && (sounding || voice.PunchSounding()))
            {
                KickVoice& slot = free_slot();
                slot = voice;
                slot.BeginFadeOut(false);
                sounding = false;
            }
            else if(voice.PunchSounding())
            {
                KickVoice& slot = free_slot();
                slot = voice;
                slot.BeginFadeOut(true);
            }
            KickHitParams hit;
            hit.frequency = kick_frequency;
            hit.punch = macro_kick_shape;
            hit.velocity = (uint8_t)hits[h].second;
            hit.delay_ms = delay;
            hit.decay_seconds = MacroDecaySeconds(macro_decay);
            hit.sub_attack_ms = test_tail_attack_ms;
            hit.sweep_time_scale = test_sweep_time_scale;
            hit.tail_mod = test_tail_mod;
            voice.Trigger(hit, sounding);
            h++;
        }
        float bound = SlopeBound(voice, pg, sg) + 1e-5f;
        for(auto& f : fading) bound += SlopeBound(f, pg, sg);
        KickVoiceOut out;
        voice.Process(out);
        for(auto& f : fading) if(f.active) f.Process(out);
        float y = out.punch * pg + out.sub * sg;
        float ratio = fabsf(y - prev) / bound;
        if(ratio > r.worst_step_ratio) { r.worst_step_ratio = ratio; r.worst_step_at = n; }
        prev = y;
        r.y.push_back(y);
    }
    return r;
}

/* Sub only: SHAPE 0, PUNCH gain 0, SUB at `gain`. */
static vector<float> Run(const vector<pair<int, int>>& hits, int length, float gain)
{
    float shape = macro_kick_shape;
    macro_kick_shape = 0.0f;
    vector<float> y = RunFull(hits, length, 0.0f, gain).y;
    macro_kick_shape = shape;
    return y;
}

/*
 * Peak above fc inside [lo, hi), relative to the whole render's peak. The
 * filter runs over the whole render so the window has no edges of its own.
 */
static double BandPeakDb(const vector<float>& y, double fc, int lo, int hi)
{
    Biquad a(fc, 0.54119610), b(fc, 1.30656296);
    double peak = 1e-12, hf = 0;
    for(int n = 0; n < (int)y.size(); n++)
    {
        double v = b.Process(a.Process(y[n]));
        peak = fmax(peak, fabs(y[n]));
        if(n >= lo && n < hi) hf = fmax(hf, fabs(v));
    }
    return 20 * log10(fmax(hf, 1e-12) / peak);
}

static void WriteWav(const char* path, const vector<float>& y)
{
    FILE* f = fopen(path, "wb");
    uint32_t bytes = y.size() * 2;
    auto w32 = [&](uint32_t x) { fwrite(&x, 4, 1, f); };
    auto w16 = [&](uint16_t x) { fwrite(&x, 2, 1, f); };
    fwrite("RIFF", 1, 4, f); w32(36 + bytes); fwrite("WAVEfmt ", 1, 8, f); w32(16); w16(1); w16(1);
    w32(48000); w32(96000); w16(2); w16(16); fwrite("data", 1, 4, f); w32(bytes);
    for(float s : y) w16((uint16_t)(int16_t)lrintf(ClampAdded(s, -1, 1) * 32767));
    fclose(f);
}

static int fails = 0;
static void Expect(bool ok, const char* what) { printf("%s %s\n", ok ? "PASS" : "FAIL", what); if(!ok) fails++; }

int main(int argc, char** argv)
{
    printf("---- punch profile stage %d ----\n", KICK_PUNCH_PROFILE_STAGE);
    bool wav = argc > 1 && strcmp(argv[1], "wav") == 0;
    const int SR = 48000;

    Expect(VelocityToSubMoveSemitones(1) == -12.0f &&
           VelocityToSubMoveSemitones(64) == 0.0f &&
           VelocityToSubMoveSemitones(127) == 12.0f,
           "velocity 1/64/127 = -12/0/+12 semitones");

    /* Sweep the controls, with ratchets landing at awkward offsets. */
    int velocities[] = {1, 40, 64, 100, 127};
    float delays[] = {0.0f, 0.3f, 0.6f, 1.0f};
    float decays[] = {0.0f, 0.34f, 0.6f, 0.9f, 0.99f};
    float notes[] = {41.2f, 55.0f, 82.4f, 130.0f};
    double worst = -999; char worst_case[160] = "";
    for(float note : notes) for(int vel : velocities) for(float dl : delays) for(float dc : decays)
    {
        kick_frequency = note; macro_decay = dc; tail_delay_enabled = dl > 0; macro_tail_delay = dl;
        vector<pair<int, int>> hits = {{100, vel}, {100 + SR / 3, vel}, {100 + SR / 3 + 1234, vel},
                                        {100 + SR / 3 + 1234 + 97, vel}, {100 + SR / 3 + 9000, vel},
                                        {100 + SR / 3 + 9000 + 2000, vel}};
        /*
         * The click band starts at 1 kHz, or six times the highest pitch
         * the sub reaches if that is higher: PITCH can take a 130 Hz note to
         * 260 Hz, whose own onset reaches toward 1 kHz without clicking.
         */
        float top = fminf(SUB_MAX_FREQUENCY_HZ, note * powf(2.0f, fmaxf(0.0f, VelocityToSubMoveSemitones(vel)) / 12.0f));
        double db = ClickDb(Run(hits, 2 * SR, 0.95f), fmax(1000.0, 6.0 * top));
        if(db > worst)
        {
            worst = db;
            snprintf(worst_case, sizeof worst_case, "note %.1f Hz, vel %d, delay %.2f, decay %.2f", note, vel, dl, dc);
        }
    }
    printf("worst click over 400 configs x 6 hits: %.1f dB (%s)\n", worst, worst_case);
    Expect(worst <= CLICK_LIMIT_DB, "no hit, delayed start, decay end or ratchet puts a click above the limit");

    /* Retrigger exactly at the old voice's peak, the worst possible phase. */
    kick_frequency = 55; macro_decay = 0.9f; tail_delay_enabled = false;
    int peak_offset = 100 + int(0.25f / 55.0f * SR) + 55 * 48 * 20 / 55;
    double db_peak = ClickDb(Run({{100, 64}, {peak_offset, 64}}, SR, 0.95f));
    printf("ratchet at the old voice's peak: %.1f dB\n", db_peak);
    Expect(db_peak <= CLICK_LIMIT_DB, "ratchet at the worst phase stays below the limit");

    /* Ten hits at the maximum MIDI note rate. */
    vector<pair<int, int>> burst;
    for(int i = 0; i < 10; i++) burst.push_back({3000 + i * 31, 64});
    double db_burst = ClickDb(Run(burst, SR / 2, 0.95f));
    printf("10 hits at max MIDI rate: %.1f dB\n", db_burst);
    Expect(db_burst <= CLICK_LIMIT_DB, "a MIDI-rate burst stays below the limit");

    /* Control: a sine switched on at its peak must register as a click. */
    {
        vector<float> y(SR / 2, 0.0f);
        for(int n = 2000; n < SR / 2; n++) y[n] = cosf(TWO_PI * 55.0f * (n - 2000) / SAMPLE_RATE) * 0.5f;
        double db = ClickDb(y);
        printf("control, sine switched on at its peak: %.1f dB\n", db);
        Expect(db > CLICK_LIMIT_DB + 30, "the measure catches a real click");
    }

    /*
     * DECAY INF: every hit lands on a full-level sine. With the same note the
     * handoff has nothing to change but the phase, so each band must stay
     * within a few dB of an uninterrupted sine, the floor of this measure.
     */
    {
        kick_frequency = 55; macro_decay = 0.99f; tail_delay_enabled = false;
        vector<float> sine(SR);
        for(int n = 0; n < SR; n++) sine[n] = sinf(TWO_PI * 55.0f * n / SAMPLE_RATE);
        double worst_excess = -999; int worst_offset = 0; double worst_fc = 0;
        for(int k = 0; k < 24; k++)  /* retrigger at 24 phases of the old sine */
        {
            int t2 = SR / 3 + k * 36;
            vector<float> y = Run({{100, 64}, {t2, 64}}, SR, 0.95f);
            for(double fc : {150.0, 300.0, 1000.0})
            {
                double excess = BandPeakDb(y, fc, t2 - 240, t2 + 4800) - BandPeakDb(sine, fc, t2 - 240, t2 + 4800);
                if(excess > worst_excess) { worst_excess = excess; worst_offset = k; worst_fc = fc; }
            }
        }
        printf("DECAY INF retrigger: worst band excess over a plain sine %.1f dB (>%.0f Hz, phase step %d/24)\n",
               worst_excess, worst_fc, worst_offset);
        Expect(worst_excess <= 6.0, "a retrigger at DECAY INF is within 6 dB of an uninterrupted sine in every band");
    }

    /* ---------------- punch ---------------- */
    printf("punch start ratio: SHAPE 0 %.3f, 64 %.3f, 127 %.3f\n", PunchStartRatio(0), PunchStartRatio(0.5f), PunchStartRatio(1));
    Expect(PunchStartRatio(0) == 1.0f, "SHAPE 0 = no sweep");
    Expect(fabsf(PunchStartRatio(0.5f) - 3.8f) < 0.05f && fabsf(PunchSweepMs(0.5f) - 88.0f) < 1e-3f, "SHAPE 64 = 3.8x over 88 ms");
    Expect(fabsf(PunchStartRatio(1.0f) - 30.0f) < 1e-3f && fabsf(PunchSweepMs(1.0f) - 110.0f) < 1e-3f, "SHAPE 127 = 30x over 110 ms");
    {
        float shapes[] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
        float gains[][2] = {{1.0f, 0.95f}, {2.0f, 1.6f}, {1.0f, 0.0f}, {0.0f, 1.6f}};
        float worst_ratio = 0; char worst_desc[160] = "";
        struct Extra { float attack, scale, mod; };
        const Extra extras[] = {{6, 1, 0}, {60, 0.25f, 1}, {6, 4, 0.5f}, {30, 2, 0.25f}};
        for(float sh : shapes) for(auto& g : gains) for(float dl : delays) for(float dc : decays) for(int vel : {1, 64, 127})
        for(const Extra& ex : extras)
        {
            test_tail_attack_ms = ex.attack; test_sweep_time_scale = ex.scale; test_tail_mod = ex.mod;
            macro_kick_shape = sh; kick_frequency = 55; macro_decay = dc; tail_delay_enabled = dl > 0; macro_tail_delay = dl;
            vector<pair<int, int>> hits = {{100, vel}, {100 + SR / 3, vel}, {100 + SR / 3 + 1234, vel},
                                            {100 + SR / 3 + 1234 + 97, vel}, {100 + SR / 3 + 9000, vel},
                                            {100 + SR / 3 + 9000 + 2000, vel}};
            Render r = RunFull(hits, SR, g[0], g[1]);
            if(r.worst_step_ratio > worst_ratio)
            {
                worst_ratio = r.worst_step_ratio;
                snprintf(worst_desc, sizeof worst_desc, "shape %.2f punch %.1f sub %.1f delay %.2f decay %.2f vel %d attack %.0f scale %.2f mod %.2f",
                         sh, g[0], g[1], dl, dc, vel, ex.attack, ex.scale, ex.mod);
            }
        }
        test_tail_attack_ms = 6; test_sweep_time_scale = 1; test_tail_mod = 0;
        printf("punch on: worst sample step over the smooth-sine bound %.3f (%s)\n", worst_ratio, worst_desc);
        Expect(worst_ratio <= 1.0f, "with the punch on, no hit or ratchet steps the waveform");

        macro_kick_shape = 0.6f; macro_decay = 0.99f; tail_delay_enabled = false;
        vector<float> fresh = RunFull({{0, 64}}, SR, 1.0f, 0.95f).y;
        vector<float> again = RunFull({{0, 64}, {7777, 64}}, 7777 + SR, 1.0f, 0.95f).y;
        float diff = 0;
        for(int n = MsToSamples(RETRIGGER_HANDOFF_MS) + 1; n < SR; n++) diff = fmaxf(diff, fabsf(fresh[n] - again[7777 + n]));
        /*
         * Within 1e-6 (-120 dB) rather than bit-identical: the punch path's
         * filters saw the handoff's bent phase, and their memory of it
         * decays over a few samples rather than vanishing exactly.
         */
        printf("punched ratchet vs fresh hit after the handoff: max diff %g\n", diff);
        Expect(diff <= 1e-6f, "with the punch on at DECAY INF, a retriggered hit matches a fresh one after the handoff");
        Expect(fresh[0] == 0.0f, "a punched hit starts at exactly zero");
        macro_kick_shape = 0.0f;
    }

    /* ---------------- v1.3.0 controls ---------------- */
    {
        /* TAIL MOD: a wobble macro, +-2 st at most, faster as it rises. */
        {
            auto wobble = [&](float intensity, float& lo_st, float& hi_st, int& crossings){
                KickHitParams hit; hit.frequency = 55; hit.velocity = 64; hit.decay_seconds = 1.0f; hit.tail_mod = intensity;
                KickVoice v; v.Trigger(hit, false);
                lo_st = 1e9f; hi_st = -1e9f; crossings = 0; float prev = 0;
                for(uint32_t n = 0; n < (uint32_t)SR; n++){
                    float st = 12.0f * log2f(v.BaseFrequency(n) / 55.0f);
                    lo_st = fminf(lo_st, st); hi_st = fmaxf(hi_st, st);
                    if(n > 0 && ((prev < 0) != (st < 0))) crossings++;
                    prev = st;
                }
            };
            float lo, hi; int c0, c_half, c_full;
            wobble(0.0f, lo, hi, c0);
            Expect(lo == 0.0f && hi == 0.0f, "TAIL MOD 0 leaves the pitch alone");
            wobble(0.5f, lo, hi, c_half);
            wobble(1.0f, lo, hi, c_full);
            printf("TAIL MOD full: %.2f .. %+.2f st; wobble crossings in 1 s: half %d, full %d\n", lo, hi, c_half, c_full);
            Expect(lo >= -2.0f - 1e-3f && hi <= 2.0f + 1e-3f && lo < -1.5f && hi > 1.5f,
                   "full TAIL MOD wobbles both ways, within +-2 semitones");
            Expect(c_full > 2 * c_half, "TAIL MOD wobbles faster as its intensity rises");
            KickHitParams hit; hit.frequency = 55; hit.tail_mod = 1.0f; KickVoice v; v.Trigger(hit, false);
            Expect(v.BaseFrequency(0) == 55.0f, "the wobble starts on the note");
        }

        /*
         * A handoff across a big pitch gap (TAIL MOD took the old hit to the
         * 440 Hz ceiling, the new one starts at 55 Hz) must glide between the
         * two, never overshoot or run the sine backwards.
         */
        {
            KickHitParams up; up.frequency = 55; up.punch = 0; up.velocity = 127;
            up.decay_seconds = 1000000.0f; up.frequency = 130.0f;
            KickVoice w; w.Trigger(up, false);
            for(int n = 0; n < SR; n++){ KickVoiceOut o; w.Process(o); }
            float f_old = w.last_increment * SAMPLE_RATE;
            KickHitParams fresh = up; fresh.frequency = 55.0f; fresh.velocity = 64;
            w.Trigger(fresh, true);
            float lo = 1e9f, hi = -1e9f;
            for(uint32_t n = 0; n < MsToSamples(RETRIGGER_HANDOFF_MS); n++)
            {
                KickVoiceOut o; w.Process(o);
                float f = w.last_increment * SAMPLE_RATE;
                if(f > 0.5f * SAMPLE_RATE) f -= SAMPLE_RATE; /* a backwards step wraps */
                lo = fminf(lo, f); hi = fmaxf(hi, f);
            }
            printf("handoff %.0f Hz -> 55 Hz: frequency stayed within %.1f .. %.1f Hz\n", f_old, lo, hi);
            Expect(f_old > 250.0f, "the handoff check really crosses a big pitch gap");
            Expect(lo > 55.0f - 12.0f && hi < f_old + 12.0f, "a handoff across a big pitch gap glides between the two pitches");
        }

        /* WAVE: sine -> supersaw. */
        {
            auto render = [&](float wave, int n_samples, vector<pair<int,float>> hits_morph) {
                KickVoice w; vector<float> y; size_t h = 0;
                for(int n = 0; n < n_samples; n++){
                    while(h < hits_morph.size() && hits_morph[h].first == n){
                        KickHitParams hp; hp.frequency = 55; hp.punch = 0; hp.velocity = 64;
                        hp.decay_seconds = 1000000.0f; hp.wave = hits_morph[h].second;
                        w.Trigger(hp, w.Sounding()); h++;
                    }
                    KickVoiceOut o; w.Process(o); y.push_back(o.sub);
                }
                (void)wave; return y;
            };
            vector<float> sine = render(0, SR / 2, {{0, 0.0f}});
            vector<float> saw1 = render(1, SR / 2, {{0, 1.0f}});
            vector<float> saw2 = render(1, SR / 2, {{0, 1.0f}});
            Expect(memcmp(saw1.data(), saw2.data(), saw1.size() * 4) == 0 && saw1[0] == 0.0f,
                   "a fresh supersaw hit is always the same waveform, starting at zero");
            double rs = 0, rw = 0;
            for(int n = SR / 10; n < SR / 2; n++){ rs += sine[n] * sine[n]; rw += saw1[n] * saw1[n]; }
            double level_db = 10 * log10(rw / rs);
            double hf_sine = BandPeakDb(sine, 1000, SR / 10, SR / 2), hf_saw = BandPeakDb(saw1, 1000, SR / 10, SR / 2);
            printf("WAVE 127 vs 0: level %+.1f dB, energy above 1 kHz %.1f dB vs %.1f dB\n", level_db, hf_saw, hf_sine);
            Expect(fabs(level_db) < 3.0, "full supersaw sits within 3 dB of the sine's level");
            /*
             * Bass: the supersaw's fundamental must add to the sine's, not
             * cancel it, and must not wander (phasing) over the hit. Measured
             * as the fundamental's level in consecutive 50 ms windows.
             */
            auto fundamental = [&](const vector<float>& y, int a, int b){
                double re = 0, im = 0;
                for(int n = a; n < b; n++){ double w = 2 * M_PI * 55.0 * n / 48000.0; re += y[n] * cos(w); im += y[n] * sin(w); }
                return 2.0 * sqrt(re * re + im * im) / (b - a);
            };
            double lo_saw = 1e9, hi_saw = 0, lo_sine = 1e9;
            for(int w0 = SR / 10; w0 + 2400 <= SR / 2; w0 += 2400){
                double fs = fundamental(saw1, w0, w0 + 2400), fn = fundamental(sine, w0, w0 + 2400);
                lo_saw = fmin(lo_saw, fs / fn); hi_saw = fmax(hi_saw, fs / fn); lo_sine = fmin(lo_sine, fn);
            }
            printf("supersaw fundamental vs the sine's: %.2f .. %.2f over the hit\n", lo_saw, hi_saw);
            Expect(lo_saw > 0.89, "the supersaw keeps the sine's bass (fundamental within 1 dB or more)");
            Expect(hi_saw / lo_saw < 1.12, "the supersaw's fundamental does not phase over the hit (< 1 dB swing)");
            /*
             * Locked to the sub: over a held tail each body harmonic keeps
             * its level, as a sine's distortion harmonics do. Windows of
             * exactly 11 cycles (9600 samples at 55 Hz) make each harmonic's
             * DFT bin leakage-free. Saws detuned in Hz swept these through
             * ~30 dB nulls. The top is allowed to shimmer; it must still move,
             * or the spread has been lost.
             */
            {
                vector<float> held = render(1, SR * 2, {{0, 1.0f}});
                auto harmonic = [&](int k, int a){
                    double re = 0, im = 0;
                    for(int n = a; n < a + 9600; n++){ double w = 2 * M_PI * 55.0 * k * n / 48000.0; re += held[n] * cos(w); im += held[n] * sin(w); }
                    return sqrt(re * re + im * im);
                };
                auto swing_db = [&](int k){
                    double lo = 1e30, hi = 0;
                    for(int a = SR / 10; a + 9600 <= 2 * SR; a += 2400){ double m = harmonic(k, a); lo = fmin(lo, m); hi = fmax(hi, m); }
                    return 20 * log10(hi / lo);
                };
                double body = 0;
                for(int k = 2; k <= 8; k++) body = fmax(body, swing_db(k));
                double top = 0;
                for(int k = 40; k <= 48; k++) top = fmax(top, swing_db(k));
                printf("supersaw over a held tail: body harmonics 2-8 swing %.2f dB, top 40-48 %.1f dB\n", body, top);
                Expect(body < 1.0, "the supersaw's body harmonics stay locked to the sub (< 1 dB swing)");
                Expect(top > 3.0, "the supersaw's top still shimmers");
            }
            /*
             * Deterministic: a hit landing on a still-sounding kick must play
             * exactly like a fresh one once the handoff has glided the saws
             * onto the new hit's wobble.
             */
            {
                vector<float> fresh = render(1, SR, {{0, 1.0f}});
                vector<float> again = render(1, SR / 3 + SR, {{0, 1.0f}, {SR / 3, 1.0f}});
                float diff = 0;
                for(int n = MsToSamples(RETRIGGER_HANDOFF_MS) + 1; n < SR; n++)
                    diff = fmaxf(diff, fabsf(fresh[n] - again[SR / 3 + n]));
                printf("supersaw: a retriggered hit vs a fresh one after the handoff: max diff %g\n", diff);
                Expect(diff <= 1e-5f, "a retriggered supersaw hit matches a fresh one after the handoff");
            }
            Expect(hf_saw > hf_sine + 30.0, "the supersaw adds the harmonics a sine does not have");
            /* Morph changed between two hits of a sounding kick: no step at the seam. */
            vector<float> seam = render(1, SR / 2, {{0, 0.0f}, {SR / 4, 1.0f}});
            float step = 0; for(int n = SR / 4 - 2; n < SR / 4 + 4; n++) step = fmaxf(step, fabsf(seam[n] - seam[n - 1]));
            float slope = 0; for(int n = SR / 4 - 200; n < SR / 4 - 2; n++) slope = fmaxf(slope, fabsf(seam[n] - seam[n - 1]));
            printf("morph 0 -> 127 across a retrigger: seam step %.4f vs the sine's own slope %.4f\n", step, slope);
            Expect(step <= slope * 1.5f, "changing WAVE between retriggered hits does not step the waveform");
        }

        KickVoice a, b; KickHitParams h1; h1.punch = 0.5f; KickHitParams h2 = h1; h2.sweep_time_scale = 2.0f;
        a.Trigger(h1, false); b.Trigger(h2, false);
        Expect(b.anatomy_end == 2 * a.anatomy_end || b.anatomy_end == 2 * a.anatomy_end + 1,
               "SWEEP TIME 2x doubles the punch window");
        Expect(fabsf(logf(b.sweep_coefficient) * 2.0f - logf(a.sweep_coefficient)) < 1e-6f,
               "SWEEP TIME 2x halves the sweep's decay rate");

        KickVoice c; KickHitParams h3; h3.sub_attack_ms = 1.0f; c.Trigger(h3, false);
        KickVoice d; KickHitParams h4; h4.sub_attack_ms = 500.0f; d.Trigger(h4, false);
        Expect(c.gap_rise == MsToSamples(TAIL_ATTACK_MIN_MS) && d.gap_rise == MsToSamples(TAIL_ATTACK_MAX_MS),
               "TAIL ATTACK (the gap's rise) is held to 6..60 ms");

        /* TAIL DELAY: the whole kick at full through the punch, silent, back at full. */
        {
            KickHitParams g; g.frequency = 55; g.punch = 0.5f; g.decay_seconds = 1000000.0f;
            g.delay_ms = 250.0f; g.sub_attack_ms = 20.0f;
            KickVoice v; v.Trigger(g, false);
            float before = 1, during = 0, after = 0;
            for(uint32_t n = 0; n < MsToSamples(400); n++){
                float lv = v.GapLevel();
                if(n < v.gap_hold) before = fminf(before, lv);
                if(n > v.gap_hold + v.gap_fall && n < v.gap_return) during = fmaxf(during, lv);
                if(n > v.gap_return + v.gap_rise) after = fmaxf(after, lv);
                KickVoiceOut o; v.Process(o);
            }
            printf("TAIL DELAY gap: %.2f through the punch, %.2f in the gap, %.2f after\n", before, during, after);
            Expect(before == 1.0f && during == 0.0f && after == 1.0f,
                   "TAIL DELAY holds the whole punch, silences the kick, then brings it back to full");
            KickHitParams early = g; early.delay_ms = 10.0f;
            KickVoice w; w.Trigger(early, false);
            float lowest = 1;
            for(uint32_t n = 0; n < MsToSamples(400); n++){ lowest = fminf(lowest, w.GapLevel()); KickVoiceOut o; w.Process(o); }
            Expect(lowest == 1.0f, "a TAIL DELAY inside the punch never opens a gap");
        }
    }

    /* Determinism. */
    kick_frequency = 55; macro_decay = 0.6f; tail_delay_enabled = true; macro_tail_delay = 0.4f;
    vector<float> a = Run({{0, 90}}, SR, 0.95f);
    vector<float> b = Run({{0, 20}, {7777, 90}}, 7777 + SR, 0.95f);
    vector<float> c = Run({{0, 90}}, SR, 0.95f);
    float diff = 0;
    for(int n = MsToSamples(RETRIGGER_HANDOFF_MS) + 1; n < SR; n++) diff = fmaxf(diff, fabsf(a[n] - b[7777 + n]));
    Expect(diff == 0.0f, "a ratcheted hit is bit-identical to a fresh one once the handoff is over");
    Expect(memcmp(a.data(), c.data(), SR * 4) == 0, "two fresh hits are bit-identical");

    /*
     * Length. (1 - u)^2 lasts exactly DECAY; the old blended decay empties
     * in about 60 % of it. Either way it must land on zero.
     */
    kick_frequency = 55; macro_decay = 0.34f; tail_delay_enabled = false;
    vector<float> d = Run({{0, 64}}, SR, 0.95f);
    int last = 0;
    for(int n = 0; n < SR; n++) if(d[n] != 0.0f) last = n;
    float expected = MacroDecaySeconds(0.34f) * SAMPLE_RATE;
    printf("decay %.0f ms: sound ends at %.1f ms\n", MacroDecaySeconds(0.34f) * 1000, last / 48.0);
    if(KICK_OLD_SUB_DECAY)
        Expect(last > 0.5f * expected && last < 0.7f * expected, "the old sub decay empties in about 60 % of the DECAY time");
    else
        Expect(fabsf(last - expected) < 2.0f, "the sub ends exactly at the DECAY time");
    Expect(fabsf(d[last]) < 1e-5f, "the sub lands on zero rather than stepping off");

    if(wav)
    {
        auto render = [&](const char* name, int vel, float dl, float dc)
        {
            kick_frequency = 55; macro_decay = dc; tail_delay_enabled = dl > 0; macro_tail_delay = dl;
            vector<pair<int, int>> hits;
            for(int i = 0; i < 8; i++) hits.push_back({i * SR / 3, vel});
            WriteWav(name, Run(hits, 3 * SR, 0.95f));
        };
        render("sub_decay200_flat.wav", 64, 0, 0.34f);
        render("sub_decay200_taildelay_max.wav", 64, 1.0f, 0.34f);
        render("sub_decay600_down.wav", 1, 0, 0.6f);
        render("sub_decay600_up.wav", 127, 0, 0.6f);
        render("sub_long_decay_ratchet.wav", 64, 0, 0.9f);
        auto punched = [&](const char* name, float shape, float decay)
        {
            kick_frequency = 55; macro_kick_shape = shape; macro_decay = decay; tail_delay_enabled = false;
            vector<pair<int, int>> hits;
            for(int i = 0; i < 8; i++) hits.push_back({i * SR / 3, 64});
            WriteWav(name, RunFull(hits, 3 * SR, 1.0f, 0.95f).y);
            macro_kick_shape = 0.0f;
        };
        punched("punch_shape064.wav", 0.5f, 0.34f);
        punched("punch_shape127_laser.wav", 1.0f, 0.34f);
        punched("punch_shape064_decay_inf.wav", 0.5f, 0.99f);
    }

    printf("%s\n", fails ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED");
    return fails;
}
