/*
 * Host test for the Daisy kick voice (daisy-kick/midi_oled_monitor.cpp).
 *
 * run.sh cuts the KickVoice block out of the firmware source verbatim and
 * includes it here with stubs for the few firmware helpers it calls, so this
 * exercises the real code.
 *
 * The click measure is spectral: a 55 Hz sub has no business above 1 kHz,
 * so the peak of a 4th-order 1 kHz high-pass of the output, relative to the
 * sub's own peak, is how loud any click is. Every scenario must stay below
 * CLICK_LIMIT_DB, and a control proves the measure catches a real click.
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

/* Peak above 1 kHz relative to the signal peak, in dB. */
static double ClickDb(const vector<float>& y)
{
    Biquad a(1000, 0.54119610), b(1000, 1.30656296);
    double peak = 1e-12, hf = 0;
    /* Renders may end mid-note; only the render itself is analysed. */
    for(float s : y) { peak = fmax(peak, fabs(s)); hf = fmax(hf, fabs(b.Process(a.Process(s)))); }
    return 20 * log10(fmax(hf, 1e-12) / peak);
}

/* The firmware's trigger + per-sample loop. */
static vector<float> Run(const vector<pair<int, int>>& hits, int length, float gain)
{
    KickVoice voice;
    vector<float> y;
    size_t h = 0;
    for(int n = 0; n < length; n++)
    {
        while(h < hits.size() && hits[h].first == n)
        {
            float delay = (tail_delay_enabled && macro_tail_delay > 0.005f) ? MacroTailDelayMs(macro_tail_delay) : 0.0f;
            voice.Trigger(kick_frequency, (uint8_t)hits[h].second, delay, MacroDecaySeconds(macro_decay));
            h++;
        }
        y.push_back(voice.Process(gain));
    }
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
        double db = ClickDb(Run(hits, 2 * SR, 0.95f));
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

    /* Determinism. */
    kick_frequency = 55; macro_decay = 0.6f; tail_delay_enabled = true; macro_tail_delay = 0.4f;
    vector<float> a = Run({{0, 90}}, SR, 0.95f);
    vector<float> b = Run({{0, 20}, {7777, 90}}, 7777 + SR, 0.95f);
    vector<float> c = Run({{0, 90}}, SR, 0.95f);
    float diff = 0;
    for(int n = MsToSamples(RETRIGGER_HANDOFF_MS) + 1; n < SR; n++) diff = fmaxf(diff, fabsf(a[n] - b[7777 + n]));
    Expect(diff == 0.0f, "a ratcheted hit is bit-identical to a fresh one once the handoff is over");
    Expect(memcmp(a.data(), c.data(), SR * 4) == 0, "two fresh hits are bit-identical");

    /* Length: the sub lasts exactly DECAY after its start, and ends at zero. */
    kick_frequency = 55; macro_decay = 0.34f; tail_delay_enabled = false;
    vector<float> d = Run({{0, 64}}, SR, 0.95f);
    int last = 0;
    for(int n = 0; n < SR; n++) if(d[n] != 0.0f) last = n;
    float expected = MacroDecaySeconds(0.34f) * SAMPLE_RATE;
    printf("decay %.0f ms: sound ends at %.1f ms\n", MacroDecaySeconds(0.34f) * 1000, last / 48.0);
    Expect(fabsf(last - expected) < 2.0f, "the sub ends exactly at the DECAY time");

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
    }

    printf("%s\n", fails ? "SOME CHECKS FAILED" : "ALL CHECKS PASSED");
    return fails;
}
