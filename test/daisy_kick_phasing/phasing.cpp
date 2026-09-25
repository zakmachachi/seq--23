/*
 * Supersaw phasing, measured in the full firmware.
 *
 * Compiles daisy-kick/midi_oled_monitor.cpp against the host stubs (as the
 * kick_host harness does), runs its init path, plays kicks over MIDI and
 * renders the kick output one sample per audio callback, logging the
 * oscillator's phase with every sample.
 *
 * Phasing is a harmonic swelling and dipping against the sub. So over the
 * tail, in windows of 4 whole oscillator cycles, each harmonic k is
 * projected onto the logged phase (k x phase): its level relative to the
 * fundamental, in dB, tracks the sub through any sweep, PITCH glide or TAIL
 * MOD wobble. Its swing is the peak-to-peak around a quadratic trend, so a
 * tail's slow change of tone (a model's harmonics falling with its level) is
 * not counted. Swings are of bands of three neighbouring harmonics; bands
 * and harmonics under -60 dB are skipped.
 *
 * Also: every retriggered kick against the one before it, after the 80 ms
 * handoff (should be identical) and inside it (the handoff's own variation).
 *
 *   phasing <key=value ...>      prints one line of numbers:
 *   band_swing_2_8 band_swing_7_16 single_harmonic_swing after_handoff_db inside_handoff_db
 *
 * Keys as the kick_host harness: line mackamt tubeamt model mackie tube sub
 * punch decay shape wave tailmod sweeptime layers bpf1 taildelay vel note
 * hits spacing_ms jitter_ms seed tail_ms.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cmath>
#include <random>
#include <vector>

#define main firmware_main
#include FIRMWARE
#undef main

namespace daisy { uint32_t System::now_ms = 0; }
GPIO_TypeDef*  GPIOC_storage_target = nullptr;
static GPIO_TypeDef  gpioc_storage{};
static USART_TypeDef usart3_storage{};
GPIO_TypeDef*  GPIOC  = &gpioc_storage;
USART_TypeDef* USART3 = &usart3_storage;

using std::vector;

static vector<float> g_out, g_phase;
static uint64_t g_n = 0;

static void Render(size_t samples)
{
    float in_l[1] = {}, in_r[1] = {}, out_l[1], out_r[1];
    float* out[2] = {out_l, out_r};
    const float* in[2] = {in_l, in_r};
    for(size_t i = 0; i < samples; i++)
    {
        hw.callback(in, out, 1);
        g_out.push_back(out_l[0]);
        g_phase.push_back(kick_voice.last_phase);
        g_n++;
        daisy::System::now_ms = (uint32_t)(g_n * 1000ull / 48000ull);
    }
}

static void Midi3(uint8_t a, uint8_t b, uint8_t c)
{ ProcessMidiByte(a); ProcessMidiByte(b); ProcessMidiByte(c); }

static int g_argc; static char** g_argv;
/* The last value given wins, so a setting can override the baseline. */
static double Arg(const char* key, double fallback)
{
    size_t k = strlen(key);
    for(int i = g_argc - 1; i >= 1; i--)
        if(strncmp(g_argv[i], key, k) == 0 && g_argv[i][k] == '=')
            return atof(g_argv[i] + k + 1);
    return fallback;
}

/* Least-squares quadratic trend removed; peak-to-peak of what is left. */
static double DetrendedSwing(const vector<double>& y)
{
    size_t n = y.size();
    if(n < 4) return 0.0;
    double s[5] = {}, t[3] = {};
    for(size_t i = 0; i < n; i++)
    {
        double x = (double)i / (double)(n - 1), p = 1;
        for(int j = 0; j < 5; j++) { s[j] += p; if(j < 3) t[j] += p * y[i]; p *= x; }
    }
    double m[3][4] = {{s[0], s[1], s[2], t[0]}, {s[1], s[2], s[3], t[1]}, {s[2], s[3], s[4], t[2]}};
    for(int c = 0; c < 3; c++)
        for(int r = 0; r < 3; r++)
            if(r != c)
            {
                double f = m[r][c] / m[c][c];
                for(int j = 0; j < 4; j++) m[r][j] -= f * m[c][j];
            }
    double a = m[0][3] / m[0][0], b = m[1][3] / m[1][1], c2 = m[2][3] / m[2][2];
    double lo = 1e30, hi = -1e30;
    for(size_t i = 0; i < n; i++)
    {
        double x = (double)i / (double)(n - 1), r = y[i] - (a + b * x + c2 * x * x);
        lo = fmin(lo, r); hi = fmax(hi, r);
    }
    return hi - lo;
}

/* Worst swing per harmonic group over [a, b). */
static void TailSwing(size_t a, size_t b, double swing[3])
{
    vector<size_t> starts;
    for(size_t n = a + 1; n < b; n++)
        if(g_phase[n] < g_phase[n - 1] - 0.5f) starts.push_back(n);

    const int K = 16, CYCLES = 4;
    size_t count = starts.size() > CYCLES ? starts.size() - CYCLES : 0;
    vector<vector<double>> amps(count, vector<double>(K + 1));
    double loudest = 0.0;
    for(size_t w = 0; w < count; w++)
    {
        for(int k = 1; k <= K; k++)
        {
            double re = 0, im = 0;
            for(size_t n = starts[w]; n < starts[w + CYCLES]; n++)
            {
                double ang = 2 * M_PI * k * g_phase[n];
                re += g_out[n] * cos(ang); im += g_out[n] * sin(ang);
            }
            amps[w][k] = sqrt(re * re + im * im) + 1e-12;
        }
        loudest = fmax(loudest, amps[w][1]);
    }

    /*
     * Bands of three neighbouring harmonics, about a critical band down here:
     * the ear hears a band's level move, and one harmonic falling into a
     * near-null of a hard-driven model's spectrum carries little energy
     * however far its own dB swings. Per harmonic is kept as a third number.
     */
    const int BANDS = K - 3;   /* bands start at harmonic 2 .. 14 */
    vector<vector<double>> band(BANDS + 2), single(K + 1);
    vector<double> level(K + 1, 0.0);
    int windows = 0;
    for(size_t w = 0; w < count; w++)
    {
        /* Only while the sub sounds: a tail fading into rounding is not phasing. */
        if(amps[w][1] < loudest * 0.01) continue;
        const vector<double>& amp = amps[w];
        double fund = amp[1] * amp[1];
        for(int k = 2; k <= K; k++)
        {
            double db = 20 * log10(amp[k] / amp[1]);
            single[k].push_back(db);
            level[k] += db;
        }
        for(int s = 2; s + 2 <= K; s++)
        {
            double p = amp[s] * amp[s] + amp[s + 1] * amp[s + 1] + amp[s + 2] * amp[s + 2];
            band[s].push_back(10 * log10(p / fund));
        }
        windows++;
    }

    swing[0] = swing[1] = swing[2] = 0.0;
    if(windows <= 3) return;
    for(int s = 2; s + 2 <= K; s++)
    {
        double band_level = -1e30;
        for(double v : band[s]) band_level = fmax(band_level, v);
        if(band_level < -60.0) continue;
        double sw = DetrendedSwing(band[s]);
        if(s + 2 <= 8) swing[0] = fmax(swing[0], sw);   /* body, harmonics 2-8 */
        else swing[1] = fmax(swing[1], sw);            /* harmonics 7-16 */
    }
    for(int k = 2; k <= K; k++)
        if(level[k] / windows > -60.0)
            swing[2] = fmax(swing[2], DetrendedSwing(single[k]));
}

int main(int argc, char** argv)
{
    g_argc = argc; g_argv = argv;
    try { firmware_main(); } catch(const daisy::HostAudioStarted&) {}

    auto cc = [](const char* key, uint8_t number) {
        double v = Arg(key, -1.0);
        if(v >= 0.0) Midi3(0xB0 | MIDI_CHANNEL_KICK, number, (uint8_t)(v * 127.0 + 0.5));
    };
    cc("line", CC_MIX_LINE_GAIN);  cc("mackie", CC_MIX_MACKIE_GAIN);
    cc("tube", CC_MIX_TUBE_GAIN);  cc("bpf", CC_MIX_BPF_GAIN);
    cc("sub", CC_MIX_SUB_GAIN);    cc("punch", CC_MIX_PUNCH_GAIN);
    cc("decay", CC_DECAY_ABSOLUTE); cc("shape", CC_KICK_SHAPE_ABSOLUTE);
    cc("mackamt", CC_MACKIE_AMOUNT); cc("tubeamt", CC_TUBE_AMOUNT);
    cc("model", CC_CHARACTER_MODEL); cc("tailmod", CC_TAIL_MOD);
    cc("wave", CC_WAVE);           cc("sweeptime", CC_PUNCH_SWEEP_TIME);
    cc("layers", CC_BPF_LAYER_COUNT); cc("bpf1", CC_BPF_LAYER1_FREQUENCY);
    cc("taildelay", CC_TAIL_DELAY_ABSOLUTE);
    if(Arg("taildelay", -1.0) >= 0.0) Midi3(0xB0 | MIDI_CHANNEL_KICK, CC_TAIL_DELAY_STATE, 127);

    Render(12000);

    const int hits = (int)Arg("hits", 6);
    const double spacing = Arg("spacing_ms", 461.5);
    const double jitter = Arg("jitter_ms", 0.5);
    const uint8_t note = (uint8_t)Arg("note", 33), vel = (uint8_t)Arg("vel", 64);
    std::mt19937 rng((unsigned)Arg("seed", 1));
    std::uniform_real_distribution<double> u(-jitter, jitter);

    vector<size_t> trig;
    for(int h = 0; h < hits; h++)
    {
        Midi3(0x90 | MIDI_CHANNEL_KICK, note, vel);
        trig.push_back(g_n);
        size_t len = (size_t)((h + 1 < hits ? spacing + u(rng) : Arg("tail_ms", 1500)) * 48.0);
        Render(960);
        Midi3(0x80 | MIDI_CHANNEL_KICK, note, 0);
        Render(len - 960);
    }
    trig.push_back(g_n);

    /* The last kick's tail, from 150 ms to its end. */
    double swing[3];
    TailSwing(trig[hits - 1] + 150 * 48, trig[hits], swing);

    /*
     * Retriggered kicks (3rd on) against the one before: the difference's
     * energy against the kick's own, inside the 80 ms handoff and after it.
     */
    double d_after = 0, e_after = 0, d_inside = 0, e_inside = 0;
    size_t span = (size_t)(spacing * 48.0) - 48 * 2;
    for(int h = 2; h + 1 < hits; h++)
        for(size_t n = 0; n < span; n++)
        {
            double y = g_out[trig[h] + n], d = y - g_out[trig[h - 1] + n];
            if(n >= 90 * 48) { d_after += d * d; e_after += y * y; }
            else { d_inside += d * d; e_inside += y * y; }
        }
    auto db = [](double d, double e) { return d > 0 && e > 0 ? 10 * log10(d / e) : -999.0; };

    printf("%.2f %.2f %.2f %.1f %.1f\n", swing[0], swing[1], swing[2],
           db(d_after, e_after), db(d_inside, e_inside));
    return 0;
}
