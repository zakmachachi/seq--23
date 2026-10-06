// Analysis-only observer of the real firmware's dry/wet sum. audit.py injects
// ObserveKickMix() into a temporary source copy; the Seed build has no hooks.
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
static void ObserveKickMix(float dry, float wet, float phase);
#define main firmware_main
#include FIRMWARE
#undef main

namespace daisy { uint32_t System::now_ms = 0; }
GPIO_TypeDef* GPIOC_storage_target = nullptr;
static GPIO_TypeDef gpioc_storage{};
static USART_TypeDef usart3_storage{};
GPIO_TypeDef* GPIOC = &gpioc_storage;
USART_TypeDef* USART3 = &usart3_storage;

struct Sample { float dry, wet, phase, out; };
static std::vector<Sample> samples;
static void ObserveKickMix(float dry, float wet, float phase)
{
    samples.push_back({dry, wet, phase, 0});
}

static void Render(int count)
{
    float in_l[16]{}, in_r[16]{}, out_l[16]{}, out_r[16]{};
    const float* in[] = {in_l, in_r};
    float* out[] = {out_l, out_r};
    for(int n = 0; n < count; n += 16)
    {
        size_t begin = samples.size();
        hw.callback(in, out, 16);
        for(int i = 0; i < 16; ++i) samples[begin + i].out = out_l[i];
        daisy::System::now_ms = samples.size() / 48;
    }
}

static int argc_; static char** argv_;
static double Arg(const char* key, double fallback)
{
    size_t len = strlen(key);
    for(int i = argc_ - 1; i >= 1; --i)
        if(strncmp(argv_[i], key, len) == 0 && argv_[i][len] == '=')
            return atof(argv_[i] + len + 1);
    return fallback;
}

static void Midi(uint8_t status, uint8_t key, uint8_t value)
{ ProcessMidiByte(status); ProcessMidiByte(key); ProcessMidiByte(value); }

int main(int argc, char** argv)
{
    argc_ = argc; argv_ = argv;
    try { firmware_main(); } catch(const daisy::HostAudioStarted&) {}
    auto cc = [](const char* key, uint8_t number, double fallback) {
        Midi(0xB0 | MIDI_CHANNEL_KICK, number,
             static_cast<uint8_t>(Arg(key, fallback) * 127 + .5));
    };
    cc("line", CC_MIX_LINE_GAIN, .12); cc("sub", CC_MIX_SUB_GAIN, 1);
    cc("punch", CC_MIX_PUNCH_GAIN, .5); cc("decay", CC_DECAY_ABSOLUTE, .5);
    cc("shape", CC_KICK_SHAPE_ABSOLUTE, .5); cc("wave", CC_WAVE, 0);
    cc("model", CC_CHARACTER_MODEL, 0);
    cc("mackamt", CC_MACKIE_AMOUNT, Arg("amount", .5));
    cc("tubeamt", CC_TUBE_AMOUNT, Arg("amount", .5));
    cc("mackie", CC_MIX_MACKIE_GAIN, 1); cc("tube", CC_MIX_TUBE_GAIN, 1);
    cc("bpf", CC_MIX_BPF_GAIN, 1); cc("layers", CC_BPF_LAYER_COUNT, 0);
    cc("bpf1", CC_BPF_LAYER1_FREQUENCY, 0);
    cc("bpf2", CC_BPF_LAYER2_FREQUENCY, .3);
    cc("bpf3", CC_BPF_LAYER3_FREQUENCY, .6);
    cc("tmod", CC_KICK_TAIL_MOD, 0); cc("belly", CC_KICK_BELLY, 64. / 127);
    cc("sweeptime", CC_KICK_SWEEP_TIME, 64./127);
    cc("bitcrush", CC_MACRO_FX_BITCRUSH, 0);
    cc("taildelay", CC_TAIL_DELAY_ABSOLUTE, 0);
    Midi(0xB0 | MIDI_CHANNEL_KICK, CC_TAIL_DELAY_STATE, 127);
    Render(12000);
    int start = 0;
    const int hits = static_cast<int>(Arg("hits", 1));
    for(int hit = 0; hit < hits; ++hit)
    {
        start = samples.size();
        Midi(0x90 | MIDI_CHANNEL_KICK, Arg("note", 38), Arg("vel", 64));
        Render(960);
        Midi(0x80 | MIDI_CHANNEL_KICK, Arg("note", 38), 0);
        Render(hit + 1 == hits ? 18240 : 11040);
    }
    const int edges[][2] = {{12, 50}, {50, 150}, {150, 350}};
    for(int window = 0; window < 3; ++window)
    {
        int a = start + 48 * edges[window][0];
        int b = start + 48 * edges[window][1];
        double cycles = 0, peak = 0, rms = 0, weight = 0;
        for(int i = a; i < b; ++i)
        {
            double step = samples[i].phase - samples[i - 1].phase;
            if(step < -.5) step += 1;
            cycles += step;
            peak = fmax(peak, fabs(samples[i].out));
            rms += samples[i].out * samples[i].out;
        }
        for(int h = 1; h <= 3; ++h)
        {
            std::complex<double> dry{}, wet{}, output{};
            weight = 0;
            for(int i = a; i < b; ++i)
            {
                double win = .5 - .5 * cos(2 * M_PI * (i - a) / (b - a - 1));
                auto rot = std::polar(win, -2 * M_PI * h * samples[i].phase);
                dry += double(samples[i].dry) * rot;
                wet += double(samples[i].wet) * rot;
                output += double(samples[i].out) * rot;
                weight += win;
            }
            auto db = [](double v) { return 20 * log10(fmax(v, 1e-12)); };
            double dominant = fmax(abs(dry), abs(wet));
            double loss = db(abs(dry + wet)) - db(dominant);
            double relative_phase = arg(wet * conj(dry)) * 180 / M_PI;
            // window,harmonic,tracked Hz,cycles,dry dB,wet dB,relative phase,
            // sum vs stronger path dB,output harmonic dB,output RMS dB,peak.
            printf("%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f\n",
                window, h, cycles * SAMPLE_RATE / (b - a), cycles,
                db(abs(dry) * 2 / weight), db(abs(wet) * 2 / weight),
                relative_phase, loss, db(abs(output) * 2 / weight),
                db(sqrt(rms / (b - a))), peak);
        }
    }
}
