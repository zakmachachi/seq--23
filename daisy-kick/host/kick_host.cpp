/*
 * Host harness for the Daisy kick engine.
 *
 * Compiles midi_oled_monitor.cpp on the desktop against stub hardware, runs
 * the firmware's own init path, feeds it real MIDI bytes and renders the kick
 * bus to a raw float32 file. Lets the retrigger click be measured rather than
 * reasoned about.
 *
 *   usage: kick_host <out.f32> [key=value ...]
 *
 *   bpm=185  hits=4  decay=<0..1>  sub=<0..1>  punch=<0..1>
 *   line=<0..1>  mackie=<0..1>  tube=<0..1>  bpf=<0..1>
 *   hpf=<0..1>   lpf=<0..1>     (DJ filter position; hpf enables itself)
 *   mackamt=<0..1>  tubeamt=<0..1> (K5 character amount, CC48 / CC49)
 *   model=<0|1>                    (K5 model, CC50: 0 Mackie, 1 tube)
 *   taildelay=<0..1>               (K3, CC42; also enables it, CC43)
 *   tailattack sweeptime tailmod wave=<0..1>   (v1.3.0, CC61..64)
 *   layers bpf1=<0..1>             (BPF layer count CC47, layer 1 frequency CC44)
 *   tail=<ms of silence after the last hit>  vel=<1..127, default 100>
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

/* The firmware's main() is the init path we want; rename it out of the way. */
#define main firmware_main
#include "../midi_oled_monitor.cpp"
#undef main

namespace daisy
{
uint32_t System::now_ms = 0;
}

GPIO_TypeDef*  GPIOC_storage_target = nullptr;
static GPIO_TypeDef  gpioc_storage{};
static USART_TypeDef usart3_storage{};
GPIO_TypeDef*  GPIOC  = &gpioc_storage;
USART_TypeDef* USART3 = &usart3_storage;


static const int    kBlock      = 8;
static const double kSampleRate = 48000.0;

static std::vector<float> g_kick_bus;
static uint64_t           g_samples_rendered = 0;


/* Render n samples through the firmware's audio callback. */
static void Render(size_t samples)
{
    float  in_l[kBlock] = {};
    float  in_r[kBlock] = {};
    float  out_l[kBlock];
    float  out_r[kBlock];
    float* out[2]       = {out_l, out_r};
    const float* in[2]  = {in_l, in_r};

    size_t done = 0;
    while(done < samples)
    {
        memset(out_l, 0, sizeof(out_l));
        memset(out_r, 0, sizeof(out_r));

        hw.callback(in, out, kBlock);

        for(int i = 0; i < kBlock; i++)
            g_kick_bus.push_back(out_l[i]);

        if(getenv("KICK_HOST_PROBE"))
        {
            static int n = 0;
            if((n++ % 250) == 0)
                fprintf(stderr,
                        "t=%7.3f gate=%d voice(act=%d age=%u level=%.4f "
                        "handoff=%d) sub=%.3f punch=%.3f out=%.6f\n",
                        g_samples_rendered / 48000.0,
                        (int)note_gate,
                        (int)kick_voice.active,
                        (unsigned)kick_voice.age,
                        kick_voice.last_level,
                        (int)kick_voice.handoff,
                        param_sub_gain,
                        param_punch_gain,
                        out_l[0]);
        }

        g_samples_rendered += kBlock;
        done += kBlock;

        daisy::System::now_ms =
            static_cast<uint32_t>(g_samples_rendered * 1000ull / 48000ull);
    }
}


static void SendCC(uint8_t channel, uint8_t cc, uint8_t value)
{
    ProcessMidiByte(static_cast<uint8_t>(0xB0 | channel));
    ProcessMidiByte(cc);
    ProcessMidiByte(value);
}


static void NoteOn(uint8_t channel, uint8_t note, uint8_t velocity)
{
    ProcessMidiByte(static_cast<uint8_t>(0x90 | channel));
    ProcessMidiByte(note);
    ProcessMidiByte(velocity);
}


static void NoteOff(uint8_t channel, uint8_t note)
{
    ProcessMidiByte(static_cast<uint8_t>(0x80 | channel));
    ProcessMidiByte(note);
    ProcessMidiByte(0);
}


static double ArgOr(int argc, char** argv, const char* key, double fallback)
{
    size_t klen = strlen(key);
    for(int i = 2; i < argc; i++)
    {
        if(strncmp(argv[i], key, klen) == 0 && argv[i][klen] == '=')
            return atof(argv[i] + klen + 1);
    }
    return fallback;
}


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        fprintf(stderr, "usage: kick_host <out.f32> [key=value ...]\n");
        return 2;
    }

    try
    {
        firmware_main();
    }
    catch(const daisy::HostAudioStarted&)
    {
        /* Init complete, audio callback captured. */
    }

    if(!hw.callback)
    {
        fprintf(stderr, "firmware never called StartAudio\n");
        return 1;
    }

    const double bpm      = ArgOr(argc, argv, "bpm", 185.0);
    const int    hits     = static_cast<int>(ArgOr(argc, argv, "hits", 4));
    const double tail_ms  = ArgOr(argc, argv, "tail", 600.0);
    const double note_ms  = ArgOr(argc, argv, "gate", 20.0);

    /* Mix: sub only unless asked otherwise, which is the reported case. */
    auto cc7 = [](double v) {
        return static_cast<uint8_t>(v * 127.0 + 0.5);
    };

    /* Only CCs actually named on the command line are sent, so anything
     * left out keeps the firmware's own power-on default. LINE is the
     * master output level, not a mix lane - zeroing it mutes everything. */
    auto maybe = [&](const char* key, uint8_t cc) {
        double v = ArgOr(argc, argv, key, -1.0);
        if(v >= 0.0)
            SendCC(MIDI_CHANNEL_KICK, cc, cc7(v));
    };

    maybe("line",    CC_MIX_LINE_GAIN);
    maybe("mackie",  CC_MIX_MACKIE_GAIN);
    maybe("tube",    CC_MIX_TUBE_GAIN);
    maybe("bpf",     CC_MIX_BPF_GAIN);
    maybe("sub",     CC_MIX_SUB_GAIN);
    maybe("punch",   CC_MIX_PUNCH_GAIN);
    maybe("decay",   CC_DECAY_ABSOLUTE);
    maybe("shape",   CC_KICK_SHAPE_ABSOLUTE);
    maybe("hpf",     CC_MACRO_FX_HPF);
    maybe("lpf",     CC_MACRO_FX_LPF);
    maybe("mackamt", CC_MACKIE_AMOUNT);
    maybe("tailattack", CC_TAIL_ATTACK);
    maybe("sweeptime", CC_PUNCH_SWEEP_TIME);
    maybe("tailmod", CC_TAIL_MOD);
    maybe("wave", CC_WAVE);
    maybe("layers", CC_BPF_LAYER_COUNT);   /* 0 / 0.33 / 0.67 / 1 = 0..3 layers */
    maybe("bpf1", CC_BPF_LAYER1_FREQUENCY);
    maybe("taildelay", CC_TAIL_DELAY_ABSOLUTE);
    if(ArgOr(argc, argv, "taildelay", -1.0) >= 0.0)
        SendCC(MIDI_CHANNEL_KICK, CC_TAIL_DELAY_STATE, 127);
    maybe("tubeamt", CC_TUBE_AMOUNT);
    maybe("model",   CC_CHARACTER_MODEL);

    /* Let the mix-gain slew settle before the first hit. */
    Render(static_cast<size_t>(0.25 * kSampleRate) / kBlock * kBlock);

    const double step_ms = 60000.0 / bpm / 4.0; /* sixteenths */
    const uint8_t note   = 36;
    const uint8_t velocity =
        static_cast<uint8_t>(ArgOr(argc, argv, "vel", 100.0));

    for(int h = 0; h < hits; h++)
    {
        NoteOn(MIDI_CHANNEL_KICK, note, velocity);
        Render(static_cast<size_t>(note_ms * kSampleRate / 1000.0) / kBlock
               * kBlock);
        NoteOff(MIDI_CHANNEL_KICK, note);
        Render(static_cast<size_t>((step_ms - note_ms) * kSampleRate / 1000.0)
               / kBlock * kBlock);
    }

    Render(static_cast<size_t>(tail_ms * kSampleRate / 1000.0) / kBlock
           * kBlock);

    FILE* f = fopen(argv[1], "wb");
    if(!f)
    {
        perror("fopen");
        return 1;
    }
    fwrite(g_kick_bus.data(), sizeof(float), g_kick_bus.size(), f);
    fclose(f);

    fprintf(stderr,
            "rendered %zu samples (%.2f s) -> %s\n",
            g_kick_bus.size(),
            g_kick_bus.size() / kSampleRate,
            argv[1]);
    return 0;
}
