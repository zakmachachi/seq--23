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
 *   model=<0|1>  (CC50: 0 = Mackie, 1 = Tube)   wave=<0..1> (CC64)
 *   taildelay=<0..1>               (K3, CC42; also enables it, CC43)
 *   tail=<ms of silence after the last hit>  vel=<0..127, default 64>
 *   note=<0..127, default 36>   reverse=<0|1> (CC41)
 *   tmod=<0..1> (CC79, 0 = single glide)   belly=<0..1> (CC61, 0.5 ~ neutral)
 *   bitcrush=<0..1> (CC37, dirty return only)
 *   externalhz=<Hz> externallevel=<amplitude> externalout=<path.f32>
 *     Optional external-input sine; output 2 captures the same combined mono mix.
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


static const int    kBlock      = 16;
static const double kSampleRate = 48000.0;

static std::vector<float> g_kick_bus;
static std::vector<float> g_external_bus;
static double             g_external_hz = 60.0;
static double             g_external_level = 0.0;
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
        for(int i = 0; i < kBlock; i++)
            in_l[i] = static_cast<float>(g_external_level * sin(
                6.283185307179586 * g_external_hz *
                (g_samples_rendered + i) / kSampleRate));
        memset(out_l, 0, sizeof(out_l));
        memset(out_r, 0, sizeof(out_r));

        hw.callback(in, out, kBlock);

        for(int i = 0; i < kBlock; i++)
        {
            g_kick_bus.push_back(out_l[i]);
            g_external_bus.push_back(out_r[i]);
        }

        if(getenv("KICK_HOST_PROBE"))
        {
            static int n = 0;
            if((n++ % 250) == 0)
                fprintf(stderr,
                        "t=%7.3f gate=%d voice(act=%d age=%u env=%.4f "
                        "residual=%.6f) sub=%.3f punch=%.3f out=%.6f\n",
                        g_samples_rendered / 48000.0,
                        (int)note_gate,
                        (int)kick_voice.active,
                        (unsigned)kick_voice.age,
                        kick_voice.body_env,
                        kick_output_bridge.residual,
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
    perf_quarter_note_ms = static_cast<float>(60000.0 / bpm);

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
    maybe("layers",  CC_BPF_LAYER_COUNT);
    maybe("bpf1",    CC_BPF_LAYER1_FREQUENCY);
    maybe("bpf2",    CC_BPF_LAYER2_FREQUENCY);
    maybe("bpf3",    CC_BPF_LAYER3_FREQUENCY);
    maybe("sub",     CC_MIX_SUB_GAIN);
    maybe("punch",   CC_MIX_PUNCH_GAIN);
    maybe("decay",   CC_DECAY_ABSOLUTE);
    maybe("shape",   CC_KICK_SHAPE_ABSOLUTE);
    if(ArgOr(argc,argv,"fxroutes",-1)>=0)
        SendCC(MIDI_CHANNEL_KICK,92,static_cast<uint8_t>(ArgOr(argc,argv,"fxroutes",14)));
    maybe("pump", CC_MACRO_FX_PUMP);
    if(ArgOr(argc,argv,"pump",-1)>=0)SendCC(MIDI_CHANNEL_KICK,52,127);
    maybe("delay", CC_MACRO_FX_DELAY);
    maybe("stut", CC_MACRO_FX_STUTTER);
    maybe("loop", CC_MACRO_FX_LOOPER);
    maybe("hpf",     CC_MACRO_FX_HPF);
    maybe("reverb",  CC_MACRO_FX_REVERB);
    maybe("bitcrush", CC_MACRO_FX_BITCRUSH);
    maybe("erosion", CC_EROSION_AMOUNT);
    maybe("erosionfreq", CC_EROSION_FREQUENCY);
    maybe("lpf",     CC_MACRO_FX_LPF);
    maybe("mackamt", CC_MACKIE_AMOUNT);
    maybe("taildelay", CC_TAIL_DELAY_ABSOLUTE);
    if(ArgOr(argc, argv, "taildelay", -1.0) >= 0.0)
        SendCC(MIDI_CHANNEL_KICK, CC_TAIL_DELAY_STATE, 127);
    maybe("tubeamt", CC_TUBE_AMOUNT);
    maybe("model",   CC_CHARACTER_MODEL);
    maybe("wave",    CC_WAVE);
    maybe("reverse", CC_REVERSE_STATE);   /* >= 0.5 = on */
    maybe("tmod",    CC_KICK_TAIL_MOD);
    maybe("belly",   CC_KICK_BELLY);
    maybe("sweeptime", CC_KICK_SWEEP_TIME);
    g_external_hz = ArgOr(argc, argv, "externalhz", 60.0);
    g_external_level = ArgOr(argc, argv, "externallevel", 0.0);

    /* Let the mix-gain slew settle before the first hit. */
    Render(static_cast<size_t>(0.25 * kSampleRate) / kBlock * kBlock);

    const double step_ms = ArgOr(argc, argv, "spacing_ms", 60000.0 / bpm / 4.0);
    const uint8_t note     =
        static_cast<uint8_t>(ArgOr(argc, argv, "note", 36.0));
    const uint8_t velocity =
        static_cast<uint8_t>(ArgOr(argc, argv, "vel", 64.0));

    for(int h = 0; h < hits; h++)
    {
        SendCC(MIDI_CHANNEL_KICK, CC_KICK_TAIL_PITCH, velocity);
        NoteOn(MIDI_CHANNEL_KICK, note, velocity ? velocity : 1);
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

    for(int i = 2; i < argc; i++)
    {
        constexpr const char* prefix = "externalout=";
        if(strncmp(argv[i], prefix, strlen(prefix)) != 0)
            continue;
        FILE* external = fopen(argv[i] + strlen(prefix), "wb");
        if(!external)
        {
            perror("external output fopen");
            return 1;
        }
        fwrite(g_external_bus.data(), sizeof(float), g_external_bus.size(), external);
        fclose(external);
    }

    fprintf(stderr,
            "rendered %zu samples (%.2f s) -> %s\n",
            g_kick_bus.size(),
            g_kick_bus.size() / kSampleRate,
            argv[1]);
    return 0;
}
