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
 *   mackie=<0..1>  tube=<0..1>  bpf=<0..1>  bpfhz=<0..1>
 *   hpf=<0..1>   lpf=<0..1>     (DJ filter position; hpf enables itself)
 *   mackamt=<0..1>  tubeamt=<0..1> (K5 character amount, CC48 / CC49)
 *   model=<0|1>  (CC50: 0 = Mackie, 1 = Tube)   wave=<0..1> (CC64)
 *   taildelay=<0..1>               (K3, CC42; also enables it, CC43)
 *   tail=<ms of silence after the last hit>  vel=<0..127, default 64>
 *   note=<0..127, default 36>   reverse=<0|1> (CC41)
 *   tmod=<0..1> (CC79, 0 = single glide)   belly=<0..1> (CC61, 0.5 ~ neutral)
 *   bitcrush=<0..1> (CC37, dirty return only)
 *   extkick=<0|1> (CC59, external input uses the kick processing path)
 *   externalhz=<Hz> externallevel=<amplitude> externalout=<path.f32>
 *   externalfile=<PCM WAV> (played once after the 250 ms settle)
 *     Optional external-input sine; output 2 captures the same combined mono mix.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

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
static std::vector<float> g_external_file;
static uint32_t           g_external_file_rate = 0;
static uint16_t           g_external_file_channels = 0;
static uint64_t           g_samples_rendered = 0;
static uint8_t            g_held_fx = 0;

static uint32_t ReadLe32(const uint8_t* p)
{
    return uint32_t(p[0]) | (uint32_t(p[1])<<8) | (uint32_t(p[2])<<16) | (uint32_t(p[3])<<24);
}

static bool LoadExternalWav(const char* path)
{
    std::ifstream f(path,std::ios::binary);
    uint8_t head[12];
    if(!f.read(reinterpret_cast<char*>(head),12) || memcmp(head,"RIFF",4) || memcmp(head+8,"WAVE",4))return false;
    uint16_t format=0,bits=0;
    uint32_t data_size=0,data_offset=0;
    while(f)
    {
        uint8_t chunk[8];
        if(!f.read(reinterpret_cast<char*>(chunk),8))break;
        uint32_t size=ReadLe32(chunk+4);
        std::streamoff body=f.tellg();
        if(!memcmp(chunk,"fmt ",4))
        {
            uint8_t fmt[16];
            if(size<16 || !f.read(reinterpret_cast<char*>(fmt),16))return false;
            format=uint16_t(fmt[0])|(uint16_t(fmt[1])<<8);
            g_external_file_channels=uint16_t(fmt[2])|(uint16_t(fmt[3])<<8);
            g_external_file_rate=ReadLe32(fmt+4);
            bits=uint16_t(fmt[14])|(uint16_t(fmt[15])<<8);
        }
        else if(!memcmp(chunk,"data",4)){data_size=size;data_offset=uint32_t(body);break;}
        f.seekg(body+size+(size&1));
    }
    if(format!=1 || (bits!=16 && bits!=24 && bits!=32) || !g_external_file_rate ||
       !g_external_file_channels || !data_size || !data_offset)return false;
    const unsigned bytes=bits/8;
    std::vector<uint8_t> raw(data_size);
    f.clear();f.seekg(data_offset);
    if(!f.read(reinterpret_cast<char*>(raw.data()),data_size))return false;
    const size_t frames=data_size/(bytes*g_external_file_channels);
    g_external_file.resize(frames);
    for(size_t n=0;n<frames;++n)
    {
        double sum=0;
        for(unsigned c=0;c<g_external_file_channels;++c)
        {
            const uint8_t* p=raw.data()+(n*g_external_file_channels+c)*bytes;
            int32_t sample=0;
            if(bits==16)sample=int16_t(uint16_t(p[0])|(uint16_t(p[1])<<8));
            else if(bits==24){sample=int32_t(p[0])|(int32_t(p[1])<<8)|(int32_t(p[2])<<16);if(sample&0x800000)sample|=int32_t(0xff000000);}
            else sample=int32_t(ReadLe32(p));
            sum+=bits==16?sample/32768.0:bits==24?sample/8388608.0:sample/2147483648.0;
        }
        g_external_file[n]=float(sum/g_external_file_channels);
    }
    return true;
}


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
        {
            if(!g_external_file.empty())
            {
                const uint64_t start=uint64_t(.25*kSampleRate);
                const uint64_t absolute=g_samples_rendered+i;
                const double pos=absolute<start?0.0:(absolute-start)*double(g_external_file_rate)/kSampleRate;
                const size_t index=size_t(pos);
                if(absolute<start || index>=g_external_file.size())in_l[i]=0.f;
                else
                {
                    const size_t next=index+1<g_external_file.size()?index+1:index;
                    const float frac=float(pos-index);
                    in_l[i]=g_external_file[index]+(g_external_file[next]-g_external_file[index])*frac;
                }
            }
            else in_l[i] = static_cast<float>(g_external_level * sin(
                6.283185307179586 * g_external_hz *
                (g_samples_rendered + i) / kSampleRate));
        }
        memset(out_l, 0, sizeof(out_l));
        memset(out_r, 0, sizeof(out_r));

        if(g_held_fx && g_samples_rendered%9600==0){
            ProcessMidiByte(0xBE);ProcessMidiByte(heldfx::KEEPALIVE);ProcessMidiByte(127);
        }
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

    /* Macros are sent only when named, keeping default raw renders clean. */
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

    g_held_fx=static_cast<uint8_t>(ArgOr(argc,argv,"heldfx",0));
    if(g_held_fx)SendCC(MIDI_CHANNEL_KICK,heldfx::SELECT,g_held_fx);
    maybe("line",    CC_MIX_LINE_GAIN); // retired in raw path; accepted for older render commands
    maybe("mackie",  CC_MACKIE_AMOUNT);
    maybe("tube",    CC_TUBE_AMOUNT);
    maybe("bpf",     CC_BPF_LAYER_COUNT);
    maybe("layers",  CC_BPF_LAYER_COUNT); // compatibility alias: 0..1 maps OFF..FULL
    maybe("bpfhz",   CC_BPF_LAYER1_FREQUENCY);
    maybe("bpf1",    CC_BPF_LAYER1_FREQUENCY);
    maybe("bpf2",    CC_BPF_LAYER2_FREQUENCY);
    maybe("bpf3",    CC_BPF_LAYER3_FREQUENCY);
    maybe("sub",     CC_MIX_SUB_GAIN);
    maybe("punch",   CC_MIX_PUNCH_GAIN);
    if(ArgOr(argc,argv,"extkick",-1)>=0)
        SendCC(MIDI_CHANNEL_KICK,CC_MIX_EXT_KICK_MODE,
               ArgOr(argc,argv,"extkick",0)>=0.5?127:0);
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
    for(int i=2;i<argc;++i)
        if(strncmp(argv[i],"externalfile=",13)==0 && !LoadExternalWav(argv[i]+13))
        {fprintf(stderr,"failed to read external WAV: %s\n",argv[i]+13);return 2;}

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
