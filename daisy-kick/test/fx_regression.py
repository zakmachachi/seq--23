#!/usr/bin/env python3
"""Compile actual firmware DSP sections on the host; no hardware SDK required."""
from pathlib import Path
import os
import re
import subprocess
import sys
import tempfile

source = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[1] / 'midi_oled_monitor.cpp'
s = source.read_text()
def block(pattern):
    match = re.search(pattern, s)
    if not match:
        raise RuntimeError('Missing DSP definition: ' + pattern)
    start = match.start()
    brace = s.index('{', match.end() - 1)
    depth = 1
    end = brace + 1
    while depth:
        depth += (s[end] == '{') - (s[end] == '}')
        end += 1
    return s[start:end]

def function(name):
    return block(r'(?:static\s+)?(?:inline\s+)?float\s+' + name + r'\s*\([^;{]*\)\s*\{')

constants = ['SAMPLE_RATE', 'TWO_PI', 'PI', 'REVERB_SEND_HP_POLE_A',
    'REVERB_SEND_LEVEL', 'REVERB_RETURN_LEVEL', 'REVERB_FEEDBACK',
    'REVERB_AMOUNT_MAX', 'REVERB_DAMPING', 'REVERB_DUCK_DEPTH',
    'REVERB_DUCK_RECOVER_A', 'REVERB_DUCK_CUT_STEP',
    'PERFORMANCE_FILTER_DRY_HOLD_MS', 'PERFORMANCE_FILTER_FADE_IN_MS',
    'PER_HIT_GATE_CLOSE_MS', 'PER_HIT_GATE_CLOSE_STEP', 'DJ_FILTER_COEFF_UPDATE_SAMPLES']
code = '#include <cmath>\n#include <cstdint>\n#include <cstdio>\n#include <cassert>\n'
for name in constants:
    code += re.search(r'static constexpr (?:float|uint32_t)\s+' + name + r'\s*=[\s\S]*?;', s).group() + '\n'
code += '''static uint32_t kick_age_samples = 0;
static bool PERF_DJ_HPF_ENABLED = false;
static float macro_fx_value_hpf = 0, macro_fx_value_lpf = 0;
'''
for name in ['ClampAdded', 'Clamp01Added', 'SmoothstepAdded', 'CloseGateWithoutStepping', 'PerformanceFilterKickOnsetBlend']:
    code += function(name) + '\n'
for name in ['KickSidechainReverb', 'AddedSmoothWet', 'AddedDjHighpass', 'MacroDjLowpass']:
    code += block(r'struct\s+' + name + r'\s*\{') + ';\n'
code += '''static MacroDjLowpass macro_dj_lowpass;
struct PassThrough { float Process(float x, const void*) { return x; } };
struct MasterPath {
    PassThrough stutter;
    AddedDjHighpass dj_hpf;
'''
# Exercise the actual aggregator route, replacing only the unrelated stutter.
code += function('ProcessMaster') + '\n};\n'
code += r'''
static int failures = 0;
void check(bool ok, const char* message) {
    if (!ok) { std::printf("FAIL: %s\n",message); ++failures; }
}
float rms(MasterPath& path, float hz, float amount) {
    macro_fx_value_lpf = amount;
    double energy = 0;
    for (int n=0;n<48000;++n) {
        float in=.4f*std::sin(TWO_PI*hz*n/SAMPLE_RATE);
        float out=path.ProcessMaster(in);
        check(std::isfinite(out),"LPF finite output");
        if (n>=24000) energy += out*out;
    }
    return std::sqrt(energy/24000);
}
int main() {
    // A repeated kick resets the age but must not switch the live HPF path
    // to dry in one sample. Compare twin states with identical audio input.
    float worst_hpf = 0;
    for (float amount : {1.f/127.f, 2.f/127.f, .5f, 1.f}) {
        AddedDjHighpass filter;
        filter.Reset(); filter.position=amount;
        macro_fx_value_hpf=amount; PERF_DJ_HPF_ENABLED=true;
        for (int n=0;n<24000;++n) {
            kick_age_samples=10000+n;
            filter.Process(.65f*std::sin(TWO_PI*73*n/SAMPLE_RATE),true);
        }
        for (int phase=0;phase<64;++phase) {
            AddedDjHighpass retrigger=filter, reference=filter;
            float in=.65f*std::sin(TWO_PI*phase/64.f);
            kick_age_samples=10000;
            float continued=reference.Process(in,true);
            kick_age_samples=0;
            float triggered=retrigger.Process(in,true);
            float error=std::fabs(triggered-continued);
            if(error>worst_hpf)worst_hpf=error;
            check(error<1.e-6f,"HPF retrigger has no one-sample dry/wet step");
        }
    }
    std::printf("HPF trigger discontinuity: %.9f\n",worst_hpf);

    // Reverb amount edges (including the lowest nonzero MIDI position)
    // and trigger ducking cannot change the gain of the current sample.
    KickSidechainReverb verb;
    verb.Reset();
    for(int n=0;n<48000;++n)
        verb.Process(.6f*std::sin(TWO_PI*379*n/SAMPLE_RATE),REVERB_AMOUNT_MAX);
    float worst_reverb=0;
    for(float amount : {0.f, REVERB_AMOUNT_MAX/127.f, REVERB_AMOUNT_MAX}) {
        KickSidechainReverb changed=verb, reference=verb;
        float expected=reference.Process(.17f,REVERB_AMOUNT_MAX);
        float actual=changed.Process(.17f,amount);
        float error=std::fabs(actual-expected);
        if(error>worst_reverb)worst_reverb=error;
        check(error<1.e-6f,"Reverb amount edge has no one-sample gain step");
    }
    {
        KickSidechainReverb triggered=verb,reference=verb;
        triggered.Trigger();
        check(std::fabs(triggered.Process(.17f,REVERB_AMOUNT_MAX)-
            reference.Process(.17f,REVERB_AMOUNT_MAX))<1.e-6f,
            "Reverb trigger starts duck without an instantaneous gain step");
    }
    // A muted tank must advance, not freeze and replay old wet samples.
    int index=verb.ci0;
    verb.Process(0,0);
    check(verb.ci0==(index+1)%KickSidechainReverb::C0,"Muted reverb tank keeps advancing");
    for(int n=0;n<96000;++n) {
        if(n%6000==0)verb.Trigger();
        float out=verb.Process(n%6000<100 ? .4f : 0.f,
                              n%12000<6000 ? REVERB_AMOUNT_MAX/127.f : 0.f);
        check(std::isfinite(out)&&std::fabs(out)<2.f,"Reverb finite/bounded during rapid changes");
    }
    std::printf("Reverb control discontinuity: %.9f\n",worst_reverb);

    PERF_DJ_HPF_ENABLED=false; macro_fx_value_hpf=0;
    MasterPath master;
    macro_dj_lowpass.Reset();
    float bypass=rms(master,2000,0);
    float closed=rms(master,2000,1);
    float bass=rms(master,50,1);
    float db=20*std::log10(closed/bypass);
    std::printf("Kick LPF at maximum: 2 kHz %.2f dB, 50 Hz RMS %.5f\n",db,bass);
    check(db < -35.f,"CC34 reaches the kick output and attenuates 2 kHz");
    check(bass>.2f,"LPF retains low-frequency kick energy");
    macro_dj_lowpass.Reset();
    macro_fx_value_lpf=0;
    check(master.ProcessMaster(.123f)==.123f,"LPF zero is exact bypass");
    // Independent LPF instances cannot contaminate the other output bus.
    MacroDjLowpass external;
    external.Reset();macro_fx_value_lpf=1;
    for(int i=0;i<2000;++i) {
        master.ProcessMaster(.4f);
        check(external.Process(0)==0,"Kick filter has no external-bus leakage");
    }
    if(failures) {std::printf("%d regression failures\n",failures);return 1;}
    std::puts("PASS: DSP transitions, muted-tank continuity, LPF routing/response and bus isolation");
}
'''
code = code.replace('#include <cassert>', '#include <cassert>\n#include <initializer_list>')
with tempfile.TemporaryDirectory(prefix='daisy-fx-test-') as directory:
    cpp = Path(directory)/'test.cpp'
    binary = Path(directory)/'test'
    cpp.write_text(code)
    subprocess.run([os.environ.get('CXX','c++'), '-std=c++17', '-O2', '-fsanitize=address,undefined', str(cpp), '-o', str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
