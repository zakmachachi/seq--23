"""Regressions on the real firmware's two physical outputs (stdlib only).

The bass shelf stays on the clean path and never enters character processing.
Tests observe the actual dirty return, use low LINE for spectral work, and
compare the complete waveform at two LINE settings to expose clipping.
"""

import array
import cmath
import math
import os
from itertools import product
from pathlib import Path
import subprocess
import sys

BINARY, WORK = sys.argv[1], Path(sys.argv[2])
RATE = 48000
BASE = dict(note=38, vel=64, tmod=0, sub=1, punch=1, taildelay=0, shape=0, wave=0,
            decay=1, line=25 / 127, mackamt=0, tubeamt=0, mackie=1, tube=1,
            bpf=0, hits=1, tail=650, spacing_ms=100)
failures = []


def render(renderer=BINARY, **settings):
    output = WORK / "output.f32"
    args = {**BASE, **settings}
    result = subprocess.run(
        [str(renderer), str(output)] + [f"{k}={v}" for k, v in args.items()],
        capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(result.stderr)
    samples = array.array("f")
    samples.frombytes(output.read_bytes())
    samples = samples[RATE // 4:]  # Harness control-settling silence.
    if not samples or not all(math.isfinite(v) and abs(v) <= .986 for v in samples):
        raise AssertionError(f"non-finite/unbounded audio: {settings}")
    return samples


def amplitude(samples, hz, start=.18, end=.58):
    segment = samples[round(start * RATE):round(end * RATE)]
    n = len(segment)
    rotation = cmath.exp(-2j * math.pi * hz / RATE)
    phase, total, weights = 1 + 0j, 0j, 0.0
    for i, value in enumerate(segment):
        weight = .5 - .5 * math.cos(2 * math.pi * i / (n - 1))
        total += value * weight * phase
        weights += weight
        phase *= rotation
    return abs(total) * 2 / weights


def difference(a, b):
    assert len(a) == len(b)
    return [x - y for x, y in zip(a, b)]


def db_ratio(a, b):
    return 20 * math.log10(max(a, 1e-12) / max(b, 1e-12))


def check(ok, message):
    if not ok:
        failures.append(message)


def pitch_from_crossings(samples, start=.2, end=.6):
    crossings = [i - 1 - samples[i - 1] / (samples[i] - samples[i - 1])
                 for i in range(round(start * RATE), round(end * RATE))
                 if samples[i - 1] < 0 <= samples[i]]
    if len(crossings) < 3:
        return 0.0
    return RATE * (len(crossings) - 1) / (crossings[-1] - crossings[0])


# Observe the real post-filter dirty return before it meets the clean sub.
# Only this temporary host build routes that local value to output 2; the
# production source and the generated-kick output are unchanged. A high-pass
# cannot remove distortion intermodulation caused by feeding the octave into
# the clipper, so output-band levels alone are insufficient to test routing.
def sub_isolation_regression():
    root = Path(__file__).resolve().parents[2]
    firmware = (root / "daisy-kick/midi_oled_monitor.cpp").read_text()
    marker = "        out[EXTERNAL_OUTPUT_CHANNEL][i] = mixed_output;"
    if firmware.count(marker) != 1:
        raise RuntimeError("Cannot locate external output for host-only wet observer")
    observed_source = WORK / "sub-isolation-firmware.cpp"
    observed_source.write_text(firmware.replace(marker,
        "        out[EXTERNAL_OUTPUT_CHANNEL][i] = wet;"))
    harness = (root / "daisy-kick/host/kick_host.cpp").read_text()
    include = '#include "../midi_oled_monitor.cpp"'
    if harness.count(include) != 1:
        raise RuntimeError("Cannot locate firmware include for host-only wet observer")
    observed_host = WORK / "sub-isolation-host.cpp"
    observed_host.write_text(harness.replace(include, f'#include "{observed_source}"'))
    observed_binary = WORK / "sub-isolation-render"
    subprocess.run([os.environ.get("CXX", "c++"), "-std=c++17", "-O2", "-w",
                    "-fsanitize=address,undefined", "-I",
                    str(root / "daisy-kick/host/stubs"), "-I", str(root / "daisy-kick"), str(observed_host),
                    "-o", str(observed_binary)], check=True)

    wet_path = WORK / "observed-wet.f32"
    clean_subs = {}
    worst_error, worst_relative, cases, wet_mismatches = 0.0, 0.0, 0, 0
    for model, shape, amount, wave, layers, crush in product(
            (0, 1), (0, .5, 1), (.3, 1), (0, 1), (0, 1), (0,1)):
        settings = dict(model=model, shape=shape, wave=wave, bpf=1, bitcrush=crush,
                        layers=layers, bpf1=.15, bpf2=.5, bpf3=.85,
                        punch=1, line=8 / 127)
        clean_key = (shape, wave, layers)
        if clean_key not in clean_subs:
            clean_subs[clean_key] = difference(render(**settings, sub=1),
                                               render(**settings, sub=0))
        clean_sub = clean_subs[clean_key]
        settings["tubeamt" if model else "mackamt"] = amount
        with_sub = render(renderer=observed_binary, **settings, sub=1,
                          externalout=wet_path)
        wet_with_sub = wet_path.read_bytes()
        without_sub = render(renderer=observed_binary, **settings, sub=0,
                             externalout=wet_path)
        identical = wet_with_sub == wet_path.read_bytes()
        wet_mismatches += not identical
        check(identical,
              f"SUB changed the dirty return: {settings}")
        muted_punch = dict(settings, punch=0)
        render(renderer=observed_binary, **muted_punch, sub=1, externalout=wet_path)
        check(wet_with_sub == wet_path.read_bytes(), f"PUNCH changed dirty drive: {settings}")
        observed_wet = array.array("f")
        observed_wet.frombytes(wet_with_sub)
        check(all(math.isfinite(v) for v in observed_wet) and
              max(map(abs, observed_wet)) > 1e-8,
              f"dirty-return isolation check had invalid/silent character audio: {settings}")
        error = max(abs((on - off) - clean)
                    for on, off, clean in zip(with_sub, without_sub, clean_sub))
        peak = max(map(abs, clean_sub))
        worst_error = max(worst_error, error)
        worst_relative = max(worst_relative, error / max(peak, 1e-12))
        # The final 15 Hz float biquad sees a different summed input with
        # drive enabled. Bound its accumulated rounding at the same 0.3%
        # tolerance as the WAVE/sub subtraction below. The wet equality
        # assertion above has no tolerance and directly proves isolation.
        check(error < max(5e-7, peak * .003),
              f"character changed the isolated final sub: error={error:g}, {settings}")
        cases += 1
    print(f"SUB isolation: {cases} cases, {wet_mismatches} dirty-return mismatches; "
          f"worst final double-difference {worst_error:.2g} "
          f"({worst_relative * 100:.3f}% of sub peak)", flush=True)


sub_isolation_regression()

# Bass boost follows the original oscillator; it must not create an octave.
body_hz = 440 * 2 ** ((38 - 69) / 12)
body = render(sub=0)
complete = render(sub=1)
for signal in (body, complete):
    measured = pitch_from_crossings(signal)
    check(abs(measured-body_hz)<.1, f"Bass boost changed tuning: {measured}")
boost = db_ratio(amplitude(complete,body_hz), amplitude(body,body_hz))
check(11 < boost < 15.1, f"Unexpected D2 shelf boost {boost}")
check(amplitude(complete,body_hz/2)<amplitude(complete,body_hz)*.01,
      "Bass shelf introduced an octave component")
print(f"D2 bass shelf: {boost:.3f} dB, original tuning retained", flush=True)

# BITCRUSH cannot touch clean/sub when the dirty fader is off.
assert render(bitcrush=0)==render(bitcrush=1), "bitcrush changed clean audio"

# DECAY no longer waits for SWEEP. Short bass hits must still speak; a
# deliberately long laser with shortest DECAY may end before settling.
for note in (33,38,45):
    short=render(note=note,shape=0,sweeptime=1,decay=0,punch=1,sub=1,vel=64,line=1)
    check(max(map(abs,short[:2500]))>.02, "short bass lost its onset energy")
    # The clean 240 Hz LPF intentionally rejects a laser that ends before
    # reaching bass. Its audible sweep comes through the dirty lane.
    laser=render(note=note,shape=1,sweeptime=1,decay=0,punch=1,sub=1,vel=64,line=1,mackamt=1)
    check(max(map(abs,laser[:8000]))>.001, "short laser lost its onset energy")
    check(max(map(abs,laser[10000:]))<.001, "SWEEP incorrectly stretched amplitude lifetime")

# The post-distortion high-pass prevents both bass cancellation and hidden
# bass gain. Include intermediate drive, both models, WAVE and pitch clamps.
worst = (0.0, None)
for note in (22, 28, 33, 38, 45, 47):
    hz = max(30, min(120, 440 * 2 ** ((note - 69) / 12)))
    for wave in (0, 1):
        clean = amplitude(render(note=note, wave=wave), hz)
        for model in (0, 1):
            for amount in (.1, .5, 1):
                settings = dict(note=note, wave=wave, model=model, bpf=1, layers=1,
                                **{("tubeamt" if model else "mackamt"): amount})
                change = db_ratio(amplitude(render(**settings), hz), clean)
                if abs(change) > abs(worst[0]):
                    worst = (change, settings)
                check(abs(change) <= 1.,
                      f"character changed the boosted body {change:+.2f} dB: {settings}")
print(f"Worst boosted-body change across character settings: {worst[0]:+.3f} dB", flush=True)

# All mixer controls at 100% must leave the transient below the safety ceiling.
# A limiter can pass a simple abs(output)<=1 assertion, so also demand exact
# linear LINE scaling (the low test value is an exact MIDI CC, avoiding 7-bit
# rounding ambiguity). BPF is explicitly at full here, not its BASE zero.
peak_max, linear_error_max = 0.0, 0.0
for model in (0, 1):
    for note in (23, 38, 47):
        for (shape, wave), center in product(((0, 0), (.5, 1), (1, 1)), (0, .5, 1)):
            # Three fully enabled bands at low/middle/high frequency settings.
            settings = dict(model=model, note=note, wave=wave, punch=1,
                            shape=shape, hits=5, mackamt=1, tubeamt=1,
                            mackie=1, tube=1, sub=1, bpf=1, layers=1,
                            bpf1=center, bpf2=center, bpf3=center, vel=127,
                            belly=1, tmod=1, sweeptime=1)
            quiet = render(**settings, line=25 / 127)
            full = render(**settings, line=1)
            peak = max(map(abs, full))
            error = max(abs(loud - soft * 127 / 25)
                        for soft, loud in zip(quiet, full))
            peak_max = max(peak_max, peak)
            linear_error_max = max(linear_error_max, error)
            check(.02 < peak < .93,
                  f"all-max output lacks usable headroom: peak={peak:.5f}, {settings}")
            # Float arithmetic in the 15 Hz high-pass contributes <0.2% peak
            # residual after scaling; saturation would violate the peak bound
            # or create a substantially larger, signal-correlated residual.
            check(error < peak * .002,
                  f"all-max LINE scaling became nonlinear: error={error:g}, {settings}")
print(f"All-max mixer: peak {peak_max:.4f}, worst LINE scaling error {linear_error_max:.2g}", flush=True)

# HPF filters external audio before the shared mixer/glue; no external
# signal means it must leave the generated kick bit-identical.
external_path = WORK / "external.f32"
dry_kick = render()
for position in (.1, .5, 1):
    check(render(hpf=position) == dry_kick, f"HPF {position} changed generated audio")
    ref = render(punch=0, sub=0, externalhz=60, externallevel=.1)
    filtered = render(punch=0, sub=0, hpf=position, externalhz=60,
                      externallevel=.1, externalout=external_path)
    output2 = array.array("f"); output2.frombytes(external_path.read_bytes())
    check(filtered == output2[RATE // 4:], "the two physical outputs differ")
    change = db_ratio(amplitude(filtered, 60), amplitude(ref, 60))
    if position >= .5:
        check(change < -25, f"external HPF ineffective: {change:.2f} dB")
    print(f"External HPF {position}: {change:+.2f} dB; generated kick unchanged", flush=True)
# Shared mixer needs headroom with simultaneous full-level external audio.
combined = render(punch=1, sub=1, line=1, shape=1, wave=1, mackamt=1,
                  externalhz=55, externallevel=1, hits=5)
check(max(map(abs, combined)) < .93, "shared output reached its emergency ceiling")
# PUNCH mutes the clean lane including its bass boost; DIST is independent.
check(max(map(abs, render(punch=0, sub=0))) < 1e-9, "PUNCH zero did not mute clean lane")
check(max(map(abs, render(punch=0, sub=1))) < 1e-9, "PUNCH failed to mute boosted clean lane")
check(max(map(abs, render(punch=0, sub=0, mackamt=1))) > .001, "PUNCH muted dirty lane")

# Tail Delay must make an actual break between the initial kick and its bass
# return, including maximum BELLY. Inspect the physical output: filtering the
# generator alone would leave the dirty filter tail ringing through the gap.
for model in (0, 1):
    for belly in (0, 1):
        for delay, bpm in product((.5, 1), (120, 240)):
            audio = render(shape=.5, punch=1, belly=belly, taildelay=delay,
                           model=model, mackamt=1, tubeamt=1, bpf=1,
                           bpm=bpm, spacing_ms=100, tail=650)
            peak = max(map(abs, audio))
            longest, run, gap_end = 0, 0, 0
            for i in range(round(.015 * RATE), round(.5 * RATE)):
                run = run + 1 if abs(audio[i]) < peak * 1e-6 else 0
                if run > longest:
                    longest, gap_end = run, i
            gap_ms = longest / 48
            check(gap_ms > 20,
                  f"TAIL DELAY has no real silent gap: model={model}, belly={belly}, amount={delay}, bpm={bpm}")
            before = audio[:max(1, gap_end - longest)]
            after = audio[gap_end + round(.006 * RATE):gap_end + round(.080 * RATE)]
            check(max(map(abs, before)) > peak * .3,
                  "TAIL DELAY removed the initial kick")
            check(after and max(map(abs, after)) > peak * .08,
                  "TAIL DELAY never brought the bass back")
            print(f"Tail gap: model {model}, BELLY {belly}, amount {delay}, BPM {bpm}: {gap_ms:.1f} ms", flush=True)

# Retrigger at several unrelated points in the old waveform. After the 2 ms
# bridge, a reset hit must equal a fresh hit, including dirty/filter histories.
# 77/107/137 ms are exact multiples of the host's 8-sample audio block.
worst_reset_error = 0.0
for shape, wave, model, amount in ((0, 0, 0, 0), (.5, 1, 0, 1), (1, 1, 1, 1)):
    settings = dict(shape=shape, wave=wave, model=model, punch=1,
                    mackamt=amount, tubeamt=amount, bpf=1, gate=5)
    fresh = render(**settings, hits=1)
    fresh_peak = max(map(abs, fresh))
    check(abs(fresh[0]) < 1e-7, "a fresh kick did not begin at zero")
    for spacing in (77, 107, 137):
        again = render(**settings, hits=2, spacing_ms=spacing)
        offset = spacing * 48
        finish = min(len(fresh), len(again) - offset)
        error = max(abs(fresh[i] - again[offset + i])
                    for i in range(480, finish))  # 10 ms, beyond even the bass-endpoint bridge.
        worst_reset_error = max(worst_reset_error, error)
        check(error < 3e-6,
              f"retrigger retained old-hit state: shape={shape}, model={model}, spacing={spacing}, error={error:g}")
        # At the trigger boundary, the new output must continue from the old
        # value/slope. This specifically catches a hard reset to phase zero.
        seam_step = abs(again[offset] - again[offset - 1])
        prior_slope = abs(again[offset - 1] - again[offset - 2])
        check(seam_step <= prior_slope * 1.2 + fresh_peak * 1e-4,
              f"retrigger stepped the waveform: step={seam_step:g}, prior slope={prior_slope:g}")
print(f"Retrigger after bridge: worst fresh-hit difference {worst_reset_error:.2g}", flush=True)

# Repeating exactly the same event sequence must reproduce every output bit.
settings = dict(shape=1, wave=1, model=1, tubeamt=1, bpf=1,
                punch=1, hits=5, spacing_ms=31, gate=5)
check(render(**settings) == render(**settings),
      "identical MIDI/control sequences did not produce identical output")

for failure in failures:
    print("FAIL:", failure)
if failures:
    raise SystemExit(f"{len(failures)} low-end checks failed")
print("All low-end checks passed")
