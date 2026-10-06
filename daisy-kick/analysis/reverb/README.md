# Sustained, ducked reverb with clocked echo

Built, not uploaded. This supersedes the previous bass-shelf/attack experiment.
The clean bass shelf and its −6.02 dB kick headroom trim remain. The original
pulse/noise attack and 0.5 ms minimum onset are restored; SHAPE zero remains
8 ms. No new tick/whine fix is included. Previous bitcrush DSP remains intact.

The reference is the spacious kick/air relationship of Pilldriver's
[Pitch-Hiker](https://perctrax.bandcamp.com/track/pitch-hiker-original-mix),
not a claim to recreate the record's production chain.

## REVERB macro

- 0%: dry-only, after a 30 ms control smoothing time. Dry gain stays unity.
- 1–65%: increasingly sustained, damped reverb. The four tank feedback gains
  progress from 0.86 to 0.93 over the full knob range (roughly 1–3 s nominal
  low-frequency decay in the combs; actual filtered tail depends on content).
- Above 65%: smooth introduction of dotted-eighth echo (three sixteenths,
  or 0.75 quarter note), reaching full echo contribution at 100%.
- Echo feedback rises from 0.35 to 0.60; the repeats also feed the reverb tank.
  The echo is low-pass damped. This is an additive return, not a dry/wet
  crossfade that removes the original kick.

The previous effect emptied its tank at every hit. This version retains it.
A dry-signal detector ducks only the return (2 ms attack, 100 ms gain recovery),
with a short 15 ms trigger hold to make the initial pump consistent. Sustained
loud dry bass continues to duck the effect; the tail blooms as that dry signal
falls. Neither trigger nor parameter changes clear the buffers.

The original ~300 Hz send high-pass plus a 180 Hz second-order return high-pass
keep reverberation/echo out of the deep bass. No dry-path crossover is added.
The complete generated kick, after its chop gate, feeds this effect. External
input keeps its existing effects and does not enter the reverb.

Clock changes crossfade between fixed fractional read taps over 50 ms using
constant-sum smoothstep weights. A 1% / 0.5 ms hysteresis rejects clock jitter.
This avoids continuously moving the delay head (pitch bends) and equal-power
crossfade boosts. The time range follows the existing 120–1200 ms quarter-note
clock range. If MIDI clock stops, the last tempo remains. No MIDI transport
phase-locked oscillator is involved: each input transient repeats at that
clock-derived interval.

The 192,008-byte float echo buffer uses SDRAM, initialized after hardware init.
No allocations or large buffer clears occur on Note-On. Audio remains mono on
both physical outputs. The Teensy readout shows REV and ECHO percentages; the
existing REVERB CC is unchanged, so an older Teensy still controls it.

## Listen

Identical six-hit D2 patterns at 180 BPM, followed by tail, exported at actual
firmware output level without individual normalization:

- [Dry, 0%](reverb-0.wav)
- [Reverb, 50%](reverb-50.wav)
- [Reverb + emerging echo, 75%](reverb-75.wav)
- [Full reverb + echo, 100%](reverb-100.wav)

## Validation

Production DSP tests cover persistent tails, dry-level ducking, exact settled
zero-amount bypass, 300/450 ms clock-derived delay times, and 30 seconds of
rapid amount/tempo changes. This automation test peaks at 0.15233 with maximum
sample step 0.01174 for a 0.15-amplitude 500 Hz input; sanitizer checks pass.

72 full-bus extreme renders (both distortion models, three shapes/notes, two reverb
amounts, with/without full-level external input) peak at 0.755868, below the
0.93 emergency-ceiling knee. These sampled tests do not prove every setting
or hardware CPU timing; listening on the Seed is still needed.

The standard low-end/MIDI/retrigger suite is retained. Build and validation
fingerprints are in validation.json; raw extreme peaks are in headroom.json.

Reproduce:

```
bash test/daisy_kick_low_end/run.sh
python3 test/daisy_kick_low_end/reverb_render.py HOST_RENDERER OUTPUT_DIRECTORY
make -C daisy-kick -j2
```
