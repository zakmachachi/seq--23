# SCREECH — resettable performance voice

A third audio machine in the existing synth Seed project, MIDI channel 14,
Audio Out 1. Menu 4 knob 1 press cycles NONE / ACID / HOOVER / SCREECH;
pattern-machine selection remains independent. Kick firmware is separate.

## Controls on Menu 2

| Knob | Macro | Range / role |
| --- | --- | --- |
| 1 | LENGTH | 75 ms–2.4 s shot length; MIDI note-off releases earlier |
| 2 | SWEEP | falling to rising, ±36-semitone total travel; 64 is stationary |
| 3 | METAL | FM ratio and depth; press to cycle the six styles |
| 4 | VOWEL | paired resonant band centres and emphasis |
| 5 | ERROR | density/depth of seeded pitch mutations and amplitude breaks |
| 6 | BURN | internal saturation with level compensation |

Function + knob 6 is output trim. Knob 2 press retains the existing Tube/Mackie
selection for the synth FX send. Existing synth FX are available separately.
Voice macros and style are latched at each note-on; trim is smoothed live.
The primary display names the style; the secondary display shows the focused
macro and the knob-3 click hint. Knob numbers are physical, starting at one.

## Styles

- SQUEAL: curved pitch gesture with dense, moving FM sidebands.
- ERROR: stepped pitch trajectory and discrete modulator ratios.
- CHATTER: repeatable amplitude chops, with rounded transitions.
- LASER: fast exponential pitch trajectory and a more tonal core.
- VOWEL: saw/FM excitation of sweeping paired resonant bands.
- STATIC: seeded noise/tonal excitation through the same tuned bands.

CC90–95 are the macros; CC96 is level; CC97 selects style 0–5.
CC119=64 selects SCREECH; Acid=0 and Hoover=127 retain their existing values.
Every screech note retriggers, including tied sequencer notes. Velocity changes
amplitude. Oscillators, filter memories, envelopes and PRNG reset per shot.
An outgoing shot fades for 4 ms while the new shot attacks. Extremely rapid
retriggering can replace an already retiring shot; this is a bounded two-shot
voice, not an unlimited polyphonic engine.

Two-times oversampling, pitch-dependent FM depth bounds and a four-pole
anti-alias low-pass reduce harsh digital folding. They do not make nonlinear FM
mathematically alias-free. Gain compensation, DC removal, a rounded 4 ms attack
and 18 ms end taper manage output transitions. The shared output limiter remains
in place. Reverb/delay tails and a retiring shot deliberately retain their history:
repeatability applies to each new dry shot, not to arbitrary accumulated FX tails.

## Reference study and limits

Source: user-provided `TaMaHaTa - In The Pudding.wav`, 451.024 s, stereo 44.1 kHz.
This implementation is informed by spectral inspection; no isolated stems or
original patches are available and no exact patch/artist match is claimed.
The full-track plot covers 0–451 s; the detail covers 336–352 s. Both show
120 Hz (bottom) to 12 kHz (top), logarithmically; brighter pixels mean greater
magnitude. 2048-sample Hann FFTs, full-track snapshots every 1/3 s, detail every
10 ms. Sparse full-track sampling is insufficient to measure individual clicks.

The additional intro plot covers 0–16 s at 10 ms spacing. It shows repeated
pulsed clusters alongside a narrow upper band near 5.3 kHz and occasional
higher rising traces. These motivate separating tonal, noisy and chopped
styles instead of treating all the sounds as a single continuous sweep.

The full mix contains repeated curved low/mid trajectories and intermittent
upper-frequency bursts. Around 343–345 s sampled strongest mid/high bins shift
into roughly 6.9–7.2 kHz, interleaved with strong ~1.7–1.8 kHz material elsewhere
in the detail. These are full-mix measurements, not isolated screech fundamentals.
The design uses multiple excitation/gesture families rather than claiming one
oscillator topology explains every sound in the track.

## Listening and checks

`screech_demo.wav`: six four-second sections in style order above; four pitches
per style, shared FX off, no file normalization. Audio Out 1 duplicated to stereo
for listening. This is a first audition, not an established sonic match.

Build the host test/render with:

```
c++ -O2 -std=c++14 tests/screech.cpp -o /tmp/screech-check
/tmp/screech-check screech_demo.wav
```

Checks cover all 64 macro endpoints in each style, bit-identical repeated isolated
shots, finite samples, note-off release, MIDI routing, engine selection and demo
output bounds. Board CPU deadlines and musical listening still require hardware.

Latest host run: all 384 style/corner combinations passed; maximum dry peak
0.5817, demo peak 0.4425, zero output safety clamps. Existing Acid artifact
and Hoover/routing checks also passed. Synth Seed build: 96,324 bytes flash;
Teensy build succeeded. No boards uploaded.
