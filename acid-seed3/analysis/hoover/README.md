# Hoover movement study — 8 October 2026

## Downloaded reference

[Hoover Synth by Chameon](https://freesound.org/people/Chameon/sounds/399542/)
is a gabber/hardcore-tagged stereo sound published under CC0. The downloaded
file is Freesound's **HQ MP3 preview**, not the login-only original WAV:
https://cdn.freesound.org/previews/399/399542_6996302-hq.mp3

`reference/chameon-hoover.mp3` preserves that download; the adjacent WAV is
its decoded 44.1 kHz stereo audio. Its duration is 2.0817 seconds. File hashes
and measurements are in `metrics.json`. This is a usable sonic reference,
not verified provenance of a particular Roland unit or Rodox patch.

The manufacturer's [Modor Hoover article and waveform diagrams](https://www.modormusic.com/hoover.html)
were checked separately. The ramp slope stays constant and two windows open
within each period; the pulse rising edge aligns with the second window.
Our previous ramp-compression formula did not implement this geometry.

[AudioRealism's envelope description](https://forum.audiorealism.se/post/synthesis-parameters-11431710)
distinguishes linear first/second segments and an exponential third segment.
[TAL's manual](https://semimonitor.tal-software.com/downloads/docs/TAL-Pha-UserManual.pdf)
also documents both one- and two-octave sub waveform options. The new voice
uses a one-octave divider for its body; that is an explicit voicing decision.

## What the audio analysis establishes

The reference spectrogram shows an upward scoop, a pitch overshoot around
150–200 ms, then a slower settling contour. Harmonic spacing settles around
147 Hz; the absolute source key/octave cannot be established from this alone.
As an example, a strong partial near 790 Hz at 0.20 s is near 584 Hz by
1.20 s: roughly a five-semitone overshoot. Chorus makes individual partial
frequencies fluctuate, so these are approximate readings, not a fitted f0 curve.
The release begins around 1.84 s and falls in pitch as level decays.

At identical macros (88/94/100/0/0/25), held D3 and matched gate duration,
with the revised voice now running at the requested 1.4× movement speed:

| Actual oscillator control | Previous voice | Revised voice |
|---|---:|---:|
| Pitch peak time | 8 ms | 113 ms |
| Peak above held note | +11.61 st | +5.73 st |
| Pitch at 100 ms | +5.76 st | +3.57 st |
| Pitch at 500 ms | -2.65 st | +0.01 st |
| Pitch at 1 s | approximately 0 st | approximately 0 st |

The old movement was fundamentally different: almost immediate upward jump,
downward undershoot, then return. The new contour has an extended rise, overshoots,
settles from above and drops on release. VCA attack remains fast independently
of the longer pitch attack. The revised oscillator uses the correct interrupted
ramp/pulse relationship, rather than merely changing the old envelope numbers.

Chorus is faster/deeper and its delay positions now move every sample.
Previously they jumped at the control-block boundary. Six offset taps retain
a centre dry path; the existing slow opposite stereo band-pass motion remains
separate from this much faster ensemble movement.

## Listen / inspect

- `reference-before-after.wav`: reference → old → revised, with 250 ms gaps.
  All three have the same stereo RMS, without limiting. Reference starts at
  0 s, old at 2.332 s, revised at 4.682 s. Only this listening comparison
  resamples the preview to 48 kHz with linear interpolation.
- `after-held.wav`: revised voice at its actual output level.
- `comparison.png`: RMS envelopes and level-matched spectrograms. Each
  spectrogram uses its file's original sample rate, 2048-sample windows and
  128-sample hops. They show movement; they are not proof of sonic identity.
- `pitch-contours.png`: actual oscillator control trajectories, not audio
  pitch-estimator output. CSVs include oscillator pitch, VCA and PWM/delay state.
- `../../hoover_movement_demo.wav`: longer notes followed by shorter,
  progressively more aggressive phrases from the firmware DSP. This newer demo
  monitors the mono Audio Out 1 signal in both ears. The isolated reference
  study above retains the internal stereo signal.

`Hoover-before.h` and `before-demo.wav` preserve the previous implementation
and supplied demo. `study_hoover.cpp` renders the isolated voice;
`study_hoover.py` rebuilds both comparison renders. `analyse.py` uses
numpy/scipy/matplotlib to generate the figures, metrics and listening WAV.

This addresses measured contour and oscillator errors. It is not a claim that
the reference has been matched by ear, or that one sample establishes the
exact Alpha Juno/Rodox sound. The new spectral balance and ensemble differ
from the reference; subsequent listening remains the test of the target character.
