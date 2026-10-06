# Clean analog-reference and D2 bass comparison

Actual firmware host renders are compared with separately downloaded, attributed original TR-909 recordings. This is an engineering comparison, not a claim of circuit-level emulation. Reference WAVs are **not included**. Only the four firmware audition WAVs are exported here.

## Results

![Time, envelope, pitch](909_time_pitch.png)
![Clean spectra](909_spectrum.png)
![D2 full-LINE low end](d2_low_end.png)
![D2 output and tail gap](d2_envelope.png)

Energy percentages use the same first 600 ms, integrated relative to each signal's 20 Hz-20 kHz energy. They describe energy distribution, not frequency-response flatness. Projection columns measure a Hann-weighted sinusoidal component over 80-280 ms; the reference's slightly drifting pitch is not an exact musical note. Raw dBFS across hardware recording and firmware have different gains and must not be read as absolute loudness comparisons.

| Signal | Peak dBFS | 30-40 Hz energy | 40-90 Hz energy | Body-root 1/6-octave energy | Body projection dBFS | Octave projection dBFS |
|---|---:|---:|---:|---:|---:|---:|
| TR-909 T50 A50 D50 | -10.76 | 0.52% | 75.80% | 28.07% | -33.52 | -72.90 |
| Firmware G1, medium | -21.32 | 0.14% | 91.81% | 44.96% | -34.16 | -71.08 |
| TR-909 T50 A0 D100 | -12.14 | 0.25% | 82.03% | 46.73% | -23.34 | -72.48 |
| Firmware G1, long | -21.32 | 0.04% | 93.51% | 60.62% | -27.73 | -75.14 |
| D2 clean, SUB 0 | -21.62 | 0.00% | 85.17% | 65.60% | -28.32 | -87.07 |
| D2 clean, SUB 1 | -15.60 | 31.00% | 55.38% | 27.11% | -28.33 | -28.24 |
| D2 Mackie maximum | -7.77 | 9.75% | 17.73% | 8.70% | -28.24 | -28.24 |
| D2 Tube maximum | -11.11 | 12.54% | 22.47% | 10.97% | -28.32 | -28.24 |

Absolute energy change versus D2 clean SUB 1 at identical LINE gain (these values are not normalized per signal):

| Signal | 30-40 Hz | 40-90 Hz | 30-90 Hz |
|---|---:|---:|---:|
| D2 Mackie maximum | +0.0028 dB | +0.0812 dB | +0.0532 dB |
| D2 Tube maximum | +0.0021 dB | +0.0172 dB | +0.0118 dB |

These character examples also change WAVE to 1; the table measures the complete preset change, not distortion alone.

### Bass trough measurements

The octave pair settles at 36.7 and 73.4 Hz. The table measures the region between those tones against the common clean SUB 1 raw spectral maximum.

| Signal | Raw valley | 1/6-octave valley | Smoothed 40-to-50 Hz fall |
|---|---:|---:|---:|
| D2 clean, SUB 1 | 58.59 Hz / -15.32 dB | 57.86 Hz / -15.16 dB | 9.10 dB |
| D2 Mackie maximum | 58.41 Hz / -15.32 dB | 57.86 Hz / -15.12 dB | 9.06 dB |
| D2 Tube maximum | 58.41 Hz / -15.31 dB | 58.22 Hz / -15.13 dB | 9.04 dB |

These pitched signals do not form a flat broadband bass shelf. A spectral valley is not itself evidence of destructive mixing: compare isolated paths and phase traces as well. Raw and smoothed results are both retained; no minima are interpolated away.

D2 settles at 73.416 Hz, with its optional octave at 36.708 Hz. The octave and body now use one phase clock, so both sweep together. Clean/dirty lanes use matched 180 Hz fourth-order Linkwitz-Riley filters, with SUB excluded from distortion. PUNCH is the clean lane gain; mixer2 sums external FX and uses gentle continuous bus compression with 0.8 output trim. These two pitched components do not form a flat 30-90 Hz spectrum. SUB 0 leaves the body sounding; SUB 1 fills the octave region. The raw curves retain window-related interference and spectral minima. Strong distortion changes upper harmonics; its bass retention can be evaluated with the shared-gain D2 plots and the absolute metrics, rather than per-trace spectral normalization.

The 909 and firmware share a falling-pitch body, but their amplitude curves and harmonic distribution differ. The reference's late decay accelerates; the firmware holds through pitch settling plus one sub cycle, followed by exponential decay. The medium/long knob settings below are selected comparisons, not a numerical fit or a claim that knob percentages correspond across instruments. Observe the first 30 ms panel for transient differences and the pitch panel for the remaining trajectory mismatch. The audio reference is one producer-attributed original unit with undisclosed interface, serial number and accent setting.

## Exact settings

All renders: actual `daisy-kick/host/kick_host.cpp` -> firmware `AudioCallback`, 48 kHz; LINE=1, PUNCH=1, performance filters/reverb/reverse/tail-gap off (the fixed crossover remains active), CURVE=64/127, BELLY=64/127, sweep-time/depth trims=.55, gate=20 ms. These continuous controls are rounded to MIDI CC values by the host. The host's 250 ms control-settling silence is removed exactly; no per-file time warping or onset optimization is applied.

- G1 clean comparison: MIDI note 31 (48.999 Hz), SHAPE=.50 (~90 ms pitch-envelope time after MIDI quantization), velocity=70 (pitch depth), WAVE=0, SUB=0, both character amounts=0. DECAY=.30 for the medium reference and .50 for the long reference. The core's intentional body harmonics and short pulse/noise attack remain active.
- D2 clean: MIDI note 38 (73.416 Hz), SHAPE=.50, velocity=100, DECAY=.50, WAVE=0; SUB=0 or 1; character amounts=0.
- D2 maximum character: same D2, SUB=1, WAVE=1, selected Mackie/Tube amount=1, both character return faders=1, BPF blend=1/layers=1 (three layers, initialized centers 330/700/1450 Hz); all other effects off. Exact host settings appear in `metrics.json`.
- Tail-gap audition: D2, SUB=1, WAVE=1, Mackie amount=1, TAIL DELAY=.45. This intentionally removes and restores the body; it is shown separately from the maximum-character spectral comparison.

## Analysis and reproducibility

Run `/usr/bin/python2.7 test/daisy_kick_low_end/compare_909.py --reference-dir /path/to/AudioRealism/BassDrum`. Requires numpy and matplotlib; uses Python 2.7 or Python 3. It compiles a fresh host unless `--renderer` is supplied. Firmware SHA256 for this run: `4b676988dbeea3eb4e661013a04c74f44e057f00a08652bd8a1c7c6e0bef0027`. Renderer SHA256 and reference file hashes are recorded in `metrics.json`.

The clean comparison uses each trace's own 20 Hz-20 kHz raw spectral maximum; the D2 plot uses the clean SUB 1 maximum for **every** trace, preserving level changes. Spectra cover 0-600 ms with 2 ms cosine edge tapers, calculated at native sample rates, zero-padded to 262144 points. Zero padding samples the transform densely; actual frequency resolution remains set by 600 ms. Faint curves are unsmoothed; thick curves average linear spectral power within full 1/6-octave bands. No EQ, gain matching by frequency, or interpolation over dips is applied. Envelopes use 20 ms RMS windows, plotted at window centers. Pitch uses linearly interpolated positive-going crossings, excludes the noisy attack and low-level tail, and measures cycle averages.

The firmware auditions retain original output levels, with 100 ms silence before the single hit. They are 16-bit mono WAV, 48 kHz. The rendered output is a desktop float model with hardware stubs; this does not measure the Seed codec, output electronics, listening room, speakers or perceived loudness.

## Recording attribution and primary references

- [AudioRealism original TR-909 sample pack](https://audiorealism.se/TR-909-SamplePack.html): individual outputs, part volume 50%, no clipping, 96 kHz capture. `BassDrum909-tune050-attack050-decay050.wav` and `BassDrum909-tune050-attack000-decay100.wav`; verified mono PCM24/96 kHz. The pack's included Readme forbids redistribution of samples whole or in part without permission. Download independently; no analog-reference audio is exported here.
- [Original Roland TR-909 service notes](https://www.polynominal.com/site/studio/gear/drum/roland-tr909/roland-tr909-service-manual.pdf), pp6/9/11: triggered oscillator method, actual centered-knob scope traces, and separate tune/attack/decay paths. Page 7 documents a tune-range capacitor revision; one vintage unit is not a universal waveform target.
- [Official Jomox ModBase 09 MkII manual](https://www.jomox.de/upload/manuals/ModBase09Mk2_E.pdf), pp15-16/19: distinct base pitch and pitch-envelope amount, near-sine body with variable harmonics, pulse/noise attack, decay and smoothing filter. No calibrated ModBase recording was obtained; Jomox validation here is documentary, not quantitative.
