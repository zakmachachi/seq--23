> Historical analysis. For the current firmware and upload status, see [channel-chain](../channel-chain/README.md).

> The intermediate shared-clock revision is archived in [../locked/](../locked/README.md).

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
| Firmware G1, medium | -19.85 | 0.11% | 72.30% | 17.83% | -44.28 | -84.14 |
| TR-909 T50 A0 D100 | -12.14 | 0.25% | 82.03% | 46.73% | -23.34 | -72.48 |
| Firmware G1, long | -19.85 | 0.05% | 80.92% | 39.66% | -31.12 | -80.91 |
| D2 clean, SUB 0 | -19.68 | 0.00% | 84.38% | 67.68% | -32.40 | -90.83 |
| D2 clean, SUB 1 | -13.56 | 42.07% | 47.19% | 33.69% | -32.39 | -32.28 |
| D2 Mackie maximum | -3.81 | 7.74% | 8.66% | 6.18% | -32.38 | -32.28 |
| D2 Tube maximum | -9.01 | 11.93% | 13.35% | 9.52% | -32.38 | -32.28 |

Absolute energy change versus D2 clean SUB 1 at identical LINE gain (these values are not normalized per signal):

| Signal | 30-40 Hz | 40-90 Hz | 30-90 Hz |
|---|---:|---:|---:|
| D2 Mackie maximum | +0.0104 dB | +0.0003 dB | +0.0051 dB |
| D2 Tube maximum | +0.0114 dB | -0.0007 dB | +0.0050 dB |

The smaller bass percentages under distortion reflect added harmonic energy, while absolute bass energy stays within 0.012 dB in these cases.

### Remaining bass trough: explicit limitation

The D2 octave pair still has a deep trough between its 36.7 Hz and 73.4 Hz peaks. Levels below use the clean SUB 1 raw spectral maximum, exactly as the plot does.

| Signal | Raw valley | 1/6-octave valley | Smoothed 40-to-50 Hz fall |
|---|---:|---:|---:|
| D2 clean, SUB 1 | 54.93 Hz / -23.21 dB | 54.33 Hz / -22.84 dB | 14.58 dB |
| D2 Mackie maximum | 54.75 Hz / -23.33 dB | 54.33 Hz / -22.91 dB | 14.66 dB |
| D2 Tube maximum | 54.75 Hz / -23.26 dB | 54.33 Hz / -22.90 dB | 14.67 dB |

This does **not** meet a requirement of no 12 dB drop within a 10 Hz span: even the clean D2 has a 14.58 dB smoothed fall from 40 to 50 Hz. The valley is already present before character processing; adding maximum Mackie or Tube leaves it nearly unchanged. It reflects the spectral spacing of the two pitched components and the finite transient/window, rather than an extra null introduced by mixing in the character path. Spectral valley depth and dry/wet phase-cancellation loss are different measurements. These changes preserve existing bass and add an octave; they do not create a flat broadband 40-90 Hz shelf. The actual single-peaked 909 is not a flat shelf either.

D2 is 73.416 Hz, with its optional octave at 36.708 Hz. These two pitched components do not form a flat 30-90 Hz spectrum. SUB 0 leaves the body sounding; SUB 1 fills the octave region. The raw curves retain window-related interference and spectral minima. Strong distortion changes upper harmonics; its bass retention can be evaluated with the shared-gain D2 plots and the absolute metrics, rather than per-trace spectral normalization.

The 909 and firmware share a falling-pitch body, but their amplitude curves and harmonic distribution differ. The reference's late decay accelerates; the firmware has an initial hold followed by exponential decay. The medium/long knob settings below are selected comparisons, not a numerical fit or a claim that knob percentages correspond across instruments. Observe the first 30 ms panel for transient differences and the pitch panel for the remaining trajectory mismatch. The audio reference is one producer-attributed original unit with undisclosed interface, serial number and accent setting.

## Exact settings

All renders: actual `daisy-kick/host/kick_host.cpp` -> firmware `AudioCallback`, 48 kHz; LINE=1, PUNCH=0, filters/reverb/reverse/tail-gap off, CURVE=64/127, BELLY=64/127, sweep-time/depth trims=.55, gate=20 ms. These continuous controls are rounded to MIDI CC values by the host. The host's 250 ms control-settling silence is removed exactly; no per-file time warping or onset optimization is applied.

- G1 clean comparison: MIDI note 31 (48.999 Hz), SHAPE=.87 (~91 ms pitch-envelope time after MIDI quantization), velocity=70 (pitch depth), WAVE=0, SUB=0, both character amounts=0. DECAY=.30 for the medium reference and .50 for the long reference. The core's intentional body harmonics and short pulse/noise attack remain active.
- D2 clean: MIDI note 38 (73.416 Hz), SHAPE=.50, velocity=100, DECAY=.50, WAVE=0; SUB=0 or 1; character amounts=0.
- D2 maximum character: same D2, SUB=1, WAVE=1, selected Mackie/Tube amount=1, both character return faders=1, BPF blend=1/layers=1 (three layers, initialized centers 330/700/1450 Hz); all other effects off. Exact host settings appear in `metrics.json`.
- Tail-gap audition: D2, SUB=1, WAVE=1, Mackie amount=1, TAIL DELAY=.45. This intentionally removes and restores the body; it is shown separately from the maximum-character spectral comparison.

## Analysis and reproducibility

Run `/usr/bin/python2.7 test/daisy_kick_low_end/compare_909.py --reference-dir /path/to/AudioRealism/BassDrum`. Requires numpy and matplotlib; uses Python 2.7 or Python 3. It compiles a fresh host unless `--renderer` is supplied. Firmware SHA256 for this run: `4f5b5344c8c1a6ccbe3f140eb56721ab140c691e15b0d453403ed18034b97304`. Renderer SHA256 and reference file hashes are recorded in `metrics.json`.

The clean comparison uses each trace's own 20 Hz-20 kHz raw spectral maximum; the D2 plot uses the clean SUB 1 maximum for **every** trace, preserving level changes. Spectra cover 0-600 ms with 2 ms cosine edge tapers, calculated at native sample rates, zero-padded to 262144 points. Zero padding samples the transform densely; actual frequency resolution remains set by 600 ms. Faint curves are unsmoothed; thick curves average linear spectral power within full 1/6-octave bands. No EQ, gain matching by frequency, or interpolation over dips is applied. Envelopes use 20 ms RMS windows, plotted at window centers. Pitch uses linearly interpolated positive-going crossings, excludes the noisy attack and low-level tail, and measures cycle averages.

The firmware auditions retain original output levels, with 100 ms silence before the single hit. They are 16-bit mono WAV, 48 kHz. The rendered output is a desktop float model with hardware stubs; this does not measure the Seed codec, output electronics, listening room, speakers or perceived loudness.

## Recording attribution and primary references

- [AudioRealism original TR-909 sample pack](https://audiorealism.se/TR-909-SamplePack.html): individual outputs, part volume 50%, no clipping, 96 kHz capture. `BassDrum909-tune050-attack050-decay050.wav` and `BassDrum909-tune050-attack000-decay100.wav`; verified mono PCM24/96 kHz. The pack's included Readme forbids redistribution of samples whole or in part without permission. Download independently; no analog-reference audio is exported here.
- [Original Roland TR-909 service notes](https://www.polynominal.com/site/studio/gear/drum/roland-tr909/roland-tr909-service-manual.pdf), pp6/9/11: triggered oscillator method, actual centered-knob scope traces, and separate tune/attack/decay paths. Page 7 documents a tune-range capacitor revision; one vintage unit is not a universal waveform target.
- [Official Jomox ModBase 09 MkII manual](https://www.jomox.de/upload/manuals/ModBase09Mk2_E.pdf), pp15-16/19: distinct base pitch and pitch-envelope amount, near-sine body with variable harmonics, pulse/noise attack, decay and smoothing filter. No calibrated ModBase recording was obtained; Jomox validation here is documentary, not quantitative.
