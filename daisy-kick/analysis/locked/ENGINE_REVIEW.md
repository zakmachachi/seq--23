# Shared-clock kick revision — 2026-10-03

This revision addresses the reported SUB phasing and degraded Mackie sound before uploading to Seed3. Earlier `../coherent/` measurements describe the previous uploaded design.

## Diagnosis

The previous body and sub both reset to phase zero but then used **independent phase increments**. The body swept down while the sub immediately ran at the final octave. Their relation `body_phase - 2*sub_phase` changed through the transient and settled at a control-dependent offset. Resetting both clocks did not lock them. This is deterministic pitch/phase motion, not random oscillator instability. The plots below come from compiling both actual firmware sources.

![Actual phase clocks before and after](oscillator_phase.png)

The previous upload also sent the auxiliary sub through Mackie/Tube. Nonlinear mixing generated new harmonics and intermodulation above the return HPF; high-passing afterward could not undo those interactions. Earlier isolated-return tests found that SUB changed the dirty waveform substantially. The corrected return is bit-identical when SUB changes in all 48 tested model/shape/drive/WAVE/BPF combinations. PUNCH also leaves it bit-identical.

The Mackie transfer function itself had not changed from the pulled Git version. Its input, gain staging and return filtering had changed. The eighth-order 180 Hz Butterworth return rotated upper-bass harmonics enough to cause cancellation with the full-range clean lane. This revision fixes the routing and replaces that arrangement rather than claiming an uncalibrated new clipper is more authentic.

## Implemented routing

- Body plus its subordinate pulse/noise attack → fixed low-pass → PUNCH gain.
- The same body/attack, before PUNCH gain → BPF → Mackie/Tube → fixed high-pass → selected DIST return gain (the scalar gain is applied inside the processor, before its linear return filter).
- Optional octave sine from the shared phase clock → SUB gain. It never feeds BPF or distortion.
- Sum → existing kick performance effects/LINE → tail gate → reverb.
- External input → external performance FX, including HPF → second mixer with the kick.
- Combined mix → fixed 0.8 trim → gentle glue → emergency ceiling → both mono outputs.

The clean LPF and dirty HPF are matched fourth-order Linkwitz-Riley sections at 180 Hz. Identical linear inputs recombine with matching phase and flat magnitude (measured maximum deviation 0.00112 dB). BPF and distortion change harmonic phase, so the actual nonlinear return is not guaranteed to reconstruct a flat signal. Fixed crossover phase is also not a claim that differently pitched waves have identical zero crossings after filtering.

## Oscillators, macros and timing

There is one phase accumulator at half the instantaneous body frequency. `body_phase = fract(2*phase)` generates the body; `sin(2*pi*phase)` supplies the optional octave. The phase relation is exact throughout the sweep, not just at the trigger. It measured zero error across 675 pitch/SHAPE/DEPTH/CURVE/SWEEP combinations. D2 settles at 73.416/36.708 Hz; A1 settles at 55/27.5 Hz, with no sub frequency clamp replacing the requested octave.

A fixed-frequency low sub cannot stay octave-locked to a changing-pitch body. This revision chooses locking: a laser sweeps both. The amplitude holds until pitch is within 5% of the note, plus one complete settled sub cycle, then DECAY begins. This preserves bass duration even at short DECAY; it does not promise 35 Hz from the first sample of a high-pitched laser.

SHAPE zero has no pitch sweep, attack pulse/noise or hidden onset boost. Its zero-slope onset is 8 ms, smoothly reaching 0.5 ms by SHAPE 35%. Pitch depth enters continuously over that same range. Sweep-time anchors are 30/90/115/135 ms at 25/50/75/100%. DEPTH sets excursion, SWEEP TIME trims duration, CURVE (the former TMOD slot) changes the sweep curvature. WAVE deliberately adds harmonics without introducing another phase clock. Bright laser content is attenuated by the clean LPF; the dirty return exposes it.

PUNCH is now a true 0–100% clean-lane fader, independent of SUB and dirty drive. Zero means muted clean lane, not the old “no onset boost” setting. BELLY controls the earliest tail-gate start and does nothing when that gate is off. The gate never begins before the settled bass hold ends. Three-millisecond gate edges chop the kick before reverb; reverb may ring into its gap. There is no sub sidechain or delayed sub onset.

A finite continuity bridge preserves retrigger value/slope while resetting oscillator/filter histories: 8 ms at SHAPE zero, 2 ms in normal kick positions. This avoids hard sample discontinuities; it does not claim that an intentional clipped waveform or very short gate has no high-frequency content. Reverb and shared compressor retain their documented effect history. Identical complete MIDI/control sequences reproduce bit-identical renders.

## Glue, gain and measured output

Shared glue is 1.25:1 above −12 dBFS, with a 30 ms detector attack and 150 ms release, no makeup gain or saturator, and at most 2 dB reduction. It runs continuously across triggers. External level may therefore affect shared gain, as requested by the final-mixer topology. The external HPF never directly filters the generated kick.

LINE is kick trim 0–1; fixed kick gain is 0.12, external gain 0.62 and final mix trim 0.8. DIST returns span 0–1.25. This reserves output headroom; restore level downstream as needed. The normal full-gain grid peaked at 0.4300 versus the emergency ceiling's 0.93 onset, and preserved LINE scaling within float-filter tolerance. A full-level external sine mixed with a driven kick also stayed below that ceiling. These sampled settings do not exhaust every automation sequence or hardware input.

## Frequency and phase results

[909 comparison, spectra and firmware auditions](README.md) use actual firmware renders and separately sourced original TR-909 recordings. The clean pitch fall approaches the reference around 50 Hz; its filtered attack, initial amplitude hold and exponential tail still differ from the analog recording. It is a related drum design, not a circuit-exact 909 model.

In the D2 comparison, SUB adds substantial 30–40 Hz energy. The smoothed 40→50 Hz drop is **9.10 dB**, versus **14.58 dB** in the previous report. The smoothed interpeak minimum is still **−15.16 dB near 57.9 Hz**, relative to the common raw spectral peak. No flat 30–90 Hz shelf is claimed. In particular, tuning a sine pair to 36.7/73.4 Hz does not create an equally strong 30 Hz component. No claim about the user's horn cabinets follows from these electrical renders.

[Interactive phase map](phase/phase-map.html) covers 1,032 settings and three transient/tail windows. Among 3,209 measured 30–90 Hz windows for enabled generated components, worst dry/dirty cancellation is **−0.89 dB**, with none below −1 dB. The raw data retain 23 deeper projection nulls when PUNCH=0: these project a muted body frequency onto a finite-window octave sine and small dirty leakage. The worst is −10.53 dB at an output projection of −74.76 dBFS. They are not silently removed from the data; enabled-component results exclude the muted body projection only. Higher harmonics, very low-level tails and arbitrary settings are not covered by a “no cancellation anywhere” guarantee.

## Validation and provenance

- `bash test/daisy_kick_low_end/run.sh`: production voice/crossover tests and actual AudioCallback MIDI renders, with address/undefined-behaviour sanitizers; all passed. Includes shared phase, SHAPE-zero sweep invariance, SUB/PUNCH isolation, tuning, short-decay settled hold, headroom, mono routing, external HPF, tail gaps and retriggers.
- `bash test/kick_host/run.sh`: controller/mapping/display regression passed.
- `make -C daisy-kick -j2`: ARM build passed; flash 107000 bytes (81.63%), SRAM 453756 bytes (86.55%). Two existing unused-function warnings remain.
- `git diff --check`: passed.
- Host measurements do not measure the codec/analog output, Seed CPU deadline margin, PA response or subjective sound. Hardware listening remains the next validation.

Source SHA256: `4b676988dbeea3eb4e661013a04c74f44e057f00a08652bd8a1c7c6e0bef0027`.
Firmware binary SHA256: `bf32bcfcde9b27aa0142e3f91fb4c8ee9e7b9f355c9862e1516b26911c8f91aa`.

The Teensy preview source mirrors the revised phase/envelope/macro laws; this task uploads only Seed3. The controller must be rebuilt/uploaded separately to show the revised preview/timing labels.

Upload completed to Seed3 serial `200364500000`, internal flash at `0x08000000`. All 107000 bytes were read back over DFU and matched the tested binary byte-for-byte (SHA256 above). The subsequent leave command returned get_status error 74 as the USB device disconnected; a fresh DFU listing found no DFU device. Flash contents are verified; running audio has not been captured from hardware.

## The apparent envelope flutter

![Same renders with short-window and full-cycle RMS](rms_window_comparison.png)

The regular scalloping in `d2_envelope.png` is largely a short-window measurement effect. Its 20 ms RMS window is shorter than D2's 36.708 Hz sub cycle (27.242 ms). The body/octave waveform repeats every sub cycle, but different 20 ms portions contain different energy. This is not proof of oscillator detuning or a fluctuating decay envelope.

For the old clean SUB=1 render, subtracting the decay trend leaves 4.40 dB peak-to-peak ripple in 20 ms sliding RMS, versus 0.0058 dB when measuring consecutive complete sub cycles at a fixed settled phase. For the new render, those figures are 4.01 dB and 0.0055 dB. The latter tiny residual includes PCM16 quantization and rounded sample boundaries. Original WAVs were used without modifying their audio; only the measurement window changed. See `rms_window_metrics.json` and `test/daisy_kick_low_end/rms_window.py`.

The old independent sweep clocks and sub-induced distortion intermodulation were real architectural issues, but this periodic late-tail RMS ripple does not diagnose them. The deep notch on the green tail-gap example is the intentionally enabled gate.
