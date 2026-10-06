> Historical firmware snapshot. The current shared-clock revision is documented in [../locked/](../locked/README.md).

# Coherent kick engine review — 2026-10-02

The running DSP has two pitched oscillators: the swept body and an optional
stationary octave sine. The body's sweep becomes its settled note; there is
no second stationary body underneath a separate punch oscillator. The sub
earns its place by producing immediate low bass while a laser sweep is still
high. SUB zero removes it for a clean 909-family body. WAVE is a waveshape of
the body's existing phase, not additional detuned oscillators.

The complete generator feeds both branches. The dirty branch is a selectable
pre-distortion BPF blend, Mackie or Tube, then an eighth-order Butterworth
high-pass at 180 Hz. There is no bass notch, post-distortion BPF return,
shape-dependent send suppression, or shared nonlinear mastering stage.
The fixed output trim is 0.12; LINE now spans 0–1 rather than 0–1.5. Saturation
belongs to the character branch. Final infrasonic filtering is 15 Hz, about
−0.26 dB at 30 Hz. This is an electrical DSP model, not a measured cabinet
response or an exact Mackie circuit emulation.

PUNCH applies one gain contour to body, sub and attack. BELLY sets its duration.
Both oscillators start together, hold one sub cycle before amplitude decay,
and reset phase on each note. TAIL DELAY is a deliberate gap after BELLY with
3 ms edges. It gates the completed output, including stored filter energy;
oscillators continue underneath. Amplitude decay also continues, so a long
gap with a very short DECAY can return a quiet tail. A single 2 ms output
bridge joins retriggers without preserving an old bass oscillator.

## Measurements of actual firmware DSP

`bash test/daisy_kick_low_end/run.sh` passed with AddressSanitizer and
UndefinedBehaviorSanitizer:

| Check | Result |
| --- | --- |
| D2 settled body / octave sub | 73.417 / 36.708 Hz |
| Shortest decay with longest sweep | First-cycle sub retained |
| Sub change across tested distortion settings | At most +0.005 dB |
| Full mixer / three BPF layers / model / shape / note stress matrix | Peak 0.8772, below 0.93 ceiling knee |
| LINE gain scaling | Linear within low-frequency float-filter roundoff; worst absolute residual 0.00041 |
| Tail gate across 120/240 BPM, BELLY extremes and both models | Real silent gaps of 39–250 ms, then return |
| Fresh vs retriggered output after bridge | Exactly identical after 3 ms in tested cases |
| Retrigger boundary | No output sample step |
| External performance HPF | Kick output bit-identical; external bass attenuated |

The [interactive phase map](phase/phase-map.html) covers 1,032 sampled
combinations. Across 3,246 qualifying projections of the sub and body in
30–90 Hz, the worst sum relative to the stronger branch is **−0.576 dB**;
none loses more than 1 dB. These are sampled settings, not an exhaustive
proof for every continuous knob position. All raw projections remain
available. Third-sub-harmonic projections often measure finite-window
leakage rather than an actual stationary generator component, and are
excluded from that summary count.

[Hardware-909 comparisons, spectra and firmware audition WAVs](README.md)
describe the remaining spectral and envelope differences. In particular,
strong pitched peaks at 36.7 and 73.4 Hz still leave an inter-peak trough.
Low cancellation does not imply flat spectral energy between those notes.

A [sub-pitch-fall prototype](sub-pitch-fall-prototype/README.md) was measured
separately and **has not been applied to the firmware or uploaded**. Starting
the sub at 1.3 times its destination with a 20 ms time constant reduces the
largest smoothed 10 Hz fall in 40–90 Hz from 14.58 to 11.00 dB in the D2
example, with 0.48 dB less 30–40 Hz energy and 0.70 dB more 40–90 Hz energy.
It still does not satisfy a literal unsmoothed 12 dB/10 Hz maximum everywhere.
The uploaded version retains the fixed octave sub while that tonal preference
is unresolved. These are spectrum measurements, not a claim that one option
sounds better without auditioning it.

The controller acceptance suite passed; 11 scope previews compiled and
rendered. The scope now includes the actual SUB setting and tail gate.
Only the Seed was targeted for USB upload; controller UI changes remain in
source until the controller is separately built/uploaded.

## Seed build and upload

ARM build succeeded: flash 107,308 bytes of 131,072; SRAM 453,748 bytes of
524,288. `build/midi_oled_monitor.bin` SHA256:

`76242e5ef20ae6d0c1fdd332d205feaf775867fd78685888611ce43b46f7ecd9`

The connected STM32 DFU device accepted all 107,308 bytes at 0x08000000 and
reported `File downloaded successfully`. After `Submitting leave request`,
dfu-util returned a get_status error (exit 74). A subsequent unrestricted
DFU query found no device, consistent with leaving the bootloader. Flash
transfer is confirmed by the uploader; readback and hardware audio were not
verified. The host measurements above are not recordings of the Seed output.
