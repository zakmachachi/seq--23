# EROSION — cycling FX menu

Seed3 flashed and readback verified on 2026-10-06. Teensy GUI uploader reported successful upload on the same date; no independent Teensy flash readback. Select EROSION after BITCRUSH in the existing Menu 2 FX
cycle while on the kick channel. This is not a new top-level menu.

- Knob 1: EROSION macro / CC38, 0–127. Increases short-delay modulation depth
  quadratically (up to a 0–1 ms excursion), sine-to-filtered-noise blend, and
  noise bandwidth. At settled zero the audio bypass is exact.
- Knob 4: centre frequency / CC39, logarithmic 80 Hz–12 kHz. It takes this role
  only while EROSION is selected; other FX selections restore its BPF role.
  BPF settings are preserved. Button 4 does not change BPF count while EROSION
  is selected. Frequency defaults to approximately 1 kHz.
- Changing the selected FX does not disable its stored amount, consistent with
  the existing effects. Long-hold FX reset clears EROSION amount but retains
  frequency. Motion lanes, temporary snapshots and EEPROM save/recall include
  both new parameters. Save version 20 appends fields without shifting older
  offsets; older saves initialize EROSION off and frequency to 64.

## Signal and macro design

[Ableton describes Erosion](https://www.ableton.com/en/live-manual/12/live-audio-effect-reference/)
as a short delay modulated by sine and filtered noise. This implementation
uses that published principle; it is not a bit-exact recreation of Ableton.
Noise modulates the delay time rather than being added as audible hiss.

The macro controls noise blend (0–100%), delay depth, and bandpass damping.
A stable topology-preserving state-variable filter shapes the noise around
knob 4's centre frequency. Amount and frequency slew over approximately 30 ms.
The modulation runs continuously; triggering a kick does not reset a delay
buffer containing external audio. Global DSP reset uses a fixed noise seed.

The combined kick-plus-external bus enters EROSION before the existing master
trim/glue/ceiling. Both physical outputs remain the same mono mix. Set
`EROSION_INCLUDE_INTERNAL=false` in midi_oled_monitor.cpp for external-only
processing later; this is a code switch, not a current UI option.

The delay read wraps integer indices and interpolates adjacent samples. There
is no feedback, added noise floor or extra gain. Interpolation is bounded by
the peak of stored input samples. High-frequency roughness at extreme amounts
is intentional degradation; the model does not claim alias-free processing.

## Listening examples

Identical generated kick patterns, actual output level without normalization:
[off](erosion-0.wav), [50%](erosion-50.wav), [100%](erosion-100.wav).
All use the midpoint frequency setting. These demonstrate the effect, not a
measurement against Ableton audio.

## Checks

- Controller: cycling/CC mapping, knob 4 reassignment, retained BPF values,
  reset, all parameter values, and display rendering pass.
- DSP sanitizer test: exact settled bypass, no output from silent input,
  30 seconds of rapid control changes, finite states and bounded sample peaks.
- Actual firmware renders verify both internal and external routing; 12
  combined-bus extreme cases peak at 0.649624, below the 0.93 ceiling knee.
- GPIO trigger/clock and existing reverb regression tests pass.
- Seed and Teensy builds pass. Hardware timing/listening has not been measured.

Run `bash test/daisy_kick_low_end/run.sh` and `bash test/kick_host/run.sh`.
`erosion_render.py HOST_RENDERER OUTPUT_DIR` regenerates the routing checks and
WAVs. The full validation record is in validation.json.
