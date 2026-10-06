> Historical experiment. The bass boost remains, but its attack changes were
> reverted at the user’s request. See [../reverb/](../reverb/README.md) for current
> code, listening examples and validation. Measurements below describe this
> earlier snapshot.

# Clean bass shelf — built, not uploaded

This revision replaces the octave oscillator with a bass boost on the original
clean kick. The existing SUB/CC57 control blends a fixed +15 dB, 120 Hz low
shelf; PUNCH controls the whole boosted clean lane. The dirty send is unchanged.
At full SUB, the shelf adds 14.92 dB at 30 Hz, 14.73 dB at 40 Hz, 12.59 dB at
73.4 Hz and 10.80 dB at 90 Hz. It boosts existing content; it does not invent
30 Hz content under a sustained D2. Intermediate blends have a small upper-mid
undershoot (largest sampled value 0.15 dB), not a bass notch.

Kick output trim is reduced from 0.12 to 0.06 (6.02 dB) to reserve headroom.
Thus D2 at full bass is approximately 6.6 dB louder than the previous clean
body alone, rather than 12.6 dB louder at the DAC. Glue settings are unchanged.

The separate sub-millisecond attack pulse and noise layer are removed. The
shortest raised-cosine onset is now 1.5 ms rather than 0.5 ms; SHAPE zero stays
8 ms. SHAPE still controls depth, SWEEP controls time, and DECAY controls
amplitude duration. A very long laser with minimum DECAY can die before
reaching bass; the clean 240 Hz LPF rejects its high pitch, while the dirty
lane carries the laser. There is no second low oscillator underneath it.

## Measurements

- Actual DSP host renders: D2 boost 12.606 dB, original fundamental retained.
- 96 isolation cases: the dirty return is bit-identical when SUB/PUNCH change.
- 1,080 parameter combinations: worst tracked fundamental cancellation in
  eligible 30–90 Hz windows is 0.862 dB. Higher harmonic projections are retained
  in raw data, but are not meaningful clean components for a sine-only setting.
- All-max mixer grid: peak 0.3582; maximum LINE scaling residual 0.00028.
- 675 voice configurations: phase resets and SHAPE-zero invariance pass.
- MIDI parser/queue stress and 30-second FX stress pass; no nonfinite samples.
- The measured first-20-ms energy above 5 kHz changes only −0.28 to +0.48 dB
  across 40 attack comparisons after compensating output trim. Removing the
  pulse is therefore **not proof that the reported audible tick is fixed**.
  Exact failing settings/audio are still needed for that diagnosis.

[Rendered D2 spectrum and waveform](d2-bass.png), [shelf response](shelf.csv),
[interactive phase survey](phase-map.html), [phase summary](summary.json),
[attack measurements](attack.json).

## Idle whine remains a hardware diagnosis

Both old and new firmware produce exact digital zero with zero input at boot,
including maximum distortion/reverb settings without a trigger. After a hit,
residual host samples fall to denormal magnitudes; the MCU flushes denormals.
This does not test the codec, external input, power supply or grounding.

Daisy documents analog callback noise at sample rate / block size:
https://docs.daisy.audio/tutorials/eliminating-callback-noise/
At 48 kHz / 16 samples, that predicts 3 kHz. The earlier block-size change from
8 to 16 would move such a tone from 6 kHz to 3 kHz. This is a hypothesis,
not a confirmed cause. No notch, blind callback-rate change or input gate has
been added. Measure the whine frequency and compare with the external input
removed; an unplugged ADC can still pick up noise. LINE mutes the kick only.

## Validation and reproduction

`bash test/daisy_kick_low_end/run.sh` exercises the production source, including
sanitizers, bass-shelf response, mixer/headroom, MIDI, gates and retriggers.
`python3 test/daisy_kick_low_end/audit.py daisy-kick/analysis/bass-shelf`
regenerates the phase survey. `bass_analysis.py BEFORE_RENDERER AFTER_RENDERER
OUTPUT_DIR` uses Python with numpy/matplotlib for the attack comparison and plot.
Both Seed and Teensy build successfully. Teensy updates only the clean preview
and bitcrush readout; the existing controller CCs work with this Seed revision.
Neither device was uploaded during this change.
