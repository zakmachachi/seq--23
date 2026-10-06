# Current kick: serial mid drive, independent pitch timing, buffered MIDI

**Seed3 uploaded and flash readback verified. Teensy upload confirmed by the user. Runtime/audio soak testing remains outstanding.**
Previous `coherent/` and `locked/` plots describe earlier firmware.
The measurements here render the production Seed DSP on a desktop. They do not
measure Seed CPU timing, its analog output, an actual 909, or a horn system.

## Reliability findings

The previously uploaded source (`4b676988…`) has a reproducible MIDI parser
buffer overflow: send a pitch-bend status followed by repeated data under
running status. Ignored messages were never completed, so a third data byte
wrote beyond `midi_data[2]`. AddressSanitizer and UndefinedBehaviorSanitizer
both caught it. This is a real defect, but we have not captured the user's
specific failing MIDI stream and cannot call it the proven cause of that event.

The same firmware polled one UART receive register in the foreground. Heavy
DSP work could delay reads long enough to lose bytes. STM32's hardware requires
reading the register/FIFO before it overflows; clearing the error afterward
cannot recover the missing bytes. See [ST's RM0433 USART section](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743-753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf).

Changes:

- Every MIDI channel message consumes the correct one or two data bytes,
  including ignored messages. Realtime bytes do not disturb running status.
- USART3 FIFO + interrupt-fed 1024-entry queue, about 327 ms at full DIN rate.
  Overflow/error markers invalidate partial messages at the loss boundary.
- Backlogged Note-Ons wait for the prior trigger to be consumed.
- BPF coefficients update at 1 kHz while moving, then stop recomputing.
  Mirrored FIR rings remove division from every filter tap.
- 16-sample audio blocks; flush denormals; stop audio before bulk panic reset.
- Linked vector checked explicitly: USART3 points to `KickMidiRxIrq`.
  `--defsym` in the Makefile avoids depending on a modified libDaisy handler.
- Hardware debug counters: `audio_overruns`, `audio_max_cycles`,
  `midi_uart_errors`, `midi_rx.overflow`. They have **not** yet been measured
  while playing the physical board.

The production-code stress test covers all channel-status types with repeated
running-status data, 100 queued hits with their own exact tail-pitch values,
queue overflow/recovery, and 30 seconds of changing BPF/drive/model/bitcrush.
This is supplemented by the output and phase regressions described below.

## Controls and routing

Menu 1 K3 **SWEEP** is CC78, 4–240 ms logarithmic. **SHAPE** keeps its attack
pitch-depth/character role; **DECAY** changes only the amplitude envelope.
There is a one-base-sub-cycle hold, independent of sweep/depth. A short decay
can intentionally cut off a long laser before bass settling; the earlier
automatic extension has been removed.

Menu 1 K6 **TUNE** is velocity/CC77, ±12 semitones with 64 neutral and fine
resolution near centre. K5 **TMOD** is CC79: zero gives a one-shot glide;
1–127 gives .125–16 Hz. TUNE supplies signed depth, TMOD supplies rate. The
LFO resets at each trigger, then starts when the initial attack has settled.
With TUNE at 64, there is no modulation to hear. The sub tracks exactly half
the body frequency throughout; no independently drifting oscillator exists.

Menu 2's eighth FX page is **BITCRUSH**, CC37. It processes only the dirty
return before the 240 Hz high-pass. The macro blends clean-to-crushed wet audio
and reduces resolution from 16 to 2 bits. Adjacent integer-depth quantizers
are interpolated, with 50 ms parameter smoothing and no added latency.
A full parameter jump at fixed .371 input produced at most .001511 per-sample
change before output trim. The quantized waveform itself deliberately contains
rough steps. This is not an alias-free effect.

```
body/attack -> LR4 LPF 240 Hz -> PUNCH -> 24-sample alignment --+
octave sine -> SUB ------------------> same alignment -------+-- mixer1
body/attack -> serial mid EQ/preamp -> BITCRUSH -> LR4 HPF ----+
mixer1 -> kick FX / LINE -> tail gate -> reverb ----+
external -> external FX (including performance HPF) +-- mixer2 -> trim/glue -> both mono outputs
```

## Mid EQ and drive

BPF is the existing panel label; its filters are now mid **boosts**, not
replacement band-pass audio. CC44–46 share the displayed 85–3200 Hz law.
Boot frequencies match controller values 35/65/95. Each stage has two-octave
bandwidth, up to +15 dB boost. Measured centre-gain error across all 128
positions at both processing rates is at most 0.002 dB.

Mackie count 0: one preamp without mid EQ. Counts 1–3: corresponding serial
EQ/preamp stages, all at 192 kHz. Tube retains its two-stage character with
the EQ bank before it. SUB never enters either drive path.

The [Mackie 8-bus manual](https://umlsrt.com/StudioDocuments/Mackie_24-8_Console.pdf)
specifies a two-octave low-mid section with ±15 dB gain and a 45 Hz–3 kHz sweep.
We deliberately retain this instrument's 85 Hz–3.2 kHz CC range for display
compatibility. This is a musical desk/preamp approximation, not a calibrated
24-8 circuit emulation.

The former model's later clipping stage ran at the base sample rate. The new
Mackie cascade uses 97-tap anti-imaging/decimation FIRs; the clean path receives
the matching 24-sample delay. Coherent-tone probes at 997 and 3001 Hz show
about 28–72 dB less nonharmonic spectral energy relative to harmonic energy.
That metric is not THD+N and compares changed models, not just resamplers.
See [aliasing](aliasing.png), [mid EQ](mid_eq.png), and [raw metrics](channel_metrics.json).

## Bass phase and glue

[Interactive phase map](phase/phase-map.html): 1,080 combinations, including
wet bitcrush. Across 2,994 enabled body/sub projections in 30–90 Hz, worst
sum-versus-stronger-component reduction was **1.415 dB**; seven windows exceeded
1 dB. Deeper raw projection nulls also exist, often at the muted body's
frequency when PUNCH=0, with low absolute levels. All are retained in the raw
CSV/JSON; the map does not establish perfect phase at every nonlinear harmonic.
The separate generator test measured zero octave-clock error in 675 settings.
D2 settles at 73.415 Hz body and 36.708 Hz sub.

The output compressor is gentle bus glue: 1.25:1 above approximately −12 dBFS,
30 ms detector attack, 150 ms release, maximum 2 dB attenuation, no makeup.
It stays continuous across hits. Aggression comes from the drive cascade.
Measured 185/250 BPM solo kick trains did not engage it; a hot external-input
mix reached 0.689 dB reduction. A sustained .8-peak 36.7 Hz sine reached
1.699 dB reduction, with .044 dB settled gain ripple and no reduction in the
first 5 ms from silence. It preserves the transient rather than trying to
make a gabber sound with heavy mastering compression.

See [glue on kick trains](glue_kick_trains.png), [hot-sub glue](glue.png), and
[tail pitch / TMOD](tail_pitch.png). These tests do not imply a flat spectrum:
a deterministic tonal kick has pitched peaks, and a lower octave cannot
supply equal energy at every frequency down to 30 Hz.

## Reproduce and resume

From the repository root:

```
bash test/daisy_kick_low_end/run.sh
bash test/kick_host/run.sh
make -C daisy-kick -j2
python3 test/daisy_kick_low_end/verify_seed_vector.py
python3 test/daisy_kick_low_end/audit.py daisy-kick/analysis/channel-chain/phase
platformio run -e teensy41
```

The source/binary checksums and final build/test results are recorded in
`validation.json`. Upload both prepared firmwares when the user connects the
boards. Read back Seed3 flash before leaving DFU. USB DFU verifies flash bytes,
not runtime timing or sound; a hardware MIDI/FX soak remains the next check.
