# Host harness for the Daisy kick

Compiles `daisy-kick/midi_oled_monitor.cpp` on the desktop against stub
hardware, runs the firmware's own `main()` init path, feeds it real MIDI bytes
and writes the kick bus to a raw float32 file. It exists because reasoning
about the retrigger click from the source repeatedly produced changes that
were no better on the bench; rendering the waveform settled it in one pass.

```sh
cd daisy-kick
c++ -std=c++17 -O2 -I host/stubs -o /tmp/kick_host host/kick_host.cpp
/tmp/kick_host /tmp/out.f32 bpm=185 hits=6 sub=0.75 punch=0 decay=0.75 line=0.12
```

Read it back with `numpy.fromfile(path, dtype=numpy.float32)` at 48 kHz mono.

## Arguments

`bpm` `hits` `gate` (ms) `tail` (ms of silence after the last hit), and the
mix CCs as 0..1: `line` `mackie` `sherman` `bpf` `sub` `punch` `decay`.

**Only CCs named on the command line are sent**; anything omitted keeps the
firmware's power-on default. `line` is the master output level, not a mix
lane — setting `line=0` mutes everything. Keep it around `0.12` so the render
stays below `OutputCeiling()`, or the clipping hides what you are measuring.

## How it works

`kick_host.cpp` renames the firmware's `main` and `#include`s the .cpp, so the
harness shares the translation unit and can read its statics. The stub
`DaisySeed::StartAudio()` throws, which lets the real init path run to
completion and then hands control back without entering the firmware's
`while(1)`. `System::GetNow()` is driven from the sample count, so the
firmware's millisecond timing is real.

`KICK_HOST_PROBE=1` prints a per-block state line to stderr.

## Stubs

`host/stubs/` covers only what the firmware touches: `DaisySeed`, `I2CHandle`,
`System`, the `AudioHandle` buffer typedefs, and the handful of STM32 HAL
symbols used by the MIDI UART. There is no OLED and no UART; MIDI is injected
by calling `ProcessMidiByte()` directly.

## Caveats

The engine is deterministic — identical arguments give a byte-identical file.
If two renders disagree, suspect the shell script rather than the engine.
