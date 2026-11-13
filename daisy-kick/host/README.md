# Host harness for the Daisy kick

Compiles `daisy-kick/midi_oled_monitor.cpp` on the desktop against stub
hardware, runs the firmware's own `main()` init path, feeds it real MIDI bytes
and writes the kick bus to a raw float32 file. It exists because reasoning
about the retrigger click from the source repeatedly produced changes that
were no better on the bench; rendering the waveform settled it in one pass.

```sh
cd daisy-kick
c++ -std=c++17 -O2 -I host/stubs -o /tmp/kick_host host/kick_host.cpp
/tmp/kick_host /tmp/out.f32 bpm=185 hits=6 decay=0.5 shape=0.5 sweeptime=0.5
```

Read it back with `numpy.fromfile(path, dtype=numpy.float32)` at 48 kHz mono.

## Arguments

`bpm`, `hits`, `gate` (note-on duration in ms), `tail` (render time after the
last step) and `spacing_ms` (time between hits; defaults to a sixteenth at
`bpm`). The drum continues its decay after note-off. Every file begins with
250 ms of silent control-settling time.

The MIDI CC controls use normalized values 0..1: `mackie` / `mackamt`,
`tube` / `tubeamt`, `bpf` (four send positions), `bpfhz` (one band centre),
`decay`, `shape`, `sweeptime`, `model`, and `extkick`. The host accepts old
arguments such as `line`, `sub`, `punch`, `hpf` and `lpf` for script
compatibility, but those controls are bypassed in this raw phase. `bpm` sets the
firmware tempo used for the tail gate.
`model=0` selects Mackie; `model=1` selects Tube. `reverse=1` enables reverse.
`note` is a MIDI note number. `sweeptime` maps to 4..100 ms, with the measured
909 sweep at the midpoint (31 ms). SHAPE reaches the measured 909 start ratio
at its midpoint, has finer travel near a steady bass note, and extends to a
four-octave pitch start at maximum. DECAY maps to a 40..600 ms T30 after the
fixed 60 ms body plateau. BPF is one variable-centre band on the Mackie/Tube
send, not a stack; the dry kick remains present.

`externalhz` and `externallevel` generate a sine on the external input.
`externalout=/tmp/external.f32` saves physical output 2 separately. The
primary file and output 2 now both contain the combined mono mix, including any external input.

**Only CCs named on the command line are sent**; anything omitted keeps the
firmware's power-on default. The raw generator has a fixed calibrated level;
there is no output-gain control in this path.

For a clean body comparison, explicitly disable the BPF and character send:

```sh
/tmp/kick_host /tmp/clean-body.f32 hits=1 spacing_ms=1000 tail=1000 note=38 \
  bpf=0 shape=0.5 sweeptime=0.5 decay=0.5 mackamt=0 tubeamt=0
```

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

The host captures the firmware DSP at 48 kHz. It does not capture converter,
analog-output, amplifier or loudspeaker behavior. Identical fresh runs should
produce identical files; test retriggers separately because their continuity
bridge intentionally depends on the preceding output. The controller's OLED
scope is a simplified clean-generator preview, not a measurement of this bus.

`heldfx=1..16` selects a held industrial gesture; the harness renews its MIDI
lease every 200 ms. `heldfx=0` (default) leaves the new subsystem bypassed.

## External delay transition check

The external dotted-quarter delay now crossfades stationary taps on tempo
changes, fades its input send on engagement and filters both send and feedback.
The feedback path no longer applies the old soft clip on every repeat.

```
c++ -std=c++17 -O2 -I host/stubs host/delay_test.cpp -o /tmp/kick-delay-check
/tmp/kick-delay-check
```

This checks finite output and continuity on a sine through a tempo jump and
on/off transitions. It does not establish hardware callback CPU headroom.

## External-path integrity audit

The automatic external input noise gate has been removed from the normal path.
It was introduced during the grounding investigation and attenuated quiet input
and tails without an explicit FX setting. Input 2 remains excluded. Held FX
now separate the command generation counter from the pad number; pad-9 release
can finish on a quarter edge without another kick, and lease expiry clears the
kick BPF override as well as the effect return.

```
c++ -std=c++17 -O2 -fsanitize=address,undefined -I host/stubs host/external_integrity_test.cpp -o /tmp/external-integrity
/tmp/external-integrity
```

Checks: production callback at input amplitudes 0.00001, 0.005, 0.1 and 0.5,
with/without kick triggers (generated lane muted); expected linear return gain
and output filters; synth-channel CC isolation; each external effect's release
to exact bypass; all ten external effects layered then released; held command
sequence flags and lease/quarter release. Address/undefined-behavior sanitizer
checks pass. All 16 held gesture tests also pass. These tests found no extra
input distortion with settled FX bypass, but cannot establish hardware callback
deadlines or diagnose the reported audible recovery when kicks are added.
