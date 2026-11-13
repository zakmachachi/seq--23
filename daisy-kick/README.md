# Daisy kick firmware

Daisy Seed kick/audio firmware, originally imported from `DaisyExamples/files (1)/midi_oled_monitor.cpp`. This is separate firmware from the Teensy controller in the parent project's `src/` directory. Build it with Make, not PlatformIO.

See [Eurorack trigger/clock wiring](EURORACK_OUTPUTS.md) for D12 kick triggers
and D3 MIDI clock outputs (3.3 V logic).

## Held industrial performance FX

Step 1 suppresses new generated hits while held, step 2 holds a random
140–800 Hz kick HPF, step 3 raises kick pitch by one semitone, step 4 holds
40 ms decay for new hits. Step 11 ramps the kick down over about two seconds to
a quiet held level, then restores it over about half a second. Step 12 shortens
new kick decays and enables the clocked, high-passed dub echo/reverb; release
stops its send immediately and lets repeats fade over three sixteenth-note
steps. Step 13 starts with a sharp 12 ms static crackle, then 500 ms of silence,
followed by a distorted 50 Hz ground-loop scream for four seconds on its own.
Kicks then return at 150 ms decay and ramp from silence to full level over four
seconds; each kick ducks the scream.
On release the scream fades over 800 ms, starting at the next kick boundary.
These controls work from every selected sequencer channel, row and page, and
always target the Kick Seed. Step 12's send release
is immediate; Step 13 transitions take effect on MIDI-15 kick boundaries. They
do not stop MIDI trigger or clock
outputs. The other step keys keep their held industrial FX when KICK is
selected. Existing kick tails ring through step 1; changing DECAY affects the
next hit without shortening the current one. The HPF sits on the kick bus, so
live external passthrough remains clear. MIDI CC60 controls temporary hit
suppression, CC66 step 11's level ramp, CC62 the kick HPF, CC63 temporary decay,
CC65 step 12's dub echo, and CC70 step 13's ground machine. See
[controls, MIDI and limits](../HELD_FX.md).
This addition is built/tested for both boards, not uploaded.

## Dedicated FX menu

With KICK selected, MENU3 replaces Euclid with six FX slots: DELAY/LOOP/STUT/PITCH,
HPF/LPF, PUMP, REVERB, BITCRUSH and EROSION. The first two cycle EXT-only
choices; the other four toggle EXT / EXT+INT, with INT-only also available
for REVERB. Hold one second for a bar-end
zero. Function + EROSION sets its centre frequency, shown on OLED2.
See [controls, routing and CC protocol](../KICK_MENU2.md).

The previous INT-only-reverb revision was uploaded to both boards. The latest
changes preserve effects while cycling and add EXT-only PITCH (±12 semitones,
CC93, centre/hold reset = 64). Both new firmware images are built for manual
upload; this latest revision has not been uploaded. See [validation and screen previews](analysis/fx_menu/README.md).

## Kick signal path

**Current raw-character phase:** a committed CC110–115 next-hit packet feeds
one Seed-owned oscillator; fixed MIDI note 60 is only the trigger edge. The
Seed snapshots pitch, decay, shape, sweep and tail tune together before audio
generation. SHAPE is a pitch-depth macro (zero to +48 semitones;
finer near the bass end, with the 909 reference at centre). SWEEP is 4–120 ms
(58 ms at centre), with finer travel around the useful 50–65 ms region.
DECAY controls the amplitude fall after a fixed 60 ms body plateau. TUNE/velocity
sets a deterministic ±12-semitone tail movement; TMOD sets its reset-per-hit
triangle rate. HARM leaves the existing sine/body untouched at zero and adds
phase-locked 2nd, 3rd and 5th partials toward a harder, parabolic drum-machine
character. The partials fade smoothly before Nyquist; no detuned voices are used.
Retriggers crossfade from the prior voice over 5 ms while the new oscillator
still resets phase on every hit.

EXT IN1 is disabled in software at the input multiply, so a floating or
unplugged input contributes no ADC noise. Function + Menu 2 on KICK maps one
control to each knob: tail-gate start (0–200 ms, centre about 130 ms), Acid
clock division, tanh compressor amount (K3 press toggles its clean
level-matched bypass), kick gain (−15 to +21 dB), cutoff fade (K5 press
enables 2–60 ms; off restores the existing 3 ms edge), and internal reverb.

The three editable centres (85 Hz–3.2 kHz) feed up to three serial, broad
two-octave Mackie-style mid boosts before the character stage. K4 edits the
current centre and its button cycles 0–3 active sections. Both displays show
all three centre frequencies, active or stored. The Mackie path restores the
older body/presence shaping and dual-stage overload, with each nonlinear stage
oversampled and the 48-sample wet latency matched by the clean path. The dry
return remains in the blend, which tops out at 65% to preserve body and
headroom. The tail gate starts at the adjustable back-menu time (about 130 ms
at default), cuts to zero over 3 ms, holds for the selected gap, then reopens
over 3 ms. Reverb follows it and can ring through. Reverse switches at a MIDI
kick edge, and held sequencer pads 1/9 restore random BPF and the clocked
eighth-note kick loop. There is no standalone HPF/LPF or separate sub
oscillator in this phase.
The maximum tested internal output peak was 0.907 full scale; the model stays
below clipping across the tested sweep and BPF-centre grid. See
[the 909 and hardcore reference comparison](analysis/raw_909/README.md).

The following notes describe earlier multi-lane revisions and are retained
as history; they do not describe the current raw-character callback.

One oscillator produces the swept body and resets deterministically on
Note-On. HARM adds phase-derived harmonics. SUB boosts the original clean
body with a 120 Hz low shelf, up to +15 dB in the lows, rather than generating
an octave below it. PUNCH controls the entire boosted clean lane.

```text
body + attack -> fixed LPF -> SUB bass shelf -> PUNCH --┐
body + attack -> mid EQ / Mackie cascade (or Tube) -> BITCRUSH -> HPF -> DIST -- mixer1
mixer1 -> LINE -> tail gate -> routed reverb / pump / erosion -- mixer2
external input -> pump / delay / stut / loop / HPF / LPF
               -> reverb / bitcrush / erosion ------------------┘
mixer2 -> fixed trim -> gentle glue -> safety ceiling -> both mono outputs
```

The fixed clean LPF and dirty HPF are matched 240 Hz fourth-order
Linkwitz-Riley filters. The older BPF implementation used 0..3 serial mid
boosts, 85..3200 Hz, two-octave width and 0..15 dB boost. Mackie interleaved
these with one to three nonlinear preamp stages, all at 4x sample rate.
97-tap interpolation/decimation filters suppress unwanted aliasing; the
clean return is delayed by the same 48 samples (1 ms). SUB never feeds
EQ/distortion, and PUNCH does not alter the distortion drive. These are
musical models, not calibrated circuit emulations.

SHAPE=0 disables the kick sweep and attack harmonics but preserves oscillator
phase across notes; sequencer note-off gates the bass. DECAY controls the body
fall, with its top setting holding sustain while the MIDI gate is on and a
smooth release after note-off. It no longer forces infinite sustain by itself.
SHAPE adds pitch depth (0..48 semitones) and attack character. Menu 1 K3
SWEEP / CC78 sets duration directly: 4..240 ms, independent of SHAPE and
DECAY. DECAY sets amplitude fall only, following a fixed two-base-note-cycle
hold. A long laser with minimum DECAY can intentionally end before settling.

Menu 1 K6 TUNE/velocity sets signed tail pitch, -12..+12 semitones with fine
resolution near neutral 64. CC77 preserves value zero without sending a
MIDI note-off. K5 TMOD / CC79 selects rate: 0 is static; 1..127 rises
smoothly from .02 to 16 Hz and resets each hit. TUNE=64 means zero modulation
depth at every rate. At SHAPE zero this modulation and the sweep are disabled.

BITCRUSH / CC37 is FX knob 5. It processes external input and optionally the
kick dirty return, leaving boosted clean bass intact. The macro
blends 0..100% crushed audio while reducing depth from 16 to 1 bit, with
50 ms control smoothing and interpolation between adjacent integer depths.
The final dirty HPF follows quantization. Quantization deliberately adds
rough edges; the smoothing prevents abrupt parameter-switching steps.

The output AHD envelope now follows the kick through its effects, including
the clean SUB lane. CC42 concentrates most of its range across 90–400 ms,
with sweep-length and whole-note endpoints. Attack is 5 ms and release is
20 ms before internal reverb/delay; retriggers continue smoothly from the
current envelope level, with the original 5 ms voice handoff. CC43
toggles a separate fixed 32 ms tail
gap, beginning at the selected gate-start time. Reverb is also cut by the
output envelope while it is closed.
REVERB now retains its tank across hits, ducks from the dry kick level, and
introduces damped eighth-note repeats above 65%. Amount changes are smoothed;
tempo changes crossfade fixed delay taps. See the auditions below.

Mixer2 sums the processed external input and kick. Both outputs carry that
mono mix. The external HPF affects only its input branch. Shared glue is a
1.25:1 compressor above -12 dBFS with 30 ms detector attack, 150 ms release,
no makeup gain and at most 2 dB reduction. It does not reset on kicks. LINE
trims only the kick (0..1); fixed kick trim is .06, external trim .62/2, and
final mix trim .8, followed by a fixed 2x (+6 dB) output makeup after the glue.
The external input is gated on a 10 ms average level (opens at -36 dBFS,
closes below -46 dBFS after 250 ms), so an unplugged input adds exact
silence instead of board noise. Audio blocks are 2 samples: a hardware
recording showed the idle whine at exactly 48 kHz / block size (6 kHz with
8-sample blocks, 3 kHz with 16), i.e. per-block CPU bursts rippling the
supply. At 2 samples it sits at 24 kHz, above hearing.

See [KICK_MENU2.md](../KICK_MENU2.md), the current
[analysis and auditions](analysis/reverb/README.md), and
[host harness](host/README.md). Earlier `analysis/coherent/` is historical.

## MIDI and scheduling reliability

USART3 receives into a 1024-entry interrupt-fed queue with its hardware FIFO
enabled. The linker explicitly routes the USART3 vector to `KickMidiRxIrq`;
no local libDaisy patch is required. Parsing consumes every channel-message
type, even ignored pitch bend/aftertouch/program changes. A receive gap
invalidates partial messages and running status. Pending triggers are consumed
before parsing another hit, preserving ordering during backlogs.

BPF coefficients update only while controls move, at 1 kHz. FIR history uses
mirrored rings to avoid per-tap division. Audio blocks are 2 samples (42 µs).
FPU denormals are flushed to zero; full DSP recovery stops audio before clearing
shared state. Debug symbols `midi_uart_errors`, `midi_rx.overflow`,
`audio_max_cycles`, and `audio_overruns` distinguish reception errors from
missed audio deadlines. Desktop tests cannot establish MCU deadline headroom.

## Build

Requires GNU Make, the ARM embedded GCC toolchain (`arm-none-eabi-g++`), and built libDaisy/DaisySP dependencies. Dependencies and generated binaries are not included in this folder.

The Makefile defaults to `../libDaisy` and `../DaisySP`, relative to this folder. Override these paths to use an existing DaisyExamples checkout:

```sh
make -C daisy-kick \
  LIBDAISY_DIR="/path/to/DaisyExamples/libDaisy" \
  DAISYSP_DIR="/path/to/DaisyExamples/DaisySP"
```

To save those paths on your machine, create `daisy-kick/local.mk` (ignored by
Git):

```make
LIBDAISY_DIR = /path/to/DaisyExamples/libDaisy
DAISYSP_DIR = /path/to/DaisyExamples/DaisySP
```

Then, from the `seq--23` directory, run `make -C daisy-kick`.

## Upload over USB

Connect USB to the Daisy Seed itself. Hold BOOT, press and release RESET, then
release BOOT to enter the STM32 DFU bootloader. Check that `dfu-util -l` lists
the device, then run from `seq--23`:

```sh
make -C daisy-kick program-dfu
```

This builds the current source before uploading. PlatformIO's upload command
in this repository targets the Teensy 4.1 controller, not the Seed.

## Dependency versions

The local dependency revisions when this source was imported were:

- libDaisy: `9498417add4a4c76bc10737d5e9f43750a4da639`
- DaisySP: `a0494a3adb67f549e18dfd71a35fa656f65b38b6`

Initialize dependency submodules and build the libraries before building this application if using a fresh checkout. The outputs are generated under `daisy-kick/build/`, with target name `midi_oled_monitor`.

Consult the source's hardware and MIDI initialization sections before wiring or changing mappings. The Teensy controller's mapping is documented separately in [KICK_MENU2.md](../KICK_MENU2.md).
