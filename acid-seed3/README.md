# Synth Seed3: ACID and HOOVER

Two separate seq23 **audio machines**, one separate synth Seed firmware. The folder
and binary retain the earlier `acid-seed3` / `midi_acid_monitor` names for build
compatibility; the binary contains both engines. `daisy-kick` is independent.
The Teensy owns both OLEDs, notes, scale, accent/velocity, slide and transport.

In **Menu 4**, **press knob 1** to cycle **NONE → ACID → HOOVER → NONE**.
Default is NONE. Turning knob 1 still selects the independent pattern generator
(OFF/KICK/HIHAT/SNARE/ANTIKICK/PERC/EUCLID). Selecting audio never regenerates
or clears the pattern. For example, KICK pattern + HOOVER audio uses the kick
rhythm with Hoover notes, gates, slides and synth menus. Kick-only held FX and
kick CC updates are disabled on that overridden track. NONE restores its normal
MIDI routing and kick controls where applicable.

Physical **Menu 2** shows the selected engine's controls. Up to three sequencer
tracks can each select a synth engine. Teensy row 4 defaults to MIDI channel
**15** for the kick and row 5 to MIDI channel **12** for the off-acid voice
(since v1.5.0 any row's channel can be changed; the off-bass follower only
plays while row 5 is on MIDI 12). Row 5 stays directly editable, muteable, and
parameter-controlled. Other
synth tracks are assigned MIDI channels **13 and 14** automatically; rows 6
and 7 remain freely assignable for normal MIDI output.
Real channel-15 kick hits on steps 1/5/9/13 can arm an automatic off-bass on
steps 3/7/11/15, one octave above the kick's TUNE/SHAPE/TMOD-derived tail
pitch, including fine pitch bend. This follower owns the note pitch and timing
on those hits even if row 5 has a step programmed; row 5 still supplies the
voice/FX parameters, mute and note-length setting. Outside follower hits, row
5 plays its own programmed notes normally. The follower stays silent when row
5 is muted or Fill is active. MIDI 12 has priority over the two lower lanes
until its gate ends. The Seed remains
monophonic: an open note is never stolen by another user lane, and changing
lanes fades to silence over 2 ms before the next note starts, avoiding a
two-voice crossfade through shared effects. Kick remains on channel 15.
Machine selection is sent on each user lane's channel, so the Seed keeps
independent Acid/Hoover/Screech settings per MIDI channel.
Row 5 stays locked to MIDI 12 even when its audio engine is set to NONE. Other
synth rows use a temporary synth-channel assignment; cycling one back to NONE
restores its normal MIDI route. The Menu 2 and synth-FX screens show the
assigned channel.
Audio selection is saved independently inside the existing machine-state byte
(marker plus audio/pattern bits), without changing EEPROM struct offsets. Old
ACID/HOOVER pattern IDs migrate to pattern OFF/audio NONE. Macro values remain
session state, not EEPROM presets.

| Knob | ACID | HOOVER |
|---|---|---|
| 1 | Cutoff | Body / tail |
| 2 | Resonance | PWM / SWARM |
| 3 | Filter envelope | CHORUS |
| 4 | Decay | BAND / SCREAM |
| 5 | Accent | STEREO MOTION |
| 6 | Saw ↔ square | BRUTAL |

HOOVER: click knob 5 for OFF / 4 / 8 bars. Function+knob 4 adjusts the
parallel band-pass amount; moving knob 4 normally restores its automatic
amount curve. Function+knob 6 adjusts output trim. Velocity changes articulation
level. Slide preserves the time/level envelopes and glides pitch.

HOOVER uses one divided oscillator phase for saw-PWM, pulse-PWM and sub,
four-time/three-level amplitude and pitch contours with independent timing, a six-tap interpolated stereo
ensemble, and opposite logarithmic band-pass motion mixed in parallel with dry.
See [the detailed design and reference links](HOOVER_DESIGN.md).
This is an Alpha-Juno/Rodox-inspired model, not a measured hardware clone.

ACID retains the earlier prototype with corrected gate-envelope arithmetic,
release-to-silence, BLEP oscillator edges and matching-note legato note-offs.
It has **not** been established as an accurate TB-303 circuit model; see
[its current scope](DESIGN.md). Both engines leave through **Audio Out 1**. HOOVER keeps its stereo ensemble
and opposing filters internally, then folds them down as 0.5 × (L+R) so
one connected jack receives the complete sound with no summing gain increase.
Audio Out 2 is silent.

The output stage runs at unity gain after the mono fold-down. A 64-sample
(1.33 ms) lookahead limiter with a .94 ceiling remains as overload protection,
rather than as a source of loudness, so ordinary patches retain transient
headroom. Output trim still controls voice level.

The Acid/Hoover Seed can also be pumped independently on its own output. On
the KICK FX menu, set PUMP amount and click its knob to **EXT+INT**. The
controller mirrors that amount/route to this Seed; each KICK-channel MIDI note
triggers a fast, click-safe duck and tempo-derived recovery. Pumping is applied
after synth FX and the output limiter, and received kick notes never alter the
synth voice. Keep the two Seed outputs separate at the mixer; the ordinary
EXT-only PUMP path remains on the kick Seed.

When ACID or HOOVER is selected, Menu 3 provides BPF, tube/Mackie send, HPF,
clocked delay, reverb and erosion. Press knob 2 on physical Menu 2 (or the FX
page) to crossfade its model between Tube and Mackie; the amount is retained. Zero is an exact dry bypass for each
macro. Delay follows incoming MIDI clock at an eighth note and is high-passed
at 160/180 Hz on send/return. Reverb is high-passed at 250/180 Hz on
send/return. Erosion follows the played pitch instead of remaining at one fixed
centre frequency.

ACID Decay shapes both filter-envelope decay and audible amplitude articulation
over 70 ms to 1.5 s. HOOVER Body/Tail moves from a sharp transient to a softer
attack with a strong, compact sustained stab while preserving the engine's
faster pitch movement.

## MIDI and wiring

The shared controller constants are in `../include/SoftSynthProtocol.h`.
MIDI channels 12/13/14 have independent voice and controller state; row 5
uses channel 12, which also carries the kick-following notes, while other
user-programmed synth tracks use channels 13 and 14.
CC119: 0=ACID, 64=SCREECH, 127=HOOVER. CC29: incoming slide for the following note.
ACID CC20–25 correspond to its six controls; CC26=drive, CC27=glide,
CC28=output level. HOOVER CC70–75 correspond to its six controls;
CC76=motion period (0/64/127), CC77=parallel mix, CC78=output trim.
CC80–85 control BPF, distortion send, HPF, delay, reverb and erosion; CC86
selects Tube (0) or Mackie (127). CC108 selects the Acid clocked-delay rate;
the Kick back-menu K2 sends it to every active Acid part. Kick channel 15
CC35/CC92 mirror the PUMP
amount and internal-route bit for the post-FX Acid/Hoover output duck.
Velocity >=96 accents ACID. MIDI clock is 24 PPQN; Start resets motion,
Stop releases the voice and freezes phase, Continue resumes, clock dropout
holds after one interpolated tick. All Notes Off and All Sound Off release
voices; MIDI receive discontinuities clear parser state and release notes.

MIDI RX uses the existing PC11 / Seed D1, USART3, 31250-baud receiver
architecture. Use the same appropriate MIDI receiver circuit as the kick Seed.
UART interrupts only queue bytes; DSP/control state is owned by the audio callback.
The synth Seed has no OLED and no dependency on the kick project's source.

## Build and listen

```
make -j4 LIBDAISY_DIR=/path/to/libDaisy DAISYSP_DIR=/path/to/DaisySP
python3 render_acid_demo.py
```

An optional ignored `local.mk` can set the library paths. Builds create
`build/midi_acid_monitor.bin`. No board has been flashed for this update.

HOOVER movement now defaults to **1.4× speed**: amplitude/pitch envelope times
and glide are divided by 1.4; PWM and chorus rates are multiplied by 1.4.
The MIDI-locked 4/8-bar filter cycle retains its stated musical length.

The renderer compiles the same `dsp/Engine.h` used in firmware and produces
48 kHz WAVs of Audio Out 1 duplicated into both listening channels, at actual
output levels without normalization. The reference study retains the internal
stereo branches for comparison:

- `hoover_movement_demo.wav`: held notes first, then shorter aggressive phrases.
- `analysis/hoover/reference-before-after.wav`: level-matched downloaded CC0
  reference, previous engine, revised engine. See the adjacent study README.
- `hoover_demo.wav`: 0–6.4 s raw; 6.4–12.8 s ensemble; 12.8–19.2 s aggressive;
  19.2–25.6 s full macros, then release. Last two sections share an 8-bar motion
  cycle. Notes use a diminished pattern, accents and slides at 150 BPM.
- `acid_demo.wav`: four sequences of progressively brighter/more resonant,
  driven settings with slide and accent.

The host checks routing, slide note-off handling, clock timing/dropout, and all
64 Hoover macro endpoint combinations for finite bounded output and release.
Both firmwares compile. CPU deadlines, OLED usability and sonic matching still
need a hardware listening check; the host renderer cannot establish these.

### Acid artifact correction

The Acid filter now saturates each pole's input instead of repeatedly boosting
its stored state. The previous update could latch a large offset even with
zero oscillator input. Retriggers preserve envelope levels and attack from
there; accent, VCA and audible CC changes are smoothed. Oscillator phase remains
free-running and tied notes retain their envelope continuity.

The resonant feedback is now bounded just below unity, and the envelope-open
cutoff is capped at 9 kHz. This avoids the near-Nyquist limit-cycle edge at
extreme resonance/accent. Clocked delay returns are darker and lower in level,
so a bright filter transient is less likely to turn into a harsh repeat stack.

Run `c++ -O2 -std=c++14 tests/acid_artifacts.cpp -o /tmp/acid-artifacts && /tmp/acid-artifacts`
for the silent-filter, retrigger, CC-transition, 32 parameter-corner and exact
zero-FX bypass checks. The measured worst immediate retrigger deviation dropped
from 0.757878 to 0.007555; the existing delayed-Acid check peaks at 0.04239
sample-to-sample change. These are software checks, not board CPU measurements
or proof that every audible artifact has been eliminated.

## SCREECH machine

Menu 4 knob 1 now also selects SCREECH. Menu 2 knob 3 press cycles six
resettable styles. See [SCREECH_DESIGN.md](SCREECH_DESIGN.md) for macros,
MIDI mapping, reference-study limits and tests. `screech_demo.wav` contains
six four-second dry style examples. Both Teensy and synth Seed need this update.

## Acid character and clocked delay

On Acid's physical Menu 2, press knob 1 to select **MODDED / SMOOTH**.
Seq23 starts Acid in MODDED; both screens show the selection. CC30 on channel
14 selects Smooth (0) or Modded (127). Modded restores the original nonlinear
state-feedback ladder, with 15 Hz DC removal before the VCA. Both modes retain
smooth retriggers, accent/control slew and a crossfaded mode switch. Direct Seed
startup without seq23 remains Smooth until CC30 arrives.

The synth delay now uses the kick's dotted-quarter timing (36 MIDI clocks),
500 ms at the shared 180 BPM fallback, capped at 840 ms as on the kick.
Tempo changes crossfade stationary taps over 30 ms rather than slewing playback
speed. This avoids delay-time pitch bends; a crossfade can briefly colour a
sustained tone. The return retains high-pass filtering and adds a 9 kHz low-pass.
`tests/delay_modes.cpp` checks echo timing, tempo changes and Acid mode switching.

## Skitter — pre-FX amplitude hold modulation

Available on Acid, Hoover and Screech. Physical Menu 2 knob 4 **press** toggles
Skitter; **Function + knob 4 press** cycles 16ths/2 bars, 16ths/4 bars,
8ths/2 bars, 8ths/4 bars. Default is off with 16ths/4 bars selected. Turning
knob 4 retains its voice parameter (including Hoover's Function+turn mix).
The primary OLED shows SK when enabled; knob 4 focus shows state and timing.

CC87 values: 0 off, 1 sixteenths/2 bars, 2 sixteenths/4 bars, 3 eighths/2 bars,
4 eighths/4 bars. The shared amplitude stage is after the voice and before
BPF/distortion/HPF/delay/reverb/erosion; stored FX tails remain audible in gaps.
It chops existing sound and does not extend a voice's note/envelope length.

Hold width is 7–87% of the available pulse window. A triangle phase-modulates
a second triangle over the selected cycle; the width is latched at each pulse
onset so release cannot reopen halfway through a pulse. Rounded attack/release
and a 20 ms bypass transition soften edges. Phase follows MIDI Clock/Start,
continues across notes, and repeats every selected 2/4 bars. It bypasses when
transport is stopped or no clock has been received. Existing clock dropout
behavior freezes phase after one extrapolated MIDI tick.

`tests/skitter.cpp` verifies all timings, repeatability across whole cycles,
bounded amplitude, exact settled bypass, MIDI mode routing and downstream echo.
