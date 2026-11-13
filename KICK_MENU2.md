Held kick controls target the Kick Seed across every selected channel and page;
they never redirect those effects to the channel currently being edited.
Steps 1/2/3/4/11 retain their hit suppression, random HPF, pitch, short decay
and level ramp. Step 12 is a held dub echo; its input send stops on release and
the repeats fade over three sixteenth-note steps. Step 13 starts at a kick
boundary with a 12 ms static crackle, then 500 ms of silence. The distorted
50 Hz feedback scream then plays alone for four seconds. Kick hits return at
150 ms decay and ramp from silence to full level over four seconds. The scream ducks on each kick and fades
over 800 ms after the first kick boundary after release. See
[HELD_FX.md](HELD_FX.md).

# KICK sound and FX menus

**Current audio-engine note:** the Seed is in the incremental raw-909 phase.
The active path is one oscillator or EXT KICK input, an optional
three-layer BPF bank into the selected Mackie/Tube model, then clocked reverse,
the tail gate and routed internal reverb. With EXT KICK off, EXT IN1 is
still a live passthrough; the PUMP sidechain affects only that external branch.
Function + Menu 2 now gives each knob one end-stage control. Standalone HPF/LPF,
protected bands, the old mix lanes and master glue are bypassed.
When the stored PUMP route is set to EXT+INT, its amount and kick-triggered envelope
also duck the Acid/Hoover Seed's separate output after its effects. The two
Seed outputs can therefore stay separate at the mixer. This route is ignored
by the kick synthesizer itself.
The tested pitch/time ranges and comparison plot are in
[analysis/raw_909/README.md](daisy-kick/analysis/raw_909/README.md); tables
below retain the earlier controller/FX architecture where it is not part of
this current path.

Held sequencer step 9 re-rolls a three-band mid profile per eighth-note kick
retrigger. It does not capture or replay audio. Internal reverb follows the
pump, so its filtered tail can ring through the dip.

Select KICK in Trigger Machines. Menu 1 is the kick generator page; Menu 2
contains the sound controls; Menu 3 is Euclid again. Enabling Euclid on a kick
replaces the generated trigger rhythm only. It does not rewrite key, pitch,
scale, or timbre settings. FUNCTION + MENU3 still opens Pages. Kick Seed CCs
use MIDI channel 15 (`0xBE`); Acid delay-rate CC108 is sent on each active
Acid part's configured MIDI channel.

## Kick generator — MENU1

Knob 1 sets KEY, knob 2 sets HARM, knob 3 sets SPREAD (0–60 semitones; its
button cycles scale mode), knob 4 switches between PUMP and REVERB (press to
select; each amount is remembered independently). REVERB selects the Seed's
internal reverb only, including its clocked delay; external reverb is disabled.
Knob 5 sets TMOD rate, and knob 6 sets TUNE. The kick-specific spread macro maps to the trigger machine's
existing 0–5 semitone variation range.

## Menu 2 — sound controls

| Knob | Turn | Press |
| --- | --- | --- |
| 1 | SWEEP time / CC78 | — |
| 2 | DECAY / CC40 | REVERSE / CC41 |
| 3 | AMP ENV HOLD / CC42, sweep length to one whole note | Fixed 32 ms tail gap / CC43 toggle |
| 4 | BPF1/2/3 / CC44–46 | Layer count / CC47 |
| 5 | MACKIE / CC48 or TUBE / CC49 | Model / CC50 |
| 6 | SHAPE / CC51 | — |

Pump enables from its amount; the old SHAPE-button pump toggle is removed.
The pots use relative movement, clamp at 0–127 and discard overshoot at either
limit, so reversing direction responds immediately. Page changes seed fresh
physical references. BPF/model memories remain independent.

## Menu 3 — Euclid

Knobs 1–6 control pulses, offset, scale, velocity, gate and slide; knob 1's
button toggles Euclid. On KICK, Euclid replaces the generated trigger rhythm
only, while the kick's key, pitch and sound controls remain unchanged.

Earlier FX-slot routing and amount CCs remain part of the Seed protocol, but
the kick's six-slot FX screen is no longer assigned to Menu 3.

## FX MIDI protocol

Kick progression sends a complete next-hit packet on MIDI channel 15 before
each trigger: CC110 pitch, CC111 decay, CC112 shape, CC113 sweep, and CC114
tail pitch. CC115 commits the staged values atomically. The following Note-On
uses fixed note 60 as a trigger only; the Seed snapshots the committed values
and applies them in its audio callback. If the packet cannot fit in the Teensy
UART queue, it is not committed and the Seed keeps its previous complete
preset for that hit.

Amount CCs are unchanged: STUT 30, LOOP 31, DELAY 32, HPF 33, LPF 34,
PUMP 35, REVERB 36, BITCRUSH 37, EROSION 38; Erosion frequency is CC39.
CC92 is the internal-routing bitmask: bit 0 PUMP, 1 REVERB, 2 BITCRUSH,
3 EROSION. Bit 4 disables external reverb and implies bit 1 (INT-only reverb).
For the other effects, external processing remains enabled by its amount.
CC90/91 carry the lower seven/upper three bits of the queued-zero mask, in
CC30–38, then CC93 order. Seed consumes it at the next 96-clock boundary; zero masks
cancel requests. Teensy mirrors the reset in its display on the same clock grid and re-sends
zero as a backstop for UART congestion; this fallback may be one UI pass late.
These commands are retained/retried under UART backpressure, just like amount CCs.

Save format v21 appends the two cycling-slot selections and routing mask without
moving legacy fields. Older patches use the default layout/routes; all stored effect amounts remain active. v20 Erosion/frequency and v19 bitcrush values
remain compatible. Existing motion-lane parameter IDs are unchanged.

## Tail gate and back menu (v1.3.0)

The kick's 5 ms attack / 20 ms release AHD is applied before internal reverb/delay, so the clean low/sub tail and effect send stop together while the effect tail can ring. Retriggers continue from the current envelope level instead of restarting it from zero. The original brief 5 ms raised-cosine voice handoff is retained. CC42 places most knob values across 90–400 ms, while preserving sweep-length holds at the low end and whole-note holds at the top. The separate tail gate starts at the Function + Menu 2 time (0–200 ms, CC64 about 130 ms) and holds silence for a fixed 32 ms when CC43 is on. Back-menu K5 still selects a 2–60 ms gate edge; K5 off restores the existing 3 ms edge.

The single body oscillator resets each shaped kick and follows the pitch sweep. A new hit starts its own envelope; the outgoing kick is crossfaded away over 5 ms and then stopped. With no new trigger, the current hit rings out naturally. DECAY is captured per hit, so a new shorter value applies to the next trigger without changing the current hit. At SHAPE=0, the oscillator carries phase and glides across notes without resetting its envelope. Sequencer note-off controls that legato voice; DECAY sets its body fall, while the maximum holds sustain only while the MIDI gate remains on and releases smoothly after note-off. For shaped kicks, DECAY sets the body tail after the 60 ms plateau. SHAPE controls attack sweep depth and character; SWEEP sets pitch-sweep timing independently.

Since v1.5.0 step 6 is a held REVERSE and step 8 a held DRIVE flip (see [HELD_FX.md](HELD_FX.md)); the previous/cue overlay voices they used to trigger stay disabled because they caused audio callback hangs. Step 11 ramps the kick down to a quiet level over about two seconds, then restores it over about half a second after release; both edges are quantized to a kick. Step 12 shortens each new kick and enables the clocked, high-passed internal dub echo/reverb. Its above-unity feedback builds repeated echoes into a bounded saturation; on release, new send stops immediately and the repeats fade over three sixteenth-note steps. Step 13 starts with a 12 ms static crackle and 500 ms dead-air pause, followed by a four-second solo distorted 50 Hz ground-loop feedback scream. Kicks then return with a 150 ms decay and a four-second volume ramp; they sidechain the scream. Release is quantized to a kick, then the scream fades over 800 ms.

The back menu (FUNCTION + MENU2 while KICK is selected) has one sound parameter per knob:

| Knob | Control |
| --- | --- |
| 1 | Tail gate start / CC67, 0–200 ms; CC64 remains about 130 ms, with fine control across the audible zone. |
| 2 | Acid Seed clock division / CC108: 1/16, 1/8, 1/4, dotted quarter, 1/2, 3/4 or 1 bar. Sent to active Acid parts. |
| 3 | Tanh compressor amount / CC68. Press K3 / CC69 to enable or cleanly bypass it. A smoothed level matcher supplies makeup gain; bypass returns the untouched signal at unity. |
| 4 | Kick output gain / CC53, −15 to +21 dB; centre is unity. |
| 5 | Tail-gate fade / CC71, 2–60 ms; press K5 / CC72 to enable or restore the default 3 ms edge. |
| 6 | Internal reverb amount / CC36. |

K6 reverb stays synchronized with its sound-control page value. The six-byte saved menu payload keeps its size; older patches load safe
defaults for the redesigned controls.

The body/attack is cloned into clean LPF → PUNCH and serial mid EQ / Mackie preamps (or Tube) → BITCRUSH → HPF → DIST. SUB boosts the clean LPF output before PUNCH, never distortion's input. Clean LPF and dirty HPF are matched fourth-order Linkwitz-Riley filters at 240 Hz. Filtering remains phase-shifting, but the generators cannot drift and the linear crossover paths have matching phase. Nonlinear/BPF settings still change dirty harmonic phase; they are not guaranteed to reconstruct the original waveform.

Mixer1 feeds the tail gate, then reverb. An enabled reverb can fill the chopped gap. Processed external audio meets the kick at mixer2; both physical outputs carry the combined mono mix. Mixer2 uses fixed ×0.8 trim then gentle 1.25:1 glue above −12 dBFS, 30 ms detector attack / 150 ms release and maximum 2 dB reduction, followed by a fixed 2× (+6 dB) output makeup. The external branch is trimmed by the same factor, and an unplugged input is gated to silence. Kick LINE reaches unity at 127 with fixed ×0.06 trim; DIST spans 0–1.25, PUNCH/BPF/SUB 0–1. The kick's 15 Hz infrasonic filter precedes the gate (about −0.26 dB at 30 Hz). Performance HPF affects external input only. Emergency ceiling remains above tested normal peaks.

Four per-hit kick controls live on Menu 1:

| Knob | Control | MIDI / range |
| --- | --- | --- |
| 2 | HARM | CC64: existing sine/body at 0; adds phase-locked harmonics toward a harder character |
| 3 | SWEEP | CC78: 4–120 ms, with 58 ms at centre; independent of SHAPE and DECAY |
| 5 | TMOD | CC79: 0 = static pitch, 1–127 = smoothly increasing rate from .02–16 Hz; button or Function + turn resets to 0 |
| 6 | TUNE | Velocity / CC77: −12 to +12 semitones; 64 neutral, fine resolution near centre |

TMOD resets its triangle LFO on every hit. TUNE supplies its signed ±12-semitone depth; at 64, TMOD has no audible effect. TMOD zero holds pitch still; increasing it introduces a slow wobble first, then higher rates. The triangle begins at zero excursion, so there is no pitch step at note-on or at the bottom of the control. Note-On velocity carries TUNE; CC77 is used for the value zero, which Note-On cannot carry. At SHAPE zero, pitch sweep, attack harmonics, HARM and TUNE/TMOD are removed. Phase and envelope continue across legato sequencer notes; matching note-off gates the bass. DECAY remains active: maximum holds while the note is gated, then uses a smooth release. SWEEP, TMOD and HARM are latched per hit. The screen scope shows the generator before external FX.

BPF is three serial, broad peaking-EQ mid sections. Each CC44–46 tunes one centre logarithmically from 85–3200 Hz; each active section reaches +15 dB with a two-octave bandwidth. CC47 selects 0–3 sections and CC56 controls their common boost. The BPF display shows every stored centre and which sections are active. The Mackie restores the earlier input rolloff, asymmetric overload, broad body/presence shaping and second overload stage. Both nonlinear stages run at 4x with anti-alias FIR conversion; the dry lane is delayed 48 samples to match. This remains a musical approximation of a Mackie channel, not a component-exact circuit model.

Pre-v19 patches load with bitcrush off; pre-v18 CURVE is not imported as an LFO rate.

## Motion lane

On the sound menu, holding Function makes knob edits provisional: releasing it restores every value through the same snapshot the patch uses, which re-sends all CCs. Function + step 1 commits instead. While Function is held the focused parameter is sampled once per sequencer step; Function + step 2 turns that recording into a loop that keeps driving the parameter one value per step. Steps the gesture never reached hold the previous captured value, so a move shorter than a bar still loops as a complete shape.

On the FX menu Function is a persistent modifier: edits are kept on release. Function + step 2 can still record a lane, including Erosion frequency. A held reset or cycling an effect out releases its lane; a queued reset blocks lane writes until completion or cancellation.

Committing re-arms recording immediately, so reaching for another knob and pressing step 2 again layers a second loop rather than replacing the first. Each parameter owns one lane, so committing the same parameter twice replaces only its own loop. Function + step 3 is the only thing that clears them, and it clears all of them at once. Reaching for a different knob mid-gesture restarts the recording for that knob instead of splicing two parameters into one lane. A cleared parameter keeps whatever value the loop last wrote, which is the value the page is already showing.

STUT and LOOP cannot be driven this way. Their CC is a position inside a repeat session rather than a plain amount, so replaying values into it would desync the session from the Daisy's ladder. Every other parameter on the page is available. Playback runs from the foreground loop rather than the engine interrupt, so a value lands within a loop pass of its step rather than exactly on it — inaudible on a swept parameter, which is all this can address.

## Repeat sessions

STUT and LOOP keep virtual pot positions separately from canonical MIDI rate values. Positions 0–3 are OFF. The next accepted movement into ON starts a session. The two repeats no longer share a ladder or a starting rule.

STUT spans a quarter note down to an eighth triplet, alternating straight and triplet divisions (1/4T is 1/6, 1/8T is 1/12). It always opens at the slowest division and sweeps up as the knob rises — there is no random start, so a given knob position always produces the same rate.

| Division | STUT CC30 |
| --- | ---: |
| 1/4 | 16 |
| 1/4T | 48 |
| 1/8 | 80 |
| 1/8T | 112 |

LOOP still opens on a locally weighted random division:

| Division | LOOP CC31 | Start weight |
| --- | ---: | ---: |
| 1/2 | 13 | 55% |
| 1/4 | 38 | 25% |
| 1/8 | 63 | 12% |
| 1/16 | 88 | 6% |
| 1/32 | 114 | 2% |

A repeated LOOP starting draw is retried once, then replaced by a neighboring division if still equal. Starts through 1/16 move faster as the knob rises; faster starts move slower. Remaining travel to 127 is divided evenly, with two-count hysteresis around rate boundaries. Turning back retraces the session; returning to OFF ends it. Cycling a repeat slot preserves its current session. Neither repeat has a wet control.

## Implementation and display

`KickPerformance` separates authoritative parameter state, relative physical movement, button edges, sticky focus and pending MIDI output. The existing ten-millisecond button debounce supplies press/release edges. Signed circular angle differences handle the dual-track endless pots, including continuous turns through the angle seam. A full revolution corresponds to 128 value units. Net movement below two units is accumulated to reject one-count ADC jitter without delaying reversal with a low-pass filter. Switching FX/model/layer or entering Menu 2 clears residual movement and seeds a new angle reference; cycling an EXT-only slot sends no amount change. At a limit, outward motion and fractional overshoot cannot build up. The first deliberate inward movement reduces/increases the value normally.

MIDI is sent immediately from controller events if the UART has room. Each complete three-byte CC is enqueued with interrupts briefly masked, preventing an engine note from splitting the packet. When full, a fixed-size per-CC pending array retains the latest requested state for the next foreground pass. It never waits for UART space or OLED refresh. No MIDI originates in drawing functions.

The two displays redraw when dirty and at most 25 FPS; the idle FX overview animates at the same capped rate. Screen transfers remain foreground I2C operations; the existing timer engine continues handling sequencer timing. Parameter graphics use the specified logarithmic filters/BPF frequencies, decay and shape equations, tail timing from BPM, character landmarks, and pump depth. BPF inactive markers use sparse lines and unfilled labels because the OLED is monochrome. Existing save still writes sequencer EEPROM, but its full-screen splash is suppressed while the permanent performance grid is active.

## Validation

Run the host acceptance suite from the project root:

```sh
bash test/kick_host/run.sh
```

It compiles the actual controller implementation with hardware/drawing stubs and address/undefined-behavior sanitizers. It covers relative edits from arbitrary physical angles, repeated over-travel at both limits and immediate reversal, crossing the physical angle seam in either direction, switching destinations without jumps, the exact boot snapshot, UART backpressure, weighted starts, independent stored values, absolute buttons, repeat recall, long-press timing, sticky focus, ADC/seam noise, dirty-only 25 FPS rendering, and text bounds across all 128 values of every parameter. Optional test-binary argument `preview` writes SVG panel captures in the current directory.

Firmware compilation: `pio run -e teensy41`. Simulated checks and a successful firmware build do not replace a final physical pot/OLED/MIDI bench check. No firmware upload is performed by the tests.

REVERB retains its tail across triggers and ducks its return from the dry kick. Above 65%, damped eighth-note repeats fade in; tempo changes crossfade delay taps over 50 ms. See [listening examples](daisy-kick/analysis/reverb/README.md).
