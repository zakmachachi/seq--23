# KICK sound and FX menus

Select KICK in Trigger Machines. MENU2 keeps the sound controls; MENU3 (the
Euclid button) opens the six-slot **FX menu**. Other machines retain Euclid.
FUNCTION + MENU3 still opens Pages. All CCs use MIDI channel 15 (`0xBE`).

## FX menu — MENU3

| Knob | Effect / turn | Short click |
| --- | --- | --- |
| 1 | DELAY → LOOP → STUT → PITCH; amount or repeat position | Cycle these EXT-only effects |
| 2 | HPF → LPF; cutoff macro | Cycle these EXT-only effects |
| 3 | PUMP amount | EXT ↔ EXT+INT |
| 4 | REVERB amount | EXT → EXT+INT → INT → EXT |
| 5 | BITCRUSH amount | EXT ↔ EXT+INT |
| 6 | EROSION amount/noise macro | EXT ↔ EXT+INT |

**Function + turn knob 6 sets Erosion frequency, 80 Hz–12 kHz.** OLED2
shows the frequency on a logarithmic axis, with a band illustrating centre
and noise spread. Releasing Function keeps the edit. Knob 6 returns to amount
without carrying accumulated motion into that parameter.

Hold any knob for one second to queue **that effect alone** to zero at the
end of the current four-quarter bar (96 MIDI clock ticks from Start).
An asterisk marks queued resets. Releasing a held knob does not also switch its
route/model. Turning that effect again cancels its queued reset. Resets continue
when leaving the page. With transport stopped, resets apply immediately; stopping
with a reset queued also clears it. Existing smoothing/fades apply at the boundary,
so zeroing the control does not truncate a waveform abruptly. Clock must be received
by Seed for a running bar to complete.

Clicking a cycling slot changes the edited effect only. Its previous effect
keeps playing at its stored amount; cycling back resumes editing that value.
Hold the selected knob to reset that effect explicitly.

PITCH is EXT-only, ±12 semitones (CC93): centre 64 is dry/zero shift;
0 shifts down an octave, 127 up an octave. Hold resets to zero semitones at the
bar boundary, not to the bottom of the knob range. This is a lightweight
windowed-delay shifter with smoothed changes. Patch format v22 saves its value.

OLED1 permanently shows knobs 1–3 above 4–6, amounts and tiny `E` (EXT) or `+`
(EXT+INT) indicators; `I` marks internal-only reverb. OLED2 follows the latest deliberate manipulation, with
repeat cells, filter curves, pump envelopes, reverb/delay decay graphics, quantized
waves and the Erosion band. After five seconds it shows an animated six-slot
overview. These graphics illustrate controls, not measured audio levels.

PUMP defaults to EXT; REVERB, BITCRUSH and EROSION default to EXT+INT. Reverb additionally offers INT-only, leaving external audio dry; its route changes
use the existing 30 ms smoothing. Those four effects have separate internal/external DSP state where required. Turning
off the internal route fades its processing back to dry; it does not switch the
external processor or move kick history into it. BITCRUSH preserves its original
kick placement: **dirty return before HPF**, protecting clean bass. On EXT it
processes the external bus. Thus internal bitcrush needs an audible dirty return.

## Sound menu — MENU2

| Knob | Turn | Press |
| --- | --- | --- |
| 1 | Navigation hint: FX on MENU3 | — |
| 2 | DECAY / CC40 | REVERSE / CC41 |
| 3 | TAIL DELAY / CC42 | TAIL ENABLE / CC43 |
| 4 | BPF1/2/3 / CC44–46 | Layer count / CC47 |
| 5 | MACKIE / CC48 or TUBE / CC49 | Model / CC50 |
| 6 | SHAPE / CC51 | — |

Pump enables from its FX amount; the old SHAPE-button pump toggle is removed.
The pots use relative movement, clamp at 0–127 and discard overshoot at either
limit, so reversing direction responds immediately. Page changes seed fresh
physical references. BPF/model memories remain independent.

## FX MIDI protocol

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

## Tail delay and the mix page (v1.3.0)

TAIL DELAY makes a **kick–break–bass gate**. The front plays for the BELLY window, closes over 3 ms, stays silent for the selected gap, then opens over 3 ms. Both clean and dirty kick audio are gated together, after their filters, while the oscillators keep running. The gap length is half a quarter note × (amount / 127)^1.7; its maximum is one eighth note. Switching TAIL ENABLE off, or setting the amount to zero, removes the gate. There is no delayed copy, automatic sub ducking or slow recovery envelope.

The single body oscillator resets every hit and follows the pitch sweep. SUB is a clean bass shelf, not an octave oscillator. The amplitude holds for two cycles of the base note, then DECAY sets 45 ms–2.4 s to −30 dB. SHAPE zero disables the attack sweep/pulse/noise with an 8 ms onset. SHAPE controls attack sweep depth up to 48 semitones and attack character; SWEEP sets timing independently. Long SWEEP with shortest DECAY may end before reaching settled bass, deliberately allowing short laser sounds.

The mix page (FUNCTION + MENU2) has these roles:

| Control | Function |
| --- | --- |
| LINE / CC53 | Linear kick-bus level, 0–100%; external level is separate. Internal output trim reserves headroom for the boosted body and driven return. |
| DIST / CC54 + CC55 | Dirty-return level. The controller sends Tube at 80/65 of Mackie's value, capped at 127. This is separate from K5's distortion amount. |
| BELLY / CC61 | Gate start: 50 ms × 0.35–2, never before pitch settling plus one sub cycle. Value 64 is 1×. No effect with tail gate off. |
| BPF / CC56 | Blend from the full kick into the normalized pre-distortion BPF bank. Zero layers bypasses the bank. |
| SUB / CC57 | Clean bass boost: 0 is flat; 100% blends in the full +15 dB / 120 Hz low shelf. No new lower pitch is generated. |
| PUNCH / CC58 | Clean low-pass lane gain, 0–100%. Zero mutes this lane including its SUB bass boost; DIST remains independent. |

The body/attack is cloned into clean LPF → PUNCH and serial mid EQ / Mackie preamps (or Tube) → BITCRUSH → HPF → DIST. SUB boosts the clean LPF output before PUNCH, never distortion's input. Clean LPF and dirty HPF are matched fourth-order Linkwitz-Riley filters at 240 Hz. Filtering remains phase-shifting, but the generators cannot drift and the linear crossover paths have matching phase. Nonlinear/BPF settings still change dirty harmonic phase; they are not guaranteed to reconstruct the original waveform.

Mixer1 feeds the tail gate, then reverb. An enabled reverb can fill the chopped gap. Processed external audio meets the kick at mixer2; both physical outputs carry the combined mono mix. Mixer2 uses fixed ×0.8 trim then gentle 1.25:1 glue above −12 dBFS, 30 ms detector attack / 150 ms release and maximum 2 dB reduction, followed by a fixed 2× (+6 dB) output makeup. The external branch is trimmed by the same factor, and an unplugged input is gated to silence. Kick LINE reaches unity at 127 with fixed ×0.06 trim; DIST spans 0–1.25, PUNCH/BPF/SUB 0–1. The kick's 15 Hz infrasonic filter precedes the gate (about −0.26 dB at 30 Hz). Performance HPF affects external input only. Emergency ceiling remains above tested normal peaks.

Four per-hit kick controls live on Menu 1:

| Knob | Control | MIDI / range |
| --- | --- | --- |
| 2 | WAVE | CC64: sine toward phase-derived saw harmonics |
| 3 | SWEEP | CC78: 4–240 ms, logarithmic; independent of SHAPE and DECAY |
| 5 | TMOD | CC79: 0 = single glide, 1–127 = .125–16 Hz; button or Function + turn resets to 0 |
| 6 | TUNE | Velocity / CC77: −12 to +12 semitones; 64 neutral, fine resolution near centre |

TMOD resets its LFO on every hit. TUNE supplies its direction/depth; at 64, TMOD has no audible effect. With TMOD off, tail pitch glides once toward TUNE over the SWEEP duration. With TMOD on it moves between the base note and that excursion, beginning after the initial attack sweep settles. The Note-On velocity carries TUNE. Only value 0, which a Note-On cannot carry, goes out as CC77, and only if it can precede that hit; the velocity itself is clamped to at least 1. SWEEP, TMOD and WAVE are deduplicated per-hit CCs. The screen scope shows the clean generator, not the output filters/distortion/glue.

BPF is a panel label for **mid boost**: each CC44–46 maps logarithmically to 85–3200 Hz, with two-octave bandwidth and up to +15 dB from the mix-page BPF gain. Count 0 means one preamp without mid EQ; counts 1–3 enable one to three EQ/preamp stages in series for Mackie. Tube uses the same EQ bank before its two-stage model. Mackie runs at 192 kHz with anti-imaging/alias FIRs; the clean lane has matching 0.5 ms latency. It is a musical approximation, not an exact Mackie circuit model.

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
