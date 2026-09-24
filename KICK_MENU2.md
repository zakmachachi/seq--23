# KICK performance controller — Menu 2

Select KICK in the existing Trigger Machines menu, then press MENU2. The controller owns all parameter state. Other sequencer menus, note/velocity routing, transport and pattern generation retain their existing behavior.

OLED1 always shows K1–K3 above K4–K6. The border identifies the focused knob. OLED2 stays on the last deliberately used knob or button indefinitely. All six endless pots use relative movement: their current physical angle becomes a movement reference, and turning immediately adjusts the selected parameter from its remembered value. Clockwise increases and counterclockwise decreases. Values clamp at 0 and 127; extra turns at either limit are discarded, so reversing responds without unwinding overshoot. There is no pickup target or PICK display.

All performance CCs use MIDI **channel 15** (status `0xBE`).

| Control | Turn | Press |
| --- | --- | --- |
| K1 / B1 | Selected FX: STUT 30, LOOP 31, DLY 32, HPF 33, LPF 34, PUMP 35, REV 36 | Short release cycles FX locally. Hold 500 ms resets only CC30–34 and CC36 to zero, once; no page advance. |
| K2 / B2 | DECAY 40 | REVERSE 41: 0 / 127 |
| K3 / B3 | TAIL DELAY 42 | TAIL ENABLE 43: 0 / 127 |
| K4 / B4 | BPF1 44, BPF2 45, BPF3 46 | Layer count 47: 0 / 42 / 85 / 127 |
| K5 / B5 | MACKIE 48 or TUBE 49, independently remembered | MODEL 50: 0 / 127 (Mackie / Tube) |
| K6 / B6 | SHAPE 51 | PUMP ENABLE 52: 0 / 127 |

BPF count 0 stages L1; counts 1, 2, 3 edit L1, L2, L3 respectively. Changing count sends no frequency. Changing character model sends no amount. Pump and tail enable buttons retain their amounts. REV is a plain 0..127 amount displayed as a percentage, OFF at zero. B1's reset preserves pump amount/enable and all K2–K6 state. Its OLED2 overlay lasts 700 ms.

FX lanes are not all the same. STUT processes both the kick and the Digitakt return, each with its own filter state. HPF and LPF also cover both. LOOP, DLY and PUMP are external-only; REV is kick-only. The kick lane is therefore STUT → HPF → LPF, and the external lane is PUMP → DLY → STUT → LOOP → HPF → LPF. The kick's chop still engages on its next kick boundary, but everything on the external lane engages on the next sixteenth instead, so the Digitakt keeps chopping when the kick channel is silent. With the transport stopped the external lane applies changes immediately, since no sixteenth will arrive.

While the transport is running, TAIL ENABLE and B1's reset are applied by the Daisy on the next quarter-note pulse rather than on receipt, so both land on the beat. With the transport stopped there is no pulse to wait for and both apply immediately; a STOP arriving with either still pending applies it there and then rather than swallowing it. The controller's display and CC output are unchanged — the deferral happens entirely on the Daisy, which already tracks MIDI clock.

One snapshot is sent after MIDI initialization: CC30–36=0; CC40=64; CC41–43=0; CC44=35; CC45=65; CC46=95; CC47–50=0; CC51=64; CC52=0. Re-entering Menu 2 retains values and seeds fresh physical angle references. Saving the patch stores all performance and mix values in EEPROM; boot restores them and re-sends every CC through the same pending mechanism, since the Daisy has no persistence. Without a saved patch these deterministic defaults stand. There are no CC100/101–105 commands or shared CC20–25 controls.

## Tail delay and the mix page (v1.3.0)

TAIL DELAY is an envelope over the whole kick: full level through the punch, then (when enabled) the whole voice drops out over 6 ms and comes back at the tail delay time, rising over TAIL ATTACK. A delay shorter than the punch leaves no gap. The mix page (FUNCTION + MENU2) holds TAIL ATTACK on pot 3 (CC61, 6–60 ms), where the Tube gain was, and DIST on pot 2 sets both distortion gains at once: Mackie CC54 at the dial's value and Tube CC55 at 80/65 of it, the ratio their separate defaults had.

Three per-hit kick controls live on Menu 1 rather than here. On a KICK channel, pot 2 is WAVE (CC64, sine to supersaw), pot 3 is SWEEP (CC62, the punch sweep time, 0.25x–4x SHAPE's own) and pot 5 is TMOD (CC63, a wobble macro: 0 off, rising to ±2 semitones at up to 12 Hz and increasingly irregular); pot 6's velocity is shown as PITCH, in semitones. The CCs are sent from the step engine just before each kick note, only when they change.

## Motion lane

Holding Function makes knob edits on this page provisional: releasing it restores every value through the same snapshot the patch uses, which re-sends all CCs. Function + step 1 commits instead. While Function is held the focused parameter is sampled once per sequencer step; Function + step 2 turns that recording into a loop that keeps driving the parameter one value per step. Steps the gesture never reached hold the previous captured value, so a move shorter than a bar still loops as a complete shape.

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

A repeated LOOP starting draw is retried once, then replaced by a neighboring division if still equal. Starts through 1/16 move faster as the knob rises; faster starts move slower. Remaining travel to 127 is divided evenly, with two-count hysteresis around rate boundaries. Turning back retraces the session; returning to OFF ends it. Switching FX preserves the session, including its starting rate, direction and virtual position. Neither repeat has a wet control.

## Implementation and display

`KickPerformance` separates authoritative parameter state, relative physical movement, button edges, sticky focus and pending MIDI output. The existing ten-millisecond button debounce supplies press/release edges. Signed circular angle differences handle the dual-track endless pots, including continuous turns through the angle seam. A full revolution corresponds to 128 value units. Net movement below two units is accumulated to reject one-count ADC jitter without delaying reversal with a low-pass filter. Switching FX/model/layer or entering Menu 2 clears residual movement and seeds a new angle reference; selecting a destination sends no parameter value. At a limit, outward motion and fractional overshoot cannot build up. The first deliberate inward movement reduces/increases the value normally.

MIDI is sent immediately from controller events if the UART has room. Each complete three-byte CC is enqueued with interrupts briefly masked, preventing an engine note from splitting the packet. When full, a fixed-size per-CC pending array retains the latest requested state for the next foreground pass. It never waits for UART space or OLED refresh. No MIDI originates in drawing functions.

The two displays redraw only when dirty and at most 25 FPS. Screen transfers remain foreground I2C operations; the existing timer engine continues handling sequencer timing. Parameter graphics use the specified logarithmic filters/BPF frequencies, decay and shape equations, tail timing from BPM, character landmarks, and pump depth. BPF inactive markers use sparse lines and unfilled labels because the OLED is monochrome. Existing save still writes sequencer EEPROM, but its full-screen splash is suppressed while the permanent performance grid is active.

## Validation

Run the host acceptance suite from the project root:

```sh
bash test/kick_host/run.sh
```

It compiles the actual controller implementation with hardware/drawing stubs and address/undefined-behavior sanitizers. It covers relative edits from arbitrary physical angles, repeated over-travel at both limits and immediate reversal, crossing the physical angle seam in either direction, switching destinations without jumps, the exact boot snapshot, UART backpressure, weighted starts, independent stored values, absolute buttons, repeat recall, long-press timing, sticky focus, ADC/seam noise, dirty-only 25 FPS rendering, and text bounds across all 128 values of every parameter. Optional test-binary argument `preview` writes SVG panel captures in the current directory.

Firmware compilation: `pio run -e teensy41`. Simulated checks and a successful firmware build do not replace a final physical pot/OLED/MIDI bench check. No firmware upload is performed by the tests.
