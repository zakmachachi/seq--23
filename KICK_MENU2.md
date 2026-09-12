# KICK performance controller — Menu 2

Select KICK in the existing Trigger Machines menu, then press MENU2. The controller owns all parameter state. Other sequencer menus, note/velocity routing, transport and pattern generation retain their existing behavior.

OLED1 always shows K1–K3 above K4–K6. The border identifies the focused knob. OLED2 stays on the last deliberately used knob or button indefinitely. All six endless pots use relative movement: their current physical angle becomes a movement reference, and turning immediately adjusts the selected parameter from its remembered value. Clockwise increases and counterclockwise decreases. Values clamp at 0 and 127; extra turns at either limit are discarded, so reversing responds without unwinding overshoot. There is no pickup target or PICK display.

All performance CCs use MIDI **channel 15** (status `0xBE`).

| Control | Turn | Press |
| --- | --- | --- |
| K1 / B1 | Selected FX: STUT 30, LOOP 31, DLY 32, HPF 33, LPF 34, PUMP 35 | Short release cycles FX locally. Hold 500 ms resets only CC30–34 to zero, once; no page advance. |
| K2 / B2 | DECAY 40 | REVERSE 41: 0 / 127 |
| K3 / B3 | TAIL DELAY 42 | TAIL ENABLE 43: 0 / 127 |
| K4 / B4 | BPF1 44, BPF2 45, BPF3 46 | Layer count 47: 0 / 42 / 85 / 127 |
| K5 / B5 | MACKIE 48 or SHERMAN 49, independently remembered | MODEL 50: 0 / 127 |
| K6 / B6 | SHAPE 51 | PUMP ENABLE 52: 0 / 127 |

BPF count 0 stages L1; counts 1, 2, 3 edit L1, L2, L3 respectively. Changing count sends no frequency. Changing character model sends no amount. Pump and tail enable buttons retain their amounts. B1's reset preserves pump amount/enable and all K2–K6 state. Its OLED2 overlay lasts 700 ms.

One snapshot is sent after MIDI initialization: CC30–35=0; CC40=64; CC41–43=0; CC44=35; CC45=65; CC46=95; CC47–50=0; CC51=64; CC52=0. Re-entering Menu 2 retains values and seeds fresh physical angle references. Values are session state; reboot restores these deterministic defaults. There are no CC100/101–105 commands or shared CC20–25 controls.

## Repeat sessions

STUT and LOOP keep virtual pot positions separately from canonical MIDI rate values. Positions 0–3 are OFF. The next accepted movement into ON starts a locally weighted random session:

| Division | STUT CC30 | Start weight | LOOP CC31 | Start weight |
| --- | ---: | ---: | ---: | ---: |
| 1/2 | 10 | 50% | 13 | 55% |
| 1/4 | 28 | 25% | 38 | 25% |
| 1/8 | 46 | 12% | 63 | 12% |
| 1/16 | 64 | 7% | 88 | 6% |
| 1/32 | 82 | 3% | 114 | 2% |
| 1/64 | 100 | 2% | — | — |
| 1/128 | 118 | 1% | — | — |

A repeated starting draw is retried once, then replaced by a neighboring division if still equal. Starts through 1/16 move faster as the knob rises; faster starts move slower. Remaining travel to 127 is divided evenly, with two-count hysteresis around rate boundaries. Turning back retraces the session; returning to OFF ends it. Switching FX preserves the session, including its starting rate, direction and virtual position. Neither repeat has a wet control.

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
