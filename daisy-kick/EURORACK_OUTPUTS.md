# Seed3 trigger and MIDI-clock outputs

Firmware built and host-tested; not uploaded by the assistant.

| Signal | Seed name | Physical header pin | Behavior |
| --- | --- | --- | --- |
| Kick trigger | D2 / PC10 | 3 | Positive 5 ms pulse for each nonzero-velocity Note On on MIDI channel 15 |
| MIDI clock | D3 / PC9 | 4 | Positive 1 ms pulse per received MIDI F8, 24 PPQN |
| Ground | DGND | 40 | Common reference / jack sleeves |

These pin assignments are free in this firmware and the user reports no other
GPIO wiring besides MIDI RX on D1. D2 is also an optional USART3 TX pin, but
this firmware enables USART3 RX only. Neither output is an analog audio pin.

**Electrical levels:** GPIO outputs are 0–3.3 V regardless of VIN voltage.
Use a protected, non-inverting 3.3-to-5 V trigger driver for a module that
requires 5 V. VIN is not a logic-level selector. A series resistor limits
current but is not a complete protection circuit against Eurorack mispatches.
An external pull-down is appropriate if the receiving interface must stay
low before firmware initializes / during bootloader operation.

For MeeBilt 909 Kick Rev 2, schematic sheet 3 shows TRIG -> C4 (10 nF) ->
R21 (22k) -> Q5 base, with R28 (10k) to ground. Assuming ~0.7 V base turn-on,
the unloaded divider gives a nominal ~2.24 V onset; this is not a guaranteed
manufacturer threshold. A 3.3 V edge should trigger the internal transistor
pulse stage, but pulse shape, component tolerances and board revisions matter.
A test hookup is D2 through 1k to the TRIG jack tip, DGND to sleeve. The added
1k raises that nominal estimate to ~2.31 V. Use TRIG, not ACCENT. A buffered
5 V pulse provides extra margin if 3.3 V gives weak or missed hits.

Sources:
- [Seed3 datasheet, pin functions/electrical specifications](https://daisy.nyc3.cdn.digitaloceanspaces.com/products/seed3/Daisy_Seed3_datasheet.pdf)
- [MeeBilt 909 Kick Rev 2 schematic, sheet 3](https://github.com/tkilla64/eurorack/blob/main/909-kick/909_kick_sch_v2.pdf)

## Timing and routing

Outputs are serviced once at each real audio callback boundary: nominal
resolution 16/48000 = 0.333 ms. GPIO writes are deliberately outside the sample
loop: that loop computes a block in a burst and is not a wall-clock timer.
Pulse timing depends on the audio callback meeting its deadline; these tests
do not substitute for measuring the physical pins with a scope.

Note Off and velocity-zero Note On do not trigger. Other MIDI channels are
ignored. Internal repeats/ghosts do not create external kick pulses unless a
new MIDI Note On arrives. Every received clock is forwarded, including clocks
sent while MIDI transport is stopped. No autonomous ticks are generated after
clock input ceases; a high pulse still completes and returns low.

Queued kick pulses have at least 1 ms low between them; clock pulses have at
least one callback low. Bursts are serialized, not merged. This delays excess
bursts: kick throughput is at most ~166 pulses/s with these widths. Each queue
holds up to 256 pending events, with dropped-event counters on overload.
Audio recovery clears queues and returns both pins low. Normal boot initializes
both low. Hardware reset/DFU pins require external circuitry for a guaranteed
low state before initialization.

EURO_CLOCK_DIVIDER in midi_oled_monitor.cpp defaults to 1 (24 PPQN).
Set it to 6 and rebuild for sixteenth-note pulses (4 PPQN). MIDI Start resets
the divider phase; Continue retains it. The receiving module's clock mode must
match: many step sequencers expect 4 PPQN rather than raw MIDI clock.

## Tests

The production MIDI parser and callback are exercised against GPIO stubs:
correct pins, exact nominal widths, channel/velocity filtering, running status
with interleaved realtime, stopped-clock forwarding, 100 distinct queued pulses,
minimum low gaps, bounded overflow accounting, and reset-to-low.
Run `bash test/daisy_kick_low_end/run.sh` to include these tests.
