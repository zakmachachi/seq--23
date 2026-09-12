# Sherman / VCF-4 — parked notes

Status: **implemented and on the device, parked for revisit.** Commit `514300f`.

Modelled on the VCF-4 dual switched capacitor audio filter by Skull & Circuits ©2023 —
<https://www.skullandcircuits.com/blog/write-ups-2/vcf-4-2>. Credit also sits at the top of
the filter block in `midi_oled_monitor.cpp`.

## What was built

The hardware pairs two LTC1060 switched-capacitor cores. Those chips have no voltage or
current cutoff control at all — cutoff comes purely from a square-wave clock at **100× the
wanted frequency**. Two consequences drove the whole design:

- The integrator coefficient is a **constant** (`SC_G = tan(π/100)`), because the
  clock/cutoff ratio is fixed by the topology. Cutoff moves entirely with `clock_hz`.
- The core only advances **on a clock edge**. Once the clock falls under the audio rate the
  output holds between edges and steps — the write-up's *"bit crusher kind of effect on low
  filter settings"*. At 48 kHz that begins below **~480 Hz cutoff**, which is where a kick's
  body sits, so the crunch arrives by itself as the filter sweeps down.

Also carried over from the hardware:

- Resonance is the **bandpass fed back to the input through a VCA**, not a filter
  coefficient. `k` floor dropped 0.33 → 0.10 so it can self-oscillate.
- One control sweeps **LPF → BPF → HPF** across three VCA gains (`ModeMix`).
- Bipolar **−BP / 0 / +BP** sums on top of the blend, so subtracting the bandpass *nulls*
  it rather than dulling it. Nord Lead trick, cited in the write-up.
- Core B's clock tracks core A through a **divider**: 1:1 is the 24 dB cascade, 2:1 / 3:1
  give the octave spacing the article calls "incredibly useful" and acid-like.

Came out **smaller** than the state-variable version it replaced — no per-sample `tanf` or
coefficient slewing.

## Where it was left

Four things are front-panel controls on the real VCF-4 but are **fixed constants** here,
because the redesign happened before deciding whether to spend CCs and flash on them:

| Constant | Current | Range / meaning |
|---|---|---|
| `SHERMAN_MODE` | 0.46 | 0 = LPF, 0.5 = BPF, 1 = HPF |
| `SHERMAN_BP_POLARITY` | 0.35 | −1 … +1, the −BP/0/+BP switch |
| `SHERMAN_CLOCK_RATIO` | 2.0 | core B divider: 1 / 2 / 3, or free |
| `SHERMAN_SERIAL_ROUTING` | true | serial (A→B) vs parallel |

## Open questions for next time

1. **Expose the four as CCs?** CC60–63 sit free right after the mix page (53–59). That
   needs a Teensy-side page too. Flash was at 92.10% after this change.
2. **Is the stepping landing in the right place?** Core A tracks the fundamental at ×5.5,
   clamped 220–760 Hz. Lowering that multiplier makes the clock cross under the audio rate
   sooner, so the crunch arrives earlier and harder. This is the main tone dial.
3. **Free-running ratio** (the VCF-4's "Free" divider position) isn't implemented — core B
   currently always tracks A.
4. Unverified by ear at the time of parking beyond the initial flash.
