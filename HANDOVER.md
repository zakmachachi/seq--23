> **Superseded (2026-09-24).** This describes the pre-redesign engine and its
> phase bridge, which no longer exist. The kick was rebuilt on
> `kick-redesign-v2` and merged to main: one phase-reset sine with punch and
> sub paths, retriggers handed off in phase, and the Mackie surroundings
> restored to match 3af0e35. The click causes and what not to reintroduce are
> in the comments at the top of the KICK VOICE section of
> `daisy-kick/midi_oled_monitor.cpp`. Kept for the history and the build and
> flash notes, which still apply.

# Handover — the retrigger click on the Daisy kick

Branch `main`, base `3af0e35`. Everything below is **uncommitted** in
`daisy-kick/midi_oled_monitor.cpp` plus the new `daisy-kick/host/`.

---

## The problem is found, and mostly fixed

**Root cause: the sub lane was gated off at every retrigger.**

`TriggerKickAudio()` sets `kick_age_samples = 0` on every hit, and
`GetTransientTailCrossfade()` derives the transient/tail handoff purely from
that age. So on an overlapping ratchet the tail gain **stepped from ~1 back to
`sin(0) = 0`** and needed the full ~44 ms handoff to climb out again.

On a fresh hit that is right — there is no tail yet. On an overlap it truncated
a sub sine that was still at full amplitude. The transient lane rising in its
place is what normally hides the hole, so **with PUNCH at 0% nothing filled
it**: the sub was ramped to digital silence inside the 5 ms phase bridge and
the output then sat at **exact zero for ~30 ms** before the next hit faded in.

That single mechanism accounts for every property on the old bench list:
scales with SUB, gone at SUB 0, needs an overlap, louder at long/infinite
decay, absent with a short decay, masked by PUNCH, and only obvious once the
DJ HPF removes the fundamental it is hiding under.

### Measured, sub 75% / punch 0 / decay 75% / 185 BPM 16ths

```
        retrigger at 0.331 s, 2.5 ms bins
  before                           after
  0.3300  0.26970 #############    0.3300  0.26970 #############
  0.3325  0.04326 ##               0.3325  0.06033 ###
  0.3350  0.01352 #                0.3350  0.09088 ####
  0.3375  0.00000                  0.3375  0.11272 #####
  0.3400  0.00000     <- silence   0.3400  0.21824 ##########
  ...     0.00000        ~30 ms    ...
  0.3650  0.00010                  0.3650  0.30486 ##############
```

---

## The four changes

1. **Tail handoff floor** — `ratchet_tail_handoff_floor`.
   The new hit's tail curve rises **from the level the tail was already at**
   instead of from zero. Captured once per trigger, `0` on a fresh hit.
   The transient lane is deliberately *not* touched: resuming the equal-power
   curve at the old position instead would attenuate the transient by the same
   amount, and because that position ratchets upward hit after hit, a roll
   loses its attack entirely. That was tried and rejected — it made PUNCH
   inaudible after two hits.

2. **Exact bridge continuation** — a two-pole resonator replaces the
   amplitude estimate.
   `y[n] = 2cos(w)y[n-1] - y[n-2]`, seeded with the last two samples the
   engine really produced. That is the exact continuation of the sinusoid
   those samples were on: right amplitude, right phase, **right slope**, with
   nothing estimated.
   The old bridge reconstructed `sin(phase) * guessed_amplitude` and patched
   the error with an additive residual decayed over `RATCHET_RESIDUAL_DECAY_MS`.
   That made the *value* continuous but not the *derivative* — the residual
   lost ~20 % of itself in the first sample, and a slope kink is an impulse.
   Confirmed on the waveform: `d2` spiked ~100x at the trigger sample.
   This removes `generated_output_envelope`, `ratchet_bridge_residual`,
   `ratchet_bridge_tail_gain`, `ratchet_bridge_phase` and
   `RATCHET_RESIDUAL_DECAY_MS`.

3. **Bridge 5 -> 8 ms, complementary crossfade.**
   `new_gain` goes back to `smooth_t`; the untested "voice steal"
   (`new_gain = 1.0`) is reverted. Unity only works while something else holds
   the new voice down for the length of the bridge, which is exactly the tail
   gating that change 1 removes — with the tail lane now continuous it
   double-counts, measured **1.8x worse than base at PUNCH 100**.
   8 ms is the knee: click energy still falling, attack retention still flat
   (it collapses from 9 ms upward).

4. Kept from the previous session: the **mix-gain slew** (`param_*_gain_target`)
   and the **DJ LPF on the kick chain**. Both unrelated to the click.
   `tail_env.attack` is back to **2.5 ms** — see below.

### Result (vs `3af0e35`, click = HP-500 Hz peak at the retrigger / sub level)

| case | click before | after | level hole before | after |
|---|---|---|---|---|
| sub 75, punch 0, decay 75 | 0.0315 | **0.0225** | 0.000 | 0.322 |
| sub 100, punch 0, decay 75 | 0.0488 | **0.0225** | 0.000 | 0.322 |
| sub 40, punch 0, decay 75 | 0.0339 | **0.0225** | 0.000 | 0.322 |
| sub 75, punch 0, decay 95 | 0.0316 | **0.0227** | 0.000 | 0.280 |
| sub 75, punch 0, decay 50 | 0.0294 | **0.0216** | 0.000 | 0.533 |
| sub 75, punch 100 | 0.0561 | **0.0247** | 0.526 | 0.430 |
| full mix | 0.0420 | **0.0229** | 0.326 | 0.372 |

"level hole" is the lowest envelope reached between hits as a fraction of the
level just before the retrigger; **0 means it went completely silent**.
Attack retention on a ratchet is 1.09 (1.0 = as hard as a fresh hit), so none
of this was bought by softening the transient.

**With no overlap the change is bit-identical** — verified by rendering
against a build with the floor forced to zero: identical at 60/90/120 BPM
(0 overlapping hits), differing at 150/185 BPM (3 of 4 hits overlapping).

Builds at **93.67 %** flash, against 93.64 % at `3af0e35`.

---

## Second bug: the DJ HPF had an "onset protection" that was the click

Reported on the bench after the above was flashed: the retrigger click was
gone, but engaging the DJ HPF produced a big click at **any** cutoff, and at
1.5 kHz the click was all there was.

`AddedDjHighpass::Process(x, protect_kick_onset = true)` on the kick lane held
the path **fully dry for the first 4 ms of every hit** and crossfaded into the
HPF over the next 12 ms. The stated reason was that a high-pass emphasizes a
kick's step-like onset and reads as a brittle click.

**It never did that.** Rendered onset impulse — max `|d2|` over the first 30 ms
of a hit — is the same with it and without, 0.00059 either way, at every
cutoff. What it actually did was inject 4 ms of full-level *unfiltered* kick,
which at high settings is exactly the sub the control was asked to remove.

| DJ HPF | dry burst (0–4 ms) | settled (>20 ms) | burst / settled |
|---|---|---|---|
| 40 Hz | 0.0805 | 0.0798 | 1.0x |
| 300 Hz | 0.0805 | 0.0117 | 6.9x |
| 800 Hz | 0.0805 | 0.0029 | 27.4x |
| 1500 Hz | 0.0805 | 0.0010 | **80.2x** |
| 3000 Hz | 0.0805 | 0.0003 | **278.5x** |

The burst was the loudest thing in the output at every setting above ~300 Hz.
Removing it drops the onset peak 18x at 1.5 kHz and 53x at 3 kHz, with the
onset impulse unchanged, and takes the retrigger click at 40 Hz back to what
it is with the HPF off (0.0217 vs 0.0225).

Removed: the `protect_kick_onset` parameter, the branch, and
`PerformanceFilterKickOnsetBlend()` with its two constants. A note on
`AddedDjHighpass` records why it must not come back.

**What remains at high cutoffs is the kick's own attack** — burst/settled
settles at ~6x from 800 Hz up, and that is the effect working. A kick through
a 1.5 kHz high-pass *is* mostly its click; there is nothing left to be wrong.

---

## The host harness — use this first

`daisy-kick/host/` compiles the firmware on the desktop against stub hardware,
runs its real `main()` init path, injects MIDI and renders the kick bus to a
float32 file. See `daisy-kick/host/README.md`.

Every number above came from it. It is the reason this session got somewhere
after several that did not: reasoning about this file from the source has now
failed repeatedly, and rendering the waveform answered it in one pass. **Do
not change the kick DSP without rendering before and after.**

---

## Ruled out, with numbers — do not re-tread

| Tried | Result |
|---|---|
| Preserving `sub_phase` across an overlap | **14x worse** (0.047 -> 0.67). Also worse with the bridge left on (0.083). |
| Disabling the phase bridge entirely | **16x worse** (0.75). It is load-bearing. |
| Gating `ResetDspStateForThisTrigger()` on `!kick_retrigger_active` | **12x worse** (0.59), and identical whether the bridge is on or off, so the zeroing is preventing a bang, not causing one. The comments in that file claiming the state must survive are **wrong**; the guard is right. |
| `new_gain = 1.0` voice-steal | 1.8x worse at PUNCH 100. Reverted. |
| Equal-power resumption of the handoff curve | Kills the attack on rolls — the resume position ratchets to 1 and PUNCH vanishes. |
| Ramping the floor in over 1–8 ms | Worse at 300–500 Hz; the cancellation notch costs more than the slope step it removes. |
| Bridge 12–20 ms | Lower click, but attack retention falls to 0.68 / 0.57. The click metric rewards gutting the transient — always check retention too. |
| `PSY_PHASE_LOCK_EVERY_HIT = false` | Worse (previous session). |
| `RATCHET_RESIDUAL_DECAY_MS` 0.65 <-> 3 ms | No help (previous session); the constant no longer exists. |
| `ENABLE_PROTECTED_PUNCH`, `tail_pitch_phase` | Not involved; already off / vestigial. |
| DJ HPF onset dry-hold + crossfade | Removed. It did not reduce the onset impulse at all and was itself the click — see above. |

## The "decay is no longer smooth, it is clicking" report

**Not reproducible, and the prime suspect was wrong.** `tail_env.attack` at
2.5 ms and 5 ms render *identically* on a single hit (decay-body HF 0.000390
in both), so the 5 ms attack was not causing it. Across a 900 ms decay the HF
residue tracks the level proportionally at ~0.0004 — that is the sine's own
harmonic floor, not a click or a zipper.

Most likely the report was the retrigger click heard on a rolled pattern. If
it recurs on a *single* hit, render it before changing anything.
`tail_env.attack` has been put back to 2.5 ms since 5 ms bought nothing.

---

## What is left

The residual at the trigger is now a single-sample **slope step**, from the
phase-reset sub entering at full level with slope `A*w`. It is what remains of
the click, at roughly -33 dB relative to the sub (was -26 dB).

Removing it properly needs a **phase-steered bridge**: continue the old tail
but bend its phase to meet the new hit's over the bridge, so the two are in
phase at the handover and the complementary fade becomes a pure amplitude
interpolation with no cancellation. Every cheaper alternative was tried and
measured above; they all trade the slope step for a notch or for the attack.
Sketch: solve `A` and `theta` from the resonator seed, then synthesise
`A*sin(theta + w*n + S(t)*delta)` with `delta` the wrapped phase error and `S`
a smoothstep.

Worth doing only if it is still audible on the bench. Render it first.

---

## Other open items, unrelated to the click

- **ext-in STUT** arms only on a kick boundary, so it never engages with no
  kick playing. Needs a MIDI-clock fallback — a behavioural decision.
- **ext-in LOOPER** never implemented; needs a second large history buffer and
  SRAM is at ~87 %.
- **PR #1** on the `deagleskyline-create` fork was never updated. `main`
  already contains it, so closing it is tidier.

---

## Build and flash

Homebrew's `arm-none-eabi-gcc` has no newlib and cannot build this. Use the
extracted toolchain, and note `make` does **not** expand `~` in these
variables — pass `$HOME`.

```sh
export PATH="$HOME/Documents/PlatformIO/Projects/DaisyLibs/armgcc/Payload/bin:$PATH"
cd ~/Documents/PlatformIO/Projects/Sequencer_prototybe

# Daisy — hold BOOT, tap RESET to enter DFU first
make -C daisy-kick \
  LIBDAISY_DIR="$HOME/Documents/PlatformIO/Projects/DaisyLibs/libDaisy" \
  DAISYSP_DIR="$HOME/Documents/PlatformIO/Projects/DaisyLibs/DaisySP"
dfu-util -a 0 -s 0x08000000:leave -D daisy-kick/build/midi_oled_monitor.bin -d ,0483:df11

# Teensy
~/.platformio/penv/bin/pio run -e teensy41 -t upload
~/.platformio/penv/bin/pio run -e teensy41 && bash test/kick_host/run.sh
```

`dfu-util` always ends with `Error during download get_status` — harmless.
`File downloaded successfully` above it is the real result.

### A trap when editing this file

A **non-zero default member initialiser** on a struct containing large buffers
moves the whole object from `.bss` into `.data`, storing its zero image in
FLASH. `float duck_gain = 1.0f;` on `KickSidechainReverb` (32 KB of combs)
pushed the build to 119.8 % and failed to link. Default such members to zero
and set the real value in `Reset()`.
