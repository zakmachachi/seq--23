# Held industrial FX — SEQ23 / Seed3

**When KICK is selected, the 16 step buttons are momentary FX pads on every
screen.** They no longer edit KICK steps or invoke Function + step shortcuts.
Other machines keep normal step editing. KICK's generated pattern, density
controls, transport, knob FX, MIDI receiver, trigger/clock GPIO and mono outputs
remain in place. Function edits on KICK persist without needing a step-button
commit. Function + Erosion still controls its frequency.

No new menu or knobs are required. OLED2 shows the held effect and gesture;
the existing FX screen's idle view lists the pad pairs. Blue pads are bar
builds; magenta pads are beat gestures. White marks the most recently pressed
pad; amber marks earlier keys still held.

| Bar build | Beat gesture | Effect | Transformation |
| ---: | ---: | --- | --- |
| 1 | 9 | HPF/RPT | Kick-only resonant HPF (100–1800 Hz) and 1/8 or 1/16 repeat; 1 holds its choice, 9 randomizes every sixteenth |
| 2 | 10 | TOY | Tiny speaker → unstable sample clock → low-resolution, failing-battery repeats/dropouts |
| 3 | 11 | RADIO | Narrow communication band → compressed/quantized transmission → flutter, interference and packets dropping out |
| 4 | 12 | MEMORY | Captured fragments → displaced microloops → reversals and shortened corrupted regions |
| 5 | 13 | METAL | Input excites six irregularly spaced resonances; size changes the object, character lengthens ringing |
| 6 | 14 | BINARY | PCM16 bit masking → patterned XOR corruption → bounded bit permutation, keeping the sign bit intact |
| 7 | 15 | DECAY | Preserved impact followed by progressive sample-rate/bit-depth deterioration; source transients restart degradation |
| 8 | 16 | HAZARD | Real Geiger speaker click at irregular intervals; gesture and source level increase density |

Pads 2–8 rise over four quarter notes and hold their strongest character until
released. Pads 10–16 repeat a shaped gesture each quarter note. These durations
are independent of pattern length and follow the existing MIDI BPM estimate.

Pressing another pad selects it at the next boundary. Releasing it returns to
an earlier held pad, restarting that gesture at the next boundary. Only one
algorithm runs at a time. Releasing the final pad fades to the ordinary external
FX chain over 8 ms. Changing effects fades the old one out over 8 ms, then the
new one in over 12 ms; there is no buffer clearing/allocation during that switch.

## Timing and safety

While transport runs, a request waits for the **next received kick or quarter
note, whichever arrives first**. A live press cannot be applied to a beat that
has already passed. MIDI quarter edges correct gesture phase; controls remain
smoothed. If no boundary arrives, the request falls back after one estimated
quarter note. With transport stopped a new press engages immediately. Stop
cancels an active hold; release and press again to play FX while stopped.

Channel changes clear holds. Releases from pads pressed on KICK remain consumed
when switching channels, so they cannot accidentally toggle another pattern.
The controller sends a keepalive every 200 ms; after 900 ms without one Seed
fades dry. A lost connection/timeout requires another press to resume.

The screen shows requested controls and an illustrative gesture, not Seed
acknowledgement or an audio meter. UI scan/UART latency and the 2-sample audio
block precede the actual boundary commit.

## Audio path

Existing external pump/repeat/filter, pitch, reverb, bitcrush and Erosion
processing → **held FX** → existing mono mixer/glue/ceiling.

Pads 1/9 exclusively process the generated kick before the mono mixer. A fixed
32768-sample ring captures the preceding kick fragment; playback freezes that
capture, without feedback or pitch-changing delay interpolation. Very early
presses without enough history apply the HPF to live kick until released.
Other pads affect only the external lane. The shared output glue still responds
to their combined level. HAZARD may generate clicks with silent external input.

If repeated audio callbacks consume over 90% of their deadline, the firmware
suspends the held processor and fades its last contribution away. Keepalives
cannot re-enable it; release and press again. This reserves main-loop time for
MIDI parsing when the held effect pushes a layered patch beyond the CPU budget.
The diagnostic fault count records suspensions; hardware timing still needs a
Seed test, since desktop runtimes do not establish the MCU's worst-case load.

All modes share an amplitude/transient/brightness analyser. TOY, MEMORY, BINARY
and DECAY leave the detected leading 3 ms dry and cross into processing over
the next 12 ms. A wet-return DC blocker and transparent-knee safety limiter
bound extreme material. At full release, bypass is exact. Deliberate quantization
and simulated memory faults are audible discontinuities within the sound;
selection and release themselves use fades.

HAZARD uses a 256-sample (5.33 ms) speaker click extracted from the CC0 Eberline
E-120 recording by cookies+policy:
https://freesound.org/people/cookies%2Bpolicy/sounds/555919/
The downloaded preview's isolated clicks contain approximately 95% of their
energy within 2 ms. The extracted click is DC-centred, normalized and tapered;
the source is CC0 and the table is embedded in HazardFx.h. Playback has no
oscillator, randomized pitch, steam, motor or warning tone. Four bounded voices
allow close clicks to overlap, with a 1.5 ms minimum event separation. Its PRNG
resets with each gesture and never calls system RNG.

## MIDI reference

Shared definitions: [HeldFxProtocol.h](include/HeldFxProtocol.h).
All CCs use **channel 15**; the existing receiver and its USART3 IRQ are unchanged.

| CC | Meaning | Values / default |
| ---: | --- | --- |
| 80 | Held pad selection | 0 releases; 1–16 address the pads above; invalid values release |
| 81 | CHARACTER ceiling | 0–127, default 127; scales the internally performed gesture |
| 82 | MOTION | 0–127, default 88; instability, wander and event activity |
| 83 | SIZE | 0–127, default 64; cavity scale, fragment length or degradation/release time, depending on effect |
| 84 | MIX | 0–127, default 112; smoothed dry/wet blend |
| 85 | Held-state keepalive | Any value; send at least every 200–500 ms while holding |

The Teensy sends pad selection and keepalive; the other four CCs are optional
external-sequencer overrides, not newly assigned knobs. They are global to this
held subsystem, default at Seed startup and are not saved as latched effects.
Repeated CC80 restarts/requeues a gesture; use CC85 to sustain it instead.

## Implementation / practical limits

- [HeldFxButtons.h](include/HeldFxButtons.h): priority, release ownership,
  transport/channel cancellation and nonblocking MIDI retries.
- [HeldFxEngine.h](daisy-kick/effects/HeldFxEngine.h): quantized requests,
  smoothing, fades, input analysis, safety and lease handling.
- [Blocks.h](daisy-kick/effects/Blocks.h),
  [IndustrialFx.h](daisy-kick/effects/IndustrialFx.h),
  [HazardFx.h](daisy-kick/effects/HazardFx.h): bounded history, filters,
  deterministic noise and the eight algorithms.
- Existing `SimpleSequencer`, `KickPerformance`, `KickMixer` and
  `midi_oled_monitor.cpp` provide the button/display/MIDI/audio integration.

The new engine uses about **261 KiB of SDRAM**, mostly a 65,536-sample mono
history (1.37 seconds). It introduces no large per-switch clears, callback heap
allocations, blocking calls, FFTs or parallel algorithm stack. Filter tuning runs
at 250 Hz; DECAY's expensive mapping updates every 32 samples. BANE, RADIO and
HAZARD are the main extra CPU candidates; MEMORY/BINARY are lighter. Existing
`audio_max_cycles` and `audio_overruns` instrument the entire Seed callback.
Host timings do **not** establish a Seed CPU percentage: profile on hardware
with the heaviest existing Mackie/FX settings before relying on every combination.

Deliberate approximations: BANE uses formants/ring/comb instead of adding another
pitch shifter; METAL uses six modal filters rather than physical modelling;
MEMORY is a bounded, crossfaded glitch sampler, not transparent time stretching.
Its variable delays and the intentional resonances are coloration, not a promise
of phase-transparent wet/dry mixing. The hardware's existing mono architecture is
preserved. CHIPMUNK and NIGHTCORE are excluded as requested.

Tune by ear first: BANE/RADIO level and intensity (CC81/84), METAL object size
(CC83), MEMORY fragment length (CC83), DECAY degradation time (CC83), and HAZARD
activity (CC81/82). The defaults favour controlled levels; the procedural layer
is intentionally quieter than the original audio.

## Validation

`bash test/held_fx/run.sh` checks priority, release ownership, retries, disconnect
lease, causal quantization, Stop, all 16 gestures, macro endpoints, finite bounded
output, circular-buffer wrapping, exact release and preserved DECAY attack under
ASan/UBSan. `bash test/held_fx/integration.sh` exercises the production MIDI
parser/audio callback, clock/kick start, GPIO continuity and per-pad isolation.
`bash test/kick_host/run.sh` also covers all held-effect OLED text bounds.

Build both firmware images for this feature. No upload is part of this change.
