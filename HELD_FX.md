# Held industrial FX — SEQ23 / Seed3

Step buttons **1, 2, 3, 4, 9, 10, 11, 12 and 13** control the Kick Seed on
every selected channel and page. They do not affect the selected channel's
sound. Kick edits and retriggers remain routed to MIDI channel 15. Their
quantized edges use actual kick events where noted below. The other industrial
FX pads remain available when KICK is selected; ordinary step editing and
Function shortcuts remain available elsewhere. Function + Erosion still
controls its frequency.

No new menu or knobs are required. OLED2 shows the held effect and gesture;
the existing FX screen's idle view lists the pad pairs. Blue pads are bar
builds; magenta pads are beat gestures. White marks the most recently pressed
pad; amber marks earlier keys still held.

| Step | Held control | Behavior |
| ---: | --- | --- |
| 1 | KICK MUTE | Suppresses new generated hits while held. Existing kick tails keep ringing; MIDI triggers and clock continue. The first kick after release sounds normally. |
| 2 | RANDOM KICK HPF | Chooses a held cutoff from 140–800 Hz on the first kick, then bypasses on the next kick after release. |
| 3 | KICK +1 | Raises kick MIDI pitch by one semitone from the next kick until the first kick after release. |
| 4 | KICK DECAY 40 ms | New hits use the minimum 40 ms body decay while held. A hit already ringing keeps the decay it started with. |
| 5–8, 13–16 | INDUSTRIAL FX | Existing bar-build and beat gestures on external audio. 5/13 METAL, 6/14 BINARY, 7/15 DECAY, 8/16 HAZARD. |
| 9 | BPF + RETRIGGER | On any selected channel, holds the random three-layer Kick BPF and retriggers the kick on clocked eighth notes. It does not record or replay audio. |
| 10 | RATCHET | On any selected channel, internal Kick retriggers accelerate 1/4 → 1/8 → 1/16 → 1/32 across one bar. |
| 11 | KICK LEVEL DOWN | Ramps the kick to a quiet level over about two seconds; release restores it over about half a second. Press and release take effect at the next kick. |
| 12 | DUB ECHO | On any selected channel, shortens new kick decays and adds the clocked, high-passed echo/reverb. Above-unity feedback makes repeats grow into bounded saturation. Press takes effect at a kick; release stops the send immediately and lets the stored tail fade. |
| 13 | GROUND SCREAM | On any selected channel, plays crackle, 500 ms silence and a four-second solo 50 Hz feedback scream. Kick hits then return at 150 ms decay and ramp up over four seconds; the scream ducks on kicks and fades after release. |

These kick controls are independent of the selected channel/page. Their CCs
always target the Kick Seed; they never substitute an effect on the selected
channel. Changing focus does not clear an active kick performance hold.

The external effects retain latest-held priority: pressing another external pad
selects it at the next boundary, and releasing returns to the preceding held pad.
Steps 1, 9 and 10 are independent kick controls, so mute + BPF/retrigger + ratchet
can be held together in any press order. Pad 9 starts/stops on a kick or quarter boundary; pad 10
starts on that boundary and stops retriggering on release. Its bar has 15 hits
(1 + 2 + 4 + 8); coincident ordinary kick triggers merge into one hit. Releasing the final pad fades to the ordinary external
FX chain over 8 ms. Changing effects fades the old one out over 8 ms, then the
new one in over 12 ms; there is no buffer clearing/allocation during that switch.

## Timing and safety

While transport runs, a request waits for the **next received kick or quarter
note, whichever arrives first**. A live press cannot be applied to a beat that
has already passed. MIDI quarter edges correct gesture phase; controls remain
smoothed. If no boundary arrives, the request falls back after one estimated
quarter note. With transport stopped a new press engages immediately. Stop
cancels an active hold; release and press again to play FX while stopped.

Releasing a held pad remains consumed after switching channels, so it cannot
accidentally toggle another pattern.
The controller sends a keepalive every 200 ms; after 900 ms without one Seed
fades dry. A lost connection/timeout requires another press to resume.

The screen shows requested controls and an illustrative gesture, not Seed
acknowledgement or an audio meter. UI scan/UART latency and the 2-sample audio
block precede the actual boundary commit.

## Audio path

Existing external pump/repeat/filter, pitch, reverb, bitcrush and Erosion
processing → **held FX** → existing mono mixer/glue/ceiling.

Pad 9 takes over the generated kick's three BPF layers temporarily and
retriggers the kick at eighth-note intervals through the normal MIDI trigger
path. No audio capture buffer is used. Pad 10 uses the normal internal kick trigger path, preserving the voice's
phase reset and retrigger bridge. It does not send extra MIDI or Eurorack pulses.
Other pads affect the external lane. The shared output glue still responds to
their combined level. HAZARD may generate clicks with silent external input.

The continuous FX-page crusher now spans 16 to 4 bits (about 9.4 bits at 30%)
and blends up to 80% wet, keeping quiet tails from disappearing completely.

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
| 86 | Independent kick layers | Bit 0: pad 1 BPF; bit 1: pad 9 BPF/retrigger; bit 2: pad 10 ratchet; 0 clears |

The Teensy sends external pad selection, absolute kick-layer bits and keepalive; the other four CCs are optional
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

The new engine uses about **385 KiB of SDRAM**, mostly a 65,536-sample mono
history (1.37 seconds). It introduces no large per-switch clears, callback heap
allocations, blocking calls, FFTs or parallel algorithm stack. Filter tuning runs
at 250 Hz; DECAY's expensive mapping updates every 32 samples. RADIO and
HAZARD are the main extra CPU candidates; MEMORY/BINARY are lighter. Existing
`audio_max_cycles` and `audio_overruns` instrument the entire Seed callback.
Host timings do **not** establish a Seed CPU percentage: profile on hardware
with the heaviest existing Mackie/FX settings before relying on every combination.

Deliberate approximations: METAL uses six modal filters rather than physical modelling;
MEMORY is a bounded, crossfaded glitch sampler, not transparent time stretching.
Its variable delays and the intentional resonances are coloration, not a promise
of phase-transparent wet/dry mixing. The hardware's existing mono architecture is
preserved. CHIPMUNK and NIGHTCORE are excluded as requested.

Tune by ear first: RADIO level and intensity (CC81/84), METAL object size
(CC83), MEMORY fragment length (CC83), DECAY degradation time (CC83), and HAZARD
activity (CC81/82). The defaults favour controlled levels; the procedural layer
is intentionally quieter than the original audio.

## Validation

`bash test/held_fx/run.sh` checks priority, release ownership, retries, disconnect
lease, causal quantization, Stop, legacy gestures, stacked kick layers, exact ratchet timing, loop continuity, macro endpoints, finite bounded
output, circular-buffer wrapping, exact release and preserved DECAY attack under
ASan/UBSan. `bash test/held_fx/integration.sh` exercises the production MIDI
parser/audio callback, clock/kick start, GPIO continuity and per-pad isolation.
`bash test/kick_host/run.sh` also covers all held-effect OLED text bounds.

Build both firmware images for this feature. No upload is part of this change.
