# Engine B: Hoover / Rodox-inspired performance

This implements the user's Rodox workflow, not an assertion that we have matched
an original Alpha Juno electrically or Rodox's unpublished patch. One synth Seed
runs ACID or HOOVER, selected by seq23; the kick Seed remains a separate build.

## References and decisions, before the macro implementation

Roland's [JUNO-2 manual](https://cdn.roland.com/assets/media/pdf/JUNO-2_OM.pdf)
documents the DCO waveform options and the T1/L1/T2/L2/T3/L3/T4 envelope.
[Modor's own Hoover construction](https://www.modormusic.com/hoover.html)
identifies phase-aligned saw PWM plus pulse PWM, a divided sub,
chorus, and pitch-envelope movement. This implementation follows that alignment:
one free-running phase/divider produces all three components. No detuned saw bank.
The revised voice uses the one-octave sub mode (Alpha Juno also has multiple
sub modes); this is a voicing choice for the downloaded reference, not a claim
that it is the exact factory What The patch.

The saw is an unchanged-slope ramp gated twice per period, with discontinuities
corrected by polyBLEP. The pulse rises at the second ramp window. The earlier
one-window compressed ramp was incorrect; see `analysis/hoover/README.md`.
Pulse width modulation and pitch movement share one oscillator, including the
divider-derived sub. A two-times oversampled core/drive path feeds a two-pole
anti-alias lowpass before decimation. This is a behavioral approximation, not
a reproduction of the original DCO's complete waveform ROM or analogue VCF.

## Proposed and implemented macro curves

All x values below are knob values / 127. Controls slew before reaching DSP.
The table gives the baseline curves: as requested on 8 October, envelope times
and glide are divided by **1.4**, and PWM/chorus rates multiplied by **1.4**.
The clocked OFF/4/8-bar filter period is unchanged.

| Knob | Mapping |
|---|---|
| BODY/TAIL | Rodox/“What The?” contour: fast nonzero attack/release, long middle times, and high L1/L2/L3 values. It moves from a compact stab to a dense held body without a transient-to-quiet-body collapse. Pitch movement remains 40% faster than the original timing model. |
| PWM/SWARM | Twice-gated saw gap d=.24+(.015+.215x)×triangle; bounded .01–.47 cycles. PWM rate .8+7.2x² Hz. Shared pulse edge .5+d, giving coherent changing duty/harmonic emphasis. |
| CHORUS | Wet sqrt(x)×.68, depth .25→3.5 ms, rate .35+1.7x² Hz with .81/1/1.27 rate ratios. Two taps become six continuously using x². No feedback. Delay positions interpolate every sample; slew bounded to .055 samples/sample. |
| BAND/SCREAM | Centre 220×(4200/220)^x Hz; Q .7→3.8 using x²; parallel return 0→.58 with a smooth curve. Dry remains present. |
| STEREO MOTION | Depth 0→2.5 octaves using x². Click selects OFF→4→8 bars; default 4. One triangle, opposite frequency modulation; no polarity inversion. |
| BRUTAL | Smooth x² pre/post drive (pre up to 7×), mid emphasis, brighter core filter, compensated return gain and sub proportion .24→.16. |

The contour keeps T1/L1/T2/L2/T3/L3/T4 internally. T1/T2 use linear level
segments; T3 and release use a bounded fourth-power approximation to an
exponential approach. These curves follow the behavioral distinction in
[AudioRealism's synthesis notes](https://forum.audiorealism.se/post/synthesis-parameters-11431710),
not a calibrated Alpha Juno envelope circuit. Pitch timing is independent of the
VCA so a snappy hit can retain the audible rising scoop.

The chorus uses cubic fractional-delay interpolation, six different base delays
(7.3–18.1 ms), three slow rates and deliberately offset phases. Each side has
three positive-polarity taps plus identical centre dry sound. This is a designed
ensemble inspired by the source, not a claim to reproduce a specific BBD circuit.
Mono retains the centre signal; the stereo motion never subtracts dry audio.

## Timing and MIDI

The shared protocol is `include/SoftSynthProtocol.h`. Channel 14 is reserved for
this Seed (kick remains on 15). CC119 selects 0=ACID, 127=HOOVER. Notes, velocity,
CC29 incoming-slide and clocks originate in seq23. One engine/mono note at a time;
use one active synth track per Seed. Two synth tracks are not simultaneous voices.

CC70–75 are the six Hoover macros. CC76=0/64/127 chooses OFF/4/8 bars.
Function+knob 4 edits parallel amount (CC77), Function+knob 6 output trim (CC78).
Plain knob 4 resumes its automatic amount mapping. No MIDI CC targets kick audio.

Clock position is an integer modulo 768 ticks (8 bars at 24 PPQN). Fractional
interpolation never advances beyond the next tick. Start resets position; Stop
releases notes and freezes clock motion; Continue resumes. Missing clock freezes
after at most one tick; returning clock resumes. The modulation target is smoothed
so Start/tempo/period changes cannot jump filter coefficients.

## Verification scope

Host renders compile the identical C++ DSP used on Seed; the old Python acid
render is not used. Tests cover MIDI routing, clock periods/stop/continue/dropout,
finite/bounded macro endpoints, release to silence and stereo/mono energy.
Firmware CPU deadlines and exact sonic similarity still require hardware checks.
Do not describe the earlier ACID draft as a validated accurate TB-303 model.

## Physical output

The codec callback averages internal L/R onto Audio Out 1 and silences Audio
Out 2. This preserves both moving BPF branches on the single connected jack.
No channel is discarded and no extra gain is added by the mono sum. MIDI uses
channel 14 (zero-based 13); PC11 / D1, USART3 RX matches the kick Seed wiring.

## Audio versus pattern selection and output level

Menu 4 knob-1 press independently cycles NONE/ACID/HOOVER; turning it still
chooses the rhythm generator. Audio NONE is the default. Synth selection
overrides kick DSP menus/routing but retains the selected pattern and overlays.
After the mono fold-down, the shared output stage stays at unity gain and uses
a 64-sample lookahead peak limiter only for overload safety. The earlier raw
DSP spectral comparison remains a pre-output-stage study; current main demos
render Audio Out 1 through this safety stage.
