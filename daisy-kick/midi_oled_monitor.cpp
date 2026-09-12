#include "daisy_seed.h"
#include "stm32h7xx_hal.h"

#include <math.h>
#include <stdint.h>

using namespace daisy;


/* ============================================================
   HARDWARE
   ============================================================ */

DaisySeed hw;
UART_HandleTypeDef midi_uart;
I2CHandle oled_i2c;


/* ============================================================
   GLOBAL CONSTANTS
   ============================================================ */

static constexpr float SAMPLE_RATE = 48000.0f;


static constexpr float TWO_PI =
    6.2831853071795864769f;

static constexpr float PI =
    3.14159265358979323846f;


/* ============================================================
   ADDED PERFORMANCE / MIXER FEATURES
   ============================================================

   IMPORTANT:
   The original fixed2 kick engine below remains the base.

   Added around it:

       Digitakt / external IN1 -> Audio Out 2 only
       real-kick + ghost-clock pump on external input
       MIDI-clock dotted delay on external input
       MIDI-clock 1/16 stutter on kick + external lanes
       quantized looper on generated kick lane
       DJ high-pass on kick + external lanes
       clean protected 65..95 Hz punch lane

   All performance FX start OFF.

   Channel 16 remains the ORIGINAL fixed2 character-layer control.
   ============================================================ */


/* External input. */
static constexpr float EXTERNAL_INPUT_1_GAIN = 1.00f;
static constexpr float EXTERNAL_INPUT_2_GAIN = 0.00f;

/*
 * Leave some mix headroom without modifying the original kick gain.
 * Increase later if the Digitakt return is too quiet.
 */
static constexpr float EXTERNAL_RETURN_GAIN = 0.62f;


/* ============================================================
   HARD OUTPUT SPLIT
   ============================================================

   Physical Audio Out 1:
       GENERATED KICK ONLY

   Physical Audio Out 2:
       DIGITAKT / EXTERNAL INPUT ONLY

   The two lanes never enter a shared compressor / limiter / FX bus.
   The external lane retains its dedicated pump + external delay only.
 */
static constexpr float KICK_OUTPUT_LINEAR_GAIN = 0.86f;
static constexpr float EXTERNAL_OUTPUT_LINEAR_GAIN = 1.00f;

/* Canonical libDaisy non-interleaved channel indices. */
static constexpr size_t KICK_OUTPUT_CHANNEL = 0;
static constexpr size_t EXTERNAL_OUTPUT_CHANNEL = 1;
static constexpr size_t EXTERNAL_INPUT_CHANNEL_1 = 0;
static constexpr size_t EXTERNAL_INPUT_CHANNEL_2 = 1;
static_assert(KICK_OUTPUT_CHANNEL != EXTERNAL_OUTPUT_CHANNEL,
              "Kick and external outputs must be different physical channels");


/* Performance effects: OFF initially. */
static bool PERF_PUMP_ENABLED = false;
static bool PERF_CLOCKED_DELAY_ENABLED = false;
static bool PERF_STUTTER_ENABLED = false;
static bool PERF_DJ_HPF_ENABLED = false;
static bool PERF_QUANT_LOOPER_ENABLED = false;


/*
 * Protected kick-bin punch.
 *
 * Off: enabling it also engages a sidechain that ducks the main signal by
 * the punch envelope, which pumps. The low end is protected by keeping the
 * sub out of the distortion in the first place, not by ducking.
 *
 * Set false to hear byte-for-byte fixed2 kick behavior.
 */
static bool ENABLE_PROTECTED_PUNCH = false;


/* ============================================================
   USER-TUNABLE KICK GAIN / ANATOMY CONTROLS
   ============================================================

   These are intentionally all gathered in one place.

   They are plain code-level trims, not MIDI parameters.

   START HERE if the output level / kick anatomy needs balancing.
   ============================================================ */

/*
 * Final digital gain immediately before the DAC outputs.
 *
 * 1.00 = unity
 * 1.25 = +1.94 dB
 * 1.40 = +2.92 dB
 *
 * The final safety limiter still prevents uncontrolled full-scale
 * overs. If the signal is already at DAC full scale, software cannot
 * create more analogue voltage than the hardware output stage allows.
 */
/*
 * HEADROOM FIX:
 *
 * The old +2.1 dB master boost made downstream threshold crossings much
 * easier once character/filter energy accumulated.
 *
 * Keep the kick at unity here; the analogue/output stage can provide
 * level downstream without forcing the DSP into limiters.
 */
/*
 * MIX PAGE PARAMETERS (Teensy FUNCTION + MENU2, CC53..59 on channel 15).
 *
 * These were compile-time constants. They are now live so the mix stage can
 * be dialled on the hardware instead of rebuilt. Defaults reproduce the
 * tuned values exactly, so behaviour is unchanged until a CC arrives.
 *
 * CC value 0..127 maps linearly to 0..MAX for each.
 */
static volatile float param_line_gain    = 2.8184f; /* CC53, max 4.0  */
static volatile float param_mackie_gain  = 1.01703f;/* CC54, max 2.0  */
static volatile float param_sherman_gain = 1.56710f;/* CC55, max 2.5  */
static volatile float param_bpf_gain     = 4.0f;    /* CC56, max 8.0  */
static volatile float param_sub_gain     = 0.95f;   /* CC57, max 1.6  */
static volatile float param_comp_amount  = 0.0f;    /* CC58, max 1.0  */
static volatile bool  param_limiter_on   = false;   /* CC59, 0/127    */

static constexpr float PARAM_LINE_GAIN_MAX    = 4.0f;
static constexpr float PARAM_MACKIE_GAIN_MAX  = 2.0f;
static constexpr float PARAM_SHERMAN_GAIN_MAX = 2.5f;
static constexpr float PARAM_BPF_GAIN_MAX     = 8.0f;
static constexpr float PARAM_SUB_GAIN_MAX     = 1.6f;


/*
 * Relative anatomy trims.
 *
 * These multiply the existing fixed2-derived gain staging rather than
 * replacing it, so 1.00 preserves the current balance.
 */
static constexpr float KICK_TRANSIENT_GAIN = 1.00f;
static constexpr float KICK_TAIL_GAIN      = 1.00f;
static constexpr float KICK_KNOCK_GAIN     = 1.00f;


/*
 * PARALLEL CHARACTER SEND.
 *
 * A COPY of the WHOLE kick — transient, body and tail at unity — drives the
 * nonlinear models. That is where the classic sustained gabber/industrial
 * harmonic body comes from. The originals never enter the dirty bus; the
 * clean knock and clean sub stay on their own protected lanes and are mixed
 * back in after all dirty-path dynamics, so fidelity survives the distortion.
 */
static constexpr float KICK_PROCESSED_TAIL_FEED = 1.00f;
static constexpr float KICK_PROCESSED_TRANSIENT_FEED = 1.00f;

/*
 * Wet-return trim after Mackie/Sherman and before dirty-bus management.
 */
static constexpr float KICK_CHARACTER_RETURN_GAIN = 1.00f;

/*
 * High-pass on the DISTORTED RETURN ONLY.
 *
 * The models still SEE the entire waveform — the send is full-band and
 * unity — so the fundamental is what drives them into overload. Only the
 * return is filtered, so the distorted low end does not stack on top of the
 * protected clean sub and bloat the kick bin.
 *
 * Two poles at 120 Hz: firm enough to stop the low-end buildup, far less
 * destructive than the original three at 105 Hz because the send now opens
 * at 4 ms and carries the full transient rather than a bare tail sine.
 *
 * POLE_A = expf(-2*pi*120/48000). Keep it in step with _HZ if that changes.
 */
static bool CHARACTER_PROTECT_CLEAN_SUB = true;
static constexpr float CHARACTER_SUB_PROTECT_HZ = 120.0f;
static constexpr int CHARACTER_SUB_PROTECT_POLES = 2;
static constexpr float CHARACTER_SUB_PROTECT_POLE_A = 0.98441477f;

/* ============================================================
   MUSICAL MACKIE / SHERMAN CHARACTER
   ============================================================

   The models are PARALLEL SEND/RETURN processors:

       clean knock -------------------------------> clean lane
       clean tail  -------------------------------> clean sub lane
              \-> character send -> nonlinear/BPF -> HPF -> dirty return

   This means character can be driven hard without replacing or phase-
   cancelling the clean sub. Macro 5 is a true send/return amount.

   Macro 4 BPF layers have TWO independent filter-state banks:
       - a broad pre-drive bank feeding the character models
       - a more resonant post-drive return bank

   so the BPF frequencies genuinely interact with distortion rather than
   simply being pasted on after it.
   ============================================================ */

/*
 * These _HZ values describe the hardcoded one-pole coefficients inside each
 * model's Process(). They are documentation, not inputs — the coefficients
 * are literals at the filter sites. Keep both in step.
 *
 * Raised from 7000/5600 and 6500/5000: four poles between 5 and 7 kHz across
 * the two stages rolled off most of the harmonic content the overload was
 * generating, so the distortion read as dull and filtered no matter how hard
 * the models were driven.
 */
static constexpr float MACKIE_PRE_LP_HZ      = 12000.0f;
static constexpr float MACKIE_POST_LP_HZ     = 10000.0f;
static constexpr float MACKIE_INTERNAL_GAIN  = 7.40f;
/* +3 dB: 0.72 * 10^(3/20). Drive/character untouched, level only. */

static constexpr float SHERMAN_PRE_LP_HZ     = 12000.0f;
static constexpr float SHERMAN_POST_LP_HZ    = 10000.0f;
static constexpr float SHERMAN_FILTER_RESONANCE = 0.85f;
static constexpr float SHERMAN_FEEDBACK      = 0.15f;
static constexpr float SHERMAN_INPUT_DRIVE   = 3.05f;
/* +7 dB: 0.70 * 10^(7/20). 4 dB to match Mackie, plus the shared 3 dB. */

/*
 * How much the K5 amount drives the model, rather than only fading its
 * output back in. Input gain spans 1x at zero to 1+RANGE at full.
 *
 * Without this the models always saturate by exactly the same fixed amount
 * and K5 is a level fader on a parallel return, so turning it up makes the
 * kick marginally louder instead of progressively dirtier. Every stage in
 * both models is SoftClip-bounded, so extra drive adds harmonics rather
 * than level and cannot run away.
 */
static constexpr float CHARACTER_AMOUNT_DRIVE_RANGE = 5.0f;

/* Macro 4 is the only explicit user-controlled multi-BPF bank. */
static constexpr bool CHARACTER_HIDDEN_BPF_STACK_DISABLED = true;

/* Overall strength of the BPF bank's audible return. */

/* Smooth wet changes so CC movement never creates a control-rate edge. */
static constexpr float CHARACTER_AMOUNT_SMOOTH_MS = 18.0f;

/* ============================================================
   REAL-TIME CPU / TRANSIENT SAFETY
   ============================================================ */

/*
 * The previous build kept expensive Mackie/Sherman processing alive even
 * at exactly 0% wet, and kept BOTH DJ SVFs evaluating tanf() every sample
 * while bypassed.
 *
 * With a 4-sample block this is unnecessary real-time pressure and can
 * present as a very short brittle / cymbal-like digital glitch when a
 * transient arrives.
 */
static constexpr float CHARACTER_HEAVY_PROCESS_EPSILON = 0.0005f;

/*
 * DJ-filter coefficients follow ~30 ms control smoothing. Updating the
 * expensive tanf() every sample is pointless; every 8 samples is far
 * faster than the audible control movement.
 */
static constexpr uint32_t DJ_FILTER_COEFF_UPDATE_SAMPLES = 8;


/* ============================================================
   PROTECTED CLEAN LOW LANES
   ============================================================ */

/*
 * The clean sine tail bypasses the dirty/character compressor and
 * limiter completely.
 *
 * Raised from 0.82 now that the models distort the full band including the
 * fundamental: this protected sub is what keeps the low end tight and
 * in-phase underneath a wet return that is no longer high-passed.
 */


/*
 * Dedicated kick-bin punch is now explicitly 65..95 Hz.
 */
static constexpr float PROTECTED_LOW_REGION_HZ = 65.0f;
/*
 * HARD PROTECTED-LANE CEILING:
 * no protected LF oscillator/lane is allowed above 95 Hz.
 */
static constexpr float PROTECTED_HIGH_REGION_HZ = 95.0f;


/* ============================================================
   DIRTY / CHARACTER BUS LEVEL MANAGEMENT
   ============================================================ */

/*
 * Start progressively controlling the dirty branch around the point at
 * which the character stack moves beyond its base stage into MID I.
 */
static constexpr float DIRTY_MANAGER_START_AMOUNT = 0.28f;

/*
 * At maximum character amount:
 *   - broadband dirty path gets strong compression
 *   - a little static trim prevents EQ/crest build-up
 *   - >6.5 kHz gets its own fast dynamic clamp
 *
 * NONE of this touches the protected clean sine tail or protected punch.
 */
static constexpr float DIRTY_MANAGER_MIN_THRESHOLD = 0.30f;
static constexpr float DIRTY_MANAGER_MAX_THRESHOLD = 0.56f;

static constexpr float DIRTY_MANAGER_MAX_RATIO = 8.0f;
static constexpr float DIRTY_MANAGER_MIN_GAIN = 0.24f;

static constexpr float DIRTY_MANAGER_ATTACK_MS = 3.50f;
static constexpr float DIRTY_MANAGER_RELEASE_MS = 75.0f;

static constexpr float DIRTY_MANAGER_MAX_STATIC_TRIM = 0.82f;

static constexpr float DIRTY_HF_CROSSOVER_HZ = 6500.0f;
static constexpr float DIRTY_HF_THRESHOLD = 0.030f;
static constexpr float DIRTY_HF_RATIO = 12.0f;
static constexpr float DIRTY_HF_ATTACK_MS = 0.06f;
static constexpr float DIRTY_HF_RELEASE_MS = 28.0f;
static constexpr float DIRTY_HF_MIN_GAIN = 0.10f;


/*
 * No nonlinear dirty-bus limiter anymore.
 * Character amount determines a conservative LINEAR post-gain:
 *     dry-ish character -> ~0.90
 *     full character    -> ~0.72
 */
static constexpr float DIRTY_POST_GAIN_DRY = 0.86f;
static constexpr float DIRTY_POST_GAIN_WET = 0.95f;


/*
 * TRANSIENT -> TAIL MUSICAL HANDOFF
 *
 * The rich transient progressively hands over to the clean bass tail
 * using an equal-power crossfade.
 *
 * This happens independently from Note-Off so every kick has a coherent
 * anatomy even when MIDI gate length is being played aggressively.
 */
/*
 * Default centre/"PUNCH" anatomy.
 * Per-hit values are latched from K6 / SHAPE.
 */
static constexpr float TRANSIENT_TO_TAIL_START_MS = 6.0f;
static constexpr float TRANSIENT_TO_TAIL_END_MS   = 44.0f;


/* ============================================================
   K6 SHAPE — ROUND -> PUNCH -> SNAP
   ============================================================

   0.00 ROUND:
        dark, shallow pitch transient, softer attack, minimal click

   0.50 PUNCH:
        strongest classic tekno-kick front body

   1.00 SNAP:
        bright, deep-but-short pitch transient, hard click allowed
   ============================================================ */

/*
 * K6 / SHAPE — SYNTAKT-LIKE TRANSIENT -> BODY CONTINUUM
 *
 * IMPORTANT ORIENTATION:
 *
 *     LOW SHAPE:
 *         body-dominant
 *         shallow initial pitch displacement
 *         slow / barely-perceived pitch settling
 *         transient hands over to body very early
 *
 *     HIGH SHAPE:
 *         sharp obvious pitch transient
 *         approximately 500 Hz start for a ~55 Hz root
 *         settles to root in roughly 30..50 ms
 *
 * The ratio-based high end is 9.1x:
 *     root 45 Hz -> ~410 Hz
 *     root 55 Hz -> ~500 Hz
 *     root 65 Hz -> ~592 Hz
 *
 * This keeps the effect note-relative while landing in the requested
 * ~500 Hz area for normal kick fundamentals.
 */
static constexpr float SHAPE_ROUND_START_RATIO = 1.18f;
static constexpr float SHAPE_PUNCH_START_RATIO = 3.80f;
static constexpr float SHAPE_SNAP_START_RATIO  = 9.10f;

static constexpr float SHAPE_ROUND_SWEEP_MS = 180.0f;
static constexpr float SHAPE_PUNCH_SWEEP_MS = 88.0f;
static constexpr float SHAPE_SNAP_SWEEP_MS  = 42.0f;

static constexpr float SHAPE_ROUND_TRANSIENT_GAIN = 0.16f;
static constexpr float SHAPE_PUNCH_TRANSIENT_GAIN = 0.62f;
static constexpr float SHAPE_SNAP_TRANSIENT_GAIN  = 1.00f;

static constexpr float SHAPE_ROUND_DRIVE = 1.02f;
static constexpr float SHAPE_PUNCH_DRIVE = 1.36f;
static constexpr float SHAPE_SNAP_DRIVE  = 1.82f;

static constexpr float SHAPE_ROUND_CUTOFF_HZ = 1100.0f;
static constexpr float SHAPE_PUNCH_CUTOFF_HZ = 3600.0f;
static constexpr float SHAPE_SNAP_CUTOFF_HZ  = 7200.0f;

static constexpr float SHAPE_ROUND_ATTACK_MS = 2.50f;
static constexpr float SHAPE_PUNCH_ATTACK_MS = 1.35f;
static constexpr float SHAPE_SNAP_ATTACK_MS  = 0.75f;

/*
 * LOW SHAPE: body appears almost immediately.
 *
 * HIGH SHAPE: transient remains audible through the ~40 ms mid-drop,
 * then gives way to the fundamental/body.
 */
/*
 * When the transient starts handing over to the tail.
 *
 * These were 2/7/18 ms, so the tail began fading the punch out almost as
 * soon as it started — with a long decay the tail then dominated and the
 * punch was audibly cut off. Moved past the punch's main body so the
 * attack completes first and the tail takes over underneath it.
 */
static constexpr float SHAPE_ROUND_HANDOFF_START_MS = 12.0f;
static constexpr float SHAPE_PUNCH_HANDOFF_START_MS = 22.0f;
static constexpr float SHAPE_SNAP_HANDOFF_START_MS  = 34.0f;

static constexpr float SHAPE_ROUND_HANDOFF_END_MS = 38.0f;
static constexpr float SHAPE_PUNCH_HANDOFF_END_MS = 62.0f;
static constexpr float SHAPE_SNAP_HANDOFF_END_MS  = 84.0f;

/*
 * Tail decay shape: 0 = original pure exponential, 1 = linear ramp.
 * The exponential dumped most of the level immediately after the attack.
 */
static constexpr float TAIL_DECAY_LINEARITY = 0.55f;


/* ============================================================
   PURE SINE SWEEP ONSET
   ============================================================ */

/*
 * Opening milliseconds:
 *     main phase-locked SINE pitch sweep + clean low tail only.
 *
 * Governs TRANSIENT SATURATION and output headroom. The clean sine
 * oscillator must stay unsaturated through the attack or the kick's own
 * fundamental is what gets distorted, which reads as a mushy low end no
 * amount of downstream filtering can recover.
 */
static constexpr float PURE_SWEEP_ONLY_MS = 20.0f;

/*
 * The character/dirty bus opens on its OWN, much earlier schedule.
 *
 * These were one value. Opening it early to let Mackie/Sherman hear the
 * attack also switched on transient saturation at the same moment, so the
 * sine fundamental was being clipped from 4 ms. Separate gates: the models
 * get the attack, the clean transient stays clean.
 */
static constexpr float CHARACTER_OPEN_MS = 4.0f;
static constexpr float CHARACTER_OPEN_FADE_MS = 6.0f;
static constexpr float PURE_SWEEP_BODY_FADE_MS = 10.0f;

/*
 * END_OF_CHAIN_GAIN = 1.28.
 * 0.68 * 1.28 ~= 0.87, deliberately below the final limiter knee
 * during the pure onset so the limiter cannot manufacture a crispy
 * harmonic edge from the sine transient.
 */
static constexpr float PURE_SWEEP_OUTPUT_HEADROOM = 0.68f;


/*
 * MIDI velocity is reserved for the bipolar tail-pitch gesture below.
 * It deliberately does NOT scale kick amplitude, oscillator morph, tone,
 * or distortion drive. K6 remains the dedicated transient/shape control.
 */

/*
 * Resonant performance HPF/LPF can be pinged by an impulse even when
 * their cutoff is outside the main kick body. Ramp Q in after the edge.
 */
static constexpr float PERFORMANCE_FILTER_ONSET_GUARD_MS = 12.0f;
static constexpr float PERFORMANCE_FILTER_ONSET_Q = 0.72f;


/* ============================================================
   HPF / LPF TRANSIENT + GAIN-STAGING SAFETY
   ============================================================ */

/*
 * The master HPF/LPF live AFTER the generated-kick HF guard.
 * Therefore they can otherwise recreate a high-frequency onset spike.
 *
 * For the first few milliseconds of every kick, leave the already-HF-
 * guarded dry master signal alone, then fade the selected filter in.
 */
static constexpr float PERFORMANCE_FILTER_DRY_HOLD_MS = 4.0f;
static constexpr float PERFORMANCE_FILTER_FADE_IN_MS  = 12.0f;


/*
 * Active-filter headroom.
 *
 * The output chain normally has END_OF_CHAIN_GAIN = 1.28 (+2.1 dB).
 * A resonant SVF transient can push that into the final limiter.
 *
 * 0.76 = -2.38 dB while HPF or LPF is active.
 *
 * Combined with END_OF_CHAIN_GAIN 1.28:
 *
 *      1.28 * 0.76 = 0.973
 *
 * so the filtered bus is approximately unity before the final limiter,
 * while the non-filtered instrument keeps the extra output gain.
 */
static constexpr float PERFORMANCE_FILTER_ACTIVE_HEADROOM_GAIN = 0.76f;
static constexpr float PERFORMANCE_FILTER_HEADROOM_SMOOTH_MS = 15.0f;


/*
 * Last safety filter AFTER the performance HPF/LPF.
 *
 * This is independent from the generated-kick HF guard because HPF/LPF
 * occur later in the chain.
 *
 * It is active only while HPF or LPF is engaged.
 */
static bool ENABLE_POST_PERFORMANCE_FILTER_HF_GUARD = false;

static constexpr float POST_PERF_FILTER_HF_INITIAL_HZ = 4200.0f;
static constexpr float POST_PERF_FILTER_HF_SETTLED_HZ = 7600.0f;
static constexpr float POST_PERF_FILTER_HF_OPEN_MS    = 18.0f;
static constexpr float POST_PERF_FILTER_HF_MIX_MS     = 10.0f;


/* ============================================================
   TRANSIENT A/B + ULTRA-HF GUARD
   ============================================================ */

/*
 * A/B SWITCH:
 *
 * true  = current ROUND -> PUNCH -> SNAP transient macro
 *
 * false = previous-version transient behaviour:
 *         - old K6 start-ratio map 12x -> 5.7x -> 1.05x
 *         - old K6 sweep-time map 8ms -> 69ms -> 320ms
 *         - fixed 0.8 ms transient attack
 *         - fixed 7..40 ms transient->tail handoff
 *         - historical velocity->morph behaviour removed; velocity is
 *           reserved for the tail-pitch sweep in BOTH A/B modes
 *         - old 13% morph gain compensation
 *         - old fixed transient drive
 *         - no SHAPE-controlled transient tone LP
 *
 * This only A/Bs the transient/shape architecture. All later fixes
 * (tail decay, sub protection, FX, reverse, knock, etc.) stay identical.
 */
static bool USE_NEW_SHAPE_TRANSIENT_MACRO = true;


/*
 * Independent A/B for the remaining "laser" / >8 kHz onset.
 *
 * This guard is intentionally AFTER transient saturation and BEFORE the
 * rest of the kick/FX chain, so downstream distortion/filtering never
 * receives the nastiest initial HF spike.
 */
static bool ENABLE_TRANSIENT_HF_GUARD = true;

/*
 * Two cascaded one-pole stages = roughly 12 dB/octave.
 *
 * During the first milliseconds the transient is kept deliberately
 * dark. It then opens only to a still-conservative ceiling.
 *
 * This biases kick identity toward the 65..95 Hz protected knock and
 * lower transient/body instead of a 10 kHz "laser".
 */
static constexpr float TRANSIENT_HF_GUARD_INITIAL_HZ = 3200.0f;
static constexpr float TRANSIENT_HF_GUARD_FINAL_HZ   = 6800.0f;
static constexpr float TRANSIENT_HF_GUARD_OPEN_MS    = 16.0f;


/*
 * Reverse bank-to-bank handoff.
 *
 * This is separate from reverse enable/disable fading. It specifically
 * smooths the instant a new captured hit becomes the new reverse source.
 */
static constexpr float REVERSE_BANK_HANDOFF_MS = 9.0f;


/* ============================================================
   PROTECTED KNOCK A/B + FINAL GENERATED-KICK HF CEILING
   ============================================================ */

/*
 * false = RECOMMENDED.
 *
 * Protected 65..95 Hz punch uses a sine-led low-harmonic oscillator.
 * This is what a protected kick-bin/chest lane should be.
 *
 * true = old sine->saw->supersaw->square protected-punch oscillator.
 *
 * This switch exists purely for A/B diagnosis.
 */
static bool PROTECTED_PUNCH_USE_COMPLEX_MORPH = false;


/*
 * The protected punch gets THREE cascaded 165 Hz low-pass stages.
 * That makes it genuinely low-frequency instead of letting the harmonic
 * ladder leak upward and then get re-saturated later.
 */
static constexpr int PROTECTED_PUNCH_LP_STAGES = 3;


/*
 * LAST generated-kick-only HF guard.
 *
 * This occurs after:
 *   transient
 *   Mackie/Sherman
 *   BPF
 *   resonant layer
 *   protected knock
 *   reverse
 *   generated-kick master envelope
 *
 * but BEFORE external audio is mixed in.
 *
 * Thus nothing inside the kick engine can recreate the >8 kHz laser
 * after an earlier filter.
 */
static bool ENABLE_FINAL_KICK_HF_GUARD = true;

/*
 * Very dark at the actual onset; then opens gradually.
 *
 * Even when fully open it is intentionally conservative because the
 * desired identity is 65..95 Hz punch/body, not an 8..15 kHz spike.
 */
/*
 * Three poles starting at 2400 Hz sat across the whole output, so every
 * harmonic the character models generated during the attack was removed
 * again before it reached the DAC. Opened up; it still tames the very first
 * onset without dulling the distortion.
 */
static constexpr float FINAL_KICK_HF_INITIAL_HZ = 7000.0f;
static constexpr float FINAL_KICK_HF_SETTLED_HZ = 16000.0f;
static constexpr float FINAL_KICK_HF_OPEN_MS    = 22.0f;

/*
 * Three cascaded poles gives a much steeper roll-off than the earlier
 * transient-only two-pole guard.
 */
static constexpr int FINAL_KICK_HF_POLES = 3;


/* ============================================================
   RATCHET / OSCILLATOR TEST CONTROLS
   ============================================================ */

/*
 * TEST MODE REQUESTED:
 *
 * true = every audible KICK oscillator source is sine-only.
 *
 * This bypasses saw / supersaw / square generation completely in the
 * main transient oscillator and forces the protected knock to pure sine.
 *
 * Velocity is still received for tail pitch, while K6 owns waveform/shape.
 * Sine-only mode makes waveform morphing inaudible for diagnosis.
 */
static bool SINE_ONLY_OSCILLATORS = true;


/*
 * Phase-continuous monophonic retrigger.
 *
 * true:
 *     if a new kick arrives while the previous hit is still sounding,
 *     do NOT reset audible oscillator phases or live filter memory.
 *
 * false:
 *     deterministic hard phase reset on every trigger (old behaviour).
 */
static bool PHASE_CONTINUOUS_RATCHET_RETRIGGER = true;


/*
 * Threshold for deciding whether a new Note On is an overlapping
 * retrigger rather than a fresh hit after silence.
 */
static constexpr float RATCHET_RETRIGGER_LEVEL_THRESHOLD = 0.0005f;


/* ============================================================
   PSYTRANCE-STYLE PHASE LOCK
   ============================================================ */

/*
 * RECOMMENDED DEFAULT:
 *
 * Every kick — including ratchets — starts the sine oscillators from the
 * same deterministic phase.
 *
 * This restores hit-to-hit phase stability. Click-free retriggering is
 * handled separately by the short bridge below instead of by allowing
 * the oscillator phase to free-run.
 */
static bool PSY_PHASE_LOCK_EVERY_HIT = true;


/*
 * When a new phase-locked kick steals an overlapping tail, continue a
 * tiny synthetic copy of the OLD sine tail while the NEW deterministic
 * kick fades in.
 *
 * 3.5 ms is short enough to preserve timing but long enough to remove
 * the discontinuity that a hard phase reset would otherwise create.
 */
static constexpr float RATCHET_PHASE_BRIDGE_MS = 3.5f;

/*
 * The non-sub remainder from the immediately previous sample is included
 * only long enough to make the very first bridge sample value-continuous.
 */
static constexpr float RATCHET_RESIDUAL_DECAY_MS = 0.65f;


/* ============================================================
   FINAL >8 kHz DYNAMIC DE-HARSHER
   ============================================================ */

/*
 * This stage runs AFTER the final limiter.
 *
 * That is deliberate: if the limiter/clipping itself creates the HF
 * spike, a compressor placed earlier cannot remove those new harmonics.
 *
 * The crossover is reconstructive:
 *     high = input - lowpass(input)
 * so gain=1 reproduces the original signal exactly.
 */
/*
 * Disabled: the sub-millisecond thresholded HF compressor could itself
 * make a crystalline gain-modulation transient when character/filter
 * energy crossed its detector threshold.
 */
static bool ENABLE_FINAL_HF_DYNAMIC_TAMER = false;

static constexpr float FINAL_HF_DYNAMIC_CROSSOVER_HZ = 8000.0f;

/*
 * Approx -28 dBFS high-band detector threshold.
 * This is intentionally low because normal kick energy above 8 kHz
 * should be small in the current sine-only design.
 */
static constexpr float FINAL_HF_DYNAMIC_THRESHOLD = 0.040f;

static constexpr float FINAL_HF_DYNAMIC_RATIO = 10.0f;

static constexpr float FINAL_HF_DYNAMIC_ATTACK_MS = 0.08f;
static constexpr float FINAL_HF_DYNAMIC_RELEASE_MS = 24.0f;

/*
 * Never attenuate the >8 kHz band by more than ~18 dB.
 */
static constexpr float FINAL_HF_DYNAMIC_MIN_GAIN = 0.126f;


/*
 * PROTECTED KNOCK
 *
 * Dedicated kick-bin / chest-impact lane.
 *
 * It is generated from the same deterministic oscillator architecture,
 * isolated to roughly 65..95 Hz, and inserted AFTER the original
 * distortion/compressor/limiter.
 *
 * During the knock, the rest of the generated kick is ducked so this
 * band does not need absurd gain to be audible.
 */
static constexpr float PROTECTED_PUNCH_LOW_HZ = 65.0f;
static constexpr float PROTECTED_PUNCH_HIGH_HZ = 95.0f;
static constexpr float PROTECTED_PUNCH_LENGTH_MS = 32.0f;
static constexpr float PROTECTED_PUNCH_GAIN = 0.46f;

/*
 * 0.74 means the body/tail can be reduced by up to 74% at the centre
 * of the knock envelope.
 */
static constexpr float PROTECTED_PUNCH_DUCK_DEPTH = 0.74f;


/* ============================================================
   VELOCITY -> TAIL PITCH SWEEP
   ============================================================

   Velocity is now the dedicated bipolar tail-pitch performance control.

       velocity   1 -> -12 semitones
       velocity  64 ->   0 semitones
       velocity 127 -> +12 semitones

   The target is latched at Note-On, so Note-Off/gate duration has NO
   influence on the tail pitch anymore. The pitch gesture begins only
   after the protected knock/onset window, and oscillator phase is never
   reset while the pitch moves.

   This makes every MIDI velocity step useful and deterministic.
   ============================================================ */

static bool ENABLE_VELOCITY_TAIL_SWEEP = true;

static constexpr uint8_t TAIL_SWEEP_CENTER_VELOCITY = 64;

static constexpr float TAIL_SWEEP_DOWN_SEMITONES = -12.0f;
static constexpr float TAIL_SWEEP_UP_SEMITONES   =  12.0f;

/*
 * The clean tail may deliberately leave the strict sub-only region on
 * upward sweeps. It still bypasses Mackie/Sherman entirely.
 */
static constexpr float TAIL_SWEEP_MIN_FREQUENCY_HZ = 18.0f;
static constexpr float TAIL_SWEEP_MAX_FREQUENCY_HZ = 180.0f;

/*
 * Smooth phase-continuous glide to the velocity target.
 */
static constexpr float TAIL_SWEEP_GLIDE_MS = 115.0f;


/* MIDI-clock fallback if no external clock has been received yet. */
static constexpr float DEFAULT_BPM = 180.0f;
static volatile float perf_quarter_note_ms =
    60000.0f / DEFAULT_BPM;

static uint32_t perf_clock_last_ms = 0;
static uint32_t perf_clock_pulse_count = 0;

static volatile bool perf_sixteenth_pending = false;
static volatile bool perf_quarter_pending = false;


/*
 * New Ch15 CCs. These do NOT overlap the original note/channel behavior.
 *
 * 70 Pump
 * 71 Clocked dotted delay
 * 72 Clocked stutter gate
 * 73 DJ HPF enable
 * 74 Quantized looper gate
 * 75 DJ HPF position
 * 76 Delay wet
 */
static constexpr uint8_t CC_FX_PUMP = 70;
static constexpr uint8_t CC_FX_DELAY = 71;
static constexpr uint8_t CC_FX_STUTTER = 72;
static constexpr uint8_t CC_FX_DJ_HPF = 73;
static constexpr uint8_t CC_FX_LOOPER = 74;
static constexpr uint8_t CC_FX_DJ_HPF_POSITION = 75;
static constexpr uint8_t CC_FX_DELAY_WET = 76;


/*
 * KICK-DESIGN CCs
 *
 * Ch15 velocity is the dedicated bipolar TAIL PITCH SWEEP control.
 *
 * 77 Sweep depth
 * 78 Sweep time
 * 79 Body low-pass cutoff
 * 80 Body low-pass resonance
 */
static constexpr uint8_t CC_KICK_SWEEP_DEPTH = 77;
static constexpr uint8_t CC_KICK_SWEEP_TIME = 78;
static constexpr uint8_t CC_KICK_LP_CUTOFF = 79;
static constexpr uint8_t CC_KICK_LP_RESONANCE = 80;


/* ============================================================
   SIX PHYSICAL MACROS — MIDI CHANNEL 15
   ============================================================

   FIXED CONTROLLER PAIRS:

       KNOB 1   DYNAMIC DEDICATED FX CC:
                  STUT CC30 / LOOP CC31 / DELAY CC32
                  HPF CC33 / LPF CC34 / PUMP CC35
       BUTTON 1 CC100   FX UI PAGE / LONG RESET

       KNOB 2   CC21    TAIL / DECAY LENGTH
       BUTTON 2 CC101   REVERSE-BASS TOGGLE

       KNOB 3   CC22    TAIL DELAY
       BUTTON 3 CC102   TAIL-DELAY TOGGLE

       KNOB 4   CC23    CURRENT BPF LAYER FREQUENCY
       BUTTON 4 CC103   BPF LAYERS 0 -> 1 -> 2 -> 3 -> 0

       KNOB 5   CC24    MACKIE / SHERMAN WET
       BUTTON 5 CC104   MACKIE <-> SHERMAN MODEL

       KNOB 6   CC25    KICK SHAPE
       BUTTON 6 CC105   PUMP TOGGLE

   Button messages act only on the press (value >= 64).
   Release messages are ignored.

   FX knob pages remember their values, so effects continue operating
   in parallel after Button 1 moves the physical knob to another page.
   ============================================================ */

/*
 * SOLID K1 BANKING PROTOCOL
 * -------------------------
 *
 * The ONE physical K1 knob changes the CC NUMBER it sends according
 * to the Teensy's locally selected FX page.
 *
 * This removes shared bank state from the audio engine:
 *
 *     CHOP (legacy STUTTER address)  CC30
 *     LOOPER   CC31
 *     DELAY    CC32
 *     DJ HPF   CC33
 *     DJ LPF   CC34
 *     PUMP     CC35
 *
 * Therefore a DELAY message can never mutate CHOP state.
 *
 * CC20 banked routing is retained only as an optional legacy fallback
 * and is OFF by default.
 */
/*
 * SOLID, ADDRESSABLE CC PROTOCOL
 * -----------------------------
 *
 * Every stateful/virtual parameter has its OWN CC address.
 * No CC means "whatever page/model/layer the Seed thinks is selected".
 *
 * K1 FX pages:
 *   CC30 CHOP (legacy STUTTER address)
 *   CC31 LOOPER
 *   CC32 DELAY
 *   CC33 HPF
 *   CC34 LPF
 *   CC35 PUMP AMOUNT
 *
 * Remaining macros / state:
 *   CC40 DECAY
 *   CC41 REVERSE STATE          0=OFF, 127=ON
 *   CC42 TAIL DELAY AMOUNT
 *   CC43 TAIL DELAY STATE       0=OFF, 127=ON
 *   CC44 BPF LAYER 1 FREQ
 *   CC45 BPF LAYER 2 FREQ
 *   CC46 BPF LAYER 3 FREQ
 *   CC47 BPF LAYER COUNT        absolute 0..3
 *   CC48 MACKIE AMOUNT
 *   CC49 SHERMAN AMOUNT
 *   CC50 CHARACTER MODEL        0=MACKIE, 127=SHERMAN
 *   CC51 KICK SHAPE
 *   CC52 PUMP STATE             0=OFF, 127=ON
 *
 * CC100 is retained ONLY for the K1 UI/long-hold reset gesture.
 * Old CC20/21..25/101..105 paths are optional legacy compatibility and
 * are disabled by default.
 */
static constexpr uint8_t CC_MACRO_FX_VALUE_LEGACY = 20;

static constexpr uint8_t CC_MACRO_FX_STUTTER       = 30;
static constexpr uint8_t CC_MACRO_FX_LOOPER        = 31;
static constexpr uint8_t CC_MACRO_FX_DELAY         = 32;
static constexpr uint8_t CC_MACRO_FX_HPF           = 33;
static constexpr uint8_t CC_MACRO_FX_LPF           = 34;
static constexpr uint8_t CC_MACRO_FX_PUMP          = 35;

static constexpr uint8_t CC_DECAY_ABSOLUTE          = 40;
static constexpr uint8_t CC_REVERSE_STATE           = 41;
static constexpr uint8_t CC_TAIL_DELAY_ABSOLUTE     = 42;
static constexpr uint8_t CC_TAIL_DELAY_STATE        = 43;
static constexpr uint8_t CC_BPF_LAYER1_FREQUENCY    = 44;
static constexpr uint8_t CC_BPF_LAYER2_FREQUENCY    = 45;
static constexpr uint8_t CC_BPF_LAYER3_FREQUENCY    = 46;
static constexpr uint8_t CC_BPF_LAYER_COUNT         = 47;
static constexpr uint8_t CC_MACKIE_AMOUNT           = 48;
static constexpr uint8_t CC_SHERMAN_AMOUNT          = 49;
static constexpr uint8_t CC_CHARACTER_MODEL         = 50;
static constexpr uint8_t CC_KICK_SHAPE_ABSOLUTE     = 51;
static constexpr uint8_t CC_PUMP_STATE              = 52;

/* Mix page: Teensy FUNCTION + MENU2. */
static constexpr uint8_t CC_MIX_LINE_GAIN           = 53;
static constexpr uint8_t CC_MIX_MACKIE_GAIN         = 54;
static constexpr uint8_t CC_MIX_SHERMAN_GAIN        = 55;
static constexpr uint8_t CC_MIX_BPF_GAIN            = 56;
static constexpr uint8_t CC_MIX_SUB_GAIN            = 57;
static constexpr uint8_t CC_MIX_COMPRESSOR          = 58;
static constexpr uint8_t CC_MIX_LIMITER             = 59;

static constexpr uint8_t CC_BUTTON_FX_NEXT          = 100;


/*
 * EMERGENCY REBOOT RAW BUTTON STATE
 * ---------------------------------
 *
 * Do NOT infer physical button holds from musical/UI CCs.
 *
 * The controller sends two dedicated raw physical-state CCs:
 *
 *     CC106 = B1 RAW
 *     CC107 = B3 RAW
 *
 * For both:
 *
 *     value 127 = physically DOWN
 *     value   0 = physically UP
 *
 * CC106/CC107 have NO musical or UI function.
 */
static constexpr uint8_t CC_EMERGENCY_BUTTON1_RAW = 106;
static constexpr uint8_t CC_EMERGENCY_BUTTON3_RAW = 107;

static constexpr uint32_t EMERGENCY_REBOOT_HOLD_MS = 1000;

static bool emergency_b1_down = false;
static bool emergency_b3_down = false;

static bool emergency_combo_timing = false;
static uint32_t emergency_combo_start_time = 0;

/* Legacy six-macro addresses, ignored unless explicitly enabled. */
static constexpr uint8_t CC_MACRO_DECAY_LEGACY          = 21;
static constexpr uint8_t CC_MACRO_TAIL_DELAY_LEGACY     = 22;
static constexpr uint8_t CC_MACRO_BPF_FREQUENCY_LEGACY  = 23;
static constexpr uint8_t CC_MACRO_CHARACTER_WET_LEGACY  = 24;
static constexpr uint8_t CC_MACRO_KICK_SHAPE_LEGACY     = 25;

static constexpr uint8_t CC_BUTTON_REVERSE_BASS_LEGACY  = 101;
static constexpr uint8_t CC_BUTTON_TAIL_DELAY_LEGACY    = 102;
static constexpr uint8_t CC_BUTTON_BPF_LAYERS_LEGACY    = 103;
static constexpr uint8_t CC_BUTTON_CHARACTER_LEGACY     = 104;
static constexpr uint8_t CC_BUTTON_PUMP_LEGACY          = 105;


enum class MacroFxMode : uint8_t
{
    STUTTER = 0,
    LOOPER,
    DELAY,
    DJ_HPF,
    DJ_LPF,
    PUMP,
    COUNT
};


static volatile MacroFxMode macro_fx_mode =
    MacroFxMode::STUTTER;


/*
 * One remembered value for each FX page.
 * Moving away from a page does NOT bypass it.
 */
static volatile float macro_fx_value_stutter = 0.0f;
static volatile float macro_fx_value_looper = 0.0f;
static volatile float macro_fx_value_delay = 0.0f;
static volatile float macro_fx_value_hpf = 0.0f;
static volatile float macro_fx_value_lpf = 0.0f;
static volatile float macro_fx_value_pump = 0.0f;


/* ============================================================
   CLOCKED RANDOM STUTTER / LOOPER COMMAND MAILBOXES
   ============================================================

   Control/MIDI code writes ONE 32-bit command.
   The audio callback consumes it.

   action:
       0 = none
       1 = enable on next KICK trigger
       2 = disable on next KICK trigger
       3 = safety-disable immediately in audio thread

   bits 8..15 contain the random rate index for an ENABLE command.

   This avoids main-loop code ever resetting/starting a live audio
   processor halfway through a sample block.
   ============================================================ */

static constexpr uint32_t QUANT_FX_NONE = 0u;
static constexpr uint32_t QUANT_FX_ENABLE = 1u;
static constexpr uint32_t QUANT_FX_DISABLE = 2u;
static constexpr uint32_t QUANT_FX_FORCE_DISABLE = 3u;
static constexpr uint32_t QUANT_FX_RATE_CHANGE = 4u;

static volatile uint32_t stutter_quantized_command = QUANT_FX_NONE;
static volatile uint32_t looper_quantized_command = QUANT_FX_NONE;



/*
 * K2 decay changes are also handed to the audio thread rather than
 * directly mutating its envelope from ServiceMidi().
 */
static volatile bool master_decay_retime_pending = false;


/* Macro 2: true tail length / release scale. */
static volatile float macro_decay = 0.50f;
/*
 * CC40 writes the TARGET; macro_decay slews toward it in the audio loop.
 * Writing the live value straight from MIDI stepped the decay coefficient
 * once per CC message, which zippers audibly while the knob is turning.
 */
static volatile float macro_decay_target = 0.50f;
static bool reverse_bass_enabled = false;


/*
 * REVERSE SWITCHING POLICY
 *
 * true:
 *     Reverse ENABLE may happen immediately / during the current kick.
 *
 * Reverse DISABLE is ALWAYS deferred until the next kick boundary so
 * the waveform can never suddenly un-reverse halfway through a hit.
 */
static constexpr bool ALLOW_REVERSE_ENABLE_MID_KICK = true;

static volatile bool reverse_enable_pending = false;
static volatile bool reverse_disable_pending = false;


/* Macro 3: tail delay / bass sidechain. */
static volatile float macro_tail_delay = 0.0f;
static bool tail_delay_enabled = false;


/* Macro 4: additive BPF layer bank. */
static volatile uint8_t macro_bpf_layer_count = 0;
static volatile float macro_bpf_target_hz[3] =
{
    330.0f,
    700.0f,
    1450.0f
};


/* Macro 5: character processor. */
static volatile float macro_character_wet = 0.0f;
/*
 * UI / requested character model.
 * Audio DSP owns its actual active model internally.
 */
static volatile bool macro_character_sherman = false;

/*
 * Main-loop MIDI code ONLY writes these request flags.
 * The audio callback consumes them and is the ONLY place allowed to
 * reset/switch Mackie/Sherman DSP state.
 */
static volatile bool character_switch_pending = false;
static volatile bool character_switch_target_sherman = false;

/* Button-5 edge latch: repeated held CC values cannot retrigger switching. */
static bool character_button_down = false;


/* Macro 6: laser -> long bass-note kick shape. */
static volatile float macro_kick_shape = 0.50f;


/*
 * Button 6 is a dedicated pump performance toggle.
 * Pump depth/recovery is edited from the PUMP page of Macro 1.
 */
static bool macro_pump_enabled = false;


/*
 * Six-macro controller is the authoritative performance control path.
 * Old direct/debug CC70..80 handling stays in the source but is OFF.
 */
static bool ENABLE_LEGACY_DIRECT_FX_CCS = false;

/*
 * Old one-CC bank routing is deliberately disabled.
 * Turn this on ONLY if using an old Teensy build that still sends CC20
 * for every FX page.
 */
static bool ENABLE_BANKED_K1_CC20_COMPAT = false;

/* Old ambiguous macro/button CCs are OFF by default. */
static bool ENABLE_LEGACY_AMBIGUOUS_MACRO_CCS = false;

/* Physical controller K4/K5 compatibility remains intentionally enabled. */
static bool ENABLE_LEGACY_K4_BPF_CONTROLS = true;
static bool ENABLE_LEGACY_K5_CHARACTER_CONTROLS = true;
static bool legacy_bpf_button_down = false;


/*
 * K1 soft takeover / pickup.
 *
 * Each FX remembers its own value. When Button 1 changes page, CC20 is
 * ignored until the physical pot crosses the stored value for the newly
 * selected FX. This prevents a pot sitting at (say) 50% on HPF from
 * instantly setting a previously-zero STUTTER to 50%.
 */
static bool macro_fx_pickup_active = false;
static uint8_t macro_fx_last_raw = 0;
static uint8_t macro_fx_pickup_start_raw = 0;
static constexpr uint8_t MACRO_FX_PICKUP_TOLERANCE = 2;


/*
 * K1 button:
 * short press = next FX page
 * hold >= 500 ms = clear every K1 FX except pumping.
 */
static bool macro_fx_button_down = false;
static bool macro_fx_long_press_fired = false;
static uint32_t macro_fx_button_down_time = 0;
static constexpr uint32_t MACRO_FX_LONG_PRESS_MS = 500;


/*
 * K5 stores separate positions for the two models.
 */
static volatile float macro_mackie_amount = 0.0f;
static volatile float macro_sherman_amount = 0.0f;

/*
 * true  = newly-selected model starts at 0
 * false = newly-selected model recalls its own previous amount
 */
static bool CHARACTER_RESET_ON_MODEL_SWITCH = false;

static constexpr float CHARACTER_BASE_FULL_POINT = 0.30f;


/*
 * Defaults reproduce the original fixed2 pitch sweep:
 * multiplier 5.7x, time 69 ms.
 */
static volatile float kick_sweep_depth = 0.55f;
static volatile float kick_sweep_time = 0.55f;


/*
 * BODY-only resonant low-pass.
 * The clean low-frequency tail bypasses this.
 *
 * 0.62 -> about 4.7 kHz.
 */
static bool KICK_BODY_LP_ENABLED = true;
static volatile float kick_body_lp_cutoff = 0.62f;
static volatile float kick_body_lp_resonance = 0.18f;


/* ============================================================
   MASTER KICK AMPLITUDE ENVELOPE
   ============================================================

   MIDI Note-On:
       fast smooth attack -> HOLD

   MIDI Note-Off:
       quick smooth release -> ZERO

   This envelope is applied to the COMPLETE GENERATED KICK after:
       original distortion / character
       original compressor / limiter
       resonant sweep experiment
       protected clean punch

   It does NOT touch external Digitakt passthrough.

   Later these timings can become CC parameters.
   ============================================================ */

static bool ENABLE_KICK_MASTER_GATE_ENVELOPE = true;

static volatile float kick_master_attack_ms = 2.5f;
static volatile float kick_master_release_ms = 14.0f;


/*
 * Temporarily disable the ORIGINAL fixed2 character BPF path.
 *
 * This is a true DSP bypass: the band-pass is not processed at all,
 * rather than being processed and multiplied by zero afterwards.
 *
 * Channel 16 MIDI handling remains present so it can be restored later.
 */
static bool ENABLE_ORIGINAL_CHARACTER_BPF_PATH = false;


/* ============================================================
   EXPERIMENT: RESONANT LOW-FREQUENCY SWEEP LAYER
   ============================================================

   Parallel layer derived from the already-designed kick signal.

   Filter:
       high-resonance 2-pole low-pass
       200 Hz -> 80 Hz
       LINEAR cutoff sweep
       no filter-envelope attack

   Envelope:
       independently switchable
       starts at full amplitude immediately
       holds until late in the sweep
       then smoothly falls to zero

   Gate modulation:
       independently switchable
       measured Note-On -> Note-Off time controls sweep/envelope time

   These are variables rather than hard-coded DSP assumptions so they
   can become CC parameters later.
   ============================================================ */

/*
 * HARD-DISABLED:
 *
 * This layer contained an independent 200 -> 80 Hz resonant sweep and
 * its own phase-locked sine exciter. It is removed from the audible kick
 * while eliminating every protected/additional source above 100 Hz.
 */
static bool ENABLE_RESONANT_SWEEP_LAYER = false;
static bool ENABLE_RESONANT_SWEEP_AMP_ENVELOPE = true;
static bool ENABLE_RESONANT_SWEEP_GATE_MODULATION = true;


/* Filter character. */
static volatile float resonant_sweep_start_hz = 95.0f;
static volatile float resonant_sweep_end_hz = 72.0f;

/*
 * Large resonance, intentionally Elektron-like in spirit.
 * Kept below pathological/self-oscillating territory for this first test.
 */
static volatile float resonant_sweep_q = 2.6f;


/*
 * Parallel layer level before the original fixed2 bus compressor/limiter.
 */
static volatile float resonant_sweep_layer_gain = 0.13f;


/*
 * Make the resonant layer belong to the note rather than imposing the
 * exact same 200 -> 80 Hz event on every key.
 *
 * 0 = completely fixed absolute frequencies
 * 1 = fully tracks the kick fundamental
 *
 * 0.65 keeps the intended region while audibly following pitch.
 */
static volatile float resonant_sweep_key_tracking = 0.65f;


/*
 * Quiet deterministic sine excitation at the instantaneous resonant
 * frequency. Phase resets at every kick.
 *
 * This helps the resonance feel like a kick oscillator component rather
 * than a free filter "ping".
 */
static bool ENABLE_RESONANT_SWEEP_PHASE_LOCKED_EXCITER = true;
static volatile float resonant_sweep_exciter_amount = 0.0f;


/*
 * Only a small amount of dirty BODY drives the resonator.
 * The clean tail is its main natural excitation.
 */
static volatile float resonant_sweep_body_feed = 0.18f;


/*
 * Default ~1/10 second sweep.
 *
 * With gate modulation ON:
 *     sweep time ~= gate time * 1.10
 *
 * so the filter motion is naturally a little longer than the held note.
 */
static volatile float resonant_sweep_default_ms = 105.0f;


/*
 * Envelope holds until this fraction of the sweep has elapsed,
 * then fades smoothly to zero at the sweep end.
 */
static volatile float resonant_sweep_env_hold_ratio = 0.78f;


/*
 * Practical gate-derived sweep range.
 */
static constexpr float RESONANT_SWEEP_MIN_MS = 55.0f;
static constexpr float RESONANT_SWEEP_MAX_MS = 420.0f;


/* ============================================================
   ADDED SMALL UTILITIES
   ============================================================ */

static inline float ClampAdded(
    float x,
    float lo,
    float hi)
{
    if(x < lo)
        return lo;

    if(x > hi)
        return hi;

    return x;
}


static inline float Clamp01Added(float x)
{
    return
        ClampAdded(
            x,
            0.0f,
            1.0f
        );
}


static inline float SmoothstepAdded(float x)
{
    x =
        Clamp01Added(
            x
        );

    return
        x *
        x *
        (
            3.0f -
            2.0f *
            x
        );
}


static inline float EqualPowerA(float t)
{
    t = Clamp01Added(t);
    return cosf(t * 1.57079632679f);
}


static inline float EqualPowerB(float t)
{
    t = Clamp01Added(t);
    return sinf(t * 1.57079632679f);
}


/* ============================================================
   MUSICAL MACRO MAPPINGS
   ============================================================ */

/*
 * Macro 2 — tail decay.
 *
 * 0.00  ~35 ms    almost just the tock
 * 0.50  ~460 ms   classic useful kick tail
 * 0.80  ~2.1 s    long bass
 * 0.97  ~5.1 s    near-drone
 * >=.99            infinite sustain until the next trigger / parameter move
 */
static float MacroDecaySeconds(float x)
{
    x = Clamp01Added(x);

    if(x >= 0.99f)
        return 1000000.0f;

    return
        0.035f *
        powf(
            171.428571f,
            x
        );
}


static bool MacroDecayInfinite(float x)
{
    return
        x >= 0.99f;
}


/*
 * The master release follows the same musical scale, but bottoms out
 * slightly faster so Macro 2 at minimum truly cuts to the tock.
 */
static float MacroMasterReleaseMs(float x)
{
    x = Clamp01Added(x);

    if(MacroDecayInfinite(x))
        return 1000000000.0f;

    /*
     * 0 -> 4 ms       hard but click-safe chop
     * 0.5 -> ~155 ms  short kick
     * 0.8 -> ~1.4 s   long bass
     * top -> several seconds, then explicit INF
     */
    /*
     * 8 ms minimum keeps a hard rhythmic chop but removes the nearly
     * instantaneous low-frequency discontinuity of the old 4 ms floor.
     *
     * The curve is re-scaled so the useful middle remains close to the
     * previous timing.
     */
    return
        8.0f *
        powf(
            375.0f,
            x
        );
}


/*
 * Macro 6 — Syntakt-like kick-shape continuum.
 *
 * 0.00   BODY:
 *        shallow pitch displacement, slow/soft settling, body dominant
 *
 * 0.50   PUNCH:
 *        useful tekno middle ground
 *
 * 1.00   SNAP:
 *        obvious ~500 Hz -> root mid-drop in roughly 30..50 ms
 */
static float MacroKickShapeStartRatio(float x)
{
    x = Clamp01Added(x);


    if(!USE_NEW_SHAPE_TRANSIENT_MACRO)
    {
        /*
         * EXACT PREVIOUS MAPPING:
         * 0.0 -> 12x
         * 0.5 -> 5.7x
         * 1.0 -> 1.05x
         */
        if(x <= 0.50f)
        {
            float t =
                SmoothstepAdded(
                    x / 0.50f
                );

            return
                12.0f +
                (
                    5.70f -
                    12.0f
                ) *
                t;
        }


        float t =
            SmoothstepAdded(
                (x - 0.50f) /
                0.50f
            );

        return
            5.70f +
            (
                1.05f -
                5.70f
            ) *
            t;
    }


    /*
     * NEW ROUND -> PUNCH -> SNAP mapping.
     */
    if(x <= 0.50f)
    {
        float t =
            SmoothstepAdded(
                x / 0.50f
            );

        return
            SHAPE_ROUND_START_RATIO +
            (
                SHAPE_PUNCH_START_RATIO -
                SHAPE_ROUND_START_RATIO
            ) *
            t;
    }


    float t =
        SmoothstepAdded(
            (x - 0.50f) /
            0.50f
        );


    return
        SHAPE_PUNCH_START_RATIO +
        (
            SHAPE_SNAP_START_RATIO -
            SHAPE_PUNCH_START_RATIO
        ) *
        t;
}


static float MacroKickShapeSweepSeconds(float x)
{
    x = Clamp01Added(x);


    if(!USE_NEW_SHAPE_TRANSIENT_MACRO)
    {
        /*
         * EXACT PREVIOUS MAPPING:
         * 0.0 ->   8 ms
         * 0.5 ->  69 ms
         * 1.0 -> 320 ms
         */
        if(x <= 0.50f)
        {
            float t =
                SmoothstepAdded(
                    x / 0.50f
                );

            return
                0.008f +
                (
                    0.069f -
                    0.008f
                ) *
                t;
        }


        float t =
            SmoothstepAdded(
                (x - 0.50f) /
                0.50f
            );


        return
            0.069f +
            (
                0.320f -
                0.069f
            ) *
            t;
    }


    float ms;


    if(x <= 0.50f)
    {
        float t =
            SmoothstepAdded(
                x / 0.50f
            );

        ms =
            SHAPE_ROUND_SWEEP_MS +
            (
                SHAPE_PUNCH_SWEEP_MS -
                SHAPE_ROUND_SWEEP_MS
            ) *
            t;
    }
    else
    {
        float t =
            SmoothstepAdded(
                (x - 0.50f) /
                0.50f
            );

        ms =
            SHAPE_PUNCH_SWEEP_MS +
            (
                SHAPE_SNAP_SWEEP_MS -
                SHAPE_PUNCH_SWEEP_MS
            ) *
            t;
    }


    return
        ms /
        1000.0f;
}


static float ShapeThreePoint(
    float x,
    float round_value,
    float punch_value,
    float snap_value)
{
    x = Clamp01Added(x);

    if(x <= 0.50f)
    {
        float t =
            SmoothstepAdded(
                x / 0.50f
            );

        return
            round_value +
            (
                punch_value -
                round_value
            ) *
            t;
    }

    float t =
        SmoothstepAdded(
            (x - 0.50f) /
            0.50f
        );

    return
        punch_value +
        (
            snap_value -
            punch_value
        ) *
        t;
}


static float MacroKickShapeTransientGain(float x)
{
    return
        ShapeThreePoint(
            x,
            SHAPE_ROUND_TRANSIENT_GAIN,
            SHAPE_PUNCH_TRANSIENT_GAIN,
            SHAPE_SNAP_TRANSIENT_GAIN
        );
}


static float MacroKickShapeTransientDrive(float x)
{
    return
        ShapeThreePoint(
            x,
            SHAPE_ROUND_DRIVE,
            SHAPE_PUNCH_DRIVE,
            SHAPE_SNAP_DRIVE
        );
}


static float MacroKickShapeTransientCutoff(float x)
{
    return
        ShapeThreePoint(
            x,
            SHAPE_ROUND_CUTOFF_HZ,
            SHAPE_PUNCH_CUTOFF_HZ,
            SHAPE_SNAP_CUTOFF_HZ
        );
}


static float MacroKickShapeAttackSeconds(float x)
{
    return
        ShapeThreePoint(
            x,
            SHAPE_ROUND_ATTACK_MS,
            SHAPE_PUNCH_ATTACK_MS,
            SHAPE_SNAP_ATTACK_MS
        )
        /
        1000.0f;
}


static float MacroKickShapeHandoffStartMs(float x)
{
    return
        ShapeThreePoint(
            x,
            SHAPE_ROUND_HANDOFF_START_MS,
            SHAPE_PUNCH_HANDOFF_START_MS,
            SHAPE_SNAP_HANDOFF_START_MS
        );
}


static float MacroKickShapeHandoffEndMs(float x)
{
    return
        ShapeThreePoint(
            x,
            SHAPE_ROUND_HANDOFF_END_MS,
            SHAPE_PUNCH_HANDOFF_END_MS,
            SHAPE_SNAP_HANDOFF_END_MS
        );
}


/*
 * Macro 3 — PUMP -> TAIL-SEPARATION MORPH
 *
 * Full scale is still:
 *
 *     two 1/16 steps = one 1/8 note = half a quarter note
 *
 * But the time offset is deliberately NONLINEAR.
 *
 * Low K3:
 *     mostly a little sidechain-style body duck under the punch
 *     very little actual rhythmic displacement
 *
 * High K3:
 *     increasingly obvious body separation
 *     reaches the full two-step delay at maximum
 */
static float MacroTailDelayMs(float x)
{
    x = Clamp01Added(x);


    /*
     * Power curve keeps the first part of the knob in "pump" territory
     * instead of immediately sounding like a delayed/gated kick.
     */
    float delay_curve =
        powf(
            x,
            1.70f
        );


    return
        perf_quarter_note_ms *
        0.50f *
        delay_curve;
}


/*
 * Macro 4 — musical logarithmic BPF centre.
 */
static float MacroBpfFrequencyHz(float x)
{
    x = Clamp01Added(x);

    /*
     * Extended downward from the previous 140 Hz floor.
     */
    const float low_hz = 115.0f;
    const float high_hz = 3200.0f;

    return
        low_hz *
        powf(
            high_hz /
            low_hz,
            x
        );
}


/* MIDI */

static constexpr uint8_t MIDI_CHANNEL_KICK = 14; // MIDI ch 15
static constexpr uint8_t MIDI_CHANNEL_LAYER = 15; // MIDI ch 16


/* OLED */

static constexpr uint8_t OLED_ADDRESS = 0x3C;


/* ============================================================
   MIDI STATE
   ============================================================ */

static bool midi_running = false;

static uint8_t midi_running_status = 0;
static uint8_t midi_data[2];
static uint8_t midi_data_count = 0;

static uint8_t last_note = 36;
static uint8_t last_velocity = 100;
static bool note_gate = false;

static uint32_t trigger_count = 0;


/*
 * Timing of current MIDI note.
 */
static uint32_t note_on_time = 0;


/*
 * Gate duration is calculated when Note Off arrives.
 */
static volatile float current_gate_ms = 80.0f;


/* ============================================================
   MIDI → AUDIO EVENTS
   ============================================================ */

static volatile bool kick_trigger_pending = false;
static volatile bool kick_release_pending = false;


/*
 * Channel 16 character layer.
 */
static volatile bool layer16_active = false;
static volatile float layer16_level = 0.0f;


/* ============================================================
   KICK PARAMETERS
   ============================================================ */

/*
 * MIDI note controls this.
 */
static volatile float kick_frequency = 55.0f;


/*
 * MIDI velocity controls this.
 *
 * 0 = sine
 * 1 = square
 */
static volatile float oscillator_morph = 0.5f;


/*
 * Gate-derived temporal separation.
 *
 * This is calculated from the actual MIDI gate length.
 */
static volatile float separation = 0.50f;


/*
 * Current pitch character.
 *
 * This is deliberately internal for now.
 *
 * Later this can become a physical knob / CC macro.
 */
static constexpr float DEFAULT_PITCH_CHARACTER = 0.55f;


/*
 * Current transient character.
 *
 * Internal macro for now.
 */
static constexpr float DEFAULT_TRANSIENT_CHARACTER = 0.55f;


/*
 * Bass character.
 *
 * Internal macro for now.
 */
static constexpr float DEFAULT_BASS_CHARACTER = 0.40f;


/*
 * Overall time character.
 *
 * Gate length is currently the dominant time control.
 */
static constexpr float DEFAULT_TIME_CHARACTER = 0.50f;


/* ============================================================
   PHASE-LOCKED OSCILLATORS
   ============================================================ */

/*
 * Every oscillator is explicitly reset at every kick.
 *
 * This is important:
 *
 *     phase = 0
 *
 * at every trigger.
 *
 * Therefore a static parameter state produces the same initial
 * waveform relationship every time.
 */


/* Main transient phase. */
static float transient_phase = 0.0f;


/* Clean sub phase. */
static float sub_phase = 0.0f;


/*
 * Supersaw phases.
 *
 * These are deliberately initialised to fixed offsets.
 */
static constexpr int SUPER_COUNT = 5;

static float super_phase[SUPER_COUNT];


/*
 * Small fixed detuning.
 *
 * Deliberately very modest because this is a kick, not a pad.
 */
static constexpr float super_detune[SUPER_COUNT] =
{
    -0.012f,
    -0.006f,
     0.000f,
     0.006f,
     0.012f
};


/*
 * Fixed initial phase offsets.
 *
 * They never change between triggers.
 */
static constexpr float super_phase_offset[SUPER_COUNT] =
{
    0.000f,
    0.071f,
    0.143f,
    0.217f,
    0.291f
};


/* ============================================================
   ENVELOPE ENGINE
   ============================================================ */

struct Envelope
{
    float value = 0.0f;

    float attack = 0.001f;
    float decay = 0.5f;

    /*
     * 0 = pure exponential (unchanged), 1 = straight linear ramp to zero
     * over `decay`. Only the one-shot decay is affected. Per-instance, so
     * the transient envelope keeps its original exponential curve.
     */
    float decay_linearity = 0.0f;

    bool active = false;
    bool holding = false;
    bool releasing = false;

    /*
     * One-shot ratchet-safe reinforcement:
     * rise smoothly from current value toward 1, then enter normal decay.
     */
    bool attack_then_decay = false;


    void Trigger()
    {
        value = 0.0f;
        active = true;
        holding = false;
        releasing = false;
        attack_then_decay = false;
    }


    void RetriggerFromCurrent()
    {
        /*
         * Ratchet-safe retrigger:
         *
         * Preserve the current amplitude instead of jumping to zero.
         * The corrected attack law then approaches the new target from
         * this exact value without a waveform discontinuity.
         */
        if(!active)
            value = 0.0f;

        active = true;
        holding = false;
        releasing = false;
        attack_then_decay = false;
    }


    void AttackThenDecayFromCurrent()
    {
        /*
         * Used by the bass tail on both fresh hits and overlapping
         * ratchets. Never steps amplitude; it approaches 1 smoothly and
         * automatically falls into the normal one-shot decay.
         */
        if(!active)
            value = 0.0f;

        active = true;
        holding = false;
        releasing = false;
        attack_then_decay = true;
    }


    void Gate()
    {
        active = true;
        holding = true;
        releasing = false;
        attack_then_decay = false;
    }


    void Release(float release_time)
    {
        if(!active)
            return;

        holding = false;
        releasing = true;
        attack_then_decay = false;

        if(release_time < 0.001f)
            release_time = 0.001f;

        decay = release_time;
    }


    float Process(float sr)
    {
        if(!active)
            return 0.0f;


        if(releasing)
        {
            /*
             * Exponential release.
             *
             * About 60 dB over the requested release period.
             */
            float coefficient =
                expf(-6.9078f / (decay * sr));

            value *= coefficient;


            if(value < 0.0001f)
            {
                value = 0.0f;
                active = false;
                releasing = false;
            }

            return value;
        }


        if(attack_then_decay)
        {
            /*
             * Correct click-safe approach to full level.
             */
            float coefficient =
                expf(
                    -6.9078f /
                    (
                        attack *
                        sr
                    )
                );


            float alpha =
                1.0f -
                coefficient;


            value +=
                (
                    1.0f -
                    value
                ) *
                alpha;


            if(value >= 0.9990f)
            {
                value = 1.0f;
                attack_then_decay = false;
            }


            return value;
        }


        if(holding)
        {
            /*
             * CLICK-SAFE ATTACK.
             *
             * IMPORTANT BUG FIX:
             *
             * The old code multiplied the distance-to-target by the
             * exponential coefficient itself. For a 0.8 ms attack that
             * jumped to roughly 83% amplitude on sample ONE.
             *
             * The correct one-pole approach coefficient is:
             *
             *     alpha = 1 - exp(...)
             *
             * so the waveform actually rises over the requested attack
             * time instead of creating a broadband discontinuity.
             */
            float coefficient =
                expf(
                    -6.9078f /
                    (
                        attack *
                        sr
                    )
                );


            float alpha =
                1.0f -
                coefficient;


            value +=
                (
                    1.0f -
                    value
                ) *
                alpha;


            if(value > 0.9999f)
                value = 1.0f;


            return value;
        }


        /*
         * Normal one-shot decay, blended between exponential and linear.
         *
         * A pure exponential drops most of its level in the first fraction
         * of the decay time, which reads as the tail collapsing straight
         * after the attack. Blending toward a linear ramp holds the body up
         * and lets it fall away evenly.
         */
        float coefficient =
            expf(-6.9078f / (decay * sr));

        float exponential_value = value * coefficient;

        if(decay_linearity > 0.0001f)
        {
            float linear_value =
                value - (1.0f / (decay * sr));

            value =
                exponential_value +
                (linear_value - exponential_value) * decay_linearity;
        }
        else
        {
            value = exponential_value;
        }


        if(value < 0.0001f)
        {
            value = 0.0f;
            active = false;
        }


        return value;
    }
};


/*
 * Separate envelopes:
 *
 * transient amplitude
 * tail amplitude
 */
static Envelope transient_env;
static Envelope tail_env;


/* ============================================================
   PITCH ENVELOPES
   ============================================================ */

static float transient_pitch_phase = 1.0f;
static float tail_pitch_phase = 1.0f;


/*
 * These are time constants rather than arbitrary linear
 * envelopes. Pitch therefore behaves musically.
 */
static float transient_pitch_time = 0.055f;
static float tail_pitch_time = 0.18f;


/*
 * Per-hit age drives the transient->tail handoff and guarantees that
 * velocity-controlled tail bending cannot disturb the protected knock.
 */
static uint32_t kick_age_samples = 0;


/*
 * Set once at the beginning of an audio-callback trigger event.
 * All DSP trigger functions may inspect it.
 */
static bool kick_retrigger_active = false;


/*
 * Phase-lock ratchet bridge.
 */
static bool ratchet_phase_bridge_active = false;

static float ratchet_bridge_phase = 0.0f;
static float ratchet_bridge_frequency = 55.0f;
static float ratchet_bridge_tail_gain = 0.0f;
static float ratchet_bridge_residual = 0.0f;

static uint32_t ratchet_bridge_pos = 0;
static uint32_t ratchet_bridge_samples = 1;

static float last_generated_kick_signal = 0.0f;
static float current_clean_tail_gain_for_bridge = 0.0f;
static float current_tail_frequency_for_bridge = 55.0f;


/*
 * Final >8 kHz dynamics state.
 */
static float final_hf_low_state = 0.0f;
static float final_hf_envelope = 0.0f;
static float final_hf_gain = 1.0f;


/*
 * Per-hit SHAPE values latched on Note-On.
 */
static float transient_shape_gain_current = SHAPE_PUNCH_TRANSIENT_GAIN;
static float transient_shape_drive_current = SHAPE_PUNCH_DRIVE;
static float transient_shape_cutoff_current = SHAPE_PUNCH_CUTOFF_HZ;

static float transient_handoff_start_ms_current =
    SHAPE_PUNCH_HANDOFF_START_MS;

static float transient_handoff_end_ms_current =
    SHAPE_PUNCH_HANDOFF_END_MS;

static float transient_tone_lp_state = 0.0f;

/*
 * Two-pole-ish onset HF guard states.
 */
static float transient_hf_guard_state_1 = 0.0f;
static float transient_hf_guard_state_2 = 0.0f;


/*
 * Final whole-generated-kick HF guard states.
 */
static float final_kick_hf_state_1 = 0.0f;
static float final_kick_hf_state_2 = 0.0f;
static float final_kick_hf_state_3 = 0.0f;


/*
 * Performance-filter bus safety state.
 *
 * These are NEVER reset on Note-On; preserving their history is part of
 * the de-click design.
 */
static float performance_filter_headroom_gain = 1.0f;

static float post_perf_hf_state_1 = 0.0f;
static float post_perf_hf_state_2 = 0.0f;
static float post_perf_hf_mix = 0.0f;


/*
 * ABSOLUTE FINAL LPF ENFORCEMENT
 *
 * Purely linear. No nonlinear stage is permitted after this.
 */
static float final_lpf_enforce_state_1 = 0.0f;
static float final_lpf_enforce_state_2 = 0.0f;
static float final_lpf_enforce_state_3 = 0.0f;
static float final_lpf_enforce_state_4 = 0.0f;

static constexpr float FINAL_LPF_ENFORCE_BEGIN = 0.72f;
static constexpr float FINAL_LPF_LINEAR_HEADROOM = 0.90f;


/*
 * Velocity-latched tail tuning state.
 *
 * The sub/tail oscillator phase is NEVER reset when pitch changes:
 * only its phase increment is moved, which gives a continuous glide.
 */
static bool tail_velocity_sweep_armed = false;
static bool tail_velocity_sweep_started = false;

static float tail_velocity_sweep_target_ratio = 1.0f;
static float tail_velocity_sweep_start_ratio = 1.0f;
static float tail_velocity_sweep_current_ratio = 1.0f;

static uint32_t tail_velocity_sweep_age_samples = 0;


/*
 * State used ONLY for the optional Mackie/Sherman character-delta
 * sub protector. Declared with the core kick state because it is reset
 * deterministically by TriggerKickAudio().
 */
static float character_delta_hp_state = 0.0f;
static float character_delta_hp_state_2 = 0.0f;
static float character_delta_hp_state_3 = 0.0f;



/* ============================================================
   KICK STATE
   ============================================================ */

static float transient_start_frequency = 300.0f;
static float tail_frequency = 55.0f;


/*
 * Character layer filter frequency.
 */
static float character_frequency = 500.0f;


/*
 * Current filter pattern.
 */
static float filter_pattern[16];

static uint8_t filter_pattern_step = 0;


/*
 * Pattern generation seed.
 */
static uint32_t random_seed = 0x13579BDF;


/*
 * Only generate a new pattern when NOTE VALUE changes.
 */
static int last_pattern_note = -1;


/* ============================================================
   RANDOM NUMBER GENERATOR
   ============================================================ */

static uint32_t RandomU32()
{
    random_seed =
        random_seed * 1664525u +
        1013904223u;

    return random_seed;
}


static float Random01()
{
    return static_cast<float>(
        (RandomU32() >> 8) & 0x00FFFFFF
    ) / 16777215.0f;
}


/* ============================================================
   FREQUENCY UTILITIES
   ============================================================ */

static float MidiNoteToFrequency(uint8_t note)
{
    float semitones =
        static_cast<float>(note) - 69.0f;

    return 440.0f *
           powf(
               2.0f,
               semitones / 12.0f
           );
}


/*
 * Logarithmic interpolation.
 */
static float LogInterpolate(float a,
                            float b,
                            float t)
{
    return a *
           powf(
               b / a,
               t
           );
}


/* ============================================================
   FILTER PATTERN GENERATION
   ============================================================ */

/*
 * Generates a deliberately correlated 16-step pattern.
 *
 * We use four anchor points and interpolate between them.
 * Random jitter is added afterwards.
 *
 * This gives:
 *
 *     related neighbouring steps
 *     repeated broad movement
 *     occasional bigger changes
 *
 * rather than 16 independent random values.
 */

static void GenerateCharacterPattern()
{
    float anchors[4];


    /*
     * Musical-ish frequency regions.
     */
    const float min_freq = 180.0f;
    const float max_freq = 3200.0f;


    for(int i = 0; i < 4; i++)
    {
        float r = Random01();

        /*
         * Slight preference toward lower mids.
         */
        r = r * r * 0.75f + r * 0.25f;

        anchors[i] =
            LogInterpolate(
                min_freq,
                max_freq,
                r
            );
    }


    /*
     * Interpolate four sections.
     */
    for(int i = 0; i < 16; i++)
    {
        int section =
            i / 4;

        int position =
            i % 4;


        float t =
            static_cast<float>(position)
            / 4.0f;


        float a =
            anchors[section];


        float b =
            anchors[
                (section + 1) % 4
            ];


        float f =
            LogInterpolate(
                a,
                b,
                t
            );


        /*
         * Controlled local variation.
         */
        float jitter =
            0.90f +
            Random01() * 0.20f;


        f *= jitter;


        if(f < min_freq)
            f = min_freq;

        if(f > max_freq)
            f = max_freq;


        filter_pattern[i] = f;
    }
}


/* ============================================================
   POLYBLEP
   ============================================================ */

static inline float PolyBlep(float t,
                             float dt)
{
    if(t < dt)
    {
        t /= dt;

        return t + t - t * t - 1.0f;
    }


    if(t > 1.0f - dt)
    {
        t = (t - 1.0f) / dt;

        return t * t + t + t + 1.0f;
    }


    return 0.0f;
}


/* ============================================================
   WAVEFORMS
   ============================================================ */

static inline float Sine(float phase)
{
    return sinf(
        phase *
        6.28318530718f
    );
}


static inline float Saw(float phase,
                        float phase_increment)
{
    float y =
        2.0f * phase - 1.0f;


    y -= PolyBlep(
        phase,
        phase_increment
    );


    return y;
}


static inline float Square(float phase,
                           float phase_increment)
{
    float y =
        phase < 0.5f
            ? 1.0f
            : -1.0f;


    y += PolyBlep(
        phase,
        phase_increment
    );


    float shifted =
        phase + 0.5f;

    if(shifted >= 1.0f)
        shifted -= 1.0f;


    y -= PolyBlep(
        shifted,
        phase_increment
    );


    return y;
}


/* ============================================================
   OSCILLATOR MORPH
   ============================================================ */

static float GenerateTransientOscillator(
    float frequency,
    float morph)
{
    if(frequency < 20.0f)
        frequency = 20.0f;

    if(frequency > 1200.0f)
        frequency = 1200.0f;


    float dt =
        frequency /
        SAMPLE_RATE;


    if(dt > 0.45f)
        dt = 0.45f;


    float sine =
        Sine(
            transient_phase
        );


    if(SINE_ONLY_OSCILLATORS)
    {
        /*
         * Diagnostic / musical sine-only mode.
         *
         * No saw, supersaw or square is even evaluated, so they cannot
         * contribute HF or transient discontinuities.
         */
        transient_phase +=
            dt;


        while(transient_phase >= 1.0f)
            transient_phase -= 1.0f;


        return sine;
    }


    /*
     * Original fixed2 morph architecture retained behind the switch.
     */
    float saw =
        Saw(
            transient_phase,
            dt
        );


    float square =
        Square(
            transient_phase,
            dt
        );


    float supersaw = 0.0f;


    for(int i = 0;
        i < SUPER_COUNT;
        i++)
    {
        float p =
            super_phase[i];


        float f =
            frequency *
            (
                1.0f +
                super_detune[i]
            );


        float sdt =
            f /
            SAMPLE_RATE;


        if(sdt > 0.45f)
            sdt = 0.45f;


        supersaw +=
            Saw(
                p,
                sdt
            );
    }


    supersaw /=
        static_cast<float>(
            SUPER_COUNT
        );


    float result;


    if(morph < 0.333333f)
    {
        float t =
            morph /
            0.333333f;


        float a =
            cosf(
                t *
                1.5707963f
            );


        float b =
            sinf(
                t *
                1.5707963f
            );


        result =
            sine * a +
            saw * b;


        result *=
            0.93f;
    }
    else if(morph < 0.666666f)
    {
        float t =
            (
                morph -
                0.333333f
            )
            /
            0.333333f;


        float a =
            cosf(
                t *
                1.5707963f
            );


        float b =
            sinf(
                t *
                1.5707963f
            );


        result =
            saw * a +
            supersaw * b;


        result *=
            0.90f;
    }
    else
    {
        float t =
            (
                morph -
                0.666666f
            )
            /
            0.333334f;


        float a =
            cosf(
                t *
                1.5707963f
            );


        float b =
            sinf(
                t *
                1.5707963f
            );


        result =
            supersaw * a +
            square * b;


        result *=
            0.86f;
    }


    transient_phase +=
        dt;


    while(transient_phase >= 1.0f)
        transient_phase -= 1.0f;


    for(int i = 0;
        i < SUPER_COUNT;
        i++)
    {
        float f =
            frequency *
            (
                1.0f +
                super_detune[i]
            );


        super_phase[i] +=
            f /
            SAMPLE_RATE;


        while(super_phase[i] >= 1.0f)
            super_phase[i] -= 1.0f;
    }


    return result;
}


static bool ResetDspStateForThisTrigger()
{
    /*
     * Fresh hits always reset.
     *
     * In PSY phase-lock mode, overlapping ratchets also deliberately
     * reset so each new kick is a repeatable one-shot.
     */
    return
        !kick_retrigger_active ||
        PSY_PHASE_LOCK_EVERY_HIT;
}


static void BeginRatchetPhaseBridge()
{
    if(!kick_retrigger_active ||
       !PSY_PHASE_LOCK_EVERY_HIT)
    {
        ratchet_phase_bridge_active = false;
        return;
    }


    ratchet_bridge_phase =
        sub_phase;


    ratchet_bridge_frequency =
        ClampAdded(
            current_tail_frequency_for_bridge,
            24.0f,
            95.0f
        );


    ratchet_bridge_tail_gain =
        current_clean_tail_gain_for_bridge;


    /*
     * Predict the next old-tail sample. Build the tiny residual so the
     * first bridge output equals the exact previous generated-kick value.
     */
    float predicted_old_tail =
        sinf(
            ratchet_bridge_phase *
            TWO_PI
        )
        *
        ratchet_bridge_tail_gain;


    ratchet_bridge_residual =
        last_generated_kick_signal -
        predicted_old_tail;


    ratchet_bridge_pos = 0;


    ratchet_bridge_samples =
        static_cast<uint32_t>(
            RATCHET_PHASE_BRIDGE_MS *
            SAMPLE_RATE /
            1000.0f
        );


    if(ratchet_bridge_samples < 16)
        ratchet_bridge_samples = 16;


    ratchet_phase_bridge_active = true;
}


static float ProcessRatchetPhaseBridge(
    float new_kick)
{
    if(!ratchet_phase_bridge_active)
        return new_kick;


    float t =
        static_cast<float>(
            ratchet_bridge_pos
        )
        /
        static_cast<float>(
            ratchet_bridge_samples
        );


    t =
        Clamp01Added(
            t
        );


    float smooth_t =
        SmoothstepAdded(
            t
        );


    /*
     * Continue the OLD low-frequency sine for a few milliseconds.
     */
    float old_tail =
        sinf(
            ratchet_bridge_phase *
            TWO_PI
        )
        *
        ratchet_bridge_tail_gain;


    ratchet_bridge_phase +=
        ratchet_bridge_frequency /
        SAMPLE_RATE;


    while(ratchet_bridge_phase >= 1.0f)
        ratchet_bridge_phase -= 1.0f;


    /*
     * Preserve exact sample continuity at t=0, but discard the old
     * nonlinear/HF remainder extremely quickly.
     */
    float elapsed_ms =
        static_cast<float>(
            ratchet_bridge_pos
        )
        *
        1000.0f /
        SAMPLE_RATE;


    float residual_gain =
        expf(
            -6.9078f *
            elapsed_ms /
            RATCHET_RESIDUAL_DECAY_MS
        );


    float old_bridge =
        old_tail +
        ratchet_bridge_residual *
        residual_gain;


    /*
     * Equal-power handoff:
     * old deterministic continuation -> new deterministic phase-reset hit.
     */
    float old_gain =
        cosf(
            smooth_t *
            1.57079632679f
        );


    float new_gain =
        sinf(
            smooth_t *
            1.57079632679f
        );


    float output =
        old_bridge *
        old_gain +
        new_kick *
        new_gain;


    ratchet_bridge_pos++;


    if(ratchet_bridge_pos >=
       ratchet_bridge_samples)
    {
        ratchet_phase_bridge_active = false;
    }


    return output;
}


static float ProcessFinalHfDynamicTamer(
    float input)
{
    if(!ENABLE_FINAL_HF_DYNAMIC_TAMER)
    {
        /*
         * Keep states tracking so enabling the feature cannot click.
         */
        final_hf_low_state = input;
        final_hf_envelope = 0.0f;
        final_hf_gain = 1.0f;

        return input;
    }


    /*
     * Reconstructive one-pole crossover.
     */
    float lp_alpha =
        1.0f -
        expf(
            -TWO_PI *
            FINAL_HF_DYNAMIC_CROSSOVER_HZ /
            SAMPLE_RATE
        );


    final_hf_low_state +=
        lp_alpha *
        (
            input -
            final_hf_low_state
        );


    float high =
        input -
        final_hf_low_state;


    float detector =
        fabsf(
            high
        );


    float attack_samples =
        FINAL_HF_DYNAMIC_ATTACK_MS *
        SAMPLE_RATE /
        1000.0f;


    float release_samples =
        FINAL_HF_DYNAMIC_RELEASE_MS *
        SAMPLE_RATE /
        1000.0f;


    if(attack_samples < 1.0f)
        attack_samples = 1.0f;


    if(release_samples < 1.0f)
        release_samples = 1.0f;


    float env_alpha =
        detector >
        final_hf_envelope
        ? (
            1.0f -
            expf(
                -1.0f /
                attack_samples
            )
          )
        : (
            1.0f -
            expf(
                -1.0f /
                release_samples
            )
          );


    final_hf_envelope +=
        (
            detector -
            final_hf_envelope
        )
        *
        env_alpha;


    float target_gain = 1.0f;


    if(final_hf_envelope >
       FINAL_HF_DYNAMIC_THRESHOLD)
    {
        /*
         * Standard amplitude-domain compressor curve:
         *
         * gain = (threshold / envelope)^(1 - 1/ratio)
         */
        float exponent =
            1.0f -
            1.0f /
            FINAL_HF_DYNAMIC_RATIO;


        target_gain =
            powf(
                FINAL_HF_DYNAMIC_THRESHOLD /
                final_hf_envelope,
                exponent
            );


        if(target_gain <
           FINAL_HF_DYNAMIC_MIN_GAIN)
        {
            target_gain =
                FINAL_HF_DYNAMIC_MIN_GAIN;
        }
    }


    /*
     * Fast gain reduction, smooth recovery.
     */
    float gain_samples =
        target_gain <
        final_hf_gain
        ? attack_samples
        : release_samples;


    float gain_alpha =
        1.0f -
        expf(
            -1.0f /
            gain_samples
        );


    final_hf_gain +=
        (
            target_gain -
            final_hf_gain
        )
        *
        gain_alpha;


    /*
     * When final_hf_gain == 1:
     *
     *     low + high == input
     *
     * exactly, so there is no permanent crossover coloration.
     */
    return
        final_hf_low_state +
        high *
        final_hf_gain;
}


/* ============================================================
   PHASE RESET
   ============================================================ */

static void ResetKickPhases()
{
    bool preserve_phase =
        !PSY_PHASE_LOCK_EVERY_HIT &&
        PHASE_CONTINUOUS_RATCHET_RETRIGGER &&
        kick_retrigger_active;


    if(!preserve_phase)
    {
        /*
         * Psytrance-style deterministic phase:
         * every new kick starts from the same point.
         */
        transient_phase = 0.0f;
        sub_phase = 0.0f;


        for(int i = 0;
            i < SUPER_COUNT;
            i++)
        {
            super_phase[i] =
                super_phase_offset[i];
        }
    }


    transient_pitch_phase = 1.0f;
    tail_pitch_phase = 1.0f;
}


/* ============================================================
   MUSICAL KICK SWEEP MAPPINGS
   ============================================================ */

/*
 * Middle position 0.55 is the exact original fixed2 value.
 *
 * DEPTH:
 *   0.00  1.5x fundamental
 *   0.55  5.7x  (fixed2)
 *   1.00 11.5x
 */
static float KickSweepDepthToRatio(float x)
{
    x = Clamp01Added(x);

    if(x <= 0.55f)
    {
        float t =
            SmoothstepAdded(
                x / 0.55f
            );

        return
            1.50f +
            (5.70f - 1.50f) *
            t;
    }

    float t =
        SmoothstepAdded(
            (x - 0.55f) /
            0.45f
        );

    return
        5.70f +
        (11.50f - 5.70f) *
        t;
}


/*
 * TIME:
 *   0.00    8 ms
 *   0.55   69 ms  (fixed2)
 *   1.00  260 ms
 */
static float KickSweepTimeToSeconds(float x)
{
    x = Clamp01Added(x);

    if(x <= 0.55f)
    {
        float t =
            SmoothstepAdded(
                x / 0.55f
            );

        return
            0.008f +
            (0.069f - 0.008f) *
            t;
    }

    float t =
        SmoothstepAdded(
            (x - 0.55f) /
            0.45f
        );

    return
        0.069f +
        (0.260f - 0.069f) *
        t;
}


/*
 * Logarithmic BODY LP range:
 * 650 Hz .. 16 kHz.
 */
static float KickBodyLpCutoffHz(float x)
{
    x = Clamp01Added(x);

    const float low_hz = 650.0f;
    const float high_hz = 16000.0f;

    return
        low_hz *
        powf(
            high_hz / low_hz,
            x
        );
}


/*
 * Resonance/Q range:
 * 0.60 .. 2.20.
 */
static float KickBodyLpQ(float x)
{
    x = Clamp01Added(x);

    return
        0.60f +
        x * 1.60f;
}


/* ============================================================
   KICK ANATOMY / TAIL-SWEEP UTILITIES
   ============================================================ */

static float SemitonesToRatio(
    float semitones)
{
    return
        powf(
            2.0f,
            semitones /
            12.0f
        );
}


static float VelocityToTailSweepSemitones(
    uint8_t velocity)
{
    if(!ENABLE_VELOCITY_TAIL_SWEEP)
        return 0.0f;


    if(velocity < 1u)
        velocity = 1u;


    if(velocity <=
       TAIL_SWEEP_CENTER_VELOCITY)
    {
        /*
         * 1 -> full DOWN
         * 64 -> flat
         *
         * Linear in semitones: every velocity step has predictable pitch
         * resolution instead of being dependent on Note-Off timing.
         */
        float t =
            static_cast<float>(
                velocity - 1u
            )
            /
            static_cast<float>(
                TAIL_SWEEP_CENTER_VELOCITY - 1u
            );


        t =
            Clamp01Added(
                t
            );


        return
            TAIL_SWEEP_DOWN_SEMITONES *
            (
                1.0f -
                t
            );
    }


    /*
     * 64 -> flat
     * 127 -> full UP
     */
    float t =
        static_cast<float>(
            velocity -
            TAIL_SWEEP_CENTER_VELOCITY
        )
        /
        static_cast<float>(
            127u -
            TAIL_SWEEP_CENTER_VELOCITY
        );


    t =
        Clamp01Added(
            t
        );


    return
        TAIL_SWEEP_UP_SEMITONES *
        t;
}


static void ArmVelocityTailSweep(
    uint8_t velocity)
{
    /*
     * Convert velocity to a pitch ratio ONCE at Note-On. Keeping powf/log2f
     * out of the per-sample audio path leaves substantially more CPU margin
     * for Sherman, Mackie, looper and performance filtering.
     */
    tail_velocity_sweep_target_ratio =
        SemitonesToRatio(
            VelocityToTailSweepSemitones(
                velocity
            )
        );


    /*
     * Do not restart oscillator phase.
     *
     * Capture the exact current tune as the start of the glide.
     */
    tail_velocity_sweep_start_ratio =
        tail_velocity_sweep_current_ratio;


    tail_velocity_sweep_age_samples = 0;

    tail_velocity_sweep_armed = true;
    tail_velocity_sweep_started = false;
}


static float ProcessVelocityTailPitchRatio()
{
    if(!tail_velocity_sweep_armed)
        return
            tail_velocity_sweep_current_ratio;


    float kick_age_ms =
        static_cast<float>(
            kick_age_samples
        )
        *
        1000.0f
        /
        SAMPLE_RATE;


    /*
     * The knock owns the first ~32 ms.
     *
     * Velocity was latched at Note-On. Wait until the protected onset
     * window has completed before beginning the bass-tail pitch gesture.
     */
    if(!tail_velocity_sweep_started)
    {
        if(kick_age_ms <
           PROTECTED_PUNCH_LENGTH_MS)
        {
            return
                tail_velocity_sweep_current_ratio;
        }


        tail_velocity_sweep_started = true;
        tail_velocity_sweep_age_samples = 0;

        tail_velocity_sweep_start_ratio =
            tail_velocity_sweep_current_ratio;
    }


    float glide_samples =
        TAIL_SWEEP_GLIDE_MS *
        SAMPLE_RATE /
        1000.0f;


    if(glide_samples < 1.0f)
        glide_samples = 1.0f;


    float t =
        static_cast<float>(
            tail_velocity_sweep_age_samples
        )
        /
        glide_samples;


    t =
        Clamp01Added(
            t
        );


    t =
        SmoothstepAdded(
            t
        );


    /*
     * Smooth ratio interpolation is phase-continuous and cheap. The target
     * itself is semitone-derived, but there is no powf/log2f in the audio
     * loop. Over this 115 ms gesture the result remains musically smooth.
     */
    tail_velocity_sweep_current_ratio =
        tail_velocity_sweep_start_ratio +
        (
            tail_velocity_sweep_target_ratio -
            tail_velocity_sweep_start_ratio
        )
        *
        t;


    if(t < 1.0f)
    {
        tail_velocity_sweep_age_samples++;
    }
    else
    {
        tail_velocity_sweep_current_ratio =
            tail_velocity_sweep_target_ratio;

        tail_velocity_sweep_armed = false;
    }


    return
        tail_velocity_sweep_current_ratio;
}


static void GetTransientTailCrossfade(
    float& transient_gain,
    float& tail_gain)
{
    float age_ms =
        static_cast<float>(
            kick_age_samples
        )
        *
        1000.0f
        /
        SAMPLE_RATE;


    float handoff_start_ms =
        USE_NEW_SHAPE_TRANSIENT_MACRO
        ? transient_handoff_start_ms_current
        : 7.0f;


    float handoff_end_ms =
        USE_NEW_SHAPE_TRANSIENT_MACRO
        ? transient_handoff_end_ms_current
        : 40.0f;


    float span =
        handoff_end_ms -
        handoff_start_ms;


    if(span < 1.0f)
        span = 1.0f;


    float t =
        (
            age_ms -
            handoff_start_ms
        )
        /
        span;


    t =
        Clamp01Added(
            t
        );


    t =
        SmoothstepAdded(
            t
        );


    /*
     * Equal-power handoff:
     *
     * transient: 1 -> 0
     * tail:      0 -> 1
     *
     * This avoids the audible hole of a linear crossfade.
     */
    transient_gain =
        cosf(
            t *
            1.57079632679f
        );


    tail_gain =
        sinf(
            t *
            1.57079632679f
        );
}


/* ============================================================
   KICK TRIGGER
   ============================================================ */

static void TriggerKickAudio(uint8_t velocity)
{
    ResetKickPhases();


    /*
     * New hit = new anatomy timeline.
     */
    kick_age_samples = 0;

    tail_velocity_sweep_armed = false;
    tail_velocity_sweep_started = false;
    tail_velocity_sweep_target_ratio = 1.0f;
    tail_velocity_sweep_start_ratio = 1.0f;
    tail_velocity_sweep_current_ratio = 1.0f;
    tail_velocity_sweep_age_samples = 0;

    /*
     * Velocity target is known at Note-On, so arm immediately. The
     * processor itself waits until the protected onset window has passed.
     */
    ArmVelocityTailSweep(
        velocity
    );

    /*
     * CHARACTER DELTA HPF STATE IS CONTINUOUS ACROSS KICKS.
     *
     * Previous bug:
     *     pole 1 was reset to zero on a fresh hit
     *     pole 2 retained the previous hit's state
     *
     * That mismatched two-pole state creates an impulse-like transient.
     * It also gets worse as decay/wet/filter amount leaves more residual
     * energy in pole 2, which matches the moving screech threshold.
     *
     * Never reset either pole here.
     */


    /*
     * Current MIDI fundamental.
     */
    tail_frequency =
        kick_frequency;


    /*
     * MACRO 6: KICK SHAPE.
     *
     * BODY/ROUND -> PUNCH -> SNAP transient continuum; HIGH = sharper/faster.
     */
    float pitch_multiplier =
        MacroKickShapeStartRatio(
            macro_kick_shape
        );


    transient_start_frequency =
        tail_frequency *
        pitch_multiplier;


    /*
     * Wider than the old 650 Hz ceiling, but still bounded.
     * The new resonant BODY LP is the musical way to contain the
     * extra harmonics when using extreme sweep.
     */
    if(transient_start_frequency > 850.0f)
        transient_start_frequency = 850.0f;


    /*
     * Longer gates create more temporal separation.
     */
    float sep =
        separation;


    /*
     * K6 SHAPE is latched at Note-On so an in-flight transient cannot
     * zipper when K6 moves.
     */
    if(USE_NEW_SHAPE_TRANSIENT_MACRO)
    {
        transient_shape_gain_current =
            MacroKickShapeTransientGain(
                macro_kick_shape
            );

        transient_shape_drive_current =
            MacroKickShapeTransientDrive(
                macro_kick_shape
            );

        transient_shape_cutoff_current =
            MacroKickShapeTransientCutoff(
                macro_kick_shape
            );

        transient_handoff_start_ms_current =
            MacroKickShapeHandoffStartMs(
                macro_kick_shape
            );

        transient_handoff_end_ms_current =
            MacroKickShapeHandoffEndMs(
                macro_kick_shape
            );
    }
    else
    {
        /*
         * Previous-version values.
         */
        transient_shape_gain_current = 1.0f;

        transient_shape_drive_current =
            1.05f +
            DEFAULT_TRANSIENT_CHARACTER *
            1.45f;

        transient_shape_cutoff_current = 12000.0f;

        transient_handoff_start_ms_current = 7.0f;
        transient_handoff_end_ms_current = 40.0f;
    }


    /*
     * DO NOT zero live filter memories on an overlapping ratchet.
     * Let the existing state continue from the exact previous sample.
     */
    if(ResetDspStateForThisTrigger())
    {
        transient_tone_lp_state = 0.0f;
        transient_hf_guard_state_1 = 0.0f;
        transient_hf_guard_state_2 = 0.0f;

        final_kick_hf_state_1 = 0.0f;
        final_kick_hf_state_2 = 0.0f;
        final_kick_hf_state_3 = 0.0f;
    }


    /*
     * Initial/retrigger transient envelope.
     *
     * Ratchets restart the musical attack from the CURRENT gain rather
     * than dropping the previous hit to zero.
     */
    if(
        kick_retrigger_active &&
        !PSY_PHASE_LOCK_EVERY_HIT
    )
    {
        transient_env.RetriggerFromCurrent();
    }
    else
    {
        /*
         * Phase-locked hit gets the same amplitude-envelope start too.
         * The old voice is handled by the ratchet bridge.
         */
        transient_env.Trigger();
    }


    transient_env.attack =
        USE_NEW_SHAPE_TRANSIENT_MACRO
        ? MacroKickShapeAttackSeconds(
              macro_kick_shape
          )
        : 0.0008f;


    /*
     * At Note On the transient is held.
     *
     * Note Off releases it.
     */
    transient_env.holding = true;


    /*
     * Preserve an already-sounding tail across a ratchet.
     *
     * StartTailAudio() below will retime/retrigger it smoothly. A hard
     * value=0 here was another source of broadband ratchet clicks.
     */
    if(ResetDspStateForThisTrigger())
    {
        tail_env.active = false;
        tail_env.value = 0.0f;
    }


    /*
     * MACRO 6 also controls sweep time.
     */
    transient_pitch_time =
        MacroKickShapeSweepSeconds(
            macro_kick_shape
        );


    tail_pitch_time =
        0.08f +
        sep * 0.22f;


    /*
     * Select current character pattern step.
     */
    character_frequency =
        filter_pattern[
            filter_pattern_step
        ];


    filter_pattern_step++;

    if(filter_pattern_step >= 16)
        filter_pattern_step = 0;
}


/* ============================================================
   START TAIL
   ============================================================ */

static void StartTailAudio()
{
    /*
     * Both fresh hits and ratchets use the same continuous mechanism:
     *
     * current level -> smooth 2.5 ms reinforcement toward 1 -> K2 decay
     *
     * No hard 0 or 1 assignment exists anywhere in this path.
     */
    tail_env.AttackThenDecayFromCurrent();


    tail_env.attack =
        0.0025f;


    tail_env.decay =
        MacroDecaySeconds(
            macro_decay
        );


    tail_env.decay_linearity =
        TAIL_DECAY_LINEARITY;


    /*
     * Tail pitch gesture restarts, but oscillator PHASE is preserved
     * during an overlapping ratchet.
     */
    tail_pitch_phase =
        1.0f;
}


/* ============================================================
   RELEASE TRANSIENT
   ============================================================ */

static void ReleaseKickAudio()
{
    /*
     * NOTE OFF IS NO LONGER AN AMPLITUDE EVENT.
     *
     * The transient already hands smoothly into the tail over the fixed
     * equal-power 7..40 ms anatomy window.
     *
     * K2 / CC40 exclusively owns amplitude decay.
     *
     * Note-Off is now reserved for measuring gate length and arming the
     * tail pitch sweep. Keeping this function as a no-op preserves the
     * existing event structure without allowing gate time to fight K2.
     */
}


/* ============================================================
   SIMPLE ONE-POLE FILTER
   ============================================================ */

struct OnePole
{
    float z = 0.0f;


    float Process(float input,
                  float cutoff)
    {
        if(cutoff < 10.0f)
            cutoff = 10.0f;


        if(cutoff > SAMPLE_RATE * 0.45f)
            cutoff = SAMPLE_RATE * 0.45f;


        float a =
            expf(
                -6.2831853f *
                cutoff /
                SAMPLE_RATE
            );


        z =
            (1.0f - a) * input +
            a * z;


        return z;
    }


    void Reset()
    {
        z = 0.0f;
    }
};


/*
 * Character high-pass state.
 */
static OnePole character_lowpass;


/*
 * Sub low-pass.
 */
static OnePole sub_lowpass;


/*
 * Character post filter.
 */
static OnePole character_postfilter;


/* ============================================================
   HIGH PASS
   ============================================================ */

static float HighPass(
    float input,
    float cutoff,
    float& state)
{
    if(cutoff < 20.0f)
        cutoff = 20.0f;


    float a =
        expf(
            -6.2831853f *
            cutoff /
            SAMPLE_RATE
        );


    state =
        (1.0f - a) *
        input +
        a * state;


    return input - state;
}


static inline float HighPassFixedPole(
    float input,
    float pole_a,
    float& state)
{
    state =
        (1.0f - pole_a) *
        input +
        pole_a * state;


    return input - state;
}


/* ============================================================
   CHARACTER BANDPASS
   ============================================================ */

struct Biquad
{
    float b0 = 1.0f;
    float b1 = 0.0f;
    float b2 = 0.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;

    float z1 = 0.0f;
    float z2 = 0.0f;


    void Reset()
    {
        z1 = 0.0f;
        z2 = 0.0f;
    }


    float Process(float x)
    {
        float y =
            b0 * x +
            z1;


        z1 =
            b1 * x -
            a1 * y +
            z2;


        z2 =
            b2 * x -
            a2 * y;


        return y;
    }


    void SetBandpass(float frequency,
                     float q)
    {
        if(frequency < 50.0f)
            frequency = 50.0f;


        if(frequency > 10000.0f)
            frequency = 10000.0f;


        if(q < 0.3f)
            q = 0.3f;


        if(q > 8.0f)
            q = 8.0f;


        float w0 =
            6.2831853f *
            frequency /
            SAMPLE_RATE;


        float c =
            cosf(w0);

        float s =
            sinf(w0);


        float alpha =
            s /
            (2.0f * q);


        float a0 =
            1.0f + alpha;


        b0 =
            alpha / a0;

        b1 =
            0.0f;

        b2 =
            -alpha / a0;

        a1 =
            -2.0f * c / a0;

        a2 =
            (1.0f - alpha) / a0;
    }
};


static Biquad character_bandpass;


/* ============================================================
   SOFT SATURATION
   ============================================================ */

static inline float SoftClip(float x)
{
    /*
     * Fast, smooth saturation.
     */
    return x /
           (1.0f + fabsf(x));
}


/*
 * KICK COMPRESSOR — one dial (CC58).
 *
 * Tuned for kicks rather than general programme: fast enough to catch the
 * attack, with a release in the 120 ms region so it recovers before the
 * next hit at club tempo instead of pumping across the bar. One control
 * moves threshold and ratio together, which is the only sane way to put a
 * compressor on a single knob.
 */
struct KickCompressor
{
    float envelope = 0.0f;
    float gain = 1.0f;

    void Reset()
    {
        envelope = 0.0f;
        gain = 1.0f;
    }

    float Process(float input, float amount)
    {
        if(amount <= 0.001f)
        {
            envelope = 0.0f;
            gain = 1.0f;
            return input;
        }

        /* Harder as the dial rises: 0.85 down to 0.18, 1:1 up to 8:1. */
        float threshold = 0.85f - amount * 0.67f;
        float ratio = 1.0f + amount * 7.0f;

        float detector = fabsf(input);

        /* 2 ms attack, ~120 ms release at 48 kHz. */
        float alpha = detector > envelope ? 0.010362f : 0.000173f;
        envelope += (detector - envelope) * alpha;

        float target = 1.0f;
        if(envelope > threshold && envelope > 0.0001f)
        {
            float compressed =
                threshold + (envelope - threshold) / ratio;
            target = compressed / envelope;
        }

        /* Smooth the gain itself so fast material cannot modulate it. */
        gain += (target - gain) * (target < gain ? 0.010362f : 0.000173f);

        /* Make-up keeps the dial from simply turning the kick down. */
        float makeup = 1.0f + amount * 0.85f;

        return input * gain * makeup;
    }
};

static KickCompressor kick_compressor;


/*
 * Brickwall-ish limiter (CC59). Deliberately simple: a fast-attack gain
 * reduction onto a fixed 0.95 ceiling, sitting before the output ceiling
 * so it catches peaks rather than letting them saturate.
 */
struct KickLimiter
{
    float gain = 1.0f;

    void Reset() { gain = 1.0f; }

    float Process(float input, bool enabled)
    {
        if(!enabled)
        {
            gain = 1.0f;
            return input;
        }

        constexpr float ceiling = 0.95f;

        float magnitude = fabsf(input) * gain;
        float target = 1.0f;
        if(magnitude > ceiling)
            target = gain * (ceiling / magnitude);

        /* Instant clamp down, 50 ms recovery. */
        gain += (target - gain) * (target < gain ? 1.0f : 0.000416f);

        if(gain > 1.0f)
            gain = 1.0f;

        return input * gain;
    }
};

static KickLimiter kick_limiter;


/*
 * Final output ceiling.
 *
 * Unity below the knee, then saturates smoothly and is hard-bounded by
 * OUTPUT_CEILING_LIMIT, so it can never exceed the DAC range. Replaces a bare
 * clamp: with END_OF_CHAIN_GAIN raised the peaks now reach the ceiling, and a
 * clamp would shatter a bass-heavy kick into hard digital clipping. Gain is
 * untouched below the knee, so this is not a compressor on the whole signal.
 */
static inline float OutputCeiling(float x)
{
    constexpr float knee  = 0.80f;
    constexpr float limit = 0.995f;
    constexpr float range = limit - knee;

    float magnitude = fabsf(x);
    if(magnitude <= knee)
        return x;

    float over = magnitude - knee;
    float shaped = knee + range * (over / (over + range));

    return x < 0.0f ? -shaped : shaped;
}


/* ============================================================
   COMPRESSOR
   ============================================================ */

struct SimpleCompressor
{
    float envelope = 0.0f;


    float Process(float input)
    {
        float x =
            fabsf(input);


        /*
         * Fast attack / slower release.
         */
        if(x > envelope)
        {
            envelope +=
                (x - envelope) *
                0.18f;
        }
        else
        {
            envelope +=
                (x - envelope) *
                0.0025f;
        }


        /*
         * Soft 3:1 compression above threshold.
         */
        const float threshold =
            0.62f;


        float gain = 1.0f;


        if(envelope > threshold)
        {
            float over =
                envelope -
                threshold;


            float compressed =
                over /
                3.0f;


            float target =
                threshold +
                compressed;


            if(envelope > 0.0001f)
            {
                gain =
                    target /
                    envelope;
            }
        }


        return input * gain;
    }
};


static SimpleCompressor compressor;


/* ============================================================
   CHARACTER / DIRTY BUS LEVEL MANAGER
   ============================================================ */

struct CharacterDirtyBusManager
{
    float envelope = 0.0f;
    float gain = 1.0f;


    void Reset()
    {
        envelope = 0.0f;
        gain = 1.0f;
    }


    float Process(
        float input,
        float character_amount)
    {
        character_amount =
            Clamp01Added(
                character_amount
            );


        float strength =
            SmoothstepAdded(
                Clamp01Added(
                    (
                        character_amount -
                        DIRTY_MANAGER_START_AMOUNT
                    )
                    /
                    (
                        1.0f -
                        DIRTY_MANAGER_START_AMOUNT
                    )
                )
            );


        /*
         * Slow enough not to reshape individual 5-15 kHz waveform
         * cycles, fast enough to manage the kick body.
         */
        float detector =
            fabsf(
                input
            );


        float detector_alpha =
            detector > envelope
            ? 0.00593470f   /* 3.5 ms */
            : 0.000277739f; /* 75 ms */


        envelope +=
            (
                detector -
                envelope
            )
            *
            detector_alpha;


        float threshold =
            DIRTY_MANAGER_MAX_THRESHOLD +
            (
                DIRTY_MANAGER_MIN_THRESHOLD -
                DIRTY_MANAGER_MAX_THRESHOLD
            )
            *
            strength;


        float ratio =
            1.0f +
            (
                DIRTY_MANAGER_MAX_RATIO -
                1.0f
            )
            *
            strength;


        /*
         * SOFT KNEE.
         *
         * The previous manager had a distinct "envelope > threshold"
         * crossing. The exact knob/decay value where that crossing
         * occurred moved around, matching the reported symptom.
         *
         * Blend progressively into compression over a wide knee instead.
         */
        float knee_half =
            threshold *
            0.30f;


        float knee_start =
            threshold -
            knee_half;


        float knee_end =
            threshold +
            knee_half;


        float hard_gain = 1.0f;


        if(envelope > 0.0001f)
        {
            float over =
                envelope -
                threshold;


            if(over < 0.0f)
                over = 0.0f;


            float compressed_level =
                threshold +
                over /
                ratio;


            hard_gain =
                compressed_level /
                envelope;


            if(hard_gain >
               1.0f)
            {
                hard_gain = 1.0f;
            }


            if(hard_gain <
               DIRTY_MANAGER_MIN_GAIN)
            {
                hard_gain =
                    DIRTY_MANAGER_MIN_GAIN;
            }
        }


        float knee_mix =
            SmoothstepAdded(
                Clamp01Added(
                    (
                        envelope -
                        knee_start
                    )
                    /
                    (
                        knee_end -
                        knee_start +
                        0.00001f
                    )
                )
            );


        float target_gain =
            1.0f +
            (
                hard_gain -
                1.0f
            )
            *
            knee_mix;


        /*
         * Gain itself moves smoothly too.
         */
        float gain_alpha =
            target_gain < gain
            ? 0.00593470f   /* 3.5 ms */
            : 0.000277739f; /* 75 ms */


        gain +=
            (
                target_gain -
                gain
            )
            *
            gain_alpha;


        /*
         * Gentle amount-dependent static trim.
         * Linear only — no clipper/limiter in this branch.
         */
        float static_trim =
            1.0f +
            (
                DIRTY_MANAGER_MAX_STATIC_TRIM -
                1.0f
            )
            *
            strength;


        return
            input *
            gain *
            static_trim;
    }
};


static CharacterDirtyBusManager character_dirty_bus_manager;


/* ============================================================
   FINAL LIMITER
   ============================================================ */

static inline float FinalLimiter(float x)
{
    /*
     * Normally almost invisible.
     *
     * The parameter/gain compensation should do most of the
     * level management.
     */
    if(x > 0.92f)
    {
        float excess =
            x - 0.92f;


        x =
            0.92f +
            excess /
            (1.0f + excess * 8.0f);
    }


    if(x < -0.92f)
    {
        float excess =
            -x - 0.92f;


        x =
            -0.92f -
            excess /
            (1.0f + excess * 8.0f);
    }


    return x;
}




/* ============================================================
   MASTER KICK OUTPUT DE-CLICK / SAFETY ENVELOPE
   ============================================================ */

struct AddedKickMasterEnvelope
{
    enum class State
    {
        OFF,
        ATTACK,
        HOLD,
        RELEASE
    };


    State state =
        State::OFF;

    float value = 0.0f;

    float release_start = 0.0f;
    uint32_t release_age = 0;
    uint32_t release_samples = 1;


    void Reset()
    {
        state =
            State::OFF;

        value = 0.0f;

        release_start = 0.0f;
        release_age = 0;
        release_samples = 1;
    }


    void Trigger()
    {
        if(!ENABLE_KICK_MASTER_GATE_ENVELOPE)
        {
            state =
                State::HOLD;

            value = 1.0f;

            return;
        }


        /*
         * Retriggers are smooth: start the fast attack from the current
         * value instead of forcing an instantaneous jump.
         */
        state =
            State::ATTACK;
    }


    /*
     * Legacy/manual release helper.
     *
     * The current instrument architecture does NOT call this from
     * Note-Off. K2/tail_env owns musical decay.
     */
    void Release()
    {
        if(!ENABLE_KICK_MASTER_GATE_ENVELOPE)
            return;


        if(state ==
           State::OFF)
        {
            return;
        }


        /*
         * MACRO 2 at the top becomes a genuine sustained tail.
         * Note-Off does not close the master envelope there.
         */
        if(MacroDecayInfinite(
               macro_decay
           ))
        {
            /*
             * Infinite sustain must not create an amplitude jump.
             *
             * If Note-Off arrives during the short attack, simply allow
             * that existing smooth attack to finish into HOLD. If we are
             * already holding, remain there.
             */
            if(state == State::HOLD)
                return;

            if(state == State::ATTACK)
                return;

            /*
             * A running finite release is never resurrected by merely
             * moving K2 to INF after the note has already been released.
             */
            return;
        }


        release_start =
            value;


        release_age =
            0;


        float samples =
            MacroMasterReleaseMs(
                macro_decay
            ) *
            SAMPLE_RATE /
            1000.0f;


        if(samples < 1.0f)
            samples = 1.0f;


        release_samples =
            static_cast<uint32_t>(
                samples
            );


        if(release_samples < 1)
            release_samples = 1;


        state =
            State::RELEASE;
    }


    void OnDecayMacroChanged()
    {
        /*
         * AUDIO THREAD ONLY.
         *
         * A parameter move may change the remainder of an existing
         * finite release, but it may NEVER resurrect an already-released
         * note by jumping from zero/release back to HOLD.
         */
        if(note_gate)
            return;


        if(MacroDecayInfinite(
               macro_decay
           ))
        {
            /*
             * Moving the knob to INF after Note-Off does NOT bring a note
             * back from the dead. Infinite sustain is latched only by the
             * actual Note-Off path while the note is sounding.
             */
            return;
        }


        if(state == State::HOLD ||
           state == State::ATTACK ||
           state == State::RELEASE)
        {
            /*
             * Release() starts from the CURRENT value, so changing K2
             * cannot create an amplitude discontinuity.
             */
            Release();
        }
    }


    void EnsureReleasedIfGateOff()
    {
        if(!ENABLE_KICK_MASTER_GATE_ENVELOPE)
            return;


        if(note_gate)
            return;


        if(MacroDecayInfinite(
               macro_decay
           ))
        {
            /*
             * This is the ONE intentional infinite state.
             */
            return;
        }


        /*
         * No other macro is allowed to leave the master envelope held.
         * Do not restart an already-running release.
         */
        if(state == State::HOLD ||
           state == State::ATTACK)
        {
            Release();
        }
    }


    float Process()
    {
        if(!ENABLE_KICK_MASTER_GATE_ENVELOPE)
            return 1.0f;


        switch(state)
        {
            case State::OFF:
            {
                value = 0.0f;

                return value;
            }


            case State::ATTACK:
            {
                /*
                 * Short exponential de-click attack.
                 *
                 * This is intentionally much shorter than any musical
                 * envelope movement.
                 */
                float attack_ms =
                    kick_master_attack_ms;


                if(attack_ms < 0.10f)
                    attack_ms = 0.10f;


                float a =
                    expf(
                        -5.0f /
                        (
                            SAMPLE_RATE *
                            attack_ms /
                            1000.0f
                        )
                    );


                value =
                    1.0f +
                    (
                        value -
                        1.0f
                    ) *
                    a;


                if(value >= 0.9995f)
                {
                    value = 1.0f;

                    state =
                        State::HOLD;
                }


                return value;
            }


            case State::HOLD:
            {
                value = 1.0f;

                return value;
            }


            case State::RELEASE:
            {
                /*
                 * Finite smoothstep release.
                 *
                 * Unlike a pure exponential tail, this reaches EXACTLY
                 * zero after the requested release time.
                 */
                float t =
                    static_cast<float>(
                        release_age
                    )
                    /
                    static_cast<float>(
                        release_samples
                    );


                t =
                    Clamp01Added(
                        t
                    );


                float smooth =
                    SmoothstepAdded(
                        t
                    );


                value =
                    release_start *
                    (
                        1.0f -
                        smooth
                    );


                release_age++;


                if(release_age >=
                   release_samples)
                {
                    value = 0.0f;

                    state =
                        State::OFF;
                }


                return value;
            }
        }


        return 0.0f;
    }
};


static AddedKickMasterEnvelope added_kick_master_envelope;


static bool PerformanceMasterFilterActive()
{
    /*
     * Generated-kick policy: HPF is the only kick-side DJ filter.
     * LPF remains on the external-input bus only.
     */
    return
        PERF_DJ_HPF_ENABLED &&
        macro_fx_value_hpf > 0.005f;
}


/*
 * KICK-ONLY HPF ONSET GUARD.
 *
 * The HPF itself runs from sample one so its internal state follows the
 * waveform continuously. Only its AUDIBLE contribution is held dry for a
 * few milliseconds, then crossfaded in. This prevents the differentiating
 * high-pass response from turning the kick's rising edge into a brittle
 * digital click. External-input HPF does not use this kick-onset guard.
 */
static float PerformanceFilterKickOnsetBlend()
{
    float age_ms =
        static_cast<float>(
            kick_age_samples
        )
        *
        1000.0f /
        SAMPLE_RATE;


    if(age_ms <=
       PERFORMANCE_FILTER_DRY_HOLD_MS)
    {
        return 0.0f;
    }


    float t =
        (
            age_ms -
            PERFORMANCE_FILTER_DRY_HOLD_MS
        )
        /
        PERFORMANCE_FILTER_FADE_IN_MS;


    return
        SmoothstepAdded(
            Clamp01Added(
                t
            )
        );
}


static float ProcessAbsoluteFinalLpf(
    float input)
{
    float amount =
        Clamp01Added(
            macro_fx_value_lpf
        );


    if(amount <= FINAL_LPF_ENFORCE_BEGIN)
    {
        final_lpf_enforce_state_1 = input;
        final_lpf_enforce_state_2 = input;
        final_lpf_enforce_state_3 = input;
        final_lpf_enforce_state_4 = input;

        return input;
    }


    /*
     * Same logarithmic mapping as the DJ LPF.
     * At amount=1.0, cutoff is exactly 120 Hz.
     */
    float cutoff =
        18000.0f *
        powf(
            120.0f /
            18000.0f,
            amount
        );


    float a =
        1.0f -
        expf(
            -TWO_PI *
            cutoff /
            SAMPLE_RATE
        );


    final_lpf_enforce_state_1 +=
        a *
        (
            input -
            final_lpf_enforce_state_1
        );


    final_lpf_enforce_state_2 +=
        a *
        (
            final_lpf_enforce_state_1 -
            final_lpf_enforce_state_2
        );


    final_lpf_enforce_state_3 +=
        a *
        (
            final_lpf_enforce_state_2 -
            final_lpf_enforce_state_3
        );


    final_lpf_enforce_state_4 +=
        a *
        (
            final_lpf_enforce_state_3 -
            final_lpf_enforce_state_4
        );


    float strength =
        SmoothstepAdded(
            Clamp01Added(
                (
                    amount -
                    FINAL_LPF_ENFORCE_BEGIN
                )
                /
                (
                    1.0f -
                    FINAL_LPF_ENFORCE_BEGIN
                )
            )
        );


    return
        input +
        (
            final_lpf_enforce_state_4 -
            input
        )
        *
        strength;
}


static float ProcessPerformanceFilterHeadroom(
    float input)
{
    float target =
        PerformanceMasterFilterActive()
        ? PERFORMANCE_FILTER_ACTIVE_HEADROOM_GAIN
        : 1.0f;


    /*
     * 15 ms smoothing coefficient at 48 kHz, precomputed. This function
     * runs for every kick sample even when the HPF is bypassed.
     */
    constexpr float a = 0.99307961f;


    performance_filter_headroom_gain =
        target +
        (
            performance_filter_headroom_gain -
            target
        )
        *
        a;


    return
        input *
        performance_filter_headroom_gain;
}


static float ProcessPostPerformanceFilterHfGuard(
    float input)
{
    bool active =
        ENABLE_POST_PERFORMANCE_FILTER_HF_GUARD &&
        PerformanceMasterFilterActive();


    /*
     * When bypassed, make the states FOLLOW the current signal.
     *
     * Therefore enabling the guard never starts from stale/zero state.
     */
    if(!active &&
       post_perf_hf_mix <= 0.000001f)
    {
        post_perf_hf_state_1 = input;
        post_perf_hf_state_2 = input;

        post_perf_hf_mix = 0.0f;

        return input;
    }


    float age_ms =
        static_cast<float>(
            kick_age_samples
        )
        *
        1000.0f /
        SAMPLE_RATE;


    float open_t =
        SmoothstepAdded(
            Clamp01Added(
                age_ms /
                POST_PERF_FILTER_HF_OPEN_MS
            )
        );


    float cutoff =
        POST_PERF_FILTER_HF_INITIAL_HZ +
        (
            POST_PERF_FILTER_HF_SETTLED_HZ -
            POST_PERF_FILTER_HF_INITIAL_HZ
        )
        *
        open_t;


    float a =
        expf(
            -TWO_PI *
            cutoff /
            SAMPLE_RATE
        );


    post_perf_hf_state_1 =
        (
            1.0f -
            a
        )
        *
        input
        +
        a *
        post_perf_hf_state_1;


    post_perf_hf_state_2 =
        (
            1.0f -
            a
        )
        *
        post_perf_hf_state_1
        +
        a *
        post_perf_hf_state_2;


    float target_mix =
        active
        ? 1.0f
        : 0.0f;


    float mix_samples =
        SAMPLE_RATE *
        POST_PERF_FILTER_HF_MIX_MS /
        1000.0f;


    if(mix_samples < 1.0f)
        mix_samples = 1.0f;


    float mix_a =
        expf(
            -5.0f /
            mix_samples
        );


    post_perf_hf_mix =
        target_mix +
        (
            post_perf_hf_mix -
            target_mix
        )
        *
        mix_a;


    if(post_perf_hf_mix < 0.000001f)
        post_perf_hf_mix = 0.0f;


    if(post_perf_hf_mix > 0.999999f)
        post_perf_hf_mix = 1.0f;


    return
        input +
        (
            post_perf_hf_state_2 -
            input
        )
        *
        post_perf_hf_mix;
}


/* ============================================================
   ADDED SMOOTH WET / DRY
   ============================================================ */

struct AddedSmoothWet
{
    float wet = 0.0f;


    void Reset()
    {
        wet = 0.0f;
    }


    float Process(
        float dry,
        float effected,
        bool enabled,
        float fade_ms)
    {
        float target =
            enabled
            ? 1.0f
            : 0.0f;

        float samples =
            SAMPLE_RATE *
            fade_ms /
            1000.0f;

        if(samples < 1.0f)
            samples = 1.0f;

        float a =
            expf(
                -5.0f /
                samples
            );

        wet =
            target +
            (
                wet -
                target
            ) *
            a;

        if(wet < 0.000001f)
            wet = 0.0f;

        if(wet > 0.999999f)
            wet = 1.0f;

        return
            dry +
            (
                effected -
                dry
            ) *
            wet;
    }
};


/* ============================================================
   EXTERNAL-INPUT PUMP
   ============================================================ */

struct AddedPump
{
    float gain = 1.0f;

    bool attacking = false;
    bool holding = false;

    uint32_t attack_age = 0;
    uint32_t hold_age = 0;
    uint32_t hold_samples = 0;

    /*
     * Used only to prevent a MIDI-clock ghost trigger immediately after
     * a real kick trigger at essentially the same beat boundary.
     */
    uint32_t samples_since_trigger = 0xFFFFFFFFu;


    void Reset()
    {
        gain = 1.0f;

        attacking = false;
        holding = false;

        attack_age = 0;
        hold_age = 0;
        hold_samples = 0;

        samples_since_trigger = 0xFFFFFFFFu;
    }


    bool RecentlyTriggered(float window_ms) const
    {
        uint32_t window_samples =
            static_cast<uint32_t>(
                SAMPLE_RATE *
                window_ms /
                1000.0f
            );


        return
            samples_since_trigger <
            window_samples;
    }


    void TriggerWithSweepMs(float sweep_ms)
    {
        if(!macro_pump_enabled ||
           macro_fx_value_pump <= 0.005f)
        {
            return;
        }


        /*
         * The pump now treats the kick pitch-sweep duration as the
         * sidechain-control "kick presence" window.
         *
         * Real kick:
         *     uses the actual current K6 shape sweep time.
         *
         * Ghost kick:
         *     uses the SAME K6-derived dummy sweep time.
         */
        sweep_ms =
            ClampAdded(
                sweep_ms,
                30.0f,
                220.0f
            );


        hold_samples =
            static_cast<uint32_t>(
                sweep_ms *
                SAMPLE_RATE /
                1000.0f
            );


        attacking = true;
        holding = false;

        attack_age = 0;
        hold_age = 0;

        samples_since_trigger = 0;
    }


    void Trigger()
    {
        /*
         * Compatibility wrapper for any old caller.
         */
        TriggerWithSweepMs(
            MacroKickShapeSweepSeconds(
                macro_kick_shape
            )
            *
            1000.0f
        );
    }


    float Process(
        float input,
        float quarter_note_ms)
    {
        if(samples_since_trigger != 0xFFFFFFFFu)
        {
            if(samples_since_trigger < 0xFFFFFFFEu)
                samples_since_trigger++;
        }


        float macro =
            Clamp01Added(
                macro_fx_value_pump
            );


        if(macro <= 0.005f ||
           !macro_pump_enabled)
        {
            attacking = false;
            holding = false;

            float bypass_a =
                expf(
                    -5.0f /
                    (
                        SAMPLE_RATE *
                        0.008f
                    )
                );


            gain =
                1.0f +
                (
                    gain -
                    1.0f
                ) *
                bypass_a;


            if(fabsf(gain - 1.0f) < 0.00001f)
                gain = 1.0f;


            return
                input *
                gain;
        }


        float depth =
            0.22f +
            0.74f *
            powf(
                macro,
                0.82f
            );


        float minimum_gain =
            1.0f -
            depth;


        if(attacking)
        {
            /*
             * Smooth duck onset.
             */
            float attack_ms =
                2.5f +
                macro *
                1.5f;


            float a =
                expf(
                    -5.0f /
                    (
                        SAMPLE_RATE *
                        attack_ms /
                        1000.0f
                    )
                );


            gain =
                minimum_gain +
                (
                    gain -
                    minimum_gain
                ) *
                a;


            attack_age++;


            uint32_t attack_samples =
                static_cast<uint32_t>(
                    SAMPLE_RATE *
                    (
                        attack_ms +
                        0.8f
                    )
                    /
                    1000.0f
                );


            if(attack_age >= attack_samples)
            {
                attacking = false;
                holding = true;
                hold_age = 0;
            }
        }
        else if(holding)
        {
            /*
             * Hold the duck through the dummy/real pitch-sweep window.
             *
             * This is what makes ghost pumping feel like there is an
             * inaudible kick occupying the transient region.
             */
            float hold_a =
                expf(
                    -5.0f /
                    (
                        SAMPLE_RATE *
                        0.004f
                    )
                );


            gain =
                minimum_gain +
                (
                    gain -
                    minimum_gain
                ) *
                hold_a;


            hold_age++;


            if(hold_age >= hold_samples)
            {
                holding = false;
            }
        }
        else
        {
            /*
             * Musical recovery after the transient/sweep window.
             */
            float release_fraction =
                0.10f +
                0.58f *
                powf(
                    macro,
                    1.22f
                );


            float release_ms =
                quarter_note_ms *
                release_fraction;


            if(release_ms < 32.0f)
                release_ms = 32.0f;


            float a =
                expf(
                    -5.0f /
                    (
                        SAMPLE_RATE *
                        release_ms /
                        1000.0f
                    )
                );


            gain =
                1.0f +
                (
                    gain -
                    1.0f
                ) *
                a;
        }


        return
            input *
            gain;
    }
};

/* ============================================================
   CLOCKED DOTTED-QUARTER DELAY
   EXTERNAL INPUT ONLY
   ============================================================ */

struct AddedClockedDelay
{
    static constexpr uint32_t MAX_SAMPLES = 40800;

    float buffer[MAX_SAMPLES];

    uint32_t write = 0;
    uint32_t valid_written = 0;

    float macro_smoothed = 0.0f;

    AddedSmoothWet wet;


    void Reset()
    {
        for(uint32_t i = 0;
            i < MAX_SAMPLES;
            i++)
        {
            buffer[i] = 0.0f;
        }

        write = 0;
        valid_written = 0;
        macro_smoothed = 0.0f;

        wet.Reset();
    }


    float Process(
        float input,
        float quarter_note_ms)
    {
        bool requested =
            PERF_CLOCKED_DELAY_ENABLED &&
            macro_fx_value_delay > 0.005f;


        float macro_target =
            requested
            ? Clamp01Added(
                  macro_fx_value_delay
              )
            : 0.0f;


        /*
         * Smooth every wet/feedback parameter move.
         */
        float macro_a =
            expf(
                -5.0f /
                (
                    SAMPLE_RATE *
                    0.015f
                )
            );


        macro_smoothed =
            macro_target +
            (
                macro_smoothed -
                macro_target
            ) *
            macro_a;


        /*
         * Once the wet crossfade is completely dry we can invalidate the
         * old delay history without any audible discontinuity.
         */
        if(!requested &&
           wet.wet <= 0.000001f &&
           macro_smoothed <= 0.0001f)
        {
            valid_written = 0;
            macro_smoothed = 0.0f;
            return input;
        }


        float delay_ms =
            quarter_note_ms *
            1.5f;


        delay_ms =
            ClampAdded(
                delay_ms,
                20.0f,
                840.0f
            );


        uint32_t delay_samples =
            static_cast<uint32_t>(
                delay_ms *
                SAMPLE_RATE /
                1000.0f
            );


        if(delay_samples >= MAX_SAMPLES)
            delay_samples = MAX_SAMPLES - 1;


        uint32_t read =
            (
                write +
                MAX_SAMPLES -
                delay_samples
            )
            %
            MAX_SAMPLES;


        float delayed =
            valid_written >= delay_samples
            ? buffer[read]
            : 0.0f;


        float m =
            Clamp01Added(
                macro_smoothed
            );


        float feedback =
            0.18f +
            0.60f *
            powf(
                m,
                1.25f
            );


        float user_wet =
            0.10f +
            0.66f *
            SmoothstepAdded(
                m
            );


        buffer[write] =
            input +
            SoftClip(
                delayed *
                feedback
            );


        if(valid_written < MAX_SAMPLES)
            valid_written++;


        write++;

        if(write >= MAX_SAMPLES)
            write = 0;


        float effected =
            input +
            delayed *
            user_wet;


        return
            wet.Process(
                input,
                effected,
                requested,
                20.0f
            );
    }
};

constexpr uint32_t AddedClockedDelay::MAX_SAMPLES;


/* ============================================================
   K1 CLOCKED CHOP + CLICK-SAFE LOOPER USER CONTROLS
   ============================================================ */

/*
 * STUTTER has been replaced by a LIVE freetekno-style clocked chopper.
 *
 * It does NOT sample or repeat audio.
 *
 * The selected clock division defines the length of one four-step
 * transformer pattern:
 *
 *     STEP 1  full
 *     STEP 2  deep cut
 *     STEP 3  medium accent
 *     STEP 4  deep cut
 *
 * Adjacent levels are joined with a smooth raised transition, including
 * STEP 4 -> STEP 1 at the pattern wrap, so there are no hard gates.
 */
static constexpr float CLOCKED_CHOP_LEVEL_1 = 1.00f;
static constexpr float CLOCKED_CHOP_LEVEL_2 = 0.08f;
static constexpr float CLOCKED_CHOP_LEVEL_3 = 0.68f;
static constexpr float CLOCKED_CHOP_LEVEL_4 = 0.14f;

/*
 * Fraction of the END of each quarter-pattern step used to transition
 * smoothly into the next level.
 */
static constexpr float CLOCKED_CHOP_EDGE_FRACTION = 0.26f;

/*
 * Changing rate never resets phase. Instead the phase speed glides to
 * the new clock rate over this time.
 */
static constexpr float CLOCKED_CHOP_RATE_SMOOTH_MS = 10.0f;

static constexpr float CLOCKED_CHOP_ENGAGE_MS = 10.0f;
static constexpr float CLOCKED_CHOP_RELEASE_MS = 12.0f;


/*
 * LOOPER seam:
 *
 * The looper captures a short PRE-ROLL before the nominal loop start.
 * The final part of the loop crossfades into that pre-roll. After the
 * crossfade the next sample is the actual loop-start sample, which is
 * adjacent in the original recording.
 *
 * This avoids the classic arbitrary END -> START sample jump.
 */
static constexpr float LOOPER_SEAM_MAX_MS = 18.0f;
static constexpr float LOOPER_RATE_CHANGE_XFADE_MS = 20.0f;
static constexpr float LOOPER_ENGAGE_MS = 24.0f;
static constexpr float LOOPER_RELEASE_MS = 20.0f;


/*
 * Search backwards around the nominal capture position for a seam whose
 * beginning/end values and slopes agree better.
 *
 * This does NOT alter the requested clock length; it shifts the entire
 * capture window in time by at most a few milliseconds.
 */
static constexpr uint32_t LOOPER_SEAM_SEARCH_SAMPLES = 160;

/*
 * Seam-local HF guard catches derivative mismatch that a value-continuous
 * crossfade can still turn into a tiny click.
 */
static constexpr float LOOPER_SEAM_HF_HZ = 5200.0f;


/* ============================================================
   FREETEKNO CLOCKED CHOP — KICK-QUANTIZED
   MASTER MIX

   Internal class name remains AddedStutter so CC30 / existing controller
   protocol stays compatible. It no longer contains a sampler.
   ============================================================ */

struct AddedStutter
{
    bool active = false;
    bool desired_enabled = false;

    uint8_t latched_rate_index = 3;

    /*
     * Continuous pattern phase.
     * NEVER reset on a rate change.
     */
    float phase = 0.0f;

    float period_samples_current = 1.0f;
    float period_samples_target = 1.0f;

    float engage_mix = 0.0f;


    void Reset()
    {
        active = false;
        desired_enabled = false;

        latched_rate_index = 3;

        phase = 0.0f;

        period_samples_current = 1.0f;
        period_samples_target = 1.0f;

        engage_mix = 0.0f;
    }


    void Request()
    {
        desired_enabled = true;
    }


    void Stop()
    {
        desired_enabled = false;
    }


    uint32_t RateLengthSamples(
        float quarter_note_ms,
        uint8_t rate_index) const
    {
        /*
         * Preserve the existing CC30 division ladder:
         *
         * 1/2, 1/4, 1/8, 1/16, 1/32 (1/64 and 1/128 CC codes clamp to 1/32)
         */
        static const float fractions[7] =
        {
            2.0f,
            1.0f,
            0.50f,
            0.25f,
            0.125f,
            0.0625f,
            0.03125f
        };


        /*
         * Teensy compatibility:
         *
         * Continue accepting legacy 1/64 and 1/128 rate codes, but clamp
         * both internally to the fastest allowed CHOP rate: 1/32.
         *
         * The Teensy OLED may still display the sent legacy value; MIDI
         * protocol compatibility is intentionally unchanged.
         */
        if(rate_index > 4)
            rate_index = 4;


        uint32_t samples =
            static_cast<uint32_t>(
                quarter_note_ms *
                fractions[
                    rate_index
                ] *
                SAMPLE_RATE /
                1000.0f
            );


        if(samples < 32)
            samples = 32;


        return samples;
    }


    void SetRate(
        uint8_t rate_index,
        float quarter_note_ms,
        bool immediate)
    {
        latched_rate_index =
            rate_index;


        period_samples_target =
            static_cast<float>(
                RateLengthSamples(
                    quarter_note_ms,
                    latched_rate_index
                )
            );


        if(
            immediate ||
            period_samples_current < 2.0f
        )
        {
            period_samples_current =
                period_samples_target;
        }
    }


    void ApplyKickQuantizedCommand(
        uint32_t command,
        float quarter_note_ms,
        uint32_t history_write)
    {
        /*
         * No history is used anymore — this is a LIVE processor.
         */
        (void)history_write;


        uint8_t action =
            static_cast<uint8_t>(
                command &
                0xFFu
            );


        uint8_t rate_index =
            static_cast<uint8_t>(
                (
                    command >>
                    8
                ) &
                0xFFu
            );


        if(action == QUANT_FX_ENABLE)
        {
            if(!active)
            {
                SetRate(
                    rate_index,
                    quarter_note_ms,
                    true
                );


                /*
                 * Since engage_mix begins at zero, starting pattern phase
                 * at zero cannot create an audio discontinuity.
                 */
                phase = 0.0f;

                active = true;
                engage_mix = 0.0f;
            }
            else
            {
                /*
                 * RATE CHANGE:
                 *
                 * Change speed only. Keep pattern phase continuous.
                 */
                SetRate(
                    rate_index,
                    quarter_note_ms,
                    false
                );
            }


            desired_enabled = true;
            PERF_STUTTER_ENABLED = true;
        }
        else if(action == QUANT_FX_RATE_CHANGE)
        {
            if(!active)
            {
                SetRate(
                    rate_index,
                    quarter_note_ms,
                    true
                );

                phase = 0.0f;

                active = true;
                engage_mix = 0.0f;
            }
            else
            {
                SetRate(
                    rate_index,
                    quarter_note_ms,
                    false
                );
            }


            desired_enabled = true;
            PERF_STUTTER_ENABLED = true;
        }
        else if(action == QUANT_FX_DISABLE ||
                action == QUANT_FX_FORCE_DISABLE)
        {
            desired_enabled = false;
            PERF_STUTTER_ENABLED = false;
        }
    }


    void OnSixteenth(float quarter_note_ms)
    {
        (void)quarter_note_ms;
    }


    float PatternLevel(float p) const
    {
        /*
         * Four-step transformer pattern.
         *
         * Each step stays mostly flat, then smoothly approaches the next
         * step near its end. STEP 4 -> STEP 1 is handled identically, so
         * the pattern wrap is continuous.
         */
        static const float levels[4] =
        {
            CLOCKED_CHOP_LEVEL_1,
            CLOCKED_CHOP_LEVEL_2,
            CLOCKED_CHOP_LEVEL_3,
            CLOCKED_CHOP_LEVEL_4
        };


        p -=
            floorf(
                p
            );


        float scaled =
            p *
            4.0f;


        int step =
            static_cast<int>(
                scaled
            );


        if(step < 0)
            step = 0;


        if(step > 3)
            step = 3;


        float local =
            scaled -
            static_cast<float>(
                step
            );


        int next_step =
            (
                step +
                1
            )
            &
            3;


        float current_level =
            levels[
                step
            ];


        float next_level =
            levels[
                next_step
            ];


        float edge_start =
            1.0f -
            CLOCKED_CHOP_EDGE_FRACTION;


        if(local <= edge_start)
        {
            return
                current_level;
        }


        float t =
            (
                local -
                edge_start
            )
            /
            CLOCKED_CHOP_EDGE_FRACTION;


        t =
            SmoothstepAdded(
                Clamp01Added(
                    t
                )
            );


        return
            current_level +
            (
                next_level -
                current_level
            )
            *
            t;
    }


    float Process(
        float input,
        const float* history)
    {
        /*
         * Compatibility with the old stutter call signature only.
         */
        (void)history;


        if(!active)
            return input;


        /*
         * Smooth rate changes instead of jumping the phase accumulator.
         */
        float rate_samples =
            SAMPLE_RATE *
            CLOCKED_CHOP_RATE_SMOOTH_MS /
            1000.0f;


        if(rate_samples < 1.0f)
            rate_samples = 1.0f;


        float rate_alpha =
            1.0f -
            expf(
                -5.0f /
                rate_samples
            );


        period_samples_current +=
            (
                period_samples_target -
                period_samples_current
            )
            *
            rate_alpha;


        if(period_samples_current < 32.0f)
            period_samples_current = 32.0f;


        float chop_gain =
            PatternLevel(
                phase
            );


        phase +=
            1.0f /
            period_samples_current;


        while(phase >= 1.0f)
            phase -= 1.0f;


        /*
         * Smooth effect enable/disable.
         *
         * Because the processor only changes gain on the LIVE signal,
         * there is no unrelated sampled waveform to crossfade against.
         */
        float target =
            desired_enabled
            ? 1.0f
            : 0.0f;


        float fade_ms =
            desired_enabled
            ? CLOCKED_CHOP_ENGAGE_MS
            : CLOCKED_CHOP_RELEASE_MS;


        float fade_samples =
            SAMPLE_RATE *
            fade_ms /
            1000.0f;


        if(fade_samples < 1.0f)
            fade_samples = 1.0f;


        float fade_alpha =
            1.0f -
            expf(
                -5.0f /
                fade_samples
            );


        engage_mix +=
            (
                target -
                engage_mix
            )
            *
            fade_alpha;


        if(engage_mix < 0.000001f)
            engage_mix = 0.0f;


        if(engage_mix > 0.999999f)
            engage_mix = 1.0f;


        float applied_gain =
            1.0f +
            (
                chop_gain -
                1.0f
            )
            *
            engage_mix;


        float output =
            input *
            applied_gain;


        if(
            !desired_enabled &&
            engage_mix <= 0.000001f
        )
        {
            active = false;
        }


        return output;
    }
};



/* ============================================================
   RANDOM-RATE CLOCKED LOOPER — KICK-QUANTIZED
   MASTER MIX
   ============================================================ */

struct AddedQuantizedLooper
{
    /*
     * loop_period:
     *     actual musical period in samples.
     *
     * loop_seam:
     *     pre-roll / overlap samples used only around the loop wrap.
     *
     * capture contains:
     *
     *     [ PRE-ROLL seam ][ nominal loop period ]
     *
     * Playback begins at index = seam.
     *
     * At the end of the nominal period, its final seam samples blend
     * backwards into PRE-ROLL. When that crossfade finishes, playback
     * continues at index=seam — the actual loop start — which is the
     * next adjacent sample after the final pre-roll sample.
     */
    uint32_t loop_start = 0;
    uint32_t loop_period = 0;
    uint32_t loop_seam = 0;
    uint32_t loop_index = 0;

    uint32_t next_loop_start = 0;
    uint32_t next_loop_period = 0;
    uint32_t next_loop_seam = 0;
    uint32_t next_loop_index = 0;

    bool active = false;
    bool desired_enabled = false;
    bool rate_crossfade_active = false;

    uint32_t rate_crossfade_pos = 0;
    uint32_t rate_crossfade_samples = 1;

    uint8_t latched_rate_index = 2;
    uint8_t next_rate_index = 2;

    float engage_mix = 0.0f;

    void Reset()
    {
        loop_start = 0;
        loop_period = 0;
        loop_seam = 0;
        loop_index = 0;

        next_loop_start = 0;
        next_loop_period = 0;
        next_loop_seam = 0;
        next_loop_index = 0;

        active = false;
        desired_enabled = false;
        rate_crossfade_active = false;

        rate_crossfade_pos = 0;
        rate_crossfade_samples = 1;

        latched_rate_index = 2;
        next_rate_index = 2;

        engage_mix = 0.0f;

    }


    void Arm()
    {
        desired_enabled = true;
    }


    void Stop()
    {
        desired_enabled = false;
        rate_crossfade_active = false;
    }


    uint32_t RateLengthSamples(
        float quarter_note_ms,
        uint8_t rate_index) const
    {
        static const float fractions[5] =
        {
            2.0f,
            1.0f,
            0.50f,
            0.25f,
            0.125f
        };


        if(rate_index > 4)
            rate_index = 4;


        uint32_t samples =
            static_cast<uint32_t>(
                quarter_note_ms *
                fractions[
                    rate_index
                ] *
                SAMPLE_RATE /
                1000.0f
            );


        if(samples < 96)
            samples = 96;


        /*
         * Leave enough history room for the maximum pre-roll seam.
         */
        if(samples > 42600u)
            samples = 42600u;


        return samples;
    }


    uint32_t SeamSamples(
        uint32_t period) const
    {
        uint32_t seam =
            static_cast<uint32_t>(
                SAMPLE_RATE *
                LOOPER_SEAM_MAX_MS /
                1000.0f
            );


        /*
         * On very short loops, never let the overlap consume too much of
         * the musical period.
         */
        uint32_t period_limit =
            period /
            4u;


        if(period_limit < 12u)
            period_limit = 12u;


        if(seam > period_limit)
            seam = period_limit;


        if(seam < 12u)
            seam = 12u;


        /*
         * MASTER_HISTORY_SAMPLES = 43200.
         */
        if(period + seam >= 43200u)
        {
            seam =
                43199u -
                period;
        }


        if(seam < 2u)
            seam = 2u;


        return seam;
    }


    uint32_t HistoryAt(
        uint32_t index) const
    {
        return
            index %
            43200u;
    }


    uint32_t FindBestCaptureStart(
        const float* history,
        uint32_t nominal_start,
        uint32_t period,
        uint32_t seam) const
    {
        uint32_t best_start =
            nominal_start;


        float best_score =
            1.0e30f;


        for(uint32_t shift = 0;
            shift <= LOOPER_SEAM_SEARCH_SAMPLES;
            ++shift)
        {
            uint32_t candidate =
                (
                    nominal_start +
                    43200u -
                    shift
                )
                %
                43200u;


            /*
             * Actual loop start is after the pre-roll seam.
             */
            uint32_t start_i =
                HistoryAt(
                    candidate +
                    seam
                );


            uint32_t start_prev_i =
                HistoryAt(
                    candidate +
                    seam +
                    43199u
                );


            uint32_t end_i =
                HistoryAt(
                    candidate +
                    seam +
                    period -
                    1u
                );


            uint32_t end_prev_i =
                HistoryAt(
                    candidate +
                    seam +
                    period -
                    2u
                );


            float start =
                history[
                    start_i
                ];


            float end =
                history[
                    end_i
                ];


            float start_slope =
                start -
                history[
                    start_prev_i
                ];


            float end_slope =
                end -
                history[
                    end_prev_i
                ];


            /*
             * Prefer low-amplitude boundaries AND matching value/slope.
             * This is intentionally cheap because it runs only on a
             * quantized loop/rate event, never per sample.
             */
            float score =
                fabsf(start) *
                0.65f
                +
                fabsf(end) *
                0.65f
                +
                fabsf(
                    start -
                    end
                ) *
                1.20f
                +
                fabsf(
                    start_slope -
                    end_slope
                ) *
                2.40f;


            if(score < best_score)
            {
                best_score = score;
                best_start = candidate;
            }
        }


        return best_start;
    }


    void ConfigureCurrentHead(
        uint8_t rate_index,
        float quarter_note_ms,
        uint32_t history_write,
        const float* history)
    {
        latched_rate_index =
            rate_index;


        loop_period =
            RateLengthSamples(
                quarter_note_ms,
                latched_rate_index
            );


        loop_seam =
            SeamSamples(
                loop_period
            );


        uint32_t capture_length =
            loop_period +
            loop_seam;


        uint32_t nominal_start =
            (
                history_write +
                43200u -
                capture_length
            )
            %
            43200u;


        loop_start =
            FindBestCaptureStart(
                history,
                nominal_start,
                loop_period,
                loop_seam
            );


        /*
         * Skip the pre-roll during normal playback.
         */
        loop_index =
            loop_seam;
    }


    void ConfigureNextHead(
        uint8_t rate_index,
        float quarter_note_ms,
        uint32_t history_write,
        const float* history)
    {
        next_rate_index =
            rate_index;


        next_loop_period =
            RateLengthSamples(
                quarter_note_ms,
                next_rate_index
            );


        next_loop_seam =
            SeamSamples(
                next_loop_period
            );


        uint32_t capture_length =
            next_loop_period +
            next_loop_seam;


        uint32_t nominal_start =
            (
                history_write +
                43200u -
                capture_length
            )
            %
            43200u;


        next_loop_start =
            FindBestCaptureStart(
                history,
                nominal_start,
                next_loop_period,
                next_loop_seam
            );


        next_loop_index =
            next_loop_seam;
    }


    void StartRateCrossfade(
        uint8_t rate_index,
        float quarter_note_ms,
        uint32_t history_write,
        const float* history)
    {
        ConfigureNextHead(
            rate_index,
            quarter_note_ms,
            history_write,
            history
        );


        rate_crossfade_pos = 0;


        rate_crossfade_samples =
            static_cast<uint32_t>(
                SAMPLE_RATE *
                LOOPER_RATE_CHANGE_XFADE_MS /
                1000.0f
            );


        if(rate_crossfade_samples < 32u)
            rate_crossfade_samples = 32u;


        rate_crossfade_active = true;
    }


    void ApplyKickQuantizedCommand(
        uint32_t command,
        float quarter_note_ms,
        uint32_t history_write,
        const float* history)
    {
        uint8_t action =
            static_cast<uint8_t>(
                command &
                0xFFu
            );


        uint8_t rate_index =
            static_cast<uint8_t>(
                (
                    command >>
                    8
                ) &
                0xFFu
            );


        if(action == QUANT_FX_ENABLE)
        {
            if(!active)
            {
                ConfigureCurrentHead(
                    rate_index,
                    quarter_note_ms,
                    history_write,
                    history
                );


                active = true;
                engage_mix = 0.0f;
            }
            else if(rate_index !=
                    latched_rate_index)
            {
                StartRateCrossfade(
                    rate_index,
                    quarter_note_ms,
                    history_write,
                    history
                );
            }


            desired_enabled = true;
            PERF_QUANT_LOOPER_ENABLED = true;
        }
        else if(action == QUANT_FX_RATE_CHANGE)
        {
            if(!active)
            {
                ConfigureCurrentHead(
                    rate_index,
                    quarter_note_ms,
                    history_write,
                    history
                );


                active = true;
                engage_mix = 0.0f;
            }
            else if(rate_index !=
                    latched_rate_index)
            {
                StartRateCrossfade(
                    rate_index,
                    quarter_note_ms,
                    history_write,
                    history
                );
            }


            desired_enabled = true;
            PERF_QUANT_LOOPER_ENABLED = true;
        }
        else if(action == QUANT_FX_DISABLE ||
                action == QUANT_FX_FORCE_DISABLE)
        {
            desired_enabled = false;
            rate_crossfade_active = false;
            PERF_QUANT_LOOPER_ENABLED = false;
        }
    }


    void OnQuarter(float quarter_note_ms)
    {
        (void)quarter_note_ms;
    }


    float ReadLoopHead(
        const float* history,
        uint32_t start,
        uint32_t period,
        uint32_t seam,
        uint32_t& index)
    {
        if(
            period == 0u ||
            seam < 2u
        )
        {
            return 0.0f;
        }


        uint32_t capture_length =
            period +
            seam;


        /*
         * Normal loop data occupies:
         *
         *     index seam ... capture_length-1
         *
         * The final 'seam' samples begin at index=period.
         */
        float output;


        if(index < period)
        {
            /*
             * Ordinary playback region.
             */
            output =
                history[
                    (
                        start +
                        index
                    )
                    %
                    43200u
                ];


        }
        else
        {
            /*
             * CLICK-SAFE WRAP.
             *
             * tail:
             *     last seam samples of the nominal loop
             *
             * head:
             *     pre-roll samples immediately BEFORE the nominal
             *     loop start
             *
             * At the final overlap sample output approaches pre-roll
             * sample seam-1. The next playback sample is index=seam,
             * which is the next adjacent historical sample: the actual
             * loop start.
             */
            uint32_t x =
                index -
                period;


            float tail =
                history[
                    (
                        start +
                        index
                    )
                    %
                    43200u
                ];


            float head =
                history[
                    (
                        start +
                        x
                    )
                    %
                    43200u
                ];


            float t =
                static_cast<float>(
                    x
                )
                /
                static_cast<float>(
                    seam -
                    1u
                );


            t =
                SmoothstepAdded(
                    Clamp01Added(
                        t
                    )
                );


            output =
                tail *
                (
                    1.0f -
                    t
                )
                +
                head *
                t;


            /*
             * IMPORTANT DE-CLICK FIX:
             *
             * Do not filter only the seam. The old seam-local LPF altered
             * the final overlap sample, then the next sample jumped back to
             * raw/unfiltered loop audio. That recreated a discontinuity.
             *
             * The smoothstep overlap already lands exactly on the sample
             * immediately before the actual loop start, so the following
             * raw sample is naturally contiguous.
             */
        }


        index++;


        if(index >= capture_length)
        {
            /*
             * The pre-roll samples 0..seam-1 were already consumed by
             * the overlap, so continue at the ACTUAL loop start.
             */
            index =
                seam;
        }


        return output;
    }


    float Process(
        float input,
        const float* history)
    {
        if(!active)
            return input;


        float old_loop =
            ReadLoopHead(
                history,
                loop_start,
                loop_period,
                loop_seam,
                loop_index
            );


        float looped =
            old_loop;


        if(rate_crossfade_active)
        {
            float new_loop =
                ReadLoopHead(
                    history,
                    next_loop_start,
                    next_loop_period,
                    next_loop_seam,
                    next_loop_index
                );


            float t =
                static_cast<float>(
                    rate_crossfade_pos
                )
                /
                static_cast<float>(
                    rate_crossfade_samples
                );


            t =
                SmoothstepAdded(
                    Clamp01Added(
                        t
                    )
                );


            /*
             * Old complete loop engine -> new complete loop engine.
             *
             * Neither side is restarted during this fade and there is
             * never an intermediate dry gap.
             */
            looped =
                old_loop *
                (
                    1.0f -
                    t
                )
                +
                new_loop *
                t;


            rate_crossfade_pos++;


            if(rate_crossfade_pos >=
               rate_crossfade_samples)
            {
                latched_rate_index =
                    next_rate_index;

                loop_start =
                    next_loop_start;

                loop_period =
                    next_loop_period;

                loop_seam =
                    next_loop_seam;

                loop_index =
                    next_loop_index;

                rate_crossfade_active = false;
            }
        }


        float target =
            desired_enabled
            ? 1.0f
            : 0.0f;


        /*
         * Precomputed 48 kHz fade coefficients. Render with the CURRENT
         * mix before stepping it so the first sample after arming is
         * exactly dry.
         */
        float fade_alpha =
            desired_enabled
            ? 0.00433087f   /* 24 ms */
            : 0.00519479f;  /* 20 ms */


        /*
         * Smooth dry -> sampled-loop transition.
         */
        float output =
            input +
            (
                looped -
                input
            )
            *
            engage_mix;


        engage_mix +=
            (
                target -
                engage_mix
            )
            *
            fade_alpha;


        if(engage_mix < 0.000001f)
            engage_mix = 0.0f;


        if(engage_mix > 0.999999f)
            engage_mix = 1.0f;


        if(
            !desired_enabled &&
            engage_mix <= 0.000001f
        )
        {
            active = false;

            loop_start = 0;
            loop_period = 0;
            loop_seam = 0;
            loop_index = 0;

            rate_crossfade_active = false;
        }


        return output;
    }
};



/* ============================================================
   MASTER DJ HIGH-PASS
   ============================================================ */

struct AddedDjHighpass
{
    float ic1eq = 0.0f;
    float ic2eq = 0.0f;

    float position = 0.0f;
    float position_smoothed = 0.0f;

    /*
     * Cached TPT coefficients.
     */
    float cached_g = 0.001f;
    float cached_k = 1.41421356f;
    float cached_a1 = 1.0f;

    uint32_t coeff_counter = 0;

    bool was_requested = false;

    AddedSmoothWet wet;


    void Reset()
    {
        ic1eq = 0.0f;
        ic2eq = 0.0f;

        position = 0.0f;
        position_smoothed = 0.0f;

        cached_g = 0.001f;
        cached_k = 1.41421356f;
        cached_a1 = 1.0f;

        coeff_counter = 0;
        was_requested = false;

        wet.Reset();
    }


    void UpdateCoefficients(float p)
    {
        float cutoff =
            24.0f *
            powf(
                11500.0f /
                24.0f,
                p
            );


        /*
         * Non-ringing performance HPF.
         */
        float q =
            0.707f +
            0.10f *
            SmoothstepAdded(
                p
            );


        cached_g =
            tanf(
                PI *
                cutoff /
                SAMPLE_RATE
            );


        if(cached_g > 5.0f)
            cached_g = 5.0f;


        cached_k =
            1.0f /
            q;


        cached_a1 =
            1.0f /
            (
                1.0f +
                cached_g *
                (
                    cached_g +
                    cached_k
                )
            );
    }


    float Process(
        float input,
        bool protect_kick_onset = false)
    {
        bool requested =
            PERF_DJ_HPF_ENABLED &&
            macro_fx_value_hpf > 0.001f;


        /*
         * TRUE BYPASS:
         *
         * Once the crossfade is fully dry there is no reason to evaluate
         * an inaudible SVF or tanf().
         */
        if(!requested &&
           wet.wet <= 0.000001f)
        {
            was_requested = false;

            /*
             * Keep only a trivial prime value for the next enable.
             */
            ic1eq = 0.0f;
            ic2eq = input;

            position_smoothed = 0.0f;

            return input;
        }


        float target =
            requested
            ? Clamp01Added(
                  position
              )
            : 0.0f;


        /*
         * 30 ms smoothing coefficient at 48 kHz, precomputed to avoid a
         * control-rate expf() on every sample.
         */
        constexpr float smooth_a =
            0.99653381f;


        position_smoothed =
            target +
            (
                position_smoothed -
                target
            ) *
            smooth_a;


        float p =
            Clamp01Added(
                position_smoothed
            );


        /*
         * On the first requested sample, initialise the integrators close
         * to the current waveform rather than waking from zero.
         */
        if(requested &&
           !was_requested)
        {
            ic1eq = 0.0f;
            ic2eq = input;

            coeff_counter =
                DJ_FILTER_COEFF_UPDATE_SAMPLES;
        }


        was_requested =
            requested;


        if(coeff_counter >=
           DJ_FILTER_COEFF_UPDATE_SAMPLES)
        {
            UpdateCoefficients(
                p
            );

            coeff_counter = 0;
        }
        else
        {
            coeff_counter++;
        }


        float v1 =
            cached_a1 *
            (
                ic1eq +
                cached_g *
                (
                    input -
                    ic2eq
                )
            );


        float v2 =
            ic2eq +
            cached_g *
            v1;


        ic1eq =
            2.0f *
            v1 -
            ic1eq;


        ic2eq =
            2.0f *
            v2 -
            ic2eq;


        float high =
            input -
            cached_k *
            v1 -
            v2;


        float filtered =
            wet.Process(
                input,
                high,
                requested,
                35.0f
            );


        /*
         * A high-pass naturally emphasizes a kick's step-like onset. On
         * the generated kick that reads as a brittle click. Keep the SVF
         * running from sample one so its state is settled, but hold the
         * audible kick path dry briefly and crossfade into the HPF.
         */
        if(protect_kick_onset)
        {
            float onset_mix =
                PerformanceFilterKickOnsetBlend();


            filtered =
                input +
                (
                    filtered -
                    input
                )
                *
                onset_mix;
        }


        return filtered;
    }
};

/* ============================================================
   RESONANT BODY LOW-PASS — TPT STATE VARIABLE FILTER
   ============================================================

   transient + optional character -> resonant LPF -> BODY
   clean tail bypasses this filter
   protected punch is added later, post-processing
   ============================================================ */

struct AddedBodyResonantLowpass
{
    float ic1eq = 0.0f;
    float ic2eq = 0.0f;

    float g = 0.1f;
    float k = 1.0f;


    void Reset()
    {
        ic1eq = 0.0f;
        ic2eq = 0.0f;
    }


    void Set(
        float cutoff_hz,
        float q)
    {
        cutoff_hz =
            ClampAdded(
                cutoff_hz,
                100.0f,
                18000.0f
            );

        q =
            ClampAdded(
                q,
                0.50f,
                3.00f
            );

        g =
            tanf(
                PI *
                cutoff_hz /
                SAMPLE_RATE
            );

        k =
            1.0f / q;
    }


    float Process(float input)
    {
        float a1 =
            1.0f /
            (
                1.0f +
                g *
                (g + k)
            );

        float v1 =
            a1 *
            (
                ic1eq +
                g *
                (input - ic2eq)
            );

        float v2 =
            ic2eq +
            g * v1;

        ic1eq =
            2.0f * v1 -
            ic1eq;

        ic2eq =
            2.0f * v2 -
            ic2eq;

        return v2;
    }
};


static AddedBodyResonantLowpass added_body_lowpass;


/*
 * ============================================================
 * TAIL-DELAY / BODY-PUMP MORPH CONTROLS
 * ============================================================
 *
 * K3 is no longer a gate that suddenly closes to zero as soon as the
 * parameter leaves 0.
 *
 * Instead K3 continuously morphs:
 *
 *     low  -> soft body duck under the punch
 *     mid  -> obvious pumping / kick-bass separation
 *     high -> near-muted delayed body, approaching two sequencer steps
 *
 * The front punch-through window grows slightly with K3:
 *
 *     low  = duck begins early, behaving like sidechain
 *     high = preserve more of the punch before moving the body away
 */
static constexpr float TAIL_DELAY_PUNCH_LOW_MS  = 5.0f;
static constexpr float TAIL_DELAY_PUNCH_HIGH_MS = 18.0f;


/*
 * Low K3 is deliberately soft/slow.
 * High K3 closes more decisively for a stronger separated-tail effect.
 */
static constexpr float TAIL_DELAY_CLOSE_LOW_MS  = 22.0f;
static constexpr float TAIL_DELAY_CLOSE_HIGH_MS = 10.0f;


/*
 * Low K3 recovers softly like a pump.
 * High K3 reopens after the delayed body position.
 */
static constexpr float TAIL_DELAY_OPEN_LOW_MS  = 40.0f;
static constexpr float TAIL_DELAY_OPEN_HIGH_MS = 18.0f;


/*
 * At maximum K3 the delayed body is almost gone during the hold, but
 * never mathematically forced to exactly zero.
 *
 * This avoids a hard gate edge while still giving a strong two-step
 * separation at the top of the knob.
 */
static constexpr float TAIL_DELAY_MIN_GAIN_AT_MAX = 0.035f;


/*
 * Live K3 movement / enable-disable smoothing.
 */
static constexpr float TAIL_DELAY_PARAMETER_SMOOTH_MS = 5.0f;


/* ============================================================
   MACRO 3 — SOFT PUMP -> TAIL-SEPARATION MORPH
   ============================================================

   No audio is delayed in memory.

   Low K3:
       a gentle sidechain-like body duck under the punch

   Mid K3:
       deeper pumping and obvious kick/bass separation

   High K3:
       near-muted body hold followed by a smooth delayed return,
       reaching two 1/16 steps = one 1/8 note at maximum

   CC22 remains unchanged for Teensy compatibility.

   CC102 toggles the function.
   ============================================================ */

struct MacroTailDelayEnvelope
{
    uint32_t age_samples = 0;
    float gain = 1.0f;


    void Reset()
    {
        age_samples = 0;
        gain = 1.0f;
    }


    void Trigger()
    {
        /*
         * Never force gain to either endpoint here.
         *
         * A new hit starts in the punch-through region, whose target is
         * unity, and the small parameter slew moves us there safely.
         */
        age_samples = 0;
    }


    float Process()
    {
        bool requested =
            tail_delay_enabled &&
            macro_tail_delay > 0.005f;


        float target = 1.0f;


        if(requested)
        {
            float x =
                Clamp01Added(
                    macro_tail_delay
                );


            /*
             * Smooth macro domain used for timing interpolation.
             */
            float shape =
                SmoothstepAdded(
                    x
                );


            /*
             * ------------------------------------------------
             * DUCK DEPTH
             * ------------------------------------------------
             *
             * This is the important behavioural change.
             *
             * Small K3 values do NOT gate the body to zero.
             *
             * Approximate minimum body gains:
             *
             *     K3 0.10 -> ~0.96
             *     K3 0.25 -> ~0.85
             *     K3 0.50 -> ~0.62
             *     K3 0.75 -> ~0.35
             *     K3 1.00 -> ~0.035
             *
             * So the beginning of the knob behaves like a pump, while
             * the end becomes real kick/body separation.
             */
            float depth_curve =
                powf(
                    x,
                    1.35f
                );


            float minimum_gain =
                1.0f -
                (
                    1.0f -
                    TAIL_DELAY_MIN_GAIN_AT_MAX
                )
                *
                depth_curve;


            /*
             * ------------------------------------------------
             * TIMING
             * ------------------------------------------------
             */

            float rhythmic_delay_ms =
                MacroTailDelayMs(
                    x
                );


            float age_ms =
                static_cast<float>(
                    age_samples
                )
                *
                1000.0f /
                SAMPLE_RATE;


            float punch_through_ms =
                TAIL_DELAY_PUNCH_LOW_MS +
                (
                    TAIL_DELAY_PUNCH_HIGH_MS -
                    TAIL_DELAY_PUNCH_LOW_MS
                )
                *
                shape;


            float close_ms =
                TAIL_DELAY_CLOSE_LOW_MS +
                (
                    TAIL_DELAY_CLOSE_HIGH_MS -
                    TAIL_DELAY_CLOSE_LOW_MS
                )
                *
                shape;


            float open_ms =
                TAIL_DELAY_OPEN_LOW_MS +
                (
                    TAIL_DELAY_OPEN_HIGH_MS -
                    TAIL_DELAY_OPEN_LOW_MS
                )
                *
                shape;


            float close_start_ms =
                punch_through_ms;


            float close_end_ms =
                close_start_ms +
                close_ms;


            /*
             * The actual rhythmic displacement grows much faster near the
             * top of the knob because MacroTailDelayMs() uses x^1.7.
             *
             * At low K3 there is essentially no "silent gap": the body
             * dips and immediately begins recovering.
             */
            float open_start_ms =
                punch_through_ms +
                rhythmic_delay_ms;


            if(open_start_ms <
               close_end_ms)
            {
                open_start_ms =
                    close_end_ms;
            }


            float open_end_ms =
                open_start_ms +
                open_ms;


            if(age_ms <
               close_start_ms)
            {
                target = 1.0f;
            }
            else if(age_ms <
                    close_end_ms)
            {
                /*
                 * Smooth body duck:
                 * unity -> amount-dependent minimum.
                 */
                float t =
                    (
                        age_ms -
                        close_start_ms
                    )
                    /
                    close_ms;


                t =
                    SmoothstepAdded(
                        Clamp01Added(
                            t
                        )
                    );


                target =
                    1.0f +
                    (
                        minimum_gain -
                        1.0f
                    )
                    *
                    t;
            }
            else if(age_ms <
                    open_start_ms)
            {
                /*
                 * At low K3 this is shallow and usually extremely short.
                 * At high K3 this becomes the obvious delayed-tail hold.
                 */
                target =
                    minimum_gain;
            }
            else if(age_ms <
                    open_end_ms)
            {
                /*
                 * Soft recovery:
                 * amount-dependent minimum -> unity.
                 */
                float t =
                    (
                        age_ms -
                        open_start_ms
                    )
                    /
                    open_ms;


                t =
                    SmoothstepAdded(
                        Clamp01Added(
                            t
                        )
                    );


                target =
                    minimum_gain +
                    (
                        1.0f -
                        minimum_gain
                    )
                    *
                    t;
            }
            else
            {
                target = 1.0f;
            }
        }


        /*
         * Final live-control slew.
         *
         * The envelope is already derivative-smooth; this catches K3
         * movement and enable/disable changes while audio is running.
         */
        float smooth_samples =
            SAMPLE_RATE *
            TAIL_DELAY_PARAMETER_SMOOTH_MS /
            1000.0f;


        if(smooth_samples < 1.0f)
            smooth_samples = 1.0f;


        float a =
            expf(
                -5.0f /
                smooth_samples
            );


        gain =
            target +
            (
                gain -
                target
            )
            *
            a;


        if(fabsf(gain - target) <
           0.00001f)
        {
            gain = target;
        }


        age_samples++;


        return gain;
    }
};


static MacroTailDelayEnvelope macro_tail_delay_envelope;


/* ============================================================
   MACRO 4 — THREE ADDITIVE BPF CHARACTER LAYERS
   ============================================================

   Button cycles 0 -> 1 -> 2 -> 3 -> 0.
   Knob tunes the newest/current layer.

   Filters are kept low-Q at bass frequencies and are allowed to become
   more resonant in the mids, avoiding the exhausting fixed resonator
   problem while still reaching aggressive character.
   ============================================================ */

/*
 * Broad BPF midrange de-emphasis.
 *
 * The bank was subjectively peaking too hard from ~600 Hz to 1.6 kHz.
 * This creates a smooth window:
 *
 *     ~0 below 500 Hz
 *     rises through 500..700 Hz
 *     strongest through the central mids
 *     falls through 1.4..1.8 kHz
 *
 * It is deliberately gentle — not a notch.
 */
static float MacroBpfMidTameAmount(
    float frequency_hz)
{
    float rise =
        SmoothstepAdded(
            Clamp01Added(
                (
                    frequency_hz -
                    500.0f
                )
                /
                200.0f
            )
        );


    float fall =
        1.0f -
        SmoothstepAdded(
            Clamp01Added(
                (
                    frequency_hz -
                    1400.0f
                )
                /
                400.0f
            )
        );


    return
        Clamp01Added(
            rise *
            fall
        );
}


struct MacroBpfBank
{
    Biquad drive_filters[3];
    Biquad return_filters[3];

    float current_hz[3] = {330.0f, 700.0f, 1450.0f};
    float layer_gain[3] = {0.0f, 0.0f, 0.0f};
    /* Per-layer weighting by FREQUENCY, not by layer index. */
    float layer_tilt[3] = {1.0f, 1.0f, 1.0f};

    void Reset()
    {
        for(int i = 0; i < 3; ++i)
        {
            drive_filters[i].Reset();
            return_filters[i].Reset();
            current_hz[i] = macro_bpf_target_hz[i];
            layer_gain[i] = 0.0f;
            drive_filters[i].SetBandpass(current_hz[i], 0.82f);
            return_filters[i].SetBandpass(current_hz[i], 1.20f);
        }
    }

    void Update()
    {
        for(int i = 0; i < 3; ++i)
        {
            current_hz[i] +=
                (macro_bpf_target_hz[i] - current_hz[i]) * 0.035f;

            float normalized =
                logf(current_hz[i] / 115.0f) /
                logf(3200.0f / 115.0f);
            normalized = Clamp01Added(normalized);

            /* Broad pre-drive EQ; never a whistling resonator. */
            float drive_q =
                0.78f + normalized * 0.70f + static_cast<float>(i) * 0.08f;
            if(drive_q > 1.65f)
                drive_q = 1.65f;

            /* More vocal post-drive return, bounded to musical territory. */
            float return_q =
                1.70f + normalized * 2.60f + static_cast<float>(i) * 0.30f;
            float mid_tame = MacroBpfMidTameAmount(current_hz[i]);
            return_q *= 1.0f - mid_tame * 0.10f;
            if(return_q > 5.20f)
                return_q = 5.20f;

            /*
             * Tilt toward the top of the band. A layer parked low sits on
             * the protected punch/sub and muddies it, so it is held back;
             * a layer up near 3.2 kHz is where the bank should bite.
             */
            layer_tilt[i] = 0.40f + normalized * 1.45f;

            drive_filters[i].SetBandpass(current_hz[i], drive_q);
            return_filters[i].SetBandpass(current_hz[i], return_q);
        }
    }

    float ProcessDriveFeed(float input)
    {
        float added = 0.0f;
        float source = SoftClip(input * 1.45f);

        for(int i = 0; i < 3; ++i)
        {
            float band = drive_filters[i].Process(source);
            float gain = 0.20f + static_cast<float>(i) * 0.035f;
            added += band * layer_gain[i] * gain * layer_tilt[i];
        }

        return added;
    }

    float Process(float input)
    {
        float added = 0.0f;
        constexpr float layer_smooth_a = 0.99135701f;
        uint8_t count = macro_bpf_layer_count;

        float count_compensation =
            count >= 3 ? 0.80f : (count == 2 ? 0.89f : 1.0f);

        float source = SoftClip(input * 1.80f);

        for(int i = 0; i < 3; ++i)
        {
            float target_gain = i < static_cast<int>(count) ? 1.0f : 0.0f;
            layer_gain[i] =
                target_gain + (layer_gain[i] - target_gain) * layer_smooth_a;

            float band = return_filters[i].Process(source);
            band = SoftClip(
                band * (2.70f + static_cast<float>(i) * 0.30f)
            );

            float mid_tame =
                1.0f - MacroBpfMidTameAmount(current_hz[i]) * 0.10f;
            /* 4x: the bank's audible return only. ProcessDriveFeed is left
             * alone so what the distortion models are fed is unchanged. */
            float gain =
                (0.31f + static_cast<float>(i) * 0.055f) * param_bpf_gain;

            added +=
                band * layer_gain[i] * gain * count_compensation * mid_tame *
                layer_tilt[i];
        }

        return added;
    }
};

static MacroBpfBank macro_bpf_bank;


/* ============================================================
   MACRO 5 — MACKIE-INSPIRED CHARACTER
   ============================================================

   This is a MUSICAL approximation, not a component-exact CR-1604 SPICE
   model. Its important behaviours are:

       asymmetric soft rails
       modest even-order asymmetry
       4x nonlinear oversampling
       very-low-frequency coupling / DC block
       broad low-mid emphasis after overload

   No tanh, no random modulation, no feedback.
   ============================================================ */

struct MacroMackieProcessor
{
    Biquad body_band;
    Biquad presence_band;

    float previous_input = 0.0f;
    float pre_lp_1 = 0.0f;
    float pre_lp_2 = 0.0f;
    float dc_x1 = 0.0f;
    float dc_y1 = 0.0f;
    float post_lp_1 = 0.0f;
    float post_lp_2 = 0.0f;

    static float Core(float x)
    {
        float shaped = x + 0.020f * x * fabsf(x);
        const float positive_rail = 1.00f;
        const float negative_rail = 0.955f;

        if(shaped >= 0.0f)
        {
            float u = shaped / positive_rail;
            return positive_rail * (u / sqrtf(1.0f + u * u));
        }

        float u = -shaped / negative_rail;
        return -negative_rail * (u / sqrtf(1.0f + u * u));
    }

    void Reset()
    {
        body_band.Reset();
        presence_band.Reset();
        body_band.SetBandpass(340.0f, 0.78f);
        presence_band.SetBandpass(1050.0f, 0.92f);

        previous_input = 0.0f;
        pre_lp_1 = pre_lp_2 = 0.0f;
        dc_x1 = dc_y1 = 0.0f;
        post_lp_1 = post_lp_2 = 0.0f;
    }

    void Trigger() {}

    float Process(float input, float amount)
    {
        if(amount <= CHARACTER_HEAVY_PROCESS_EPSILON)
        {
            previous_input = input;
            pre_lp_1 = pre_lp_2 = input;
            post_lp_1 = post_lp_2 = 0.0f;
            dc_x1 = dc_y1 = 0.0f;
            body_band.Process(0.0f);
            presence_band.Process(0.0f);
            return 0.0f;
        }

        constexpr float pre_a = 0.79210000f; /* MACKIE_PRE_LP_HZ */
        pre_lp_1 += pre_a * (input - pre_lp_1);
        pre_lp_2 += pre_a * (pre_lp_1 - pre_lp_2);

        float accumulated = 0.0f;
        for(int os = 1; os <= 4; ++os)
        {
            float t = static_cast<float>(os) * 0.25f;
            float x = previous_input + (pre_lp_2 - previous_input) * t;
            accumulated += Core(x * MACKIE_INTERNAL_GAIN);
        }
        previous_input = pre_lp_2;

        float stage1 = accumulated * 0.25f;

        constexpr float dc_r = 0.99935f;
        float dc_blocked = stage1 - dc_x1 + dc_r * dc_y1;
        dc_x1 = stage1;
        dc_y1 = dc_blocked;

        /* overload -> broad desk EQ boosts -> overload again */
        float body = body_band.Process(dc_blocked);
        float presence = presence_band.Process(dc_blocked);
        float eq_driven =
            dc_blocked + body * 0.62f + presence * 0.25f;

        float stage2 = Core(eq_driven * 1.65f);

        constexpr float post_a = 0.72990000f; /* MACKIE_POST_LP_HZ */
        post_lp_1 += post_a * (stage2 - post_lp_1);
        post_lp_2 += post_a * (post_lp_1 - post_lp_2);

        return post_lp_2 * param_mackie_gain;
    }
};

struct MacroShermanSVF
{
    float ic1eq = 0.0f;
    float ic2eq = 0.0f;
    float g = 0.1f;
    float k = 1.0f;
    float target_g = 0.1f;
    float target_k = 1.0f;

    void ResetState()
    {
        ic1eq = 0.0f;
        ic2eq = 0.0f;
    }

    void Set(float frequency, float resonance, bool immediate = false)
    {
        frequency = ClampAdded(frequency, 55.0f, 6000.0f);
        resonance = ClampAdded(resonance, 0.0f, 0.92f);

        /* k is 1/Q: the old 0.82 clamp and 0.48 floor capped Q near 1.4,
         * too broad to read as a resonant filter at all. */
        float new_k = 1.62f - resonance * 1.43f;
        if(new_k < 0.33f)
            new_k = 0.33f;

        float new_g = tanf(PI * frequency / SAMPLE_RATE);
        if(new_g > 3.2f)
            new_g = 3.2f;

        target_g = new_g;
        target_k = new_k;
        if(immediate)
        {
            g = target_g;
            k = target_k;
        }
    }

    void Process(float input, float& low, float& band, float& high)
    {
        constexpr float coeff_slew = 0.00415800f;
        g += (target_g - g) * coeff_slew;
        k += (target_k - k) * coeff_slew;

        float denom = 1.0f + g * (g + k);
        if(denom < 0.001f)
            denom = 0.001f;

        float v3 = input - ic2eq;
        float v1 = (ic1eq + g * v3) / denom;
        float v2 = ic2eq + g * v1;
        ic1eq = 2.0f * v1 - ic1eq;
        ic2eq = 2.0f * v2 - ic2eq;

        low = v2;
        band = v1;
        high = input - k * band - low;
    }
};

struct MacroShermanProcessor
{
    MacroShermanSVF f1;
    MacroShermanSVF f2;

    float pre_lp_1 = 0.0f;
    float pre_lp_2 = 0.0f;
    float post_lp_1 = 0.0f;
    float post_lp_2 = 0.0f;
    float feedback_memory = 0.0f;
    float input_hp_state = 0.0f;
    float prepared_fundamental = -1.0f;

    void PrepareForPitch(float fundamental, bool immediate = false)
    {
        fundamental = ClampAdded(fundamental, 25.0f, 130.0f);

        float f1_frequency =
            ClampAdded(fundamental * 5.5f, 220.0f, 760.0f);
        float f2_frequency =
            ClampAdded(f1_frequency * 2.15f, 500.0f, 1850.0f);

        f1.Set(f1_frequency, SHERMAN_FILTER_RESONANCE, immediate);
        f2.Set(f2_frequency, SHERMAN_FILTER_RESONANCE * 0.94f, immediate);
        prepared_fundamental = fundamental;
    }

    void Reset()
    {
        f1.ResetState();
        f2.ResetState();
        pre_lp_1 = pre_lp_2 = 0.0f;
        post_lp_1 = post_lp_2 = 0.0f;
        feedback_memory = 0.0f;
        input_hp_state = 0.0f;
        PrepareForPitch(kick_frequency, true);
    }

    void Trigger()
    {
        PrepareForPitch(kick_frequency, false);
    }

    float Process(float input, float amount)
    {
        if(amount <= CHARACTER_HEAVY_PROCESS_EPSILON)
        {
            pre_lp_1 = pre_lp_2 = input;
            post_lp_1 = post_lp_2 = 0.0f;
            feedback_memory = 0.0f;
            input_hp_state = input;
            f1.ResetState();
            f2.ResetState();
            return 0.0f;
        }

        if(prepared_fundamental < 0.0f)
            PrepareForPitch(kick_frequency, false);

        constexpr float pre_a = 0.79210000f; /* SHERMAN_PRE_LP_HZ */
        pre_lp_1 += pre_a * (input - pre_lp_1);
        pre_lp_2 += pre_a * (pre_lp_1 - pre_lp_2);

        constexpr float hp_a = 0.00680f;
        input_hp_state += hp_a * (pre_lp_2 - input_hp_state);
        float source = pre_lp_2 - input_hp_state;

        float feedback_signal = SoftClip(feedback_memory * 1.35f);
        float driven =
            SoftClip(
                source * SHERMAN_INPUT_DRIVE +
                feedback_signal * SHERMAN_FEEDBACK
            );

        float l1, b1, h1;
        f1.Process(driven, l1, b1, h1);
        float f1_mix = l1 * 0.12f + b1 * 0.78f + h1 * 0.10f;

        float f2_input = driven * 0.24f + f1_mix * 0.76f;
        float l2, b2, h2;
        f2.Process(SoftClip(f2_input * 1.45f), l2, b2, h2);
        float f2_mix = l2 * 0.08f + b2 * 0.78f + h2 * 0.14f;

        float wet =
            SoftClip((f1_mix * 0.42f + f2_mix * 0.82f) * 2.15f);

        constexpr float fb_post_a = 0.32f;
        feedback_memory += fb_post_a * (wet - feedback_memory);

        constexpr float post_a = 0.72990000f; /* SHERMAN_POST_LP_HZ */
        post_lp_1 += post_a * (wet - post_lp_1);
        post_lp_2 += post_a * (post_lp_1 - post_lp_2);

        return post_lp_2 * param_sherman_gain;
    }
};

struct MacroCharacterProcessor
{
    MacroMackieProcessor mackie;
    MacroShermanProcessor sherman;

    float mackie_amount_smoothed = 0.0f;
    float sherman_amount_smoothed = 0.0f;

    enum class SwitchState
    {
        STABLE,
        FADE_TO_ZERO,
        FADE_FROM_ZERO
    };

    SwitchState switch_state = SwitchState::STABLE;
    bool active_sherman = false;
    bool desired_sherman = false;
    float transition_gain = 1.0f;

    static bool AudioValueSafe(float x)
    {
        return (x == x) && fabsf(x) < 8.0f;
    }

    void PrepareMackie() { mackie.Reset(); }
    void PrepareSherman() { sherman.Reset(); }

    void Reset()
    {
        PrepareMackie();
        PrepareSherman();
        mackie_amount_smoothed = 0.0f;
        sherman_amount_smoothed = 0.0f;
        active_sherman = macro_character_sherman;
        desired_sherman = active_sherman;
        switch_state = SwitchState::STABLE;
        transition_gain = 1.0f;
        character_switch_pending = false;
        character_switch_target_sherman = active_sherman;
    }

    void Trigger()
    {
        if(active_sherman)
            sherman.Trigger();
        else
            mackie.Trigger();
    }

    void ConsumeSwitchRequest()
    {
        if(!character_switch_pending)
            return;

        desired_sherman = character_switch_target_sherman;
        character_switch_pending = false;
        if(desired_sherman != active_sherman)
            switch_state = SwitchState::FADE_TO_ZERO;
    }

    static float SmoothAmount(float current, float target)
    {
        constexpr float a = 0.99884392f;  // ~18 ms at 48 kHz
        return target + (current - target) * a;
    }

    float CurrentSmoothedAmount() const
    {
        return active_sherman ? sherman_amount_smoothed : mackie_amount_smoothed;
    }

    float ProcessSelectedWet(float input)
    {
        if(active_sherman)
        {
            float target = Clamp01Added(macro_sherman_amount);
            sherman_amount_smoothed = SmoothAmount(sherman_amount_smoothed, target);
            float drive =
                1.0f + sherman_amount_smoothed * CHARACTER_AMOUNT_DRIVE_RANGE;
            float wet = sherman.Process(input * drive, sherman_amount_smoothed);
            if(!AudioValueSafe(wet))
            {
                PrepareSherman();
                return 0.0f;
            }
            return wet * sherman_amount_smoothed;
        }

        float target = Clamp01Added(macro_mackie_amount);
        mackie_amount_smoothed = SmoothAmount(mackie_amount_smoothed, target);
        float drive =
            1.0f + mackie_amount_smoothed * CHARACTER_AMOUNT_DRIVE_RANGE;
        float wet = mackie.Process(input * drive, mackie_amount_smoothed);
        if(!AudioValueSafe(wet))
        {
            PrepareMackie();
            return 0.0f;
        }
        return wet * mackie_amount_smoothed;
    }

    float ProcessWet(float input)
    {
        ConsumeSwitchRequest();
        float wet = ProcessSelectedWet(input);
        constexpr float transition_step = 1.0f / (SAMPLE_RATE * 0.010f);

        if(switch_state == SwitchState::FADE_TO_ZERO)
        {
            transition_gain -= transition_step;
            if(transition_gain <= 0.0f)
            {
                transition_gain = 0.0f;
                active_sherman = desired_sherman;

                if(active_sherman)
                {
                    if(CHARACTER_RESET_ON_MODEL_SWITCH)
                        sherman_amount_smoothed = 0.0f;
                    PrepareSherman();
                }
                else
                {
                    if(CHARACTER_RESET_ON_MODEL_SWITCH)
                        mackie_amount_smoothed = 0.0f;
                    PrepareMackie();
                }

                switch_state = SwitchState::FADE_FROM_ZERO;
                return 0.0f;
            }
        }
        else if(switch_state == SwitchState::FADE_FROM_ZERO)
        {
            transition_gain += transition_step;
            if(transition_gain >= 1.0f)
            {
                transition_gain = 1.0f;
                switch_state = SwitchState::STABLE;
            }
        }
        else
        {
            transition_gain = 1.0f;
        }

        wet *= transition_gain;
        return AudioValueSafe(wet) ? wet : 0.0f;
    }
};

static MacroCharacterProcessor macro_character_processor;



/* ============================================================
   MACRO 2 — WHOLE-KICK REVERSE BUFFER
   ============================================================

   Captures the clean generated bass tail from NORMAL kicks.
   When reverse mode is enabled, a new kick plays that previous tail
   backwards instead of its normal clean tail.

   This is deliberately a tail-only capture; the tock/punch is never
   reversed.

   It is a triggered reverse-bass sound after the kick, not pre-roll
   audio before the MIDI event.
   ============================================================ */

struct MacroWholeKickReverse
{
    /*
     * SAME TOTAL MEMORY FOOTPRINT AS BEFORE.
     *
     * The old implementation used one 20,000-sample frozen capture.
     * While reverse was enabled it stopped capturing, which is why all
     * sound changes seemed frozen until reverse was disabled/re-enabled.
     *
     * We now use two 10,000-sample banks:
     *
     *     bank A = reverse playback
     *     bank B = capture the CURRENT live generated kick
     *
     * At each new kick the banks swap.
     *
     * Therefore parameters may be changed continuously while reverse is
     * enabled; the freshly rendered result becomes the very next reverse
     * hit without ever disabling reverse.
     */
    static constexpr uint32_t MAX_SAMPLES = 20000;
    static constexpr uint32_t BANK_SAMPLES = MAX_SAMPLES / 2;

    float buffer[MAX_SAMPLES];

    uint32_t valid_samples[2] =
    {
        0,
        0
    };

    uint8_t capture_bank = 0;
    uint8_t playback_bank = 1;

    uint32_t capture_write = 0;
    uint32_t play_index = 0;

    bool playing = false;

    /*
     * Needed to leave reverse mode without a one-sample discontinuity.
     */
    float last_reversed_sample = 0.0f;
    float last_output_sample = 0.0f;


    /*
     * A new captured hit can begin at a waveform value completely
     * unrelated to the old reversed hit. Smooth that bank swap itself.
     */
    bool bank_handoff_active = false;
    float bank_handoff_from = 0.0f;
    uint32_t bank_handoff_pos = 0;
    uint32_t bank_handoff_samples = 1;


    AddedSmoothWet wet;


    float* Bank(uint8_t bank)
    {
        return
            &buffer[
                static_cast<uint32_t>(
                    bank
                ) *
                BANK_SAMPLES
            ];
    }


    void Reset()
    {
        for(uint32_t i = 0;
            i < MAX_SAMPLES;
            ++i)
        {
            buffer[i] = 0.0f;
        }

        valid_samples[0] = 0;
        valid_samples[1] = 0;

        capture_bank = 0;
        playback_bank = 1;

        capture_write = 0;
        play_index = 0;

        playing = false;

        last_reversed_sample = 0.0f;
        last_output_sample = 0.0f;

        bank_handoff_active = false;
        bank_handoff_from = 0.0f;
        bank_handoff_pos = 0;

        bank_handoff_samples =
            static_cast<uint32_t>(
                SAMPLE_RATE *
                REVERSE_BANK_HANDOFF_MS /
                1000.0f
            );

        if(bank_handoff_samples < 16)
            bank_handoff_samples = 16;


        wet.Reset();
    }


    void Trigger()
    {
        /*
         * If reverse is already audible, preserve the exact current
         * output sample and crossfade from it into the new bank instead
         * of abruptly starting a new unrelated waveform.
         */
        bool continuing_reverse =
            reverse_bass_enabled &&
            playing &&
            wet.wet > 0.05f;


        if(continuing_reverse)
        {
            bank_handoff_active = true;
            bank_handoff_from = last_output_sample;
            bank_handoff_pos = 0;
        }
        else
        {
            bank_handoff_active = false;
        }


        /*
         * Finish the hit that has just been rendered into capture_bank.
         */
        if(capture_write > 64)
        {
            valid_samples[
                capture_bank
            ] =
                capture_write;


            playback_bank =
                capture_bank;


            capture_bank =
                static_cast<uint8_t>(
                    1u -
                    capture_bank
                );


            capture_write = 0;
        }


        if(reverse_bass_enabled &&
           valid_samples[
               playback_bank
           ] > 64)
        {
            play_index =
                valid_samples[
                    playback_bank
                ];

            playing = true;
        }
        else
        {
            playing = false;
            play_index = 0;
        }
    }


    float Process(float whole_kick)
    {
        /*
         * ALWAYS capture the CURRENT generated kick, even while we are
         * playing a reverse hit from the other bank.
         *
         * This is the critical anti-freeze behaviour.
         */
        if(capture_write <
           BANK_SAMPLES)
        {
            Bank(
                capture_bank
            )[
                capture_write
            ] =
                whole_kick;

            capture_write++;
        }


        bool want_reverse =
            reverse_bass_enabled &&
            playing &&
            play_index > 0 &&
            valid_samples[
                playback_bank
            ] > 64;


        if(!want_reverse)
        {
            /*
             * Fade safely back to current live audio if reverse is
             * disabled or the reverse bank reaches its end.
             *
             * IMPORTANT: do not call wet.Process(dry, dry, false), which
             * would bypass the wet state and create an immediate jump.
             * Hold the last reverse sample only for the tiny fade-out.
             */
            playing =
                playing &&
                play_index > 0;


            float output =
                wet.Process(
                    whole_kick,
                    last_reversed_sample,
                    false,
                    8.0f
                );


            last_output_sample =
                output;


            return output;
        }


        uint32_t reverse_index =
            play_index -
            1;


        float reversed =
            Bank(
                playback_bank
            )[
                reverse_index
            ];


        uint32_t played =
            valid_samples[
                playback_bank
            ] -
            play_index;


        /*
         * Smooth the actual ends of the reversed bank.
         */
        uint32_t edge =
            static_cast<uint32_t>(
                SAMPLE_RATE *
                0.006f
            );


        if(edge < 16)
            edge = 16;


        if(edge >
           valid_samples[
               playback_bank
           ] / 3)
        {
            edge =
                valid_samples[
                    playback_bank
                ] / 3;
        }


        float edge_gain = 1.0f;


        /*
         * Initial reverse-enable still gets an edge fade.
         *
         * During a bank-to-bank handoff the dedicated handoff crossfade
         * replaces this, otherwise we'd fade the new bank toward zero
         * and create the exact "gap/glitch" we're trying to remove.
         */
        if(!bank_handoff_active &&
           played < edge)
        {
            edge_gain *=
                SmoothstepAdded(
                    static_cast<float>(
                        played
                    )
                    /
                    static_cast<float>(
                        edge
                    )
                );
        }


        if(play_index < edge)
        {
            edge_gain *=
                SmoothstepAdded(
                    static_cast<float>(
                        play_index
                    )
                    /
                    static_cast<float>(
                        edge
                    )
                );
        }


        reversed *=
            edge_gain;


        if(bank_handoff_active)
        {
            float t =
                static_cast<float>(
                    bank_handoff_pos
                )
                /
                static_cast<float>(
                    bank_handoff_samples
                );


            t =
                SmoothstepAdded(
                    Clamp01Added(
                        t
                    )
                );


            /*
             * Start EXACTLY at the previous output sample, then move into
             * the new reverse waveform over ~9 ms.
             *
             * This guarantees value continuity at the bank boundary and
             * removes the sharp transition glitch.
             */
            reversed =
                bank_handoff_from +
                (
                    reversed -
                    bank_handoff_from
                ) *
                t;


            bank_handoff_pos++;


            if(bank_handoff_pos >=
               bank_handoff_samples)
            {
                bank_handoff_active = false;
            }
        }


        last_reversed_sample =
            reversed;


        play_index--;


        if(play_index == 0)
            playing = false;


        /*
         * This 6 ms blend is only an enable/disable de-click transition.
         * During normal reverse playback wet sits at 1.0.
         */
        float output =
            wet.Process(
                whole_kick,
                reversed,
                true,
                8.0f
            );


        last_output_sample =
            output;


        return output;
    }
};


constexpr uint32_t MacroWholeKickReverse::MAX_SAMPLES;

static MacroWholeKickReverse macro_whole_kick_reverse;


/* ============================================================
   MACRO 1 — RESONANT MASTER DJ LOW-PASS
   ============================================================ */

struct MacroDjLowpass
{
    float ic1eq = 0.0f;
    float ic2eq = 0.0f;

    float position_smoothed = 0.0f;

    float cached_g = 0.001f;
    float cached_k = 1.41421356f;
    float cached_a1 = 1.0f;

    uint32_t coeff_counter = 0;

    bool was_requested = false;

    AddedSmoothWet wet;


    void Reset()
    {
        ic1eq = 0.0f;
        ic2eq = 0.0f;

        position_smoothed = 0.0f;

        cached_g = 0.001f;
        cached_k = 1.41421356f;
        cached_a1 = 1.0f;

        coeff_counter = 0;
        was_requested = false;

        wet.Reset();
    }


    void UpdateCoefficients(float p)
    {
        float cutoff =
            18000.0f *
            powf(
                120.0f /
                18000.0f,
                p
            );


        float q =
            0.707f +
            0.08f *
            SmoothstepAdded(
                p
            );


        cached_g =
            tanf(
                PI *
                cutoff /
                SAMPLE_RATE
            );


        if(cached_g > 5.0f)
            cached_g = 5.0f;


        cached_k =
            1.0f /
            q;


        cached_a1 =
            1.0f /
            (
                1.0f +
                cached_g *
                (
                    cached_g +
                    cached_k
                )
            );
    }


    float Process(float input)
    {
        bool requested =
            macro_fx_value_lpf >
            0.001f;


        if(!requested &&
           wet.wet <= 0.000001f)
        {
            was_requested = false;

            ic1eq = 0.0f;
            ic2eq = input;

            position_smoothed = 0.0f;

            return input;
        }


        float target =
            requested
            ? Clamp01Added(
                  macro_fx_value_lpf
              )
            : 0.0f;


        float smooth_a =
            expf(
                -5.0f /
                (
                    SAMPLE_RATE *
                    0.030f
                )
            );


        position_smoothed =
            target +
            (
                position_smoothed -
                target
            ) *
            smooth_a;


        float p =
            Clamp01Added(
                position_smoothed
            );


        if(requested &&
           !was_requested)
        {
            /*
             * LPF steady-ish prime: start the low integrator at the
             * current sample so the filtered branch does not wake from 0.
             */
            ic1eq = 0.0f;
            ic2eq = input;

            coeff_counter =
                DJ_FILTER_COEFF_UPDATE_SAMPLES;
        }


        was_requested =
            requested;


        if(coeff_counter >=
           DJ_FILTER_COEFF_UPDATE_SAMPLES)
        {
            UpdateCoefficients(
                p
            );

            coeff_counter = 0;
        }
        else
        {
            coeff_counter++;
        }


        float v1 =
            cached_a1 *
            (
                ic1eq +
                cached_g *
                (
                    input -
                    ic2eq
                )
            );


        float v2 =
            ic2eq +
            cached_g *
            v1;


        ic1eq =
            2.0f *
            v1 -
            ic1eq;


        ic2eq =
            2.0f *
            v2 -
            ic2eq;


        float low =
            v2;


        return
            wet.Process(
                input,
                low,
                requested,
                35.0f
            );
    }
};


static MacroDjLowpass macro_dj_lowpass;

/*
 * Lightweight independent Digitakt DJ-LPF state.
 * No large audio history buffer is duplicated.
 */
static MacroDjLowpass external_macro_dj_lowpass;



/* ============================================================
   EXPERIMENTAL RESONANT 200 -> 80 Hz SWEEP LAYER
   ============================================================ */

struct AddedResonantSweepLayer
{
    /*
     * TPT SVF state.
     */
    float ic1eq = 0.0f;
    float ic2eq = 0.0f;

    /*
     * Parallel-layer input DC/infrasonic protection only.
     */
    float input_lp_state = 0.0f;


    /*
     * Continuous sweep state.
     *
     * IMPORTANT:
     * We no longer derive cutoff directly from
     *
     *      elapsed_time / sweep_duration
     *
     * because changing sweep duration at Note-Off can jump that ratio
     * and therefore jump the high-Q filter cutoff.
     *
     * Instead cutoff itself is a continuous state. Gate changes modify
     * only how fast it moves toward the target.
     */
    float current_cutoff_hz = 200.0f;

    /*
     * Per-trigger sweep endpoints.
     * These are derived from the actual kick fundamental.
     */
    float sweep_start_hz = 200.0f;
    float sweep_end_hz = 80.0f;

    /*
     * Deterministic phase-reset resonant exciter.
     */
    float exciter_phase = 0.0f;

    float sweep_ms = 105.0f;
    float target_sweep_ms = 105.0f;


    /*
     * Independent normalized musical envelope progress.
     * Its RATE can change with gate duration, but its VALUE never jumps.
     */
    float env_progress = 0.0f;


    /*
     * Ultra-short de-click gain.
     *
     * This is NOT the musical amplitude envelope.
     * It exists solely so resetting a resonant parallel processor never
     * creates a one-sample discontinuity.
     */
    float transition_gain = 0.0f;

    bool active = false;
    bool releasing_transition = false;


    void ResetFilterState()
    {
        ic1eq = 0.0f;
        ic2eq = 0.0f;
        input_lp_state = 0.0f;
    }


    float GateToSweepMs(float gate_ms) const
    {
        float ms =
            gate_ms *
            1.10f;

        return
            ClampAdded(
                ms,
                RESONANT_SWEEP_MIN_MS,
                RESONANT_SWEEP_MAX_MS
            );
    }


    void Reset()
    {
        ResetFilterState();

        sweep_start_hz =
            resonant_sweep_start_hz;

        sweep_end_hz =
            resonant_sweep_end_hz;

        current_cutoff_hz =
            sweep_start_hz;

        exciter_phase = 0.0f;

        sweep_ms =
            resonant_sweep_default_ms;

        target_sweep_ms =
            resonant_sweep_default_ms;

        env_progress = 0.0f;

        transition_gain = 0.0f;

        active = false;
        releasing_transition = false;
    }


    void Trigger(float previous_gate_ms,
                 float fundamental_hz)
    {
        /*
         * Fresh hit: deterministic reset.
         *
         * Ratchet: preserve live SVF/exciter memory. Even a 2.5 ms
         * fade-in cannot hide an instantaneous DROP of the previous
         * resonator output caused by zeroing its state.
         */
        if(
            !kick_retrigger_active ||
            !PHASE_CONTINUOUS_RATCHET_RETRIGGER ||
            PSY_PHASE_LOCK_EVERY_HIT
        )
        {
            ResetFilterState();
        }

        /*
         * PARTIAL KEY TRACKING
         * --------------------
         *
         * Reference design is 200 -> 80 Hz around a ~55 Hz kick.
         * Tracking uses a power curve so the layer follows the key
         * without leaving the useful physical punch region too quickly.
         */
        if(fundamental_hz < 25.0f)
            fundamental_hz = 25.0f;

        if(fundamental_hz > 130.0f)
            fundamental_hz = 130.0f;

        float ratio =
            fundamental_hz /
            55.0f;

        float tracked_ratio =
            powf(
                ratio,
                Clamp01Added(
                    resonant_sweep_key_tracking
                )
            );

        sweep_start_hz =
            ClampAdded(
                resonant_sweep_start_hz *
                tracked_ratio,
                65.0f,
                95.0f
            );

        sweep_end_hz =
            ClampAdded(
                resonant_sweep_end_hz *
                tracked_ratio,
                55.0f,
                90.0f
            );

        current_cutoff_hz =
            sweep_start_hz;

        /*
         * Preserve the sine exciter's phase on overlapping ratchets.
         */
        if(
            !kick_retrigger_active ||
            !PHASE_CONTINUOUS_RATCHET_RETRIGGER ||
            PSY_PHASE_LOCK_EVERY_HIT
        )
        {
            exciter_phase = 0.0f;
        }

        if(ENABLE_RESONANT_SWEEP_GATE_MODULATION)
        {
            target_sweep_ms =
                GateToSweepMs(
                    previous_gate_ms
                );
        }
        else
        {
            target_sweep_ms =
                resonant_sweep_default_ms;
        }

        sweep_ms =
            target_sweep_ms;

        env_progress = 0.0f;

        if(
            !kick_retrigger_active ||
            !PHASE_CONTINUOUS_RATCHET_RETRIGGER ||
            PSY_PHASE_LOCK_EVERY_HIT
        )
        {
            transition_gain = 0.0f;
        }

        active = true;
        releasing_transition = false;
    }


    void UpdateCurrentGate(float measured_gate_ms)
    {
        if(!ENABLE_RESONANT_SWEEP_GATE_MODULATION)
            return;

        /*
         * DO NOT jump sweep_ms.
         *
         * Only update the TARGET. Process() smooths toward it, and the
         * actual cutoff is an independent continuous state anyway.
         */
        target_sweep_ms =
            GateToSweepMs(
                measured_gate_ms
            );
    }


    float MusicalAmplitudeEnvelope() const
    {
        if(!ENABLE_RESONANT_SWEEP_AMP_ENVELOPE)
            return 1.0f;

        float p =
            Clamp01Added(
                env_progress
            );

        float hold =
            ClampAdded(
                resonant_sweep_env_hold_ratio,
                0.20f,
                0.95f
            );


        /*
         * Musical envelope still has NO attack.
         *
         * The separate transition_gain below is only de-clicking.
         */
        if(p <= hold)
            return 1.0f;


        float release_progress =
            (
                p -
                hold
            )
            /
            (
                1.0f -
                hold
            );


        return
            1.0f -
            SmoothstepAdded(
                release_progress
            );
    }


    float ProcessFilter(
        float input,
        float cutoff_hz,
        float q)
    {
        /*
         * Very low HP only on this parallel resonator input.
         */
        float hp_a =
            1.0f -
            expf(
                -TWO_PI *
                30.0f /
                SAMPLE_RATE
            );


        input_lp_state +=
            hp_a *
            (
                input -
                input_lp_state
            );


        float filtered_input =
            input -
            input_lp_state;


        cutoff_hz =
            ClampAdded(
                cutoff_hz,
                45.0f,
                400.0f
            );


        q =
            ClampAdded(
                q,
                0.55f,
                9.0f
            );


        float g =
            tanf(
                PI *
                cutoff_hz /
                SAMPLE_RATE
            );


        float k =
            1.0f /
            q;


        float a1 =
            1.0f /
            (
                1.0f +
                g *
                (
                    g +
                    k
                )
            );


        float v1 =
            a1 *
            (
                ic1eq +
                g *
                (
                    filtered_input -
                    ic2eq
                )
            );


        float v2 =
            ic2eq +
            g *
            v1;


        ic1eq =
            2.0f *
            v1 -
            ic1eq;


        ic2eq =
            2.0f *
            v2 -
            ic2eq;


        return v2;
    }


    float Process(float input)
    {
        if(!ENABLE_RESONANT_SWEEP_LAYER)
        {
            /*
             * Even global bypass is click-safe.
             */
            if(!active)
                return 0.0f;

            releasing_transition = true;
        }


        if(!active)
            return 0.0f;


        /*
         * ====================================================
         * SMOOTH GATE-DERIVED TIME CHANGES
         * ====================================================
         *
         * ~12 ms smoothing. This changes the sweep RATE gently.
         */
        float time_smooth_a =
            expf(
                -1.0f /
                (
                    SAMPLE_RATE *
                    0.012f
                )
            );


        sweep_ms =
            target_sweep_ms +
            (
                sweep_ms -
                target_sweep_ms
            ) *
            time_smooth_a;


        sweep_ms =
            ClampAdded(
                sweep_ms,
                RESONANT_SWEEP_MIN_MS,
                RESONANT_SWEEP_MAX_MS
            );


        /*
         * ====================================================
         * CONTINUOUS LINEAR FILTER SWEEP
         * ====================================================
         *
         * Current cutoff never jumps.
         *
         * Sweep slope = remaining nominal 120 Hz span / current
         * gate-derived duration.
         */
        float sweep_samples =
            sweep_ms *
            SAMPLE_RATE /
            1000.0f;


        if(sweep_samples < 1.0f)
            sweep_samples = 1.0f;


        float cutoff_step =
            (
                sweep_start_hz -
                sweep_end_hz
            )
            /
            sweep_samples;


        if(current_cutoff_hz >
           sweep_end_hz)
        {
            current_cutoff_hz -=
                cutoff_step;


            if(current_cutoff_hz <
               sweep_end_hz)
            {
                current_cutoff_hz =
                    sweep_end_hz;
            }
        }


        /*
         * Envelope progress also advances CONTINUOUSLY.
         * Changing gate time changes only this increment.
         */
        env_progress +=
            1.0f /
            sweep_samples;


        if(env_progress > 1.0f)
            env_progress = 1.0f;


        /*
         * PHASE-RESET RESONANT EXCITER
         * ----------------------------
         *
         * A very quiet sine follows the exact instantaneous resonant
         * frequency. It starts at phase zero on every kick, so there is
         * no free-running resonant element slowly drifting against the
         * rest of the kick.
         */
        float exciter = 0.0f;

        if(ENABLE_RESONANT_SWEEP_PHASE_LOCKED_EXCITER)
        {
            exciter =
                sinf(
                    TWO_PI *
                    exciter_phase
                ) *
                resonant_sweep_exciter_amount;

            exciter_phase +=
                current_cutoff_hz /
                SAMPLE_RATE;

            while(exciter_phase >= 1.0f)
                exciter_phase -= 1.0f;
        }


        float resonator_input =
            input +
            exciter;


        float filtered =
            ProcessFilter(
                resonator_input,
                current_cutoff_hz,
                resonant_sweep_q
            );


        float musical_amp =
            MusicalAmplitudeEnvelope();


        /*
         * ====================================================
         * DE-CLICK TRANSITION GAIN
         * ====================================================
         *
         * Attack ~4 ms.
         * Release ~4 ms.
         *
         * These are deliberately much shorter than the musical filter
         * sweep/envelope and should not be heard as an envelope shape.
         */
        if(!releasing_transition)
        {
            float attack_a =
                expf(
                    -5.0f /
                    (
                        SAMPLE_RATE *
                        0.0040f
                    )
                );


            transition_gain =
                1.0f +
                (
                    transition_gain -
                    1.0f
                ) *
                attack_a;
        }
        else
        {
            float release_a =
                expf(
                    -5.0f /
                    (
                        SAMPLE_RATE *
                        0.0040f
                    )
                );


            transition_gain =
                (
                    transition_gain
                ) *
                release_a;
        }


        float output =
            filtered *
            musical_amp *
            transition_gain *
            resonant_sweep_layer_gain;


        /*
         * ====================================================
         * CLICK-SAFE ENDING
         * ====================================================
         */

        if(ENABLE_RESONANT_SWEEP_AMP_ENVELOPE &&
           env_progress >= 1.0f)
        {
            /*
             * Musical envelope is already at zero, but still perform a
             * transition release for complete numerical safety.
             */
            releasing_transition = true;
        }


        if(!ENABLE_RESONANT_SWEEP_AMP_ENVELOPE &&
           current_cutoff_hz <=
           sweep_end_hz + 0.001f)
        {
            /*
             * No musical envelope: do NOT hard-stop at 80 Hz.
             * Start the smooth transition fade instead.
             */
            releasing_transition = true;
        }


        if(releasing_transition &&
           transition_gain <
           0.0001f)
        {
            transition_gain = 0.0f;

            active = false;
            releasing_transition = false;

            /*
             * State may now be safely cleared because it is inaudible.
             */
            ResetFilterState();
        }


        return output;
    }
};

static AddedResonantSweepLayer added_resonant_sweep_layer;


/* ============================================================
   PROTECTED PUNCH
   SAME OSCILLATOR ARCHITECTURE AS FIXED2
   ============================================================ */

struct AddedProtectedPunch
{
    float phase = 0.0f;

    float super_phase_local[SUPER_COUNT];

    float hp_state = 0.0f;

    float lp_state = 0.0f;
    float lp_state_2 = 0.0f;
    float lp_state_3 = 0.0f;

    uint32_t age_samples = 0;

    bool active = false;


    void Reset()
    {
        phase = 0.0f;

        for(int i = 0;
            i < SUPER_COUNT;
            i++)
        {
            super_phase_local[i] =
                super_phase_offset[i];
        }

        hp_state = 0.0f;

        lp_state = 0.0f;
        lp_state_2 = 0.0f;
        lp_state_3 = 0.0f;

        age_samples = 0;
        active = false;
    }


    void Trigger()
    {
        if(
            !kick_retrigger_active ||
            !PHASE_CONTINUOUS_RATCHET_RETRIGGER ||
            PSY_PHASE_LOCK_EVERY_HIT
        )
        {
            phase = 0.0f;

            for(int i = 0;
                i < SUPER_COUNT;
                i++)
            {
                super_phase_local[i] =
                    super_phase_offset[i];
            }

            hp_state = 0.0f;

            lp_state = 0.0f;
            lp_state_2 = 0.0f;
            lp_state_3 = 0.0f;
        }

        /*
         * Restart the musical knock envelope, not the live sine/filter
         * waveform state.
         */
        age_samples = 0;
        active = true;
    }


    float LowHarmonicOscillator(
        float frequency,
        float morph)
    {
        /*
         * Protected punch should be a PUNCH generator, not a laser.
         *
         * Fundamental sine + a small amount of 2nd/3rd harmonic gives
         * chest presence and audibility without a saw/square ladder.
         *
         * K6/morph still affects the harmonic amount slightly, but MIDI
         * velocity is reserved for tail pitch and cannot brighten this lane.
         */
        float fundamental =
            sinf(
                phase *
                TWO_PI
            );


        float second =
            sinf(
                phase *
                TWO_PI *
                2.0f
            );


        float third =
            sinf(
                phase *
                TWO_PI *
                3.0f
            );


        float harmonic_2 =
            SINE_ONLY_OSCILLATORS
            ? 0.0f
            : (
                0.05f +
                Clamp01Added(
                    morph
                ) *
                0.10f
              );


        float harmonic_3 =
            SINE_ONLY_OSCILLATORS
            ? 0.0f
            : (
                0.015f +
                Clamp01Added(
                    morph
                ) *
                0.035f
              );


        float result =
            fundamental +
            second *
            harmonic_2 +
            third *
            harmonic_3;


        /*
         * Keep roughly comparable RMS to the old protected lane.
         */
        result *= 0.90f;


        phase +=
            frequency /
            SAMPLE_RATE;


        while(phase >= 1.0f)
            phase -= 1.0f;


        return result;
    }


    float ComplexMorphOscillator(
        float frequency,
        float morph)
    {
        /*
         * OLD A/B path: exact fixed2 sine -> saw -> supersaw -> square.
         */
        float dt =
            frequency /
            SAMPLE_RATE;


        if(dt > 0.45f)
            dt = 0.45f;


        float sine =
            Sine(
                phase
            );


        float saw =
            Saw(
                phase,
                dt
            );


        float square =
            Square(
                phase,
                dt
            );


        float supersaw = 0.0f;


        for(int i = 0;
            i < SUPER_COUNT;
            i++)
        {
            float f =
                frequency *
                (
                    1.0f +
                    super_detune[i]
                );


            float sdt =
                f /
                SAMPLE_RATE;


            if(sdt > 0.45f)
                sdt = 0.45f;


            supersaw +=
                Saw(
                    super_phase_local[i],
                    sdt
                );
        }


        supersaw /=
            static_cast<float>(
                SUPER_COUNT
            );


        float result;


        if(morph < 0.333333f)
        {
            float t =
                morph /
                0.333333f;


            float a =
                cosf(
                    t *
                    1.5707963f
                );


            float b =
                sinf(
                    t *
                    1.5707963f
                );


            result =
                sine * a +
                saw * b;


            result *= 0.93f;
        }
        else if(morph < 0.666666f)
        {
            float t =
                (
                    morph -
                    0.333333f
                )
                /
                0.333333f;


            float a =
                cosf(
                    t *
                    1.5707963f
                );


            float b =
                sinf(
                    t *
                    1.5707963f
                );


            result =
                saw * a +
                supersaw * b;


            result *= 0.90f;
        }
        else
        {
            float t =
                (
                    morph -
                    0.666666f
                )
                /
                0.333334f;


            float a =
                cosf(
                    t *
                    1.5707963f
                );


            float b =
                sinf(
                    t *
                    1.5707963f
                );


            result =
                supersaw * a +
                square * b;


            result *= 0.86f;
        }


        phase += dt;


        if(phase >= 1.0f)
            phase -= 1.0f;


        for(int i = 0;
            i < SUPER_COUNT;
            i++)
        {
            float f =
                frequency *
                (
                    1.0f +
                    super_detune[i]
                );


            super_phase_local[i] +=
                f /
                SAMPLE_RATE;


            while(
                super_phase_local[i] >=
                1.0f
            )
            {
                super_phase_local[i] -=
                    1.0f;
            }
        }


        return result;
    }


    float Oscillator(
        float frequency,
        float morph)
    {
        frequency =
            ClampAdded(
                frequency,
                PROTECTED_PUNCH_LOW_HZ,
                PROTECTED_PUNCH_HIGH_HZ
            );


        if(PROTECTED_PUNCH_USE_COMPLEX_MORPH &&
           !SINE_ONLY_OSCILLATORS)
        {
            return
                ComplexMorphOscillator(
                    frequency,
                    morph
                );
        }


        return
            LowHarmonicOscillator(
                frequency,
                morph
            );
    }


    float Process(
        float source_frequency,
        float morph,
        float& envelope_out)
    {
        envelope_out = 0.0f;


        if(!ENABLE_PROTECTED_PUNCH ||
           !active)
        {
            return 0.0f;
        }


        float time_ms =
            static_cast<float>(
                age_samples
            ) *
            1000.0f /
            SAMPLE_RATE;


        if(time_ms >=
           PROTECTED_PUNCH_LENGTH_MS)
        {
            active = false;
            return 0.0f;
        }


        /*
         * Slightly gentler attack than before.
         *
         * The corrected global envelope attack already removes the main
         * click bug; this additionally makes the protected lane itself
         * impossible to start with a hard edge.
         */
        float attack =
            SmoothstepAdded(
                Clamp01Added(
                    time_ms /
                    4.00f
                )
            );


        float life =
            Clamp01Added(
                time_ms /
                PROTECTED_PUNCH_LENGTH_MS
            );


        float decay =
            0.5f +
            0.5f *
            cosf(
                PI *
                life
            );


        float envelope =
            attack *
            decay;


        envelope_out =
            envelope;


        float raw =
            Oscillator(
                source_frequency,
                morph
            );


        /*
         * One gentle HP pole keeps DC/sub-rumble out.
         */
        float hp_a =
            1.0f -
            expf(
                -TWO_PI *
                PROTECTED_PUNCH_LOW_HZ /
                SAMPLE_RATE
            );


        hp_state +=
            hp_a *
            (
                raw -
                hp_state
            );


        float highpassed =
            raw -
            hp_state;


        /*
         * THREE cascaded 95 Hz LP stages.
         *
         * The protected lane is now genuinely protected LOW-FREQUENCY
         * energy. Harmonics above the knock region are aggressively
         * removed before this lane is mixed back into the kick.
         */
        float lp_a =
            1.0f -
            expf(
                -TWO_PI *
                PROTECTED_PUNCH_HIGH_HZ /
                SAMPLE_RATE
            );


        lp_state +=
            lp_a *
            (
                highpassed -
                lp_state
            );


        lp_state_2 +=
            lp_a *
            (
                lp_state -
                lp_state_2
            );


        lp_state_3 +=
            lp_a *
            (
                lp_state_2 -
                lp_state_3
            );


        age_samples++;


        return
            lp_state_3 *
            envelope *
            PROTECTED_PUNCH_GAIN *
            KICK_KNOCK_GAIN;
    }
};


/* ============================================================
   PERFORMANCE FX AGGREGATOR
   ============================================================ */

struct AddedPerformanceFx
{
    /*
     * External-only:
     * pump + delay
     */
    AddedPump pump;
    AddedClockedDelay delay;

    /*
     * Kick-side:
     * STUTTER + LOOPER + HPF ONLY
     */
    AddedStutter stutter;
    AddedQuantizedLooper looper;
    AddedDjHighpass dj_hpf;

    /*
     * Digitakt-side lightweight copies:
     * STUTTER + HPF + LPF.
     *
     * No second looper/history buffer is allocated.
     */
    AddedStutter external_stutter;
    AddedDjHighpass external_dj_hpf;


    /*
     * Ghost-pump timing is owned by AddedPump::RecentlyTriggered().
     * There is deliberately no "did a kick happen last quarter?" flag:
     * that logic caused one full unpumped hole when the kick stopped.
     */

    /*
     * Shared rolling pre-FX master history.
     *
     * 43200 samples = 900 ms at 48 kHz. This replaces the old separate
     * stutter + looper buffers (14400 + 28800 = exactly 43200 floats),
     * so SRAM usage does not grow.
     */
    static constexpr uint32_t MASTER_HISTORY_SAMPLES = 43200;
    float master_history[MASTER_HISTORY_SAMPLES];
    uint32_t master_history_write = 0;


    void Reset()
    {
        pump.Reset();
        delay.Reset();
        stutter.Reset();
        looper.Reset();
        dj_hpf.Reset();
        macro_dj_lowpass.Reset();

        external_stutter.Reset();
        external_dj_hpf.Reset();
        external_macro_dj_lowpass.Reset();

        for(uint32_t i = 0;
            i < MASTER_HISTORY_SAMPLES;
            ++i)
        {
            master_history[i] = 0.0f;
        }

        master_history_write = 0;
    }


    void TriggerKick()
    {
        /*
         * Real audible kick drives the external sidechain envelope using
         * the SAME K6 sweep duration as the kick itself.
         */
        pump.TriggerWithSweepMs(
            transient_pitch_time *
            1000.0f
        );
    }


    void OnSixteenth()
    {
        /*
         * CHOP rate is clock-derived, but state changes are now tied
         * to kick boundaries.
         */
    }


    void OnQuarter()
    {
        /*
         * ====================================================
         * GAPLESS GHOST PUMP
         * ====================================================
         *
         * Every quarter-note clock boundary is eligible for a ghost.
         *
         * If a real kick already triggered the pump within ~30 ms of this
         * SAME boundary, suppress only that duplicate trigger.
         *
         * This fixes the old logical hole:
         *
         *     last beat had real kick
         *         -> old code suppressed next ghost
         *     kick then stops
         *         -> one beat of full Digitakt audio leaks through
         *
         * There is no previous-quarter state anymore.
         */
        if(
            macro_pump_enabled &&
            macro_fx_value_pump > 0.005f &&
            !pump.RecentlyTriggered(
                30.0f
            )
        )
        {
            pump.TriggerWithSweepMs(
                MacroKickShapeSweepSeconds(
                    macro_kick_shape
                )
                *
                1000.0f
            );
        }
    }


    void OnKickBoundary()
    {
        /*
         * AUDIO THREAD ONLY.
         *
         * Atomic-ish aligned 32-bit mailbox read/clear. The control
         * thread never touches the live DSP objects.
         */
        uint32_t stutter_command =
            stutter_quantized_command;


        if(stutter_command !=
           QUANT_FX_NONE)
        {
            stutter_quantized_command =
                QUANT_FX_NONE;

            stutter.ApplyKickQuantizedCommand(
                stutter_command,
                perf_quarter_note_ms,
                master_history_write
            );


            external_stutter.ApplyKickQuantizedCommand(
                stutter_command,
                perf_quarter_note_ms,
                0
            );
        }


        uint32_t looper_command =
            looper_quantized_command;


        if(looper_command !=
           QUANT_FX_NONE)
        {
            looper_quantized_command =
                QUANT_FX_NONE;

            looper.ApplyKickQuantizedCommand(
                looper_command,
                perf_quarter_note_ms,
                master_history_write,
                master_history
            );
        }
    }


    void ServiceImmediateSafety()
    {
        /*
         * Transport-stop failsafe: if there will be no next kick, a
         * FORCE_DISABLE request must still be consumed.
         */
        uint32_t stutter_command =
            stutter_quantized_command;


        if(
            (
                stutter_command &
                0xFFu
            )
            ==
            QUANT_FX_FORCE_DISABLE
        )
        {
            stutter_quantized_command =
                QUANT_FX_NONE;

            stutter.ApplyKickQuantizedCommand(
                stutter_command,
                perf_quarter_note_ms,
                master_history_write
            );


            external_stutter.ApplyKickQuantizedCommand(
                stutter_command,
                perf_quarter_note_ms,
                0
            );
        }


        uint32_t looper_command =
            looper_quantized_command;


        if(
            (
                looper_command &
                0xFFu
            )
            ==
            QUANT_FX_FORCE_DISABLE
        )
        {
            looper_quantized_command =
                QUANT_FX_NONE;

            looper.ApplyKickQuantizedCommand(
                looper_command,
                perf_quarter_note_ms,
                master_history_write,
                master_history
            );
        }
    }


    float ProcessExternal(float input)
    {
        /*
         * Digitakt path:
         *
         * PUMP + DELAY remain external-only.
         * CHOP + HPF + LPF use cheap independent DSP state.
         *
         * LOOPER remains kick-only in this memory-safe build because a
         * genuinely isolated second looper requires another large history
         * buffer.
         */
        float x =
            pump.Process(
                input,
                perf_quarter_note_ms
            );


        x =
            delay.Process(
                x,
                perf_quarter_note_ms
            );


        x =
            external_stutter.Process(
                x,
                nullptr
            );


        x =
            external_dj_hpf.Process(
                x,
                false
            );


        x =
            external_macro_dj_lowpass.Process(
                x
            );


        return x;
    }


    float ProcessMaster(float input)
    {
        /*
         * ====================================================
         * SHARED HISTORY / TRUE LOOPER FREEZE
         * ====================================================
         *
         * CHOP is now a LIVE non-sampling processor, so LOOPER is the
         * only effect that needs this history memory.
         *
         * IMPORTANT BUG FIX:
         *
         * The previous "looper" kept writing new audio into the same
         * circular history buffer it was reading as a supposedly frozen
         * loop. Eventually the write head entered the captured region and
         * mutated samples underneath the playback head, causing random
         * ticks/glitches even when the seam itself was crossfaded.
         *
         * While LOOPER is active (including its release fade), STOP the
         * history writer completely. The capture therefore becomes truly
         * immutable.
         *
         * Once the looper is fully inactive, rolling history recording
         * resumes automatically.
         */
        if(!looper.active)
        {
            master_history[
                master_history_write
            ] =
                input;


            master_history_write++;


            if(master_history_write >=
               MASTER_HISTORY_SAMPLES)
            {
                master_history_write = 0;
            }
        }


        float x =
            stutter.Process(
                input,
                master_history
            );


        x =
            looper.Process(
                x,
                master_history
            );


        x =
            dj_hpf.Process(
                x,
                true
            );


        /*
         * Kick performance chain deliberately ends here:
         *
         *     STUTTER -> LOOPER -> HPF
         *
         * LPF, pump and delay are external-input effects only.
         */
        return x;
    }
};

constexpr uint32_t AddedPerformanceFx::MASTER_HISTORY_SAMPLES;

static AddedPerformanceFx added_performance_fx;
static AddedProtectedPunch added_protected_punch;


/* ============================================================
   OLED BUFFER
   ============================================================ */

static uint8_t oled_buffer[128 * 64 / 8];

static uint8_t DMA_BUFFER_MEM_SECTION oled_dma_packet[129];
static uint8_t DMA_BUFFER_MEM_SECTION oled_cmd_packet[2];


enum class OledTransfer
{
    IDLE,
    PAGE_ADDRESS,
    COLUMN_LOW,
    COLUMN_HIGH,
    DATA
};


static OledTransfer oled_transfer =
    OledTransfer::IDLE;


static bool oled_dma_done = false;
static bool oled_dma_error = false;
static bool oled_dirty = false;

static uint8_t oled_page = 0;


/* ============================================================
   OLED COMMANDS
   ============================================================ */

static void OledSendCommandBlocking(uint8_t command)
{
    uint8_t packet[2];

    packet[0] = 0x00;
    packet[1] = command;


    oled_i2c.TransmitBlocking(
        OLED_ADDRESS,
        packet,
        2,
        100
    );
}


static void OledSendCommand2Blocking(
    uint8_t command,
    uint8_t value)
{
    uint8_t packet[3];

    packet[0] = 0x00;
    packet[1] = command;
    packet[2] = value;


    oled_i2c.TransmitBlocking(
        OLED_ADDRESS,
        packet,
        3,
        100
    );
}


/* ============================================================
   OLED INITIALIZATION
   ============================================================ */

static void InitOled()
{
    I2CHandle::Config cfg;


    cfg.periph =
        I2CHandle::Config::Peripheral::I2C_1;


    cfg.speed =
        I2CHandle::Config::Speed::I2C_1MHZ;


    cfg.mode =
        I2CHandle::Config::Mode::I2C_MASTER;


    cfg.pin_config.scl =
        hw.GetPin(11);


    cfg.pin_config.sda =
        hw.GetPin(12);


    oled_i2c.Init(cfg);


    /*
     * WORKING SSD1309 INITIALIZATION.
     *
     * DO NOT ADD:
     *
     *     0x20, 0x00
     *
     * Runtime updates use page addressing.
     */

    OledSendCommandBlocking(0xAE);

    OledSendCommand2Blocking(0xD5, 0x80);

    OledSendCommand2Blocking(0xA8, 0x3F);

    OledSendCommand2Blocking(0xDA, 0x12);

    OledSendCommand2Blocking(0xD3, 0x00);

    OledSendCommandBlocking(0x40);

    OledSendCommandBlocking(0xA6);

    OledSendCommandBlocking(0xA4);

    OledSendCommand2Blocking(0x8D, 0x14);

    OledSendCommandBlocking(0xA1);

    OledSendCommandBlocking(0xC8);

    OledSendCommand2Blocking(0x81, 0x8F);

    OledSendCommand2Blocking(0xD9, 0x25);

    OledSendCommand2Blocking(0xDB, 0x34);

    OledSendCommandBlocking(0xAF);


    for(size_t i = 0;
        i < sizeof(oled_buffer);
        i++)
    {
        oled_buffer[i] = 0;
    }


    oled_dirty = true;
}


/* ============================================================
   OLED DMA CALLBACK
   ============================================================ */

static void OledDmaCallback(
    void*,
    I2CHandle::Result result)
{
    if(result ==
       I2CHandle::Result::OK)
    {
        oled_dma_done = true;
    }
    else
    {
        oled_dma_error = true;

        oled_transfer =
            OledTransfer::IDLE;
    }
}


/* ============================================================
   OLED DMA TRANSFERS
   ============================================================ */

static void StartOledPage(uint8_t page)
{
    oled_cmd_packet[0] = 0x00;
    oled_cmd_packet[1] =
        0xB0 | page;


    oled_transfer =
        OledTransfer::PAGE_ADDRESS;


    oled_dma_done = false;
    oled_dma_error = false;


    oled_i2c.TransmitDma(
        OLED_ADDRESS,
        oled_cmd_packet,
        2,
        OledDmaCallback,
        nullptr
    );
}


static void StartOledData()
{
    oled_dma_packet[0] = 0x40;


    for(int i = 0; i < 128; i++)
    {
        oled_dma_packet[i + 1] =
            oled_buffer[
                oled_page * 128 + i
            ];
    }


    oled_transfer =
        OledTransfer::DATA;


    oled_i2c.TransmitDma(
        OLED_ADDRESS,
        oled_dma_packet,
        129,
        OledDmaCallback,
        nullptr
    );
}


/* ============================================================
   OLED DMA SERVICE
   ============================================================ */

static void ServiceOledTransfer()
{
    if(oled_transfer ==
       OledTransfer::IDLE)
        return;


    if(oled_dma_error)
    {
        oled_transfer =
            OledTransfer::IDLE;

        oled_dma_error = false;

        return;
    }


    if(!oled_dma_done)
        return;


    oled_dma_done = false;


    switch(oled_transfer)
    {
        case OledTransfer::PAGE_ADDRESS:
        {
            oled_cmd_packet[0] = 0x00;
            oled_cmd_packet[1] = 0x00;


            oled_transfer =
                OledTransfer::COLUMN_LOW;


            oled_i2c.TransmitDma(
                OLED_ADDRESS,
                oled_cmd_packet,
                2,
                OledDmaCallback,
                nullptr
            );

            break;
        }


        case OledTransfer::COLUMN_LOW:
        {
            oled_cmd_packet[0] = 0x00;
            oled_cmd_packet[1] = 0x10;


            oled_transfer =
                OledTransfer::COLUMN_HIGH;


            oled_i2c.TransmitDma(
                OLED_ADDRESS,
                oled_cmd_packet,
                2,
                OledDmaCallback,
                nullptr
            );

            break;
        }


        case OledTransfer::COLUMN_HIGH:
        {
            StartOledData();

            break;
        }


        case OledTransfer::DATA:
        {
            oled_page++;


            if(oled_page >= 8)
            {
                oled_page = 0;

                oled_transfer =
                    OledTransfer::IDLE;


                /*
                 * If another MIDI event changed the display
                 * while we were transmitting, oled_dirty will
                 * be set again.
                 */
                oled_dirty = false;
            }
            else
            {
                StartOledPage(oled_page);
            }

            break;
        }


        case OledTransfer::IDLE:
        default:
            break;
    }
}


/* ============================================================
   OLED FONT
   ============================================================ */

static const uint8_t font5x7[][5] =
{
    {0x7E,0x11,0x11,0x11,0x7E},
    {0x7F,0x49,0x49,0x49,0x36},
    {0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},
    {0x7F,0x49,0x49,0x49,0x41},
    {0x7F,0x09,0x09,0x09,0x01},
    {0x3E,0x41,0x49,0x49,0x7A},
    {0x7F,0x08,0x08,0x08,0x7F},
    {0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},
    {0x7F,0x08,0x14,0x22,0x41},
    {0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x0C,0x02,0x7F},
    {0x7F,0x04,0x08,0x10,0x7F},
    {0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},
    {0x3E,0x41,0x51,0x21,0x5E},
    {0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},
    {0x01,0x01,0x7F,0x01,0x01},
    {0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},
    {0x3F,0x40,0x38,0x40,0x3F},
    {0x63,0x14,0x08,0x14,0x63},
    {0x07,0x08,0x70,0x08,0x07},
    {0x61,0x51,0x49,0x45,0x43},

    {0x3E,0x45,0x49,0x51,0x3E},
    {0x00,0x21,0x7F,0x01,0x00},
    {0x23,0x45,0x49,0x51,0x31},
    {0x42,0x41,0x51,0x69,0x46},
    {0x0C,0x14,0x24,0x7F,0x04},
    {0x72,0x51,0x51,0x51,0x4E},
    {0x1E,0x29,0x49,0x49,0x06},
    {0x40,0x47,0x48,0x50,0x60},
    {0x36,0x49,0x49,0x49,0x36},
    {0x30,0x49,0x49,0x4A,0x3C},

    {0x00,0x00,0x00,0x00,0x00},
    {0x08,0x08,0x08,0x08,0x08},
    {0x00,0x36,0x36,0x00,0x00}
};


static int FontIndex(char c)
{
    if(c >= 'A' && c <= 'Z')
        return c - 'A';

    if(c >= '0' && c <= '9')
        return 26 + c - '0';

    if(c == ' ')
        return 36;

    if(c == '-')
        return 37;

    if(c == ':')
        return 38;

    return 36;
}


static void OledDrawChar(
    uint8_t x,
    uint8_t page,
    char c)
{
    if(x > 122 || page > 7)
        return;


    int index =
        FontIndex(c);


    for(int i = 0; i < 5; i++)
    {
        oled_buffer[
            page * 128 +
            x + i
        ] =
            font5x7[index][i];
    }
}


static void OledDrawString(
    uint8_t x,
    uint8_t page,
    const char* text)
{
    while(*text && x < 123)
    {
        OledDrawChar(
            x,
            page,
            *text
        );

        x += 6;
        text++;
    }
}


static void OledClear()
{
    for(size_t i = 0;
        i < sizeof(oled_buffer);
        i++)
    {
        oled_buffer[i] = 0;
    }
}


/* ============================================================
   OLED NUMBER / NOTE HELPERS
   ============================================================ */

static void NumberToString(
    uint32_t value,
    char* output,
    int digits)
{
    for(int i = digits - 1;
        i >= 0;
        i--)
    {
        output[i] =
            '0' +
            value % 10;

        value /= 10;
    }


    output[digits] = '\0';
}


static void NoteToString(
    uint8_t note,
    char* output)
{
    static const char* names[] =
    {
        "C", "C-", "D", "D-", "E", "F",
        "F-", "G", "G-", "A", "A-", "B"
    };


    uint8_t n =
        note % 12;


    int octave =
        static_cast<int>(
            note / 12
        ) - 1;


    output[0] =
        names[n][0];


    if(names[n][1] == '-')
    {
        output[1] = '-';
        output[2] =
            '0' + octave;
        output[3] = '\0';
    }
    else
    {
        output[1] =
            '0' + octave;
        output[2] = '\0';
    }
}


/* ============================================================
   OLED SCREEN
   ============================================================ */

static const char* MacroFxModeName()
{
    switch(macro_fx_mode)
    {
        case MacroFxMode::STUTTER:
            return "CHOP";

        case MacroFxMode::LOOPER:
            return "LOOP";

        case MacroFxMode::DELAY:
            return "DLY";

        case MacroFxMode::DJ_HPF:
            return "HPF";

        case MacroFxMode::DJ_LPF:
            return "LPF";

        case MacroFxMode::PUMP:
            return "PUMP";

        case MacroFxMode::COUNT:
        default:
            return "FX";
    }
}


static void PrepareMidiScreen()
{
    OledClear();


    if(midi_running)
        OledDrawString(
            0,
            0,
            "MIDI RUN"
        );
    else
        OledDrawString(
            0,
            0,
            "MIDI STOP"
        );


    /*
     * Macro-1 selected FX page.
     */
    OledDrawString(
        0,
        1,
        "FX:"
    );


    OledDrawString(
        24,
        1,
        MacroFxModeName()
    );


    char text[16];


    /*
     * NOTE
     */
    char note_name[8];

    NoteToString(
        last_note,
        note_name
    );


    OledDrawString(
        0,
        2,
        "NOTE:"
    );


    OledDrawString(
        36,
        2,
        note_name
    );


    /*
     * MIDI NUMBER
     */
    NumberToString(
        last_note,
        text,
        3
    );


    OledDrawString(
        0,
        3,
        "NUM:"
    );


    OledDrawString(
        30,
        3,
        text
    );


    /*
     * VELOCITY
     */
    NumberToString(
        last_velocity,
        text,
        3
    );


    OledDrawString(
        0,
        4,
        "VEL:"
    );


    OledDrawString(
        30,
        4,
        text
    );


    /*
     * GATE
     */
    OledDrawString(
        0,
        5,
        "GATE:"
    );


    if(note_gate)
        OledDrawString(
            36,
            5,
            "ON"
        );
    else
        OledDrawString(
            36,
            5,
            "OFF"
        );


    /*
     * CHANNEL 16 LAYER
     */
    OledDrawString(
        0,
        6,
        "L16:"
    );


    if(layer16_active)
        OledDrawString(
            30,
            6,
            "PAT"
        );
    else
        OledDrawString(
            30,
            6,
            "OFF"
        );


    /*
     * Macro-4 / Macro-5 status.
     */
    OledDrawString(
        0,
        7,
        "BPF:"
    );


    NumberToString(
        macro_bpf_layer_count,
        text,
        1
    );


    OledDrawString(
        30,
        7,
        text
    );


    OledDrawString(
        42,
        7,
        macro_character_sherman
        ? "SHER"
        : "MACK"
    );


    oled_dirty = true;
}


/* ============================================================
   MIDI UART INITIALIZATION
   ============================================================ */

static void InitMidiUart()
{
    /*
     * EXACT WORKING ARCHITECTURE:
     *
     * Daisy physical D1
     *       |
     *      PC11
     *       |
     *    USART3 RX
     *
     * 31250 baud / 8N1
     */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();


    GPIO_InitTypeDef gpio = {};


    gpio.Pin =
        GPIO_PIN_11;


    gpio.Mode =
        GPIO_MODE_AF_PP;


    gpio.Pull =
        GPIO_PULLUP;


    gpio.Speed =
        GPIO_SPEED_FREQ_VERY_HIGH;


    gpio.Alternate =
        GPIO_AF7_USART3;


    HAL_GPIO_Init(
        GPIOC,
        &gpio
    );


    midi_uart.Instance =
        USART3;


    midi_uart.Init.BaudRate =
        31250;


    midi_uart.Init.WordLength =
        UART_WORDLENGTH_8B;


    midi_uart.Init.StopBits =
        UART_STOPBITS_1;


    midi_uart.Init.Parity =
        UART_PARITY_NONE;


    midi_uart.Init.Mode =
        UART_MODE_RX;


    midi_uart.Init.HwFlowCtl =
        UART_HWCONTROL_NONE;


    midi_uart.Init.OverSampling =
        UART_OVERSAMPLING_16;


    midi_uart.Init.OneBitSampling =
        UART_ONE_BIT_SAMPLE_DISABLE;


    midi_uart.AdvancedInit.AdvFeatureInit =
        UART_ADVFEATURE_NO_INIT;


    if(HAL_UART_Init(&midi_uart) != HAL_OK)
    {
        hw.SetLed(true);

        while(1)
        {
        }
    }


    while(
        USART3->ISR &
        USART_ISR_RXNE_RXFNE
    )
    {
        (void)USART3->RDR;
    }
}


/* ============================================================
   MIDI NOTE ON
   ============================================================ */

static void HandleKickNoteOff(uint8_t note);


static void HandleKickNoteOn(
    uint8_t note,
    uint8_t velocity)
{
    last_note = note;
    last_velocity = velocity;

    /*
     * MIDI convention:
     * Note On with velocity 0 is Note Off.
     *
     * Do not use goto here: the normal Note Off handler already
     * contains the complete gate/separation/release logic.
     */
    if(velocity == 0)
    {
        HandleKickNoteOff(note);
        return;
    }

    /*
     * --------------------------------------------------------
     * NOTE = FUNDAMENTAL
     * --------------------------------------------------------
     */
    float f = MidiNoteToFrequency(note);

    /*
     * Keep kick fundamentals in a useful range.
     */
    if(f < 25.0f)
        f = 25.0f;

    if(f > 130.0f)
        f = 130.0f;

    kick_frequency = f;

    /*
     * --------------------------------------------------------
     * VELOCITY = BIPOLAR TAIL PITCH
     * --------------------------------------------------------
     *
     *      1 = one octave down
     *     64 = flat
     *    127 = one octave up
     *
     * Waveform/timbre is decoupled from velocity. In the current
     * sine-only build this remains pure sine; if multi-wave mode is
     * restored later, K6/SHAPE owns morph instead.
     */
    oscillator_morph =
        SINE_ONLY_OSCILLATORS
        ? 0.0f
        : Clamp01Added(
              macro_kick_shape
          );

    /*
     * --------------------------------------------------------
     * PATTERN GENERATION
     * --------------------------------------------------------
     *
     * Only generate when the MIDI note VALUE changes.
     */
    if(last_pattern_note != static_cast<int>(note))
    {
        GenerateCharacterPattern();
        last_pattern_note = static_cast<int>(note);
    }

    /*
     * Gate starts now.
     */
    note_on_time = System::GetNow();
    note_gate = true;
    trigger_count++;

    /*
     * Kick starts immediately.
     */
    kick_trigger_pending = true;

    PrepareMidiScreen();
}


/* ============================================================
   MIDI NOTE OFF
   ============================================================ */

static void HandleKickNoteOff(
    uint8_t note)
{
    (void)note;


    uint32_t now =
        System::GetNow();


    uint32_t duration =
        now - note_on_time;


    if(duration < 5)
        duration = 5;


    if(duration > 2000)
        duration = 2000;


    current_gate_ms =
        static_cast<float>(
            duration
        );


    /*
     * Feed the newly measured gate time into the experimental resonant
     * sweep / amplitude-envelope timing.
     */
    added_resonant_sweep_layer.UpdateCurrentGate(
        current_gate_ms
    );


    /*
     * Gate duration no longer controls amplitude, tail pitch, tail level,
     * temporal separation, or release.
     *
     * It is retained only for the currently-disabled experimental
     * resonant-sweep timing and UI/debug visibility.
     */
    note_gate = false;


    kick_release_pending = true;


    PrepareMidiScreen();
}


/* ============================================================
   CHANNEL 16 LAYER
   ============================================================ */

static void HandleLayer16NoteOn(
    uint8_t note,
    uint8_t velocity)
{
    (void)note;


    if(velocity == 0)
    {
        layer16_active = false;
        layer16_level = 0.0f;

        PrepareMidiScreen();

        return;
    }


    /*
     * Channel 16 currently acts as the first secondary
     * performance function:
     *
     * QUANTISED CHARACTER FILTER PATTERN
     *
     * It does not replace the kick.
     *
     * It adds the evolving character layer on top.
     */
    layer16_active = true;


    /*
     * Keep its level controlled.
     */
    layer16_level =
        0.12f +
        (
            static_cast<float>(velocity)
            / 127.0f
        ) * 0.28f;


    PrepareMidiScreen();
}


static void HandleLayer16NoteOff(
    uint8_t note)
{
    (void)note;


    layer16_active = false;
    layer16_level = 0.0f;


    PrepareMidiScreen();
}


/* ============================================================
   ADDED PERFORMANCE CC HANDLER
   ============================================================ */

static bool AddedCcOn(uint8_t value)
{
    return
        value >= 64;
}



static float MacroFxStoredValue(MacroFxMode mode)
{
    switch(mode)
    {
        case MacroFxMode::STUTTER:
            return macro_fx_value_stutter;
        case MacroFxMode::LOOPER:
            return macro_fx_value_looper;
        case MacroFxMode::DELAY:
            return macro_fx_value_delay;
        case MacroFxMode::DJ_HPF:
            return macro_fx_value_hpf;
        case MacroFxMode::DJ_LPF:
            return macro_fx_value_lpf;
        case MacroFxMode::PUMP:
            return macro_fx_value_pump;
        case MacroFxMode::COUNT:
        default:
            return 0.0f;
    }
}


static void MacroFxBeginPickup()
{
    macro_fx_pickup_active = true;
    macro_fx_pickup_start_raw = macro_fx_last_raw;
}


static bool MacroFxPickupAllows(uint8_t raw)
{
    if(!macro_fx_pickup_active)
        return true;

    int target =
        static_cast<int>(
            MacroFxStoredValue(
                macro_fx_mode
            ) *
            127.0f +
            0.5f
        );

    int now = static_cast<int>(raw);
    int previous = static_cast<int>(macro_fx_last_raw);

    int difference =
        now - target;

    if(difference < 0)
        difference = -difference;


    bool close =
        difference <=
        static_cast<int>(
            MACRO_FX_PICKUP_TOLERANCE
        );

    bool crossed =
        (
            previous <= target &&
            now >= target
        )
        ||
        (
            previous >= target &&
            now <= target
        );

    if(close || crossed)
    {
        macro_fx_pickup_active = false;
        return true;
    }

    return false;
}


/*
 * Zero means a real dry/off state for each K1 effect.
 * These are deliberately lightweight state clears: no giant buffer is
 * memset from the MIDI thread.
 */
static uint32_t MakeQuantFxCommand(
    uint8_t action,
    uint8_t rate_index)
{
    return
        static_cast<uint32_t>(
            action
        )
        |
        (
            static_cast<uint32_t>(
                rate_index
            )
            <<
            8
        );
}


static uint8_t StutterRateFromCc(
    uint8_t raw)
{
    /*
     * Explicit addressable encoding. 0 is OFF.
     *
     * Teensy should normally send the centre values:
     *   10  = 1/2
     *   28  = 1/4
     *   46  = 1/8
     *   64  = 1/16
     *   82  = 1/32
     *   100 = 1/64
     *   118 = 1/128
     */
    if(raw <= 18)  return 0;
    if(raw <= 36)  return 1;
    if(raw <= 54)  return 2;
    if(raw <= 72)  return 3;
    if(raw <= 90)  return 4;
    if(raw <= 108) return 5;
    return 6;
}


static uint8_t LooperRateFromCc(
    uint8_t raw)
{
    /*
     * 0 is OFF.
     *
     * Teensy centre values:
     *   13  = 1/2
     *   38  = 1/4
     *   63  = 1/8
     *   88  = 1/16
     *   114 = 1/32
     */
    if(raw <= 25)  return 0;
    if(raw <= 50)  return 1;
    if(raw <= 75)  return 2;
    if(raw <= 100) return 3;
    return 4;
}


static void ClearStutterMacro()
{
    macro_fx_value_stutter = 0.0f;

    stutter_quantized_command =
        MakeQuantFxCommand(
            QUANT_FX_DISABLE,
            0
        );
}


static void ClearLooperMacro()
{
    macro_fx_value_looper = 0.0f;

    looper_quantized_command =
        MakeQuantFxCommand(
            QUANT_FX_DISABLE,
            0
        );
}


static void ClearDelayMacro()
{
    macro_fx_value_delay = 0.0f;
    PERF_CLOCKED_DELAY_ENABLED = false;
}


static void ClearHpfMacro()
{
    macro_fx_value_hpf = 0.0f;
    PERF_DJ_HPF_ENABLED = false;
}


static void ClearLpfMacro()
{
    macro_fx_value_lpf = 0.0f;
}


static void SetMacroStutterRaw(
    uint8_t raw)
{
    macro_fx_mode =
        MacroFxMode::STUTTER;


    bool was_off =
        macro_fx_value_stutter <=
        0.005f;


    bool now_on =
        raw >
        0;


    macro_fx_value_stutter =
        static_cast<float>(
            raw
        )
        /
        127.0f;


    if(!now_on)
    {
        stutter_quantized_command =
            MakeQuantFxCommand(
                QUANT_FX_DISABLE,
                0
            );

        return;
    }


    uint8_t rate =
        StutterRateFromCc(
            raw
        );


    if(was_off)
    {
        stutter_quantized_command =
            MakeQuantFxCommand(
                QUANT_FX_ENABLE,
                rate
            );
    }
    else
    {
        /*
         * CC value is now an explicit rate address. If it enters a new
         * division zone, change rate at the next kick boundary.
         */
        stutter_quantized_command =
            MakeQuantFxCommand(
                QUANT_FX_RATE_CHANGE,
                rate
            );
    }
}


static void SetMacroLooperRaw(
    uint8_t raw)
{
    macro_fx_mode =
        MacroFxMode::LOOPER;


    bool was_off =
        macro_fx_value_looper <=
        0.005f;


    bool now_on =
        raw >
        0;


    macro_fx_value_looper =
        static_cast<float>(
            raw
        )
        /
        127.0f;


    if(!now_on)
    {
        looper_quantized_command =
            MakeQuantFxCommand(
                QUANT_FX_DISABLE,
                0
            );

        return;
    }


    uint8_t rate =
        LooperRateFromCc(
            raw
        );


    if(was_off)
    {
        looper_quantized_command =
            MakeQuantFxCommand(
                QUANT_FX_ENABLE,
                rate
            );
    }
    else
    {
        looper_quantized_command =
            MakeQuantFxCommand(
                QUANT_FX_RATE_CHANGE,
                rate
            );
    }
}


static void SetMacroDelay(float v)
{
    v =
        Clamp01Added(
            v
        );


    macro_fx_mode =
        MacroFxMode::DELAY;


    macro_fx_value_delay =
        v;


    PERF_CLOCKED_DELAY_ENABLED =
        v >
        0.005f;
}


static void SetMacroHpf(float v)
{
    v =
        Clamp01Added(
            v
        );


    macro_fx_mode =
        MacroFxMode::DJ_HPF;


    macro_fx_value_hpf =
        v;


    added_performance_fx.dj_hpf.position =
        v;


    added_performance_fx.external_dj_hpf.position =
        v;


    PERF_DJ_HPF_ENABLED =
        v >
        0.005f;
}


static void SetMacroLpf(float v)
{
    v =
        Clamp01Added(
            v
        );


    macro_fx_mode =
        MacroFxMode::DJ_LPF;


    macro_fx_value_lpf =
        v;
}


static void SetMacroPump(float v)
{
    v =
        Clamp01Added(
            v
        );


    macro_fx_mode =
        MacroFxMode::PUMP;


    /*
     * DSP itself slews smoothly back to unity when this becomes zero.
     */
    macro_fx_value_pump =
        v;
}


static void ClearAllK1FxExceptPump()
{
    ClearStutterMacro();
    ClearLooperMacro();
    ClearDelayMacro();
    ClearHpfMacro();
    ClearLpfMacro();

    /*
     * Pump state/value deliberately survives.
     */
}


/*
 * ============================================================
 * EMERGENCY B1 + B3 REBOOT
 * ============================================================
 *
 * Runs only in the main/control loop.
 *
 * Requirement:
 *     B1 raw physical state held
 *     +
 *     B3 raw physical state held
 *     continuously for > 1 second
 *
 * One of the buttons being released immediately cancels the timer.
 *
 * NVIC_SystemReset() requests an STM32 system reset through CMSIS.
 * No DSP state is manually torn down first; this is intentionally a
 * hard emergency restart.
 */
static void ServiceEmergencyReboot()
{
    if(
        emergency_b1_down &&
        emergency_b3_down
    )
    {
        uint32_t now =
            System::GetNow();


        if(!emergency_combo_timing)
        {
            emergency_combo_timing = true;
            emergency_combo_start_time = now;

            return;
        }


        if(
            now -
            emergency_combo_start_time >=
            EMERGENCY_REBOOT_HOLD_MS
        )
        {
            /*
             * Make the request once. NVIC_SystemReset() should never
             * return on target hardware.
             */
            emergency_combo_timing = false;


            NVIC_SystemReset();


            /*
             * Defensive fallback in the impossible event reset is
             * delayed or the call unexpectedly returns.
             */
            while(1)
            {
            }
        }


        return;
    }


    /*
     * Combo broken before one second -> completely cancel.
     * A later B1+B3 hold must start a fresh full one-second timer.
     */
    emergency_combo_timing = false;
    emergency_combo_start_time = 0;
}


/*
 * Called continuously from the main loop so the long-press action occurs
 * at 500 ms rather than only after the button is released.
 */
static void ServiceMacroFxButtonHold()
{
    if(!macro_fx_button_down ||
       macro_fx_long_press_fired)
    {
        return;
    }


    /*
     * B1 + physical B3 is reserved for emergency reboot.
     *
     * While B3 is physically held, do NOT fire B1's normal 500 ms
     * "clear K1 FX" action on the way to the one-second emergency hold.
     */
    if(emergency_b3_down)
    {
        return;
    }

    uint32_t now = System::GetNow();

    if(now - macro_fx_button_down_time >=
       MACRO_FX_LONG_PRESS_MS)
    {
        ClearAllK1FxExceptPump();

        macro_fx_long_press_fired = true;

        /*
         * Force pickup on the current page because its stored value may
         * just have been reset to zero.
         */
        MacroFxBeginPickup();

        PrepareMidiScreen();
    }
}


static bool HandleSixMacroCC(
    uint8_t cc,
    uint8_t value)
{
    float v =
        static_cast<float>(
            value
        )
        /
        127.0f;


    switch(cc)
    {
        /* ====================================================
           EMERGENCY RAW BUTTON STATE — NO MUSICAL FUNCTION
           ==================================================== */

        case CC_EMERGENCY_BUTTON1_RAW:
        {
            emergency_b1_down =
                value >= 64;


            if(!emergency_b1_down)
            {
                emergency_combo_timing = false;
                emergency_combo_start_time = 0;
            }


            return true;
        }


        case CC_EMERGENCY_BUTTON3_RAW:
        {
            emergency_b3_down =
                value >= 64;


            if(!emergency_b3_down)
            {
                /*
                 * Release cancels any incomplete emergency hold
                 * immediately.
                 */
                emergency_combo_timing = false;
                emergency_combo_start_time = 0;
            }


            return true;
        }


        /* ====================================================
           K1 — EACH FX HAS ITS OWN ADDRESS
           ==================================================== */

        case CC_MACRO_FX_STUTTER:
            SetMacroStutterRaw(value);
            return true;

        case CC_MACRO_FX_LOOPER:
            SetMacroLooperRaw(value);
            return true;

        case CC_MACRO_FX_DELAY:
            SetMacroDelay(v);
            return true;

        case CC_MACRO_FX_HPF:
            SetMacroHpf(v);
            return true;

        case CC_MACRO_FX_LPF:
            SetMacroLpf(v);
            return true;

        case CC_MACRO_FX_PUMP:
            SetMacroPump(v);
            return true;


        /* ====================================================
           K2 — ABSOLUTE DECAY + ABSOLUTE REVERSE STATE
           ==================================================== */

        case CC_DECAY_ABSOLUTE:
        {
            macro_decay_target = v;

            /*
             * K2 is the single source of truth for kick length.
             *
             * Keep the older energy/separation compensation tied to the
             * SAME macro so every part of the kick moves together.
             */
            separation = v;

            /*
             * Audio callback safely updates the currently-running tail
             * envelope from its present amplitude.
             */
            master_decay_retime_pending = true;

            return true;
        }

        case CC_REVERSE_STATE:
        {
            bool requested_on =
                value >= 64;


            if(requested_on)
            {
                /*
                 * An ON request cancels any not-yet-applied OFF request.
                 */
                reverse_disable_pending = false;


                if(ALLOW_REVERSE_ENABLE_MID_KICK)
                {
                    /*
                     * Preserve the old immediate-enable behaviour.
                     */
                    reverse_bass_enabled = true;
                    reverse_enable_pending = false;
                }
                else
                {
                    reverse_enable_pending = true;
                }
            }
            else
            {
                /*
                 * NEVER un-reverse in the middle of a kick.
                 * The audio thread consumes this at the NEXT kick.
                 */
                reverse_enable_pending = false;
                reverse_disable_pending = true;
            }


            return true;
        }


        /* ====================================================
           K3 — ABSOLUTE TAIL-DELAY AMOUNT + STATE
           ==================================================== */

        case CC_TAIL_DELAY_ABSOLUTE:
        {
            macro_tail_delay = v;
            return true;
        }

        case CC_TAIL_DELAY_STATE:
        {
            tail_delay_enabled =
                value >= 64;

            return true;
        }


        /* ====================================================
           K4 — EACH BPF LAYER HAS ITS OWN FREQUENCY CC
           ==================================================== */

        case CC_BPF_LAYER1_FREQUENCY:
            macro_bpf_target_hz[0] =
                MacroBpfFrequencyHz(v);
            return true;

        case CC_BPF_LAYER2_FREQUENCY:
            macro_bpf_target_hz[1] =
                MacroBpfFrequencyHz(v);
            return true;

        case CC_BPF_LAYER3_FREQUENCY:
            macro_bpf_target_hz[2] =
                MacroBpfFrequencyHz(v);
            return true;

        case CC_BPF_LAYER_COUNT:
        {
            /*
             * Absolute layer count. Recommended Teensy values:
             *   0 = 0 layers
             *   42 = 1 layer
             *   85 = 2 layers
             *   127 = 3 layers
             */
            uint8_t count;

            if(value < 21)
                count = 0;
            else if(value < 64)
                count = 1;
            else if(value < 106)
                count = 2;
            else
                count = 3;


            macro_bpf_layer_count =
                count;

            return true;
        }


        /* ====================================================
           K5 — MACKIE/SHERMAN HAVE SEPARATE AMOUNT CCs
           ==================================================== */

        case CC_MACKIE_AMOUNT:
        {
            macro_mackie_amount = v;

            if(!macro_character_sherman)
                macro_character_wet = v;

            return true;
        }

        case CC_SHERMAN_AMOUNT:
        {
            macro_sherman_amount = v;

            if(macro_character_sherman)
                macro_character_wet = v;

            return true;
        }

        case CC_CHARACTER_MODEL:
        {
            bool requested =
                value >= 64;


            /*
             * Absolute model state: repeated messages are idempotent.
             * No toggle ambiguity and no button edge-state dependency.
             */
            if(requested !=
               macro_character_sherman)
            {
                macro_character_sherman =
                    requested;

                character_switch_target_sherman =
                    requested;

                character_switch_pending =
                    true;
            }


            macro_character_wet =
                requested
                ? macro_sherman_amount
                : macro_mackie_amount;

            return true;
        }


        /* ====================================================
           K6 — ABSOLUTE SHAPE + ABSOLUTE PUMP STATE
           ==================================================== */

        case CC_KICK_SHAPE_ABSOLUTE:
        {
            macro_kick_shape = v;
            return true;
        }


        /* ====================================================
           MIX PAGE — FUNCTION + MENU2 on the Teensy
           ==================================================== */

        case CC_MIX_LINE_GAIN:
            param_line_gain = v * PARAM_LINE_GAIN_MAX;
            return true;

        case CC_MIX_MACKIE_GAIN:
            param_mackie_gain = v * PARAM_MACKIE_GAIN_MAX;
            return true;

        case CC_MIX_SHERMAN_GAIN:
            param_sherman_gain = v * PARAM_SHERMAN_GAIN_MAX;
            return true;

        case CC_MIX_BPF_GAIN:
            param_bpf_gain = v * PARAM_BPF_GAIN_MAX;
            return true;

        case CC_MIX_SUB_GAIN:
            param_sub_gain = v * PARAM_SUB_GAIN_MAX;
            return true;

        case CC_MIX_COMPRESSOR:
            param_comp_amount = v;
            return true;

        case CC_MIX_LIMITER:
            param_limiter_on = value >= 64;
            return true;

        case CC_PUMP_STATE:
        {
            macro_pump_enabled =
                value >= 64;

            PERF_PUMP_ENABLED =
                macro_pump_enabled;

            return true;
        }


        /* ====================================================
           K1 BUTTON — DISPLAY PAGE / LONG RESET ONLY
           ==================================================== */

        case CC_BUTTON_FX_NEXT:
        {
            uint32_t now =
                System::GetNow();


            if(value >= 64)
            {
                if(!macro_fx_button_down)
                {
                    macro_fx_button_down = true;
                    macro_fx_long_press_fired = false;
                    macro_fx_button_down_time = now;
                }
            }
            else if(macro_fx_button_down)
            {
                bool was_long =
                    macro_fx_long_press_fired
                    ||
                    (
                        now -
                        macro_fx_button_down_time >=
                        MACRO_FX_LONG_PRESS_MS
                    );


                macro_fx_button_down = false;


                if(was_long)
                {
                    if(!macro_fx_long_press_fired)
                        ClearAllK1FxExceptPump();
                }
                else
                {
                    /*
                     * Short press is intentionally a NO-OP on the Seed.
                     *
                     * The Teensy owns the UI page. The next dedicated
                     * CC30..35 message tells the Seed which FX is really
                     * being edited, so UI packet loss cannot desynchronise
                     * DSP routing.
                     */
                }


                macro_fx_long_press_fired = false;
            }


            return true;
        }


        /* ====================================================
           OPTIONAL LEGACY PATHS — OFF BY DEFAULT
           ==================================================== */

        case CC_MACRO_FX_VALUE_LEGACY:
        {
            if(!ENABLE_BANKED_K1_CC20_COMPAT)
                return true;

            switch(macro_fx_mode)
            {
                case MacroFxMode::STUTTER:
                    SetMacroStutterRaw(value);
                    break;
                case MacroFxMode::LOOPER:
                    SetMacroLooperRaw(value);
                    break;
                case MacroFxMode::DELAY:
                    SetMacroDelay(v);
                    break;
                case MacroFxMode::DJ_HPF:
                    SetMacroHpf(v);
                    break;
                case MacroFxMode::DJ_LPF:
                    SetMacroLpf(v);
                    break;
                case MacroFxMode::PUMP:
                    SetMacroPump(v);
                    break;
                case MacroFxMode::COUNT:
                default:
                    break;
            }

            return true;
        }

        /* K4 physical knob: edit the currently selected/last BPF layer. */
        case CC_MACRO_BPF_FREQUENCY_LEGACY:
        {
            if(ENABLE_LEGACY_K4_BPF_CONTROLS)
            {
                uint8_t index =
                    macro_bpf_layer_count == 0
                    ? 0
                    : static_cast<uint8_t>(macro_bpf_layer_count - 1);
                if(index > 2)
                    index = 2;
                macro_bpf_target_hz[index] = MacroBpfFrequencyHz(v);
            }
            return true;
        }

        /* K4 button: 0 -> 1 -> 2 -> 3 -> 0 layers, press edge only. */
        case CC_BUTTON_BPF_LAYERS_LEGACY:
        {
            bool down = value >= 64;
            if(ENABLE_LEGACY_K4_BPF_CONTROLS && down && !legacy_bpf_button_down)
            {
                macro_bpf_layer_count =
                    static_cast<uint8_t>((macro_bpf_layer_count + 1) % 4);
            }
            legacy_bpf_button_down = down;
            return true;
        }

        /* K5 physical knob: amount for whichever character model is active. */
        case CC_MACRO_CHARACTER_WET_LEGACY:
        {
            if(ENABLE_LEGACY_K5_CHARACTER_CONTROLS)
            {
                if(macro_character_sherman)
                    macro_sherman_amount = v;
                else
                    macro_mackie_amount = v;
                macro_character_wet = v;
            }
            return true;
        }

        /* K5 button: Mackie <-> Sherman, press edge only. */
        case CC_BUTTON_CHARACTER_LEGACY:
        {
            bool down = value >= 64;
            if(ENABLE_LEGACY_K5_CHARACTER_CONTROLS && down && !character_button_down)
            {
                /*
                 * One physical K5 knob means model switching should not
                 * unexpectedly recall a silent amount. Carry the current
                 * live wet value into the newly selected model.
                 */
                float live_amount = Clamp01Added(macro_character_wet);

                macro_character_sherman = !macro_character_sherman;

                if(macro_character_sherman)
                    macro_sherman_amount = live_amount;
                else
                    macro_mackie_amount = live_amount;

                character_switch_target_sherman = macro_character_sherman;
                character_switch_pending = true;
                macro_character_wet = live_amount;
            }
            character_button_down = down;
            return true;
        }

        case CC_MACRO_DECAY_LEGACY:
        case CC_MACRO_TAIL_DELAY_LEGACY:
        case CC_MACRO_KICK_SHAPE_LEGACY:
        case CC_BUTTON_REVERSE_BASS_LEGACY:
        case CC_BUTTON_TAIL_DELAY_LEGACY:
        case CC_BUTTON_PUMP_LEGACY:
        {
            /*
             * Consume but ignore. This prevents old controller traffic
             * from corrupting the new explicit-state protocol.
             */
            if(!ENABLE_LEGACY_AMBIGUOUS_MACRO_CCS)
                return true;

            return true;
        }


        default:
            return false;
    }
}


static void HandleAddedPerformanceCC(
    uint8_t cc,
    uint8_t value)
{
    /*
     * LEGACY / DEBUG ONLY.
     *
     * This function deliberately NEVER writes the six-macro parameter
     * storage. It cannot corrupt K1 state.
     */
    if(!ENABLE_LEGACY_DIRECT_FX_CCS)
        return;


    switch(cc)
    {
        case CC_FX_PUMP:
            PERF_PUMP_ENABLED = AddedCcOn(value);
            break;

        case CC_FX_DELAY:
            PERF_CLOCKED_DELAY_ENABLED = AddedCcOn(value);
            break;

        case CC_FX_STUTTER:
            PERF_STUTTER_ENABLED = AddedCcOn(value);
            if(PERF_STUTTER_ENABLED)
                added_performance_fx.stutter.Request();
            else
                added_performance_fx.stutter.Stop();
            break;

        case CC_FX_DJ_HPF:
            PERF_DJ_HPF_ENABLED = AddedCcOn(value);
            break;

        case CC_FX_LOOPER:
            PERF_QUANT_LOOPER_ENABLED = AddedCcOn(value);
            if(PERF_QUANT_LOOPER_ENABLED)
                added_performance_fx.looper.Arm();
            else
                added_performance_fx.looper.Stop();
            break;

        default:
            break;
    }
}


/* ============================================================
   MIDI BYTE PARSER
   ============================================================ */

static void ProcessMidiByte(uint8_t byte)
{
    /*
     * START
     */
    if(byte == 0xFA)
    {
        midi_running = true;

        /*
         * Reset the performance quantization grid.
         */
        perf_clock_pulse_count = 0;
        perf_sixteenth_pending = false;
        perf_quarter_pending = false;

        hw.SetLed(true);

        PrepareMidiScreen();

        return;
    }


    /*
     * STOP
     */
    if(byte == 0xFC)
    {
        midi_running = false;

        /*
         * There may be no next kick on which to consume a normal
         * quantized OFF request. Force-disable is still processed inside
         * the audio callback and therefore still fades out safely.
         */
        stutter_quantized_command =
            QUANT_FX_FORCE_DISABLE;

        looper_quantized_command =
            QUANT_FX_FORCE_DISABLE;

        macro_fx_value_stutter = 0.0f;
        macro_fx_value_looper = 0.0f;

        hw.SetLed(false);

        PrepareMidiScreen();

        return;
    }


    /*
     * CLOCK.
     *
     * The original kick itself still does not depend on MIDI clock.
     * Clock is only used by the ADDED performance effects.
     */
    if(byte == 0xF8)
    {
        uint32_t now =
            System::GetNow();


        if(perf_clock_last_ms != 0)
        {
            uint32_t dt =
                now -
                perf_clock_last_ms;


            /*
             * Realistic MIDI clock period range.
             */
            if(dt >= 2 &&
               dt <= 100)
            {
                float measured_quarter =
                    static_cast<float>(
                        dt
                    ) *
                    24.0f;


                /*
                 * Smooth clock timing enough to avoid delay zipper/jitter.
                 */
                perf_quarter_note_ms =
                    perf_quarter_note_ms *
                    0.88f +
                    measured_quarter *
                    0.12f;


                perf_quarter_note_ms =
                    ClampAdded(
                        perf_quarter_note_ms,
                        120.0f,
                        1200.0f
                    );
            }
        }


        perf_clock_last_ms =
            now;


        perf_clock_pulse_count++;


        if(
            (
                perf_clock_pulse_count %
                6
            ) == 0
        )
        {
            perf_sixteenth_pending =
                true;
        }


        if(
            (
                perf_clock_pulse_count %
                24
            ) == 0
        )
        {
            perf_quarter_pending =
                true;
        }


        return;
    }


    /*
     * Other realtime messages.
     */
    if(byte >= 0xF8)
        return;


    /*
     * Status byte.
     */
    if(byte & 0x80)
    {
        /*
         * System common messages are not needed here.
         */
        if(byte >= 0xF0)
        {
            midi_running_status = 0;
            midi_data_count = 0;

            return;
        }


        midi_running_status = byte;
        midi_data_count = 0;

        return;
    }


    /*
     * Ignore stray data.
     */
    if(midi_running_status == 0)
        return;


    midi_data[
        midi_data_count++
    ] =
        byte & 0x7F;


    uint8_t type =
        midi_running_status & 0xF0;


    uint8_t channel =
        midi_running_status & 0x0F;


    /*
     * --------------------------------------------------------
     * CHANNEL 15 CONTROL CHANGE = ADDED PERFORMANCE FX
     * --------------------------------------------------------
     */
    if(type == 0xB0)
    {
        if(midi_data_count < 2)
            return;


        uint8_t cc =
            midi_data[0];


        uint8_t value =
            midi_data[1];


        midi_data_count = 0;


        if(channel ==
           MIDI_CHANNEL_KICK)
        {
            /*
             * Six physical macro pairs get first refusal.
             * Legacy/direct CCs remain available for debugging.
             */
            if(HandleSixMacroCC(
                   cc,
                   value
               ))
            {
                PrepareMidiScreen();
            }
            else if(ENABLE_LEGACY_DIRECT_FX_CCS)
            {
                HandleAddedPerformanceCC(
                    cc,
                    value
                );
            }
        }


        return;
    }


    if(
        type == 0x90 ||
        type == 0x80
    )
    {
        if(midi_data_count < 2)
            return;


        uint8_t note =
            midi_data[0];


        uint8_t velocity =
            midi_data[1];


        midi_data_count = 0;


        /*
         * ----------------------------------------------------
         * CHANNEL 15 = KICK
         * ----------------------------------------------------
         */
        if(channel == MIDI_CHANNEL_KICK)
        {
            if(type == 0x90)
            {
                HandleKickNoteOn(
                    note,
                    velocity
                );
            }
            else
            {
                HandleKickNoteOff(
                    note
                );
            }

            return;
        }


        /*
         * ----------------------------------------------------
         * CHANNEL 16 = SECONDARY CHARACTER LAYER
         * ----------------------------------------------------
         */
        if(channel == MIDI_CHANNEL_LAYER)
        {
            if(type == 0x90)
            {
                HandleLayer16NoteOn(
                    note,
                    velocity
                );
            }
            else
            {
                HandleLayer16NoteOff(
                    note
                );
            }

            return;
        }
    }
}


/* ============================================================
   MIDI SERVICE
   ============================================================ */

static void ServiceMidi()
{
    /*
     * MIDI ALWAYS GETS SERVICED FIRST.
     *
     * Raw polling.
     *
     * No UartHandler.
     * No DMA listener.
     * No BlockingReceive.
     * No custom USART3 IRQ.
     */
    while(
        USART3->ISR &
        USART_ISR_RXNE_RXFNE
    )
    {
        uint8_t byte =
            static_cast<uint8_t>(
                USART3->RDR
            );


        ProcessMidiByte(byte);
    }


    /*
     * Existing MIDI RUN heartbeat.
     */
    if(midi_running)
    {
        static uint32_t last_blink = 0;

        uint32_t now =
            System::GetNow();


        if(now - last_blink >= 250)
        {
            last_blink = now;

            static bool state = false;

            state = !state;

            hw.SetLed(state);
        }
    }
}


/* ============================================================
   AUDIO CALLBACK
   ============================================================ */

static void AudioCallback(
    AudioHandle::InputBuffer in,
    AudioHandle::OutputBuffer out,
    size_t size)
{
    /*
     * --------------------------------------------------------
     * EVENT: KICK TRIGGER
     * --------------------------------------------------------
     */
    if(kick_trigger_pending)
    {
        kick_trigger_pending = false;


        /*
         * ====================================================
         * NEXT-KICK REVERSE STATE COMMIT
         * ====================================================
         *
         * OFF is always committed here, never mid-hit.
         * ON is also committed here only if the code-level option above
         * is changed to disallow mid-kick enable.
         */
        if(reverse_disable_pending)
        {
            reverse_bass_enabled = false;
            reverse_disable_pending = false;
        }


        if(reverse_enable_pending)
        {
            reverse_bass_enabled = true;
            reverse_enable_pending = false;
        }


        /*
         * ====================================================
         * OVERLAPPING RATCHET DETECTION
         * ====================================================
         *
         * If any meaningful kick envelope is still alive, this Note On is
         * treated as a monophonic retrigger rather than a fresh voice.
         */
        kick_retrigger_active =
            tail_env.active &&
            tail_env.value >
            RATCHET_RETRIGGER_LEVEL_THRESHOLD;


        /*
         * Snapshot the old sine tail BEFORE the deterministic phase reset.
         */
        BeginRatchetPhaseBridge();


        /*
         * ORIGINAL fixed2 kick trigger.
         */
        TriggerKickAudio(
            last_velocity
        );


        /*
         * Start the independent clean bass tail at Note-On.
         *
         * The new master gate envelope determines how long the COMPLETE
         * kick is audible.
         */
        StartTailAudio();


        /*
         * MASTER OUTPUT DE-CLICK ENVELOPE:
         * Note-On -> short attack -> unity hold.
         *
         * Musical decay is handled by tail_env / K2, not by MIDI gate.
         */
        added_kick_master_envelope.Trigger();


        /*
         * ADDED layers only.
         */
        macro_tail_delay_envelope.Trigger();
        macro_whole_kick_reverse.Trigger();
        macro_character_processor.Trigger();

        added_protected_punch.Trigger();

        /*
         * Experimental resonant sweep layer.
         *
         * current_gate_ms is the most recently measured MIDI gate.
         */
        added_resonant_sweep_layer.Trigger(
            current_gate_ms,
            kick_frequency
        );

        added_performance_fx.TriggerKick();


        /*
         * CHOP / LOOPER ON-OFF is quantized to THIS kick boundary.
         */
        added_performance_fx.OnKickBoundary();
    }


    /*
     * --------------------------------------------------------
     * EVENT: NOTE OFF / TRANSIENT RELEASE
     * --------------------------------------------------------
     */
    if(kick_release_pending)
    {
        kick_release_pending = false;


        /*
         * Original transient release.
         */
        ReleaseKickAudio();


        /*
         * Tail pitch is no longer armed here.
         *
         * Velocity latched the bipolar target at Note-On. Note-Off only
         * releases the transient envelope.
         */


        /*
         * IMPORTANT:
         * Note-Off no longer closes any amplitude envelope.
         *
         * K2 / CC40 is now the exclusive amplitude-decay control.
         */
    }


    /*
     * Consume any transport-stop safety commands in the AUDIO thread.
     */
    added_performance_fx.ServiceImmediateSafety();


    if(master_decay_retime_pending)
    {
        master_decay_retime_pending = false;


        /*
         * Retiming is click-free because Envelope::Process() multiplies
         * the CURRENT amplitude by a new coefficient; value itself is
         * never stepped.
         */
        if(tail_env.active)
        {
            tail_env.decay =
                MacroDecaySeconds(
                    macro_decay
                );
        }
    }


    /*
     * Note-gate state is intentionally NOT used to release the master
     * amplitude envelope anymore.
     *
     * Velocity owns the tail-pitch gesture; Note-Off only releases the
     * transient. K2 owns amplitude decay.
     */


    /*
     * Local filter state.
     */
    static float character_hp_state = 0.0f;


    /*
     * Current character frequency.
     *
     * It is updated once per kick, not continuously.
     */
    float target_character_frequency =
        character_frequency;


    /*
     * Smooth character filter movement by a small amount.
     * This avoids zipper noise but still keeps the pattern
     * quantised from kick to kick.
     */
    static float smoothed_character_frequency = 500.0f;


    /*
     * Gain compensation for oscillator morph.
     */
    float morph_gain =
        USE_NEW_SHAPE_TRANSIENT_MACRO
        ? (
            1.0f -
            oscillator_morph *
            0.04f
          )
        : (
            1.0f -
            oscillator_morph *
            0.13f
          );


    /*
     * Bass character gain.
     */
    float bass_character =
        DEFAULT_BASS_CHARACTER;


    /*
     * ADDED clock-grid events.
     *
     * They are consumed at the audio-block boundary so buffer state
     * changes cannot occur halfway through a sample operation.
     */
    if(perf_sixteenth_pending)
    {
        perf_sixteenth_pending =
            false;

        added_performance_fx.OnSixteenth();
    }


    if(perf_quarter_pending)
    {
        perf_quarter_pending =
            false;

        added_performance_fx.OnQuarter();
    }


    /*
     * Update BODY filter coefficients once per 8-sample block.
     */
    added_body_lowpass.Set(
        KickBodyLpCutoffHz(
            kick_body_lp_cutoff
        ),
        KickBodyLpQ(
            kick_body_lp_resonance
        )
    );


    /*
     * Macro-4 BPF layer frequency smoothing/coefficient update.
     */
    macro_bpf_bank.Update();


    /*
     * Slew DECAY toward its CC target once per block. Stepping the decay
     * coefficient straight from each MIDI message zippers while K2 turns.
     */
    {
        float decay_error =
            macro_decay_target - macro_decay;

        if(fabsf(decay_error) > 0.00005f)
        {
            macro_decay += decay_error * 0.06f;
            master_decay_retime_pending = true;
        }
        else if(macro_decay != macro_decay_target)
        {
            macro_decay = macro_decay_target;
            master_decay_retime_pending = true;
        }
    }


    for(size_t i = 0;
        i < size;
        ++i)
    {
        /* Hard digital isolation: begin both physical outputs at silence. */
        out[KICK_OUTPUT_CHANNEL][i] = 0.0f;
        out[EXTERNAL_OUTPUT_CHANNEL][i] = 0.0f;
        /* ====================================================
           TRANSIENT PITCH
           ==================================================== */

        /*
         * Exponential pitch descent.
         *
         * transient_pitch_phase starts at 1 and decays toward 0.
         */
        float transient_decay =
            expf(
                -6.9078f /
                (
                    transient_pitch_time
                    * SAMPLE_RATE
                )
            );


        transient_pitch_phase *=
            transient_decay;


        float transient_frequency =
            kick_frequency +
            (
                transient_start_frequency
                - kick_frequency
            )
            *
            transient_pitch_phase;


        if(transient_frequency < kick_frequency)
            transient_frequency =
                kick_frequency;


        /* ====================================================
           TAIL PITCH — VELOCITY PERFORMANCE CONTROL
           ====================================================

           The old source calculated a tail_frequency_current but then
           advanced the actual tail oscillator with plain kick_frequency,
           so that pitch calculation was inaudible.

           The clean tail now uses the velocity-derived ratio for its
           REAL oscillator increment.
         */

        float tail_pitch_ratio =
            ProcessVelocityTailPitchRatio();


        float tail_frequency_current =
            kick_frequency *
            tail_pitch_ratio;


        /*
         * Keep the playable sweep inside a safe sub/kick range.
         */
        if(tail_frequency_current <
           TAIL_SWEEP_MIN_FREQUENCY_HZ)
        {
            tail_frequency_current =
                TAIL_SWEEP_MIN_FREQUENCY_HZ;
        }


        if(tail_frequency_current >
           TAIL_SWEEP_MAX_FREQUENCY_HZ)
        {
            tail_frequency_current =
                TAIL_SWEEP_MAX_FREQUENCY_HZ;
        }


        /*
         * Musical anatomy handoff for this sample.
         */
        float transient_handoff_gain = 1.0f;
        float tail_handoff_gain = 0.0f;


        GetTransientTailCrossfade(
            transient_handoff_gain,
            tail_handoff_gain
        );


        /*
         * 0 through first 20 ms.
         * Smoothly rises to 1 through 20..30 ms.
         */
        float current_kick_age_ms =
            static_cast<float>(
                kick_age_samples
            )
            *
            1000.0f /
            SAMPLE_RATE;


        float onset_processed_mix =
            SmoothstepAdded(
                Clamp01Added(
                    (
                        current_kick_age_ms -
                        PURE_SWEEP_ONLY_MS
                    )
                    /
                    PURE_SWEEP_BODY_FADE_MS
                )
            );

        /* Character/dirty bus only — see CHARACTER_OPEN_MS. */
        float character_open_mix =
            SmoothstepAdded(
                Clamp01Added(
                    (
                        current_kick_age_ms -
                        CHARACTER_OPEN_MS
                    )
                    /
                    CHARACTER_OPEN_FADE_MS
                )
            );


        /* ====================================================
           TRANSIENT OSCILLATOR
           ==================================================== */

        float transient =
            GenerateTransientOscillator(
                transient_frequency,
                oscillator_morph
            );


        if(USE_NEW_SHAPE_TRANSIENT_MACRO)
        {
            /*
             * NEW:
             * SHAPE decides whether a bright transient is admitted.
             * Velocity remains a timbral modifier, never loudness.
             */
            float transient_drive =
                transient_shape_drive_current;


            float driven_transient =
                SoftClip(
                    transient *
                    transient_drive
                );


            /*
             * Preserve the actual sine oscillator for the first 20 ms.
             * Saturation only fades in with the body after that point.
             */
            transient =
                transient +
                (
                    driven_transient -
                    transient
                )
                *
                onset_processed_mix;


            float transient_cutoff =
                transient_shape_cutoff_current;


            transient_cutoff =
                ClampAdded(
                    transient_cutoff,
                    500.0f,
                    12000.0f
                );


            float transient_lp_a =
                expf(
                    -TWO_PI *
                    transient_cutoff /
                    SAMPLE_RATE
                );


            transient_tone_lp_state =
                (
                    1.0f -
                    transient_lp_a
                ) *
                transient
                +
                transient_lp_a *
                transient_tone_lp_state;


            transient =
                transient_tone_lp_state;
        }
        else
        {
            /*
             * PREVIOUS VERSION:
             * fixed transient character drive, no SHAPE tone LP.
             */
            float transient_drive =
                1.05f +
                DEFAULT_TRANSIENT_CHARACTER *
                1.45f;


            transient =
                SoftClip(
                    transient *
                    transient_drive
                );
        }


        /*
         * ====================================================
         * INDEPENDENT ULTRA-HF / LASER GUARD
         * ====================================================
         *
         * Two cascaded one-poles strongly suppress the >8 kHz impulse
         * that otherwise gets exaggerated by distortion and resonant FX.
         *
         * The filter opens smoothly from ~3.2 kHz to only ~6.8 kHz
         * during the first 16 ms. This deliberately makes the protected
         * 65..95 Hz knock/body carry more of the kick's identity.
         */
        if(ENABLE_TRANSIENT_HF_GUARD)
        {
            float age_ms =
                static_cast<float>(
                    kick_age_samples
                ) *
                1000.0f /
                SAMPLE_RATE;


            float open_t =
                SmoothstepAdded(
                    Clamp01Added(
                        age_ms /
                        TRANSIENT_HF_GUARD_OPEN_MS
                    )
                );


            float guard_cutoff =
                TRANSIENT_HF_GUARD_INITIAL_HZ +
                (
                    TRANSIENT_HF_GUARD_FINAL_HZ -
                    TRANSIENT_HF_GUARD_INITIAL_HZ
                ) *
                open_t;


            float guard_a =
                expf(
                    -TWO_PI *
                    guard_cutoff /
                    SAMPLE_RATE
                );


            transient_hf_guard_state_1 =
                (
                    1.0f -
                    guard_a
                ) *
                transient
                +
                guard_a *
                transient_hf_guard_state_1;


            transient_hf_guard_state_2 =
                (
                    1.0f -
                    guard_a
                ) *
                transient_hf_guard_state_1
                +
                guard_a *
                transient_hf_guard_state_2;


            transient =
                transient_hf_guard_state_2;
        }


        /* ====================================================
           TRANSIENT ENVELOPE
           ==================================================== */

        float transient_amp =
            transient_env.Process(
                SAMPLE_RATE
            );


        /*
         * Small gain compensation as harmonic content rises.
         */
        transient *=
            transient_amp *
            morph_gain *
            0.52f *
            KICK_TRANSIENT_GAIN *
            (
                USE_NEW_SHAPE_TRANSIENT_MACRO
                ? transient_shape_gain_current
                : 1.0f
            ) *
            transient_handoff_gain;


        /* ====================================================
           CLEAN SUB
           ==================================================== */

        float sub_increment =
            tail_frequency_current /
            SAMPLE_RATE;


        if(sub_increment > 0.45f)
            sub_increment = 0.45f;


        float sub =
            sinf(
                sub_phase *
                6.28318530718f
            );


        sub_phase +=
            sub_increment;


        if(sub_phase >= 1.0f)
            sub_phase -= 1.0f;


        /*
         * SHARED K2 DECAY ENVELOPE.
         *
         * This exact tail_amp controls both the clean sub below and the
         * phase-coherent processed-tail feed into Mackie/Sherman.
         */
        float tail_amp =
            tail_env.Process(
                SAMPLE_RATE
            );


        /*
         * Clean low-frequency tail.
         */
        float clean_tail_gain =
            tail_amp *
            (
                0.52f +
                separation *
                0.20f
            ) *
            KICK_TAIL_GAIN *
            tail_handoff_gain;


        float clean_tail =
            sub *
            clean_tail_gain;


        /*
         * Continuously expose the old sine's exact musical state so an
         * overlapping phase-locked retrigger can bridge it cleanly.
         */
        current_clean_tail_gain_for_bridge =
            clean_tail_gain;


        current_tail_frequency_for_bridge =
            tail_frequency_current;


        /*
         * K2 reverse and K3 tail-delay no longer touch only clean_tail.
         * Both are routed later, after the complete kick body has been
         * assembled.
         */


        /* ====================================================
           CHARACTER FILTER SMOOTHING
           ==================================================== */

        smoothed_character_frequency +=
            (
                target_character_frequency
                -
                smoothed_character_frequency
            )
            *
            0.0025f;


        /* ====================================================
           CHARACTER PATH
           ==================================================== */

        /*
         * Temporarily TRUE-BYPASS the original BPF character system.
         *
         * This is useful for diagnosing the strange off-overtones.
         * When false, NO character-bandpass processing occurs.
         */
        float character = 0.0f;


        if(ENABLE_ORIGINAL_CHARACTER_BPF_PATH)
        {
            /*
             * Feed mainly transient/body material into the character
             * path, keeping deepest sub out of the distortion.
             */
            float character_input =
                transient *
                (
                    0.55f +
                    bass_character *
                    0.30f
                );


            float hp_character =
                HighPass(
                    character_input,
                    105.0f,
                    character_hp_state
                );


            character_bandpass.SetBandpass(
                smoothed_character_frequency,
                1.05f
            );


            character =
                character_bandpass.Process(
                    hp_character
                );


            float drive =
                1.1f +
                bass_character *
                2.4f;


            character =
                SoftClip(
                    character *
                    drive
                );


            character =
                character_postfilter.Process(
                    character,
                    6500.0f
                );


            /*
             * Original Channel 16 character-layer amplitude behavior.
             */
            if(layer16_active)
            {
                character *=
                    layer16_level;
            }
            else
            {
                character = 0.0f;
            }


            character *=
                1.0f -
                separation *
                0.30f;
        }


        /* ====================================================
           KICK/BASS BUS
           ==================================================== */

        /*
         * ====================================================
         * CHARACTER PARALLEL SEND — FULL KICK COPY
         * ====================================================
         *
         * Old Dutch gabber/industrial character depends on driving the
         * sustained kick body/tail into overload. FIXED3 removed the tail
         * from this send and then delayed character processing until 20 ms,
         * which left almost no useful source for Mackie/Sherman.
         *
         * The DIRTY SEND receives a copy of transient + clean tail +
         * optional channel-16 colour. The original clean tail NEVER leaves
         * its protected lane; only the high-passed WET RETURN comes back.
         */
        float character_send =
            transient * KICK_PROCESSED_TRANSIENT_FEED +
            clean_tail * KICK_PROCESSED_TAIL_FEED +
            character * 0.65f;

        /*
         * The send stays FULL-BAND on purpose.
         *
         * High-passing here removed the tail before the models ever saw it:
         * the tail is a 45..70 Hz sine, so a 120 Hz corner gutted it while
         * the broadband punch passed, and the distortion ended up acting
         * almost entirely on the attack. The models need the tail to
         * generate sustained harmonics from it.
         *
         * The fundamental is kept out of the MIX by high-passing the return
         * instead, so the harmonics come back but the distorted sub does
         * not stack on the protected clean one.
         */

        /* Macro-4 broad BPF EQ is pushed INTO the distortion model. */
        character_send +=
            macro_bpf_bank.ProcessDriveFeed(character_send);

        /* Keep the exact kick edge clean; wake character under body fade. */
        float character_return = 0.0f;
        if(character_open_mix > 0.000001f)
        {
            character_return =
                macro_character_processor.ProcessWet(character_send) *
                KICK_CHARACTER_RETURN_GAIN;
        }

        /* Wet-return-only sub protection. */
        if(CHARACTER_PROTECT_CLEAN_SUB)
        {
            character_return =
                HighPassFixedPole(
                    character_return,
                    CHARACTER_SUB_PROTECT_POLE_A,
                    character_delta_hp_state
                );

            if(CHARACTER_SUB_PROTECT_POLES >= 2)
            {
                character_return =
                    HighPassFixedPole(
                        character_return,
                        CHARACTER_SUB_PROTECT_POLE_A,
                        character_delta_hp_state_2
                    );
            }

            if(CHARACTER_SUB_PROTECT_POLES >= 3)
            {
                character_return =
                    HighPassFixedPole(
                        character_return,
                        CHARACTER_SUB_PROTECT_POLE_A,
                        character_delta_hp_state_3
                    );
            }
        }

        /* TRUE PARALLEL CHARACTER SEND / RETURN. */
        float clean_knock =
            transient;


        /*
         * Body LP is a clean kick tone control, not a dirty-bus processor.
         * It may shape the knock but remains independent of character wet.
         */
        if(KICK_BODY_LP_ENABLED)
        {
            clean_knock =
                added_body_lowpass.Process(
                    clean_knock
                );
        }


        float dirty_character_signal =
            character +
            character_return;


        /*
         * Macro 4 is an additive dirty/colour return. Feed it the body so
         * it responds musically, but add only its BPF OUTPUT to the dirty
         * lane; the clean body itself remains on clean_knock.
         */
        dirty_character_signal +=
            macro_bpf_bank.Process(
                clean_knock +
                character_return +
                character
            );


        /* ====================================================
           PROTECTED-PARALLEL KICK ARCHITECTURE
           ====================================================

           1) CLEAN KNOCK / TRANSIENT lane
              never enters character-bus gain management

           2) DIRTY / CHARACTER return
              Mackie/Sherman delta + channel-16/BPF + resonant colour

           3) CLEAN SINE TAIL lane
              mixed AFTER all dirty-path dynamics

           4) Optional protected 65..95 Hz punch lane
              mixed later, also AFTER dirty-path dynamics

           Mackie/Sherman therefore ADD texture around the clean kick rather
           than replacing it or consuming its headroom through a shared bus.
         */


        /*
         * Resonant layer belongs to the DIRTY / character side. Its input
         * may be excited by clean kick energy, but only its OUTPUT is sent
         * through dirty-path level management.
         */
        float resonant_sweep_input =
            clean_tail +
            clean_knock *
            resonant_sweep_body_feed;


        float resonant_sweep_layer =
            added_resonant_sweep_layer.Process(
                resonant_sweep_input
            );


        float dirty_signal =
            dirty_character_signal +
            resonant_sweep_layer;


        /*
         * Parameter-based compensation now applies ONLY to the dirty
         * branch. The clean sine fundamental is fully independent of character energy.
         */
        float time_compensation =
            1.0f -
            separation * 0.13f;


        dirty_signal *=
            time_compensation;


        /*
         * Amount-aware management begins around MID I and becomes
         * increasingly assertive as Mackie/Sherman amount rises.
         */
        dirty_signal =
            character_dirty_bus_manager.Process(
                dirty_signal,
                macro_character_processor.CurrentSmoothedAmount()
            );


        /*
         * NO sample-fast compressor and NO nonlinear limiter here.
         *
         * Those stages changed behaviour once signal energy crossed their
         * thresholds, so the screech onset moved with wet %, filter level
         * and decay. The slow soft-knee manager above is now the only
         * dirty-bus dynamics.
         */
        float dirty_post_gain =
            DIRTY_POST_GAIN_DRY +
            (
                DIRTY_POST_GAIN_WET -
                DIRTY_POST_GAIN_DRY
            )
            *
            macro_character_processor.CurrentSmoothedAmount();


        /*
         * Macro-BPF layers add parallel energy. Compensate LINEARLY,
         * rather than waiting for a limiter to clip when a layer pushes
         * the branch across a threshold.
         */
        dirty_post_gain -=
            static_cast<float>(
                macro_bpf_layer_count
            )
            *
            0.035f;


        if(dirty_post_gain < 0.62f)
            dirty_post_gain = 0.62f;


        dirty_signal *=
            dirty_post_gain;


        /*
         * ====================================================
         * PROTECTED CLEAN SINE TAIL
         * ====================================================
         *
         * This never enters:
         *   - Mackie/Sherman
         *   - character/BPF dirty manager
         *   - shared compressor
         *   - dirty limiter
         *
         * It retains K2 decay and velocity-controlled pitch movement because it
         * is still derived from the actual clean_tail oscillator.
         */
        float protected_clean_tail =
            clean_tail *
            param_sub_gain;


        /*
         * Recombine only AFTER the wet/distorted branch has been brought
         * under control.
         */
        float full_processed_signal =
            clean_knock +
            dirty_signal +
            protected_clean_tail;


        /*
         * PURE FIRST-20-ms SIGNAL:
         *
         * The pitch sweep itself is the punch.
         * No character/BPF/dirty dynamics are audible yet.
         */
        float clean_sweep_signal =
            transient +
            protected_clean_tail;


        float signal =
            clean_sweep_signal +
            (
                full_processed_signal -
                clean_sweep_signal
            )
            *
            character_open_mix;


        /* ====================================================
           MACRO 3 — POST-PROCESSING DELAYED KICK-BODY BUS
           ====================================================

           This gain is AFTER the original kick distortion,
           time compensation, compressor and limiter.

           The delayed second segment now contains:
               - protected clean knock/transient lane
               - level-managed dirty/character return
               - protected clean sine tail lane

           The dedicated protected 65..95 Hz punch is still added AFTER
           this gain and remains the immediate first segment.
         */

        float delayed_kick_bus_gain =
            macro_tail_delay_envelope.Process();


        signal *=
            delayed_kick_bus_gain;


        /* ====================================================
           ADDED CLEAN PROTECTED PUNCH
           ==================================================== */

        float punch_envelope =
            0.0f;


        float protected_punch = 0.0f;


        if(ENABLE_PROTECTED_PUNCH)
        {
            protected_punch =
                added_protected_punch.Process(
                    transient_frequency,
                    oscillator_morph,
                    punch_envelope
                );
        }


        /*
         * SELF-SIDECHAIN:
         *
         * The already-distorted/compressed/limited original kick moves
         * out of the way briefly. The clean punch is then reintroduced
         * AFTER all original kick processing.
         */
        if(ENABLE_PROTECTED_PUNCH)
        {
            /*
             * Hold the sidechain action slightly longer than the raw
             * punch amplitude so the anatomy reads clearly as:
             *
             *      CLEAN PUNCH -> BODY / TAIL BLOOM
             */
            float duck_envelope =
                sqrtf(
                    Clamp01Added(
                        punch_envelope
                    )
                );


            float duck =
                1.0f -
                duck_envelope *
                PROTECTED_PUNCH_DUCK_DEPTH;


            /*
             * Keep some body underneath, but let the 65..95 Hz knock
             * dominate the first 20..40 ms so dedicated kick bins see a
             * clear, high-crest impact instead of bass-tail masking.
             */
            if(duck < 0.14f)
                duck = 0.14f;


            signal *=
                duck;


            signal +=
                protected_punch;
        }


        /* ====================================================
           MACRO 2 — WHOLE-WAVEFORM REVERSE
           ====================================================

           Reverse sees the COMPLETE generated kick, including:
               tock/transient
               all oscillator morph content
               sub/tail
               BPF layers
               Mackie/Sherman
               resonant layer
               protected punch

           It now ping-pongs two capture banks. While one hit is being
           played backwards, the CURRENT fully processed kick is rendered
           into the other bank. Parameter changes therefore update the
           next reverse hit without disabling/re-enabling reverse.

           External Digitakt audio is deliberately outside this buffer.
         */

        signal =
            macro_whole_kick_reverse.Process(
                signal
            );


        /* ====================================================
           MASTER GENERATED-KICK AMPLITUDE ENVELOPE
           ====================================================

           Applied AFTER all kick layers, distortion, original dynamics,
           resonant experiment and protected punch.

           External Digitakt passthrough remains outside this envelope.
         */

        float kick_master_gain =
            added_kick_master_envelope.Process();


        signal *=
            kick_master_gain;


        /* ====================================================
           FINAL GENERATED-KICK HF GUARD
           ====================================================

           Last chance to remove the onset laser AFTER every internal
           generated-kick layer, including reverse and protected punch.

           External audio is mixed later and is therefore NOT filtered.
         */

        if(ENABLE_FINAL_KICK_HF_GUARD)
        {
            float age_ms =
                static_cast<float>(
                    kick_age_samples
                ) *
                1000.0f /
                SAMPLE_RATE;


            float open_t =
                SmoothstepAdded(
                    Clamp01Added(
                        age_ms /
                        FINAL_KICK_HF_OPEN_MS
                    )
                );


            float cutoff =
                FINAL_KICK_HF_INITIAL_HZ +
                (
                    FINAL_KICK_HF_SETTLED_HZ -
                    FINAL_KICK_HF_INITIAL_HZ
                ) *
                open_t;


            float a =
                expf(
                    -TWO_PI *
                    cutoff /
                    SAMPLE_RATE
                );


            final_kick_hf_state_1 =
                (
                    1.0f -
                    a
                ) *
                signal
                +
                a *
                final_kick_hf_state_1;


            final_kick_hf_state_2 =
                (
                    1.0f -
                    a
                ) *
                final_kick_hf_state_1
                +
                a *
                final_kick_hf_state_2;


            final_kick_hf_state_3 =
                (
                    1.0f -
                    a
                ) *
                final_kick_hf_state_2
                +
                a *
                final_kick_hf_state_3;


            signal =
                final_kick_hf_state_3;
        }


        /*
         * Deterministic ratchet handoff:
         * old continuing sine tail -> new reset-phase kick.
         */
        signal =
            ProcessRatchetPhaseBridge(
                signal
            );


        last_generated_kick_signal =
            signal;


        /* ====================================================
           HARD TWO-BUS OUTPUT SPLIT
           ====================================================

           AUDIO OUT 1 = KICK ONLY
           AUDIO OUT 2 = DIGITAKT / EXTERNAL ONLY

           There is NO shared compressor, limiter, looper, chopper,
           HPF/LPF or final mix bus between these two outputs.
         */


        /* ----------------------------------------------------
           KICK-ONLY OUTPUT BUS
           ---------------------------------------------------- */

        float kick_output =
            signal;


        /*
         * Kick output management remains kick-only.
         */
        /*
         * Keep the pure sine onset below the limiter knee.
         * Normal output level returns with the same 20..30 ms body fade.
         */
        float onset_output_headroom =
            PURE_SWEEP_OUTPUT_HEADROOM +
            (
                1.0f -
                PURE_SWEEP_OUTPUT_HEADROOM
            )
            *
            onset_processed_mix;


        /*
         * LINEAR OUTPUT HEADROOM.
         *
         * No nonlinear final limiter here: character/filter combinations
         * should not suddenly enter a different transfer curve at 0.92.
         */
        kick_output *=
            param_line_gain *
            onset_output_headroom;


        /*
         * Kept as a call for code continuity, but the feature is disabled
         * by ENABLE_FINAL_HF_DYNAMIC_TAMER=false above.
         */
        kick_output =
            ProcessFinalHfDynamicTamer(
                kick_output
            );


        /*
         * STUTTER / LOOPER / DJ HPF are the ONLY performance FX on the kick lane.
         */
        kick_output =
            added_performance_fx.ProcessMaster(
                kick_output
            );


        /*
         * Keep the kick post-FX chain linear. The old post-HPF HF guard
         * plus absolute LPF formed another moving filter cascade after the
         * HPF and could create a second onset transient.
         */
        kick_output =
            ProcessPerformanceFilterHeadroom(
                kick_output
            );


        kick_output *=
            KICK_OUTPUT_LINEAR_GAIN;


        /* Mix-page dynamics, ahead of the ceiling so they catch peaks
         * rather than leaving them to saturate. */
        kick_output =
            kick_compressor.Process(
                kick_output,
                param_comp_amount
            );


        kick_output =
            kick_limiter.Process(
                kick_output,
                param_limiter_on
            );


        kick_output =
            OutputCeiling(
                kick_output
            );


        /* ----------------------------------------------------
           DIGITAKT / EXTERNAL-ONLY OUTPUT BUS
           ----------------------------------------------------

           External audio NEVER enters:
               kick synthesis / kick distortion
               kick character bus / reverse / looper
               kick master envelope
               kick HF management

           It has its OWN independent external processing state:
               PUMP
               dotted external delay
               STUTTER / CHOP
               DJ HPF
               DJ LPF

           None of that external audio is ever summed into Audio Out 1.
         */

        float external_output =
            in[EXTERNAL_INPUT_CHANNEL_1][i] *
            EXTERNAL_INPUT_1_GAIN +
            in[EXTERNAL_INPUT_CHANNEL_2][i] *
            EXTERNAL_INPUT_2_GAIN;


        external_output =
            added_performance_fx.ProcessExternal(
                external_output
            );


        external_output *=
            EXTERNAL_RETURN_GAIN *
            EXTERNAL_OUTPUT_LINEAR_GAIN;


        /*
         * Pure DAC safety. Normal external level should stay below this.
         */
        external_output =
            ClampAdded(
                external_output,
                -0.995f,
                 0.995f
            );


        /*
         * PHYSICAL ROUTING:
         *
         * Out 1 = generated kick
         * Out 2 = Digitakt / external
         */
        out[KICK_OUTPUT_CHANNEL][i] =
            kick_output;

        out[EXTERNAL_OUTPUT_CHANNEL][i] =
            external_output;


        /*
         * Advance the shared per-hit anatomy clock once per audio frame.
         */
        kick_age_samples++;
    }
}


/* ============================================================
   MAIN
   ============================================================ */

int main(void)
{
    /* --------------------------------------------------------
       DAISY
       -------------------------------------------------------- */

    hw.Configure();

    hw.Init();


    /*
     * 8-sample blocks still give sub-millisecond trigger latency while
     * providing more scheduling margin as FX are added. The previous
     * 4-sample block made audio overruns easier to hear as digital ticks.
     */
    hw.SetAudioBlockSize(8);


    /* --------------------------------------------------------
       MIDI
       -------------------------------------------------------- */

    InitMidiUart();


    /* --------------------------------------------------------
       OLED
       -------------------------------------------------------- */

    InitOled();


    /* --------------------------------------------------------
       DSP INITIAL STATE
       -------------------------------------------------------- */

    kick_frequency = 55.0f;


    oscillator_morph =
        SINE_ONLY_OSCILLATORS
        ? 0.0f
        : Clamp01Added(
              macro_kick_shape
          );


    separation = 0.35f;


    /*
     * Generate initial filter pattern.
     */
    GenerateCharacterPattern();


    last_pattern_note =
        static_cast<int>(
            last_note
        );


    character_frequency =
        filter_pattern[0];


    /*
     * Reset oscillators.
     */
    kick_retrigger_active = false;

    ratchet_phase_bridge_active = false;
    ratchet_bridge_phase = 0.0f;
    ratchet_bridge_frequency = 55.0f;
    ratchet_bridge_tail_gain = 0.0f;
    ratchet_bridge_residual = 0.0f;
    ratchet_bridge_pos = 0;
    ratchet_bridge_samples = 1;

    last_generated_kick_signal = 0.0f;
    current_clean_tail_gain_for_bridge = 0.0f;
    current_tail_frequency_for_bridge = 55.0f;

    final_hf_low_state = 0.0f;
    final_hf_envelope = 0.0f;
    final_hf_gain = 1.0f;

    ResetKickPhases();


    /*
     * Reset filters.
     */
    character_lowpass.Reset();
    sub_lowpass.Reset();
    character_postfilter.Reset();
    character_bandpass.Reset();


    /*
     * ADDED DSP initial state.
     */
    added_performance_fx.Reset();
    added_protected_punch.Reset();
    added_body_lowpass.Reset();
    added_resonant_sweep_layer.Reset();
    added_kick_master_envelope.Reset();

    macro_tail_delay_envelope.Reset();
    macro_bpf_bank.Reset();
    macro_character_processor.Reset();
    character_dirty_bus_manager.Reset();
    kick_compressor.Reset();
    kick_limiter.Reset();
    character_delta_hp_state = 0.0f;
    character_delta_hp_state_2 = 0.0f;
    character_delta_hp_state_3 = 0.0f;
    macro_whole_kick_reverse.Reset();


    /*
     * HARD KNOWN STARTUP STATE.
     *
     * No effect is enabled simply because a stale controller/menu state
     * exists. Pump also starts with amount zero and toggle OFF.
     */
    macro_fx_value_stutter = 0.0f;
    macro_fx_value_looper = 0.0f;
    macro_fx_value_delay = 0.0f;

    stutter_quantized_command = QUANT_FX_NONE;
    looper_quantized_command = QUANT_FX_NONE;
    master_decay_retime_pending = false;

    macro_fx_value_hpf = 0.0f;
    macro_fx_value_lpf = 0.0f;

    performance_filter_headroom_gain = 1.0f;

    post_perf_hf_state_1 = 0.0f;
    post_perf_hf_state_2 = 0.0f;
    post_perf_hf_mix = 0.0f;

    final_lpf_enforce_state_1 = 0.0f;
    final_lpf_enforce_state_2 = 0.0f;
    final_lpf_enforce_state_3 = 0.0f;
    final_lpf_enforce_state_4 = 0.0f;
    macro_fx_value_pump = 0.0f;

    PERF_STUTTER_ENABLED = false;
    PERF_QUANT_LOOPER_ENABLED = false;
    PERF_CLOCKED_DELAY_ENABLED = false;
    PERF_DJ_HPF_ENABLED = false;
    PERF_PUMP_ENABLED = false;

    macro_pump_enabled = false;

    macro_mackie_amount = 0.0f;
    macro_sherman_amount = 0.0f;
    macro_character_wet = 0.0f;

    character_switch_target_sherman =
        macro_character_sherman;

    character_switch_pending = false;

    character_button_down = false;

    macro_fx_button_down = false;
    macro_fx_long_press_fired = false;

    emergency_b1_down = false;
    emergency_b3_down = false;
    emergency_combo_timing = false;
    emergency_combo_start_time = 0;


    /*
     * Initial OLED.
     */
    PrepareMidiScreen();


    /* --------------------------------------------------------
       AUDIO
       -------------------------------------------------------- */

    hw.StartAudio(
        AudioCallback
    );


    /* --------------------------------------------------------
       MAIN LOOP
       -------------------------------------------------------- */

    while(1)
    {
        /*
         * ====================================================
         * MIDI FIRST
         * ====================================================
         *
         * Do not put blocking OLED operations here.
         */
        ServiceMidi();


        /*
         * HARD EMERGENCY CHORD:
         *
         * B1 + B3 continuously held for >1 second -> STM32 reboot.
         *
         * This is serviced before B1's normal long-hold action.
         */
        ServiceEmergencyReboot();


        /*
         * K1 >500 ms hold is serviced independently of further MIDI
         * traffic so it fires while the button is still held.
         *
         * This normal action is suppressed while physical B3 is held.
         */
        ServiceMacroFxButtonHold();


        /*
         * ====================================================
         * OLED DMA
         * ====================================================
         */
        ServiceOledTransfer();


        /*
         * Start a pending screen refresh only when the I2C
         * peripheral is idle.
         */
        if(
            oled_dirty &&
            oled_transfer ==
                OledTransfer::IDLE
        )
        {
            oled_page = 0;

            StartOledPage(0);
        }
    }
}