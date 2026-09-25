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

/* Where OutputCeiling() starts to bend; shared with the wet headroom below. */
static constexpr float OUTPUT_CEILING_KNEE = 0.80f;

/*
 * WET HEADROOM. At high LINE the dry kick alone already reaches the output
 * ceiling, so anything the distortion added on top was soft-limited away:
 * past the base drive K5 seemed to do nothing, and limiting the peaks pressed
 * the bass down (1-5 kHz flat and < 150 Hz -2 dB from K5 50 % to 100 % at
 * LINE 100 %). With this on, the wet only uses the room the dry kick leaves
 * under the ceiling's knee, squeezed smoothly into it: plenty near the sub's
 * zero crossings, little at its peaks. The dry kick, and so the bass, is
 * never limited by the distortion.
 *
 * Off by default: measured, it keeps the bass (-0.2 dB) but squeezes the
 * distortion hard at high LINE, the liked base drive included (1-5 kHz at
 * K5 25 % fell from +18 to +11 dB at LINE 100 %). Lowering LINE for headroom
 * works better; this stays as an option.
 */
static constexpr bool KICK_WET_HEADROOM = false;
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
static volatile float param_tube_gain = 1.56710f;/* CC55, max 2.5  */
static volatile float param_bpf_gain     = 4.0f;    /* CC56, max 8.0  */
static volatile float param_sub_gain     = 0.95f;   /* CC57, max 1.6  */
static volatile float param_punch_gain   = 1.0f;    /* CC58, max 2.0  */
static volatile float param_reverb_amount = 0.0f;   /* CC36, FX page  */

static constexpr float PARAM_LINE_GAIN_MAX    = 4.0f;
static constexpr float PARAM_MACKIE_GAIN_MAX  = 2.0f;
static constexpr float PARAM_TUBE_GAIN_MAX = 2.5f;
static constexpr float PARAM_BPF_GAIN_MAX     = 8.0f;
static constexpr float PARAM_SUB_GAIN_MAX     = 1.6f;
static constexpr float PARAM_PUNCH_GAIN_MAX   = 2.0f;

/*
 * CC wrote these directly and they are read per sample, so every message
 * stepped the gain - audible as a tick while turning. CC now writes *_target;
 * audio slews to it once per block. SUB and PUNCH are not listed: the kick
 * voice already slews them per sample (KICK_GAIN_SMOOTH_A).
 */
static volatile float param_line_gain_target    = 2.8184f;
static volatile float param_mackie_gain_target  = 1.01703f;
static volatile float param_tube_gain_target = 1.56710f;
static volatile float param_bpf_gain_target     = 4.0f;
static constexpr float PARAM_GAIN_SLEW = 0.06f;

/* Sidechain reverb tuning. HP pole = expf(-2*pi*250/48000). */
/* 300 Hz: expf(-2*pi*300/48000). Keeps the tank off the punch. */
static constexpr float REVERB_SEND_HP_POLE_A = 0.96149f;
static constexpr float REVERB_SEND_LEVEL     = 0.90f;
static constexpr float REVERB_RETURN_LEVEL   = 0.70f;
/* Shorter decay so a hit's tail is spent before the next one lands. */
static constexpr float REVERB_FEEDBACK       = 0.72f;
/* Full knob travel reaches only this much wet: 40% was the usable top. */
static constexpr float REVERB_AMOUNT_MAX     = 0.40f;
/* Light damping so it still reads through distortion, without shimmering. */
static constexpr float REVERB_DAMPING        = 0.26f;
/*
 * Sidechain duck, KEYED OFF THE KICK TRIGGER rather than an envelope
 * follower on the audio. A follower gives a soft, level-dependent dip whose
 * shape changes with how loud the hit was; keying off the note gives the same
 * clean pump every time, which is what makes it read as sidechained.
 *
 * DEPTH is how far it drops on the hit, RECOVER_A the climb back to unity
 * (~70 ms, so it is most of the way back within one 16th at club tempo).
 */
static constexpr float REVERB_DUCK_DEPTH     = 0.95f;
static constexpr float REVERB_DUCK_RECOVER_A = 0.000298f;
/* 2 ms ramp out of the duck: fast enough to still read as an instant cut,
 * slow enough that it is not a single-sample step. */
static constexpr float REVERB_DUCK_CUT_STEP  = 1.0f / (0.002f * SAMPLE_RATE);

/* ============================================================
   MUSICAL MACKIE / TUBE CHARACTER
   ============================================================

   The models are PARALLEL SEND/RETURN processors:

       dry punch + sub ------------------------------------> dry lane
              \-> wet send -> Mackie/Tube + BPF -> 120 Hz HPF -> wet
       dry + wet -> kick FX

   This means character can be driven hard without replacing or phase-
   cancelling the dry sub. Macro 5 is a true send/return amount.

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
 * The previous build kept expensive Mackie/Tube processing alive even
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
 * NONE of this touches the dry punch and sub.
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
 * K6 pump retrigger lengths (see MacroKickShapeSweepSeconds).
 */
static constexpr float SHAPE_ROUND_SWEEP_MS = 180.0f;
static constexpr float SHAPE_PUNCH_SWEEP_MS = 88.0f;
static constexpr float SHAPE_SNAP_SWEEP_MS  = 110.0f;


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


/*
 * Reverse bank-to-bank handoff.
 *
 * This is separate from reverse enable/disable fading. It specifically
 * smooths the instant a new captured hit becomes the new reverse source.
 */
static constexpr float REVERSE_BANK_HANDOFF_MS = 9.0f;



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

       KNOB 5   CC24    MACKIE / TUBE WET
       BUTTON 5 CC104   MACKIE <-> TUBE MODEL

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
 *   CC49 TUBE AMOUNT
 *   CC50 CHARACTER MODEL        0=MACKIE, 127=TUBE
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
/* Seventh K1 FX page: sidechain reverb amount. */
static constexpr uint8_t CC_MACRO_FX_REVERB        = 36;

static constexpr uint8_t CC_DECAY_ABSOLUTE          = 40;
static constexpr uint8_t CC_REVERSE_STATE           = 41;
static constexpr uint8_t CC_TAIL_DELAY_ABSOLUTE     = 42;
static constexpr uint8_t CC_TAIL_DELAY_STATE        = 43;
static constexpr uint8_t CC_BPF_LAYER1_FREQUENCY    = 44;
static constexpr uint8_t CC_BPF_LAYER2_FREQUENCY    = 45;
static constexpr uint8_t CC_BPF_LAYER3_FREQUENCY    = 46;
static constexpr uint8_t CC_BPF_LAYER_COUNT         = 47;
static constexpr uint8_t CC_MACKIE_AMOUNT           = 48;
static constexpr uint8_t CC_TUBE_AMOUNT          = 49;
static constexpr uint8_t CC_CHARACTER_MODEL         = 50;
static constexpr uint8_t CC_KICK_SHAPE_ABSOLUTE     = 51;
static constexpr uint8_t CC_PUMP_STATE              = 52;

/* Mix page: Teensy FUNCTION + MENU2. */
static constexpr uint8_t CC_MIX_LINE_GAIN           = 53;
static constexpr uint8_t CC_MIX_MACKIE_GAIN         = 54;
static constexpr uint8_t CC_MIX_TUBE_GAIN        = 55;
static constexpr uint8_t CC_MIX_BPF_GAIN            = 56;
static constexpr uint8_t CC_MIX_SUB_GAIN            = 57;
static constexpr uint8_t CC_MIX_PUNCH_GAIN          = 58;

/*
 * v1.3.0 kick shaping, all per-hit (latched at the trigger):
 *   CC61 TAIL ATTACK   mix page pot 3: how fast the tail comes back in after
 *                      the TAIL DELAY gap
 *   CC62 SWEEP TIME    Menu 1 pot 3 on the kick channel: punch sweep length
 *   CC63 TAIL MOD      Menu 1 pot 5 on the kick channel: a wobble macro on the
                      sub, 0 = none, up to +-2 st, faster and more irregular
                      as it rises
 *   CC64 WAVE          Menu 1 pot 2 on the kick channel: sine -> supersaw
 */
static constexpr uint8_t CC_TAIL_ATTACK             = 61;
static constexpr uint8_t CC_PUNCH_SWEEP_TIME        = 62;
static constexpr uint8_t CC_TAIL_MOD                = 63;
static constexpr uint8_t CC_WAVE                    = 64;


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
 * The two stutter instances take the same command but consume it at
 * different points: the kick's on its next kick boundary, the external
 * one on the next sixteenth, because the external lane must still chop
 * when no kick is playing. Separate mailboxes keep one from swallowing
 * the other's command; PostStutterCommand keeps them in step.
 */
static volatile uint32_t external_stutter_quantized_command = QUANT_FX_NONE;

static inline void PostStutterCommand(uint32_t command)
{
    stutter_quantized_command = command;
    external_stutter_quantized_command = command;
}



/*
 * K2 decay changes are also handed to the audio thread rather than
 * directly mutating its envelope from ServiceMidi().
 */
static volatile bool master_decay_retime_pending = false;

/*
 * Raised by the audio thread when the kick output stops being a finite
 * number. The recovery itself runs in the control loop: it clears the
 * 43200-sample loop capture, which is far too long to sit inside an audio
 * block.
 */
static volatile bool audio_panic_pending = false;


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

/*
 * Beat-quantized requests. The control loop records what was asked for and
 * the audio thread applies it on the next quarter-note pulse, so the tail
 * delay and the K1 reset land on the beat instead of wherever the button
 * happened to be pressed. -1 means nothing is waiting.
 */
static volatile int8_t tail_delay_pending_state = -1;
static volatile bool fx_reset_pending = false;


/* Macro 4: additive BPF layer bank. */
static volatile uint8_t macro_bpf_layer_count = 0;
/*
 * Layer count LATCHED at Note-On, exactly as K6 SHAPE already is.
 *
 * CC47 is a global continuously-applied parameter, but a kick sustains for
 * hundreds of ms. Per-step layer changes from the sequencer's fill
 * randomisation therefore landed while the PREVIOUS kick was still sounding
 * and re-filtered its body and tail. Latching means a CC only takes effect
 * from the next Note-On, so each hit keeps the count it was triggered with.
 */
static volatile uint8_t macro_bpf_layer_count_latched = 0;
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
static volatile bool macro_character_tube = false;

/*
 * Main-loop MIDI code ONLY writes these request flags.
 * The audio callback consumes them and is the ONLY place allowed to
 * reset/switch Mackie/Tube DSP state.
 */
static volatile bool character_switch_pending = false;
static volatile bool character_switch_target_tube = false;

/* Button-5 edge latch: repeated held CC values cannot retrigger switching. */
static bool character_button_down = false;


/* Macro 6: laser -> long bass-note kick shape. */
static volatile float macro_kick_shape = 0.50f;


/* CC60..63; see their declarations. Defaults are the pre-v1.3.0 kick. */
static volatile float macro_tail_attack_ms = 6.0f;
static volatile float macro_punch_time_scale = 1.0f;
static volatile float macro_tail_mod = 0.0f;  /* 0..1 intensity */
static volatile float macro_wave = 0.0f;


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
static volatile float macro_tube_amount = 0.0f;

/*
 * true  = newly-selected model starts at 0
 * false = newly-selected model recalls its own previous amount
 */
static bool CHARACTER_RESET_ON_MODEL_SWITCH = false;

static constexpr float CHARACTER_BASE_FULL_POINT = 0.30f;



/* ============================================================
   MASTER KICK AMPLITUDE ENVELOPE
   ============================================================

   MIDI Note-On:
       fast smooth attack -> HOLD

   MIDI Note-Off:
       quick smooth release -> ZERO

   This envelope is applied to the COMPLETE GENERATED KICK after the
   dry + wet sum and reverse.

   It does NOT touch external Digitakt passthrough.

   Later these timings can become CC parameters.
   ============================================================ */

static bool ENABLE_KICK_MASTER_GATE_ENVELOPE = true;

static volatile float kick_master_attack_ms = 2.5f;
static volatile float kick_master_release_ms = 14.0f;


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
 * PUMP retrigger length, following K6. Only the pump uses this; the kick's
 * own punch sweep is PunchSweepMs().
 */
static float MacroKickShapeSweepSeconds(float x)
{
    x = Clamp01Added(x);

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
/*
 * Band span, shared by the frequency map and the Q law so the two cannot
 * drift apart. Extended down from 140 to 115 and now to 85 Hz, which is low
 * enough to sit on the fundamental rather than above it.
 */
static constexpr float MACRO_BPF_LOW_HZ = 85.0f;
static constexpr float MACRO_BPF_HIGH_HZ = 3200.0f;

static float MacroBpfFrequencyHz(float x)
{
    x = Clamp01Added(x);

    return
        MACRO_BPF_LOW_HZ *
        powf(
            MACRO_BPF_HIGH_HZ /
            MACRO_BPF_LOW_HZ,
            x
        );
}


/* MIDI */

static constexpr uint8_t MIDI_CHANNEL_KICK = 14; // MIDI ch 15


/* ============================================================
   MIDI STATE
   ============================================================ */

static bool midi_running = false;

static uint8_t midi_running_status = 0;
static uint8_t midi_data[2];
static uint8_t midi_data_count = 0;

static uint8_t last_velocity = 100;
static bool note_gate = false;


/* ============================================================
   MIDI → AUDIO EVENTS
   ============================================================ */

static volatile bool kick_trigger_pending = false;


/* ============================================================
   KICK PARAMETERS
   ============================================================ */

/*
 * MIDI note controls this.
 */
static volatile float kick_frequency = 55.0f;

/*
 * Gate-derived temporal separation.
 *
 * This is calculated from the actual MIDI gate length.
 */
static volatile float separation = 0.50f;


/*
 * Per-hit age: zero at each trigger. The kick-lane HPF onset guard reads it.
 */
static uint32_t kick_age_samples = 0;


/*
 * Final >8 kHz dynamics state.
 */
static float final_hf_low_state = 0.0f;
static float final_hf_envelope = 0.0f;
static float final_hf_gain = 1.0f;


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
   CLICK ABLATION SWITCHES
   ============================================================

   Each isolates one suspect for the remaining kick click. Defaults are the
   normal instrument; flip one, rebuild, listen.

   KICK_DAC_KEEPALIVE
       Adds a constant 2^-20 (about -120 dBFS, inaudible, AC-coupled away)
       to the kick output so the codec never sees exact digital silence.
       The sub now ends at exact zero between hits; a codec that mutes on
       zero input pops every time it wakes, at any level, which matches a
       click that appears the moment SUB leaves 0 %.

   KICK_BYPASS_WET
       Skips Mackie/Tube, BPF, dirty-bus manager and the wet HPF, so
       they are not even computed.

   KICK_BYPASS_POST
       Sends the dry voice straight to line gain and the output ceiling,
       skipping reverse, master envelope, performance FX, headroom and
       reverb. With this and KICK_BYPASS_WET, Out 1 is the bare sine.
   ============================================================ */

static constexpr bool KICK_DAC_KEEPALIVE = false;
static constexpr bool KICK_BYPASS_WET = false;
static constexpr bool KICK_BYPASS_POST = false;

static constexpr float KICK_DAC_KEEPALIVE_OFFSET = 1.0f / 1048576.0f;


/* ============================================================
   PUNCH PROFILE — ITERATIVE ABLATION
   ============================================================

   The old engine's punch was stronger for reasons that are all envelope
   and tone, not oscillator count. Each stage below restores one of its
   elements on top of the ones before, in the order they are likely to
   matter. Bump KICK_PUNCH_PROFILE_STAGE, rebuild, listen. Any single one
   can also be forced on or off by editing its line.

   0  current punch: decays from the first sample, sub full from 6 ms
   1  OLD ANATOMY: the punch is HELD at full, then hands over to the sub on
      an equal-power crossfade across the SHAPE window (20-52 ms at 0,
      36-82 ms at 64, 115-165 ms at 127); the sub is quiet under the sweep
      and fades in across the same window. Old per-SHAPE punch attack
      (2.5 / 1.35 / 0.75 ms).
   2  OLD LEVEL: per-SHAPE punch level 0.52 x (0.16 / 0.62 / 1.0), tapered
      by 10 % over the top 10 % of the knob, instead of a fixed 0.32.
   3  OLD DRIVE: SoftClip(sine x 1.02 / 1.36 / 1.82) on the punch path,
      faded in from 20 to 30 ms as it was; the first 20 ms stay a pure sine.
   4  OLD TONE: one-pole low-pass on the punch path, 1.1k / 3.6k / 9.5k.
   5  OLD HF GUARD: two one-poles on the punch path opening 3.2 -> 6.8 kHz
      over the first 16 ms.

   Every value is the 3af0e35 curve (ShapeThreePoint across ROUND / PUNCH /
   SNAP). All of it acts on the punch path of the one oscillator; the sub
   path stays a clean sine.
   ============================================================ */

static constexpr int KICK_PUNCH_PROFILE_STAGE = 5;

static constexpr bool KICK_PUNCH_OLD_ANATOMY  = KICK_PUNCH_PROFILE_STAGE >= 1;
static constexpr bool KICK_PUNCH_OLD_LEVEL    = KICK_PUNCH_PROFILE_STAGE >= 2;
static constexpr bool KICK_PUNCH_OLD_DRIVE    = KICK_PUNCH_PROFILE_STAGE >= 3;
static constexpr bool KICK_PUNCH_OLD_TONE     = KICK_PUNCH_PROFILE_STAGE >= 4;
static constexpr bool KICK_PUNCH_OLD_HF_GUARD = KICK_PUNCH_PROFILE_STAGE >= 5;

/* ============================================================
   MACKIE / OUTPUT AS 3af0e35 HAD IT
   ============================================================

   The character models are unchanged, but what surrounds them made the old
   kick sound the way it did; these restore it. Measured with the host
   harness on the old engine, removing 1-3 together accounts for the whole
   brightness difference (+8.8 dB above 5 kHz at full Mackie).

   0  Mackie is fed BEFORE the PUNCH / SUB mixer gains (always on), so those
      knobs set the dry level only, never how hard the models are driven.
   1  KICK_OLD_WET_ONSET: a new hit's contribution to the wet send is held
      off for 4 ms and faded in by 10 ms. Per voice: a sub still ringing
      from the previous hit keeps feeding the models, so nothing dips.
   2  KICK_OLD_FINAL_HF_GUARD: three one-poles on the whole kick, opening
      7 -> 16 kHz over the first 22 ms of each hit. Only the cutoff moves,
      closing over ~1 ms; the filter memory is never reset, which is what
      made the old one click.
   3  KICK_OLD_ONSET_LEVEL: the whole kick at 0.68 for the first 20 ms,
      back to 1 by 30 ms. Only on a hit that starts from silence; a hit
      landing on a sounding kick stays at 1, since ducking a ringing tail
      is the tick the handoff removed.
   4  KICK_OLD_SUB_DECAY: the old sub decay, 0.45 exponential + 0.55
      linear per sample (-60 dB exponential at K2's time), which empties in
      about 60 % of it. Offset by the silence floor so it lands on zero.
      Off, the sub is (1 - u)^2 and lasts the full K2 time, about 3-5 dB
      more low end.
   ============================================================ */

static constexpr bool KICK_OLD_WET_ONSET      = true;
static constexpr bool KICK_OLD_SUB_DECAY      = true;
static constexpr float OLD_SUB_DECAY_LINEARITY = 0.55f;
static constexpr bool KICK_OLD_FINAL_HF_GUARD = true;
static constexpr bool KICK_OLD_ONSET_LEVEL    = true;

static constexpr float OLD_WET_OPEN_MS      = 4.0f;
static constexpr float OLD_WET_OPEN_FADE_MS = 6.0f;

static constexpr float OLD_FINAL_HF_INITIAL_HZ = 7000.0f;
static constexpr float OLD_FINAL_HF_SETTLED_HZ = 16000.0f;
static constexpr float OLD_FINAL_HF_OPEN_MS    = 22.0f;
/* How far the cutoff may fall per sample: 16k -> 7k in ~1 ms. */
static constexpr float OLD_FINAL_HF_CLOSE_STEP_HZ = 9000.0f / 48.0f;

static constexpr float OLD_ONSET_LEVEL      = 0.68f;
static constexpr float OLD_ONSET_HOLD_MS    = 20.0f;
static constexpr float OLD_ONSET_RELEASE_MS = 10.0f;
/* A fresh hit starts from silence, but ramp the fall over 3 ms regardless. */
static constexpr float OLD_ONSET_FALL_STEP = 1.0f / (3.0f * 48.0f);


/* 3af0e35 SHAPE landmarks: ROUND (0), PUNCH (64), SNAP (127). */
static constexpr float SHAPE_ROUND_TRANSIENT_GAIN = 0.16f;
static constexpr float SHAPE_PUNCH_TRANSIENT_GAIN = 0.62f;
static constexpr float SHAPE_SNAP_TRANSIENT_GAIN  = 1.00f;
static constexpr float SHAPE_TAPER_START = 0.90f;
static constexpr float SHAPE_TAPER_DEPTH = 0.10f;
static constexpr float OLD_TRANSIENT_LEVEL = 0.52f;

static constexpr float SHAPE_ROUND_DRIVE = 1.02f;
static constexpr float SHAPE_PUNCH_DRIVE = 1.36f;
static constexpr float SHAPE_SNAP_DRIVE  = 1.82f;
static constexpr float OLD_DRIVE_PURE_MS = 20.0f;
static constexpr float OLD_DRIVE_FADE_MS = 10.0f;

static constexpr float SHAPE_ROUND_CUTOFF_HZ = 1100.0f;
static constexpr float SHAPE_PUNCH_CUTOFF_HZ = 3600.0f;
static constexpr float SHAPE_SNAP_CUTOFF_HZ  = 9500.0f;

static constexpr float SHAPE_ROUND_ATTACK_MS = 2.50f;
static constexpr float SHAPE_PUNCH_ATTACK_MS = 1.35f;
static constexpr float SHAPE_SNAP_ATTACK_MS  = 0.75f;

static constexpr float SHAPE_ROUND_HANDOFF_START_MS = 20.0f;
static constexpr float SHAPE_PUNCH_HANDOFF_START_MS = 36.0f;
static constexpr float SHAPE_SNAP_HANDOFF_START_MS  = 115.0f;
static constexpr float SHAPE_ROUND_HANDOFF_END_MS = 52.0f;
static constexpr float SHAPE_PUNCH_HANDOFF_END_MS = 82.0f;
static constexpr float SHAPE_SNAP_HANDOFF_END_MS  = 165.0f;

static constexpr float OLD_HF_GUARD_INITIAL_HZ = 3200.0f;
static constexpr float OLD_HF_GUARD_FINAL_HZ   = 6800.0f;
static constexpr float OLD_HF_GUARD_OPEN_MS    = 16.0f;


static float ShapeThreePoint(
    float x,
    float round_value,
    float punch_value,
    float snap_value)
{
    x = Clamp01Added(x);

    if(x <= 0.50f)
        return round_value +
               (punch_value - round_value) * SmoothstepAdded(x / 0.50f);

    return punch_value +
           (snap_value - punch_value) * SmoothstepAdded((x - 0.50f) / 0.50f);
}


static float OldPunchLevel(float x)
{
    float gain =
        ShapeThreePoint(
            x,
            SHAPE_ROUND_TRANSIENT_GAIN,
            SHAPE_PUNCH_TRANSIENT_GAIN,
            SHAPE_SNAP_TRANSIENT_GAIN
        );

    x = Clamp01Added(x);

    if(x > SHAPE_TAPER_START)
    {
        gain *=
            1.0f -
            (x - SHAPE_TAPER_START) / (1.0f - SHAPE_TAPER_START) *
            SHAPE_TAPER_DEPTH;
    }

    return OLD_TRANSIENT_LEVEL * gain;
}


static inline float SoftClip(float x)
{
    /*
     * Fast, smooth saturation.
     */
    return x /
           (1.0f + fabsf(x));
}


/* ============================================================
   KICK VOICE — ONE SINE, PUNCH + SUB
   ============================================================

   One sine oscillator. Its phase is 0 at the trigger, so a fresh hit
   renders the same waveform every time.

   PITCH, one continuous curve:

       f(t) = base(t) * (1 + (R - 1) * sweep(t))

       sweep(t)  1 -> 0, exponential, -60 dB at the punch sweep time.
                 K6 / SHAPE (CC51) sets R and that time:
                     0    no sweep, flat pitch
                     64   3.8x over 88 ms
                     127  laser, 30x over 110 ms
       base(t)   the MIDI note, gliding by velocity once the sub starts:
                 1 = one octave down, 64 = flat, 127 = one octave up

   LEVEL, two envelopes on that one oscillator:

       punch     CC58 PUNCH. 0.5 ms raised-cosine attack at the trigger,
                 then -60 dB over the sweep time
       sub       CC57 SUB. Starts at the trigger, or K3 / TAIL DELAY later;
                 6 ms raised-cosine attack, then (1 - u)^2 over K2 / DECAY,
                 reaching exactly zero with zero slope at the DECAY time.
                 K2 retimes it live because u only ever advances.

   The voice renders two paths from that one sine: the PUNCH path (level,
   and optionally the old drive / tone / HF guard; see PUNCH PROFILE) and the
   SUB path, which stays a clean sine.

   RETRIGGER, NO TAIL DELAY: HANDOFF. A hit landing while the sub still
   sounds must not crossfade two sines: at different phases their sum dips
   and recovers inside the fade, heard as a tick that grows with SUB. The
   sub path instead carries on from the old sine's exact phase and level
   and glides onto the new hit's trajectory over RETRIGGER_HANDOFF_MS:

       phase = new phase + (offset + base_drift * t) * (1 - s)
       sub   = old sub * (1 - s) + new sub * s

   The old punch path is not a plain sine once it is driven or filtered, so
   it is not carried: the old voice finishes its punch on its own in a
   fading slot, exactly continuing, while the new punch starts from zero.

   with s a smoothstep 0 -> 1. The old level keeps the slope it had and
   bends flat over ~1 ms (LEVEL_SLOPE_BEND_A) instead of freezing. The
   waveform, its level and the base pitch are continuous; only the punch
   sweep jumps in, which is the punch. Once s reaches 1 and the old punch
   has faded, the output is bit-identical to a fresh hit.

   RETRIGGER WITH TAIL DELAY: the new sub is silent at the trigger, so there
   is no sub to hand off to. The old voice carries on along its own
   trajectory and fades out over the same window, while the new hit starts
   fresh at phase 0.
   ============================================================ */

/* Punch landmarks: R = 30^(p^1.35) passes 3.8x at p = 0.5. */
static constexpr float PUNCH_MAX_START_RATIO  = 30.0f;
static constexpr float PUNCH_RATIO_CURVE      = 1.35f;
static constexpr float PUNCH_START_CEILING_HZ = 3000.0f;

static constexpr float PUNCH_SWEEP_MS_LOW  = 20.0f;
static constexpr float PUNCH_SWEEP_MS_MID  = 88.0f;
static constexpr float PUNCH_SWEEP_MS_HIGH = 110.0f;

/* The punch level reaches -60 dB this many sweep times after the hit. */
static constexpr float PUNCH_AMP_DECAY_PER_SWEEP = 1.0f;

static constexpr float PUNCH_ATTACK_MS = 0.5f;
static constexpr float SUB_ATTACK_MS   = 6.0f;

/* v1.3.0 control ranges (CC60..63). */
static constexpr float TAIL_ATTACK_MIN_MS = 6.0f;   /* the click-safe floor */
static constexpr float TAIL_ATTACK_MAX_MS = 60.0f;
/*
 * TAIL MOD is a macro: one intensity drives a pitch wobble on the sub.
 *     depth       0 .. +-2 semitones, linear in intensity
 *     rate        2 Hz drift at low intensity, 12 Hz wobble at full
 *     irregular   a second, unrelated wobble (1.73x the rate) mixes in as
 *                 intensity rises; the pair is normalised, so the total
 *                 never passes the depth
 * It starts on the note (both sines at phase 0) and fades in over the sub's
 * first TAIL_MOD_FADE_IN_MS, so the punch is not smeared. Deterministic: the
 * wobble restarts with every hit.
 */
static constexpr float TAIL_GAP_FALL_MS = 6.0f;
static constexpr float TAIL_MOD_MAX_SEMITONES = 2.0f;
static constexpr float TAIL_MOD_RATE_MIN_HZ = 2.0f;
static constexpr float TAIL_MOD_RATE_MAX_HZ = 12.0f;
static constexpr float TAIL_MOD_IRREGULAR = 0.6f;
static constexpr float TAIL_MOD_FADE_IN_MS = 40.0f;

/*
 * WAVE: the oscillator morphs from the sine to a five-saw supersaw. Built so
 * it cannot phase or lose bass:
 *
 *   locked       every saw is phase-locked to the one oscillator (its phase
 *                is the sine's plus a small offset), so it follows the sweep,
 *                PITCH, TAIL MOD and the handoff glide, and resets with the
 *                oscillator on every hit.
 *   harmonics    detuned saws phase against each other at the fundamental no
 *                matter how small the detune, which is what ate the bass. So
 *                each saw's own fundamental is subtracted exactly, leaving its
 *                harmonics, and those are added on top of the sine, which
 *                stays the fundamental: the bass cannot drift, phase or cancel,
 *                while the upper partials still shimmer (harmonic k of a saw
 *                offset by d Hz beats at k * d Hz, as a real supersaw's do).
 *   detune in Hz the offsets drift at a fixed rate in Hz, not a ratio, so a
 *                3 kHz punch sweep does not turn it into a fast flutter, and
 *                from zero at every hit, so the phasing is the same each time.
 *
 * On a retrigger each saw's offset glides back to zero over the handoff.
 */
static constexpr int WAVE_SAWS = 5;
/*
 * Detune in Hz. The drift is the supersaw's phasing, wanted, and it is
 * deterministic: every offset is (detune x time since the hit), starting from
 * zero with the oscillator's phase-0 reset, so each kick phases identically.
 */
static constexpr float WAVE_SAW_DETUNE_HZ[WAVE_SAWS] = {-1.6f, -0.8f, 0.0f, 0.8f, 1.6f};
static constexpr float WAVE_SAW_WEIGHT[WAVE_SAWS] = {0.6f, 0.8f, 1.0f, 0.8f, 0.6f};
static constexpr float WAVE_SAW_WEIGHT_SUM = 3.8f;
/* The rising saw's fundamental is (2 / pi) sin(2 pi phase). */
static constexpr float WAVE_SAW_FUNDAMENTAL = 0.63661977f;
/* At full WAVE the richer tail masks the punch; the punch gains up to this. */
static constexpr float WAVE_PUNCH_BOOST = 0.6f;             /* x1.6, +4 dB */

/* The offsets move slowly, so their sin/cos are refreshed every 16 samples. */
static constexpr uint32_t WAVE_OFFSET_REFRESH = 16;
/*
 * WAVE adds the supersaw's harmonics on top of the untouched sine, so the
 * bass is identical at every setting. Scaled so that at full WAVE the
 * harmonics stand to the sine as a real saw's do to its own fundamental.
 */
static constexpr float WAVE_HARMONICS_SCALE = 1.0f / WAVE_SAW_FUNDAMENTAL;

static constexpr float RETRIGGER_HANDOFF_MS = 80.0f;

/* Level staging before the CC57 / CC58 mixer gains. */
static constexpr float KICK_PUNCH_LEVEL = 0.32f;
static constexpr float KICK_SUB_LEVEL   = 0.59f;

/*
 * Per-sample decay of the carried-over level slope: expf(-1 / (1 ms * 48 kHz)).
 * The level keeps rising or falling as it was and flattens over ~1 ms.
 */
static constexpr float LEVEL_SLOPE_BEND_A = 0.97938037f;

/* Below this a level counts as silent. */
static constexpr float KICK_VOICE_SILENT = 0.0001f;

/*
 * Voices fading out behind a tail-delayed retrigger. Past the pool, the one
 * furthest through its fade is reused.
 */
static constexpr int KICK_FADING_SLOTS = 4;

/*
 * Mixer-gain slew, so a CC step on SUB/PUNCH does not step the output.
 * 1 - expf(-1 / (10 ms * 48 kHz)).
 */
static constexpr float KICK_GAIN_SMOOTH_A = 0.00208117f;

static constexpr bool ENABLE_SUB_MOVEMENT = true;
static constexpr uint8_t SUB_MOVE_CENTER_VELOCITY = 64;
static constexpr float SUB_MOVE_DOWN_SEMITONES = -12.0f;
static constexpr float SUB_MOVE_UP_SEMITONES   =  12.0f;
static constexpr float SUB_MOVE_GLIDE_MS       = 115.0f;
static constexpr float SUB_MIN_FREQUENCY_HZ    = 12.0f;
static constexpr float SUB_MAX_FREQUENCY_HZ    = 440.0f;  /* room for TAIL MOD up */


static float PunchStartRatio(float p)
{
    p = Clamp01Added(p);

    return expf(
        logf(PUNCH_MAX_START_RATIO) *
        powf(p, PUNCH_RATIO_CURVE)
    );
}


static float PunchSweepMs(float p)
{
    p = Clamp01Added(p);

    if(p <= 0.5f)
        return PUNCH_SWEEP_MS_LOW +
               (PUNCH_SWEEP_MS_MID - PUNCH_SWEEP_MS_LOW) * (p / 0.5f);

    return PUNCH_SWEEP_MS_MID +
           (PUNCH_SWEEP_MS_HIGH - PUNCH_SWEEP_MS_MID) * ((p - 0.5f) / 0.5f);
}


static float VelocityToSubMoveSemitones(uint8_t velocity)
{
    if(!ENABLE_SUB_MOVEMENT)
        return 0.0f;

    if(velocity < 1u)
        velocity = 1u;

    if(velocity <= SUB_MOVE_CENTER_VELOCITY)
    {
        float t =
            static_cast<float>(velocity - 1u) /
            static_cast<float>(SUB_MOVE_CENTER_VELOCITY - 1u);

        return SUB_MOVE_DOWN_SEMITONES * (1.0f - Clamp01Added(t));
    }

    float t =
        static_cast<float>(velocity - SUB_MOVE_CENTER_VELOCITY) /
        static_cast<float>(127u - SUB_MOVE_CENTER_VELOCITY);

    return SUB_MOVE_UP_SEMITONES * Clamp01Added(t);
}


/* Coefficient that decays a value by 60 dB over `seconds`. */
static inline float Decay60Coefficient(float seconds)
{
    if(seconds < 0.0001f)
        seconds = 0.0001f;

    return expf(-6.9078f / (seconds * SAMPLE_RATE));
}


static inline float RaisedCosine01(float t)
{
    t = Clamp01Added(t);

    return 0.5f - 0.5f * cosf(t * PI);
}


static inline uint32_t MsToSamples(float ms)
{
    if(ms < 0.0f)
        ms = 0.0f;

    return static_cast<uint32_t>(ms * 0.001f * SAMPLE_RATE);
}


/*
 * What the voices hand the mixer each sample, all BEFORE the PUNCH / SUB
 * mixer gains: the dry paths, and the models' send.
 */
struct KickVoiceOut
{
    float punch = 0.0f;     /* dry punch path */
    float sub = 0.0f;       /* dry sub path */
    float send = 0.0f;      /* into Mackie / Tube */
    float bpf_punch = 0.0f; /* the punch as the BPF colour hears it */
};


/* Everything a hit latches at its trigger. */
struct KickHitParams
{
    float frequency = 55.0f;
    float punch = 0.5f;                /* K6 SHAPE, 0..1: sweep depth */
    uint8_t velocity = 64;             /* PITCH: the first glide */
    float delay_ms = 0.0f;             /* tail delay incl. fine offset */
    float decay_seconds = 0.5f;
    float sub_attack_ms = 6.0f;        /* TAIL ATTACK */
    float sweep_time_scale = 1.0f;     /* SWEEP TIME */
    float tail_mod = 0.0f;             /* TAIL MOD: wobble intensity 0..1 */
    float wave = 0.0f;                 /* WAVE: 0 sine .. 1 supersaw */
};


static inline float PolyBlepSaw(float phase, float dt)
{
    float y = 2.0f * phase - 1.0f;

    if(phase < dt)
    {
        float t = phase / dt;
        y -= t + t - t * t - 1.0f;
    }
    else if(phase > 1.0f - dt)
    {
        float t = (phase - 1.0f) / dt;
        y -= t * t + t + t + 1.0f;
    }

    return y;
}


struct KickVoice
{
    bool active = false;

    uint32_t age = 0;
    uint32_t start = 0;

    float phase = 0.0f;
    float base_hz = 55.0f;
    float move_log_ratio = 0.0f;

    /* TAIL MOD: the wobble, latched from its intensity at the trigger. */
    float wobble_depth = 0.0f;       /* semitones */
    float wobble_increment = 0.0f;   /* cycles per sample */
    float wobble_irregular = 0.0f;

    uint32_t sub_attack_samples = 1;

    /* WAVE: each saw's offset from the oscillator's phase, in cycles. */
    float saw_offset[WAVE_SAWS] = {};
    float saw_offset_start[WAVE_SAWS] = {};   /* glides to 0 over a handoff */
    float saw_offset_sin[WAVE_SAWS] = {};
    float saw_offset_cos[WAVE_SAWS] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

    float wave_morph = 0.0f;
    float carried_morph = 0.0f;

    /*
     * TAIL DELAY as an envelope over the whole voice: full through the punch,
     * down over TAIL_GAP_FALL_MS, silent, then back up over TAIL ATTACK at
     * the tail delay time. The kick keeps evolving underneath.
     */
    bool gap = false;
    uint32_t gap_hold = 0, gap_fall = 1, gap_return = 0, gap_rise = 1;

    /* Punch pitch sweep. */
    float sweep_depth = 0.0f;   /* R - 1 */
    float sweep = 0.0f;
    float sweep_coefficient = 0.0f;

    /* Punch level, latched from SHAPE at the trigger. */
    float punch_env = 0.0f;           /* profile 0: exponential decay */
    float punch_coefficient = 0.0f;
    float punch_level_scale = KICK_PUNCH_LEVEL;
    uint32_t punch_attack_samples = 1;
    bool punch_sharp_attack = false;  /* only when the sine starts at phase 0 */
    uint32_t anatomy_start = 0;       /* profile 1+: handoff window */
    uint32_t anatomy_end = 1;
    bool punch_done = false;

    /* Punch path tone, latched from SHAPE; per voice so a copy keeps its own. */
    float punch_drive = 1.0f;
    float tone_a = 0.0f;
    float tone_state = 0.0f;
    float guard_state_1 = 0.0f;
    float guard_state_2 = 0.0f;

    /* Sub decay progress 0 -> 1; the envelope is (1 - u)^2. */
    float u = 0.0f;
    float du = 0.0f;
    bool sub_done = false;

    /* KICK_OLD_SUB_DECAY: the old blended envelope, and its per-sample terms. */
    float sub_env = 1.0f;
    float sub_env_coefficient = 1.0f;
    float sub_env_linear_step = 0.0f;

    /*
     * The last output sample's SUB-path sine, for a retrigger to continue
     * from. The punch path is not a plain sine once it is driven or
     * filtered, so it is never carried; the caller hands it to a fading
     * slot instead.
     */
    float last_phase = 0.0f;
    float last_increment = 0.0f;
    float last_level = 0.0f;
    float last_level_step = 0.0f;
    float last_punch = 0.0f;

    /* The punch sweep's share of the last increment, above the base pitch. */
    float last_sweep_increment = 0.0f;

    /* Handoff onto the new trajectory. */
    bool handoff = false;
    uint32_t handoff_age = 0;
    float handoff_offset = 0.0f;
    float handoff_drift = 0.0f;
    float handoff_correction = 0.0f;
    float handoff_level = 0.0f;
    float handoff_slope = 0.0f;

    /* Fade-out, used only by the fading slots. */
    bool fading = false;
    bool fading_punch_only = false;
    uint32_t fade_age = 0;


    void Reset()
    {
        *this = KickVoice();
    }


    /* The sub path is sounding, so a retrigger can hand it off. */
    bool Sounding() const
    {
        return active && last_level > KICK_VOICE_SILENT;
    }


    /* The punch path still has something to finish. */
    bool PunchSounding() const
    {
        return active && !punch_done;
    }


    void SetDecay(float seconds)
    {
        if(seconds < 0.001f)
            seconds = 0.001f;

        du = 1.0f / (seconds * SAMPLE_RATE);
        sub_env_coefficient = Decay60Coefficient(seconds);
        sub_env_linear_step = du;
    }


    /*
     * The note plus both movements, n samples after the sub starts: the PITCH
     * glide over the first SUB_MOVE_GLIDE_MS, and the TAIL MOD wobble.
     */
    float BaseFrequency(uint32_t n) const
    {
        if(move_log_ratio == 0.0f && wobble_depth == 0.0f)
            return base_hz;

        float t1 =
            static_cast<float>(n) /
            static_cast<float>(MsToSamples(SUB_MOVE_GLIDE_MS));

        float semitones = 0.0f;

        if(wobble_depth > 0.0f)
        {
            float cycles = wobble_increment * static_cast<float>(n);

            float shape =
                (sinf(cycles * TWO_PI) +
                 wobble_irregular * sinf(cycles * (1.73f * TWO_PI) + 1.1f)) /
                (1.0f + wobble_irregular);

            float fade =
                SmoothstepAdded(
                    static_cast<float>(n) /
                    static_cast<float>(MsToSamples(TAIL_MOD_FADE_IN_MS))
                );

            semitones = wobble_depth * fade * shape;
        }

        return ClampAdded(
            base_hz *
            expf(move_log_ratio * SmoothstepAdded(Clamp01Added(t1)) +
                 semitones * (0.69314718f / 12.0f)),
            SUB_MIN_FREQUENCY_HZ,
            SUB_MAX_FREQUENCY_HZ
        );
    }


    uint32_t SubAge() const
    {
        return age > start ? age - start : 0;
    }


    /* Profile 1+: 0 -> 1 across the SHAPE handoff window, smoothstepped. */
    float AnatomyPosition() const
    {
        if(age <= anatomy_start)
            return 0.0f;

        return SmoothstepAdded(
            static_cast<float>(age - anatomy_start) /
            static_cast<float>(anatomy_end - anatomy_start)
        );
    }


    /*
     * hand_off: carry the previous hit's sub on. Only valid with no tail
     * delay; the caller fades the old voice out instead otherwise.
     */
    void Trigger(const KickHitParams& hit,
                 bool hand_off)
    {
        float frequency = hit.frequency;
        float punch = hit.punch;
        float decay_seconds = hit.decay_seconds;

        /* The old sub's next sample, had it carried on. */
        float old_phase = last_phase + last_increment;
        old_phase -= floorf(old_phase);

        float old_increment = last_increment;
        float old_level = last_level;
        float old_slope = last_level_step;

        /* The new hit brings its own sweep; only the base pitch carries on. */
        float old_base_increment = old_increment - last_sweep_increment;

        float old_morph = wave_morph;
        wave_morph = Clamp01Added(hit.wave);

        active = true;
        fading = false;
        fading_punch_only = false;

        age = 0;
        start = 0;   /* the sub always starts with the hit; see gap */

        phase = 0.0f;
        base_hz = frequency;

        move_log_ratio =
            VelocityToSubMoveSemitones(hit.velocity) * (0.69314718f / 12.0f);

        /* TAIL MOD: see TAIL_MOD_MAX_SEMITONES. */
        float intensity = Clamp01Added(hit.tail_mod);
        wobble_depth = TAIL_MOD_MAX_SEMITONES * intensity;
        wobble_increment =
            (TAIL_MOD_RATE_MIN_HZ +
             (TAIL_MOD_RATE_MAX_HZ - TAIL_MOD_RATE_MIN_HZ) * intensity * intensity) /
            SAMPLE_RATE;
        wobble_irregular = TAIL_MOD_IRREGULAR * intensity;

        sub_attack_samples = MsToSamples(SUB_ATTACK_MS);

        /* The gap's rise is TAIL ATTACK. */
        gap_rise =
            MsToSamples(
                ClampAdded(hit.sub_attack_ms, TAIL_ATTACK_MIN_MS, TAIL_ATTACK_MAX_MS));

        if(gap_rise < 1)
            gap_rise = 1;

        /* SWEEP TIME stretches the whole punch: its sweep and its window. */
        float time_scale = ClampAdded(hit.sweep_time_scale, 0.25f, 4.0f);

        float start_ratio = PunchStartRatio(punch);

        if(base_hz * start_ratio > PUNCH_START_CEILING_HZ)
            start_ratio = PUNCH_START_CEILING_HZ / base_hz;

        if(start_ratio < 1.0f)
            start_ratio = 1.0f;

        float sweep_seconds = PunchSweepMs(punch) * 0.001f * time_scale;

        sweep_depth = start_ratio - 1.0f;
        sweep = 1.0f;
        sweep_coefficient = Decay60Coefficient(sweep_seconds);

        punch_env = 1.0f;
        punch_coefficient =
            Decay60Coefficient(sweep_seconds * PUNCH_AMP_DECAY_PER_SWEEP);
        punch_done = false;

        punch_level_scale =
            KICK_PUNCH_OLD_LEVEL
            ? OldPunchLevel(punch)
            : KICK_PUNCH_LEVEL;

        punch_attack_samples =
            MsToSamples(
                KICK_PUNCH_OLD_ANATOMY
                ? ShapeThreePoint(punch,
                                  SHAPE_ROUND_ATTACK_MS,
                                  SHAPE_PUNCH_ATTACK_MS,
                                  SHAPE_SNAP_ATTACK_MS)
                : PUNCH_ATTACK_MS
            );

        anatomy_start =
            MsToSamples(
                time_scale *
                ShapeThreePoint(punch,
                                SHAPE_ROUND_HANDOFF_START_MS,
                                SHAPE_PUNCH_HANDOFF_START_MS,
                                SHAPE_SNAP_HANDOFF_START_MS));

        anatomy_end =
            MsToSamples(
                time_scale *
                ShapeThreePoint(punch,
                                SHAPE_ROUND_HANDOFF_END_MS,
                                SHAPE_PUNCH_HANDOFF_END_MS,
                                SHAPE_SNAP_HANDOFF_END_MS));

        if(punch_attack_samples < 1)
            punch_attack_samples = 1;

        if(anatomy_end <= anatomy_start)
            anatomy_end = anatomy_start + 1;

        /* The gap holds the whole punch through, then opens to the tail. */
        gap = hit.delay_ms > 0.0f;
        gap_hold =
            KICK_PUNCH_OLD_ANATOMY
            ? anatomy_end
            : MsToSamples(sweep_seconds * 1000.0f);
        gap_fall = MsToSamples(TAIL_GAP_FALL_MS);
        gap_return = MsToSamples(hit.delay_ms);

        punch_drive =
            ShapeThreePoint(punch,
                            SHAPE_ROUND_DRIVE,
                            SHAPE_PUNCH_DRIVE,
                            SHAPE_SNAP_DRIVE);

        tone_a =
            expf(
                -TWO_PI *
                ClampAdded(
                    ShapeThreePoint(punch,
                                    SHAPE_ROUND_CUTOFF_HZ,
                                    SHAPE_PUNCH_CUTOFF_HZ,
                                    SHAPE_SNAP_CUTOFF_HZ),
                    500.0f,
                    12000.0f) /
                SAMPLE_RATE
            );

        /* The punch path starts at zero, so its filters may too. */
        tone_state = 0.0f;
        guard_state_1 = 0.0f;
        guard_state_2 = 0.0f;

        u = 0.0f;
        sub_env = 1.0f;
        sub_done = false;
        SetDecay(decay_seconds);

        /*
         * Seed the history one step behind the first sample, so the first
         * measured increment is the real one.
         */
        last_increment =
            BaseFrequency(0) * (1.0f + sweep_depth) / SAMPLE_RATE;
        last_phase = -last_increment;
        last_level = 0.0f;
        last_level_step = 0.0f;
        last_sweep_increment = 0.0f;
        last_punch = 0.0f;

        handoff = false;

        /* A fresh hit starts at phase 0, where the old sharp attack is safe. */
        punch_sharp_attack = true;

        if(!hand_off || old_level <= KICK_VOICE_SILENT)
        {
            /* Fresh: the saws start locked to the oscillator. */
            for(int i = 0; i < WAVE_SAWS; i++)
            {
                saw_offset[i] = 0.0f;
                saw_offset_start[i] = 0.0f;
                saw_offset_sin[i] = 0.0f;
                saw_offset_cos[i] = 1.0f;
            }


            return;
        }

        /*
         * Handed off: each saw's offset glides from where it was to zero over
         * the handoff, and the carried sub keeps its own wave.
         */
        for(int i = 0; i < WAVE_SAWS; i++)
            saw_offset_start[i] = saw_offset[i];

        carried_morph = old_morph;

        /*
         * Handed off, the punch starts on the old sine's phase, where a
         * sharp attack would be a step; it rises on the raised cosine.
         */
        punch_sharp_attack = false;

        /* Shortest way round: the offset is within half a cycle. */
        handoff = true;
        handoff_age = 0;
        handoff_offset = old_phase > 0.5f ? old_phase - 1.0f : old_phase;
        handoff_drift = old_base_increment - BaseFrequency(0) / SAMPLE_RATE;

        /*
         * The drift term fades out over the window and in doing so adds
         * drift * N / 2 cycles; the correction brings the total to a whole
         * number of cycles, shortest way round, so the handoff lands exactly
         * on the new hit's phase. It is spread on a smoothstep, so it moves
         * the pitch by at most half a cycle over the window (~9 Hz).
         */
        float n_total = static_cast<float>(MsToSamples(RETRIGGER_HANDOFF_MS));
        float landing = handoff_offset + handoff_drift * n_total * 0.5f;
        handoff_correction = -(landing - floorf(landing + 0.5f));
        handoff_level = old_level;
        handoff_slope = old_slope;

        /* The first sample lands on old_phase; seed one step behind. */
        last_phase = old_phase - old_increment;
    }


    /*
     * Hand this voice to a fading slot: carry on, fade to zero. punch_only
     * silences its sub path, for when the new hit carries the sub on.
     */
    void BeginFadeOut(bool punch_only)
    {
        fading = true;
        fading_punch_only = punch_only;
        fade_age = 0;
    }


    /* Punch path level at this age, before the path's tone shaping. */
    float PunchLevel()
    {
        if(punch_done)
            return 0.0f;

        float attack =
            RaisedCosine01(
                static_cast<float>(age) /
                static_cast<float>(punch_attack_samples)
            );

        if(KICK_PUNCH_OLD_ANATOMY && punch_sharp_attack)
        {
            /*
             * The old one-pole approach: -60 dB of distance left after the
             * attack time. It starts at exactly zero and, on a sine that
             * starts at phase 0, cannot step; its sharp front is the old
             * punch's edge.
             */
            attack =
                1.0f -
                expf(
                    -6.9078f *
                    static_cast<float>(age) /
                    static_cast<float>(punch_attack_samples)
                );
        }

        if(KICK_PUNCH_OLD_ANATOMY)
        {
            /* Held, then the equal-power fade-out half of the handoff. */
            return attack * cosf(AnatomyPosition() * 1.57079632679f);
        }

        /* Offset by the silence floor so it lands on zero rather than stepping. */
        return attack *
               fmaxf(0.0f, punch_env - KICK_VOICE_SILENT) /
               (1.0f - KICK_VOICE_SILENT);
    }


    /* The punch path's tone chain, on the unit sine, as the old engine had. */
    float ShapePunch(float s)
    {
        if(KICK_PUNCH_OLD_DRIVE)
        {
            float age_ms =
                static_cast<float>(age) * 1000.0f / SAMPLE_RATE;

            float mix =
                SmoothstepAdded(
                    Clamp01Added(
                        (age_ms - OLD_DRIVE_PURE_MS) / OLD_DRIVE_FADE_MS
                    )
                );

            s += (SoftClip(s * punch_drive) - s) * mix;
        }

        if(KICK_PUNCH_OLD_TONE)
        {
            tone_state = (1.0f - tone_a) * s + tone_a * tone_state;
            s = tone_state;
        }

        if(KICK_PUNCH_OLD_HF_GUARD)
        {
            float age_ms =
                static_cast<float>(age) * 1000.0f / SAMPLE_RATE;

            float cutoff =
                OLD_HF_GUARD_INITIAL_HZ +
                (OLD_HF_GUARD_FINAL_HZ - OLD_HF_GUARD_INITIAL_HZ) *
                SmoothstepAdded(Clamp01Added(age_ms / OLD_HF_GUARD_OPEN_MS));

            float a = expf(-TWO_PI * cutoff / SAMPLE_RATE);

            guard_state_1 = (1.0f - a) * s + a * guard_state_1;
            guard_state_2 = (1.0f - a) * guard_state_1 + a * guard_state_2;
            s = guard_state_2;
        }

        return s;
    }


    /* 1 once this hit's wet onset hold is over; see KICK_OLD_WET_ONSET. */
    float WetOpen() const
    {
        if(!KICK_OLD_WET_ONSET)
            return 1.0f;

        float age_ms = static_cast<float>(age) * 1000.0f / SAMPLE_RATE;

        return SmoothstepAdded(
            Clamp01Added((age_ms - OLD_WET_OPEN_MS) / OLD_WET_OPEN_FADE_MS)
        );
    }


    /* The TAIL DELAY envelope over the whole voice, 0..1, at this age. */
    float GapLevel() const
    {
        if(!gap || age < gap_hold)
            return 1.0f;

        float down =
            1.0f -
            SmoothstepAdded(
                static_cast<float>(age - gap_hold) /
                static_cast<float>(gap_fall)
            );

        float up =
            age < gap_return
            ? 0.0f
            : SmoothstepAdded(
                  static_cast<float>(age - gap_return) /
                  static_cast<float>(gap_rise)
              );

        /*
         * Combined as 1 - (1 - down)(1 - up): 1 whenever either is, so a tail
         * delay inside the punch never closes the gap, but with no corner
         * where a fall and a rise overlap. fmaxf() left one there, a slope
         * reversal in a single sample that measured as a click.
         */
        return 1.0f - (1.0f - down) * (1.0f - up);
    }


    /* Adds this voice's sample to o, before the mixer gains. */
    void Process(KickVoiceOut& o)
    {
        if(!active)
            return;

        float gap_level = GapLevel();

        float punch_level =
            PunchLevel() *
            punch_level_scale;

        /* Sub: silent until its start, raised-cosine onset, then decay. */
        float sub_level = 0.0f;

        if(age >= start && !sub_done)
        {
            float remaining = 1.0f - u;

            float decay =
                KICK_OLD_SUB_DECAY
                ? fmaxf(0.0f, sub_env - KICK_VOICE_SILENT) /
                  (1.0f - KICK_VOICE_SILENT)
                : remaining * remaining;

            sub_level =
                decay *
                RaisedCosine01(
                    static_cast<float>(age - start) /
                    static_cast<float>(sub_attack_samples)
                ) *
                KICK_SUB_LEVEL;

            /* Profile 1+: the fade-in half of the handoff. */
            if(KICK_PUNCH_OLD_ANATOMY)
                sub_level *= sinf(AnatomyPosition() * 1.57079632679f);
        }

        float total_phase = phase;

        /* The old hit's sub still being carried, kept apart for the send. */
        float carried_level = 0.0f;

        if(handoff)
        {
            float s =
                SmoothstepAdded(
                    static_cast<float>(handoff_age) /
                    static_cast<float>(MsToSamples(RETRIGGER_HANDOFF_MS))
                );

            /*
             * Frequency glides from the old base pitch to the new one: the
             * drift decays along (1 - s), whose running sum over the window
             * is N (u - u^3 + u^4 / 2) for a smoothstep s. The old form,
             * (offset + drift * t) * (1 - s), undershot badly when the two
             * pitches were far apart (TAIL MOD makes that common) and could
             * run the sine backwards mid-window.
             */
            float u =
                static_cast<float>(handoff_age) /
                static_cast<float>(MsToSamples(RETRIGGER_HANDOFF_MS));
            float glide_sum =
                static_cast<float>(MsToSamples(RETRIGGER_HANDOFF_MS)) *
                (u - u * u * u + 0.5f * u * u * u * u);

            total_phase +=
                handoff_offset +
                handoff_drift * glide_sum +
                handoff_correction * s;

            handoff_level += handoff_slope;
            handoff_slope *= LEVEL_SLOPE_BEND_A;

            if(handoff_level < 0.0f)
                handoff_level = 0.0f;

            carried_level = handoff_level * (1.0f - s);
            sub_level *= s;

            handoff_age++;

            if(s >= 1.0f)
                handoff = false;
        }

        total_phase -= floorf(total_phase);

        float sine = sinf(total_phase * TWO_PI);

        float increment = total_phase - last_phase;
        increment -= floorf(increment);

        /*
         * WAVE. At 0 this is exactly the sine, bit for bit. The saws follow
         * the sine's actual step (sweep, glides and handoff included), so
         * they stay locked to its pitch.
         */
        float wave = sine;
        float carried_wave = sine;

        if(wave_morph > 0.0f || (handoff && carried_morph > 0.0f))
        {
            float glide =
                handoff
                ? 1.0f - SmoothstepAdded(
                             static_cast<float>(handoff_age) /
                             static_cast<float>(MsToSamples(RETRIGGER_HANDOFF_MS)))
                : 0.0f;

            float seconds = static_cast<float>(age) / SAMPLE_RATE;
            float dt = fminf(increment, 0.45f);
            bool refresh = (age % WAVE_OFFSET_REFRESH) == 0 || handoff;

            float sine_cos = cosf(total_phase * TWO_PI);
            float harmonics = 0.0f;

            for(int i = 0; i < WAVE_SAWS; i++)
            {
                /* Drift from the hit; a handoff glides from the old offsets. */
                saw_offset[i] =
                    saw_offset_start[i] * glide +
                    WAVE_SAW_DETUNE_HZ[i] * seconds;

                if(refresh)
                {
                    saw_offset_sin[i] = sinf(saw_offset[i] * TWO_PI);
                    saw_offset_cos[i] = cosf(saw_offset[i] * TWO_PI);
                }

                /* Rising saw, zero at phase 0: its fundamental is in phase. */
                float q = total_phase + saw_offset[i] + 0.5f;
                q -= floorf(q);

                /* This saw's own fundamental, sin(2 pi (phase + offset)). */
                float own =
                    sine * saw_offset_cos[i] +
                    sine_cos * saw_offset_sin[i];

                harmonics +=
                    WAVE_SAW_WEIGHT[i] *
                    (PolyBlepSaw(q, dt) - WAVE_SAW_FUNDAMENTAL * own);
            }

            harmonics *= WAVE_HARMONICS_SCALE / WAVE_SAW_WEIGHT_SUM;

            /* The sine stays the fundamental; WAVE adds the saw harmonics. */
            wave = sine + harmonics * wave_morph;
            carried_wave = sine + harmonics * carried_morph;
        }

        /* The punch filters run from its first sample so their state is real. */
        float punch_out =
            ShapePunch(wave) * punch_level * gap_level *
            (1.0f + WAVE_PUNCH_BOOST * wave_morph);
        float sub_out = wave * sub_level * gap_level;
        float carried_out = carried_wave * carried_level * gap_level;
        float open = WetOpen();

        /* What a retrigger carries on from is the level actually heard. */
        float heard = (sub_level + carried_level) * gap_level;

        last_increment = increment;
        last_phase = total_phase;
        last_level_step = heard - last_level;
        last_level = heard;
        last_punch = punch_out;

        /* Advance the one oscillator. */
        float base_frequency = BaseFrequency(SubAge());

        float frequency =
            base_frequency *
            (1.0f + sweep_depth * sweep);

        float phase_increment = frequency / SAMPLE_RATE;

        if(phase_increment > 0.45f)
            phase_increment = 0.45f;

        last_sweep_increment =
            phase_increment -
            base_frequency / SAMPLE_RATE;

        phase += phase_increment;

        if(phase >= 1.0f)
            phase -= 1.0f;

        sweep *= sweep_coefficient;
        punch_env *= punch_coefficient;

        if(KICK_PUNCH_OLD_ANATOMY
           ? age >= anatomy_end
           : punch_env < KICK_VOICE_SILENT)
        {
            punch_done = true;
        }

        if(age >= start && !sub_done)
        {
            u += du;

            float exponential = sub_env * sub_env_coefficient;

            sub_env =
                exponential +
                (sub_env - sub_env_linear_step - exponential) *
                OLD_SUB_DECAY_LINEARITY;

            if(KICK_OLD_SUB_DECAY
               ? sub_env <= KICK_VOICE_SILENT
               : u >= 1.0f)
            {
                u = 1.0f;
                sub_done = true;
            }
        }

        age++;

        float fade = 1.0f;

        if(fading)
        {
            fade =
                1.0f -
                SmoothstepAdded(
                    static_cast<float>(fade_age) /
                    static_cast<float>(MsToSamples(RETRIGGER_HANDOFF_MS))
                );

            fade_age++;

            if(fade_age >= MsToSamples(RETRIGGER_HANDOFF_MS))
                active = false;
        }

        if(fading_punch_only)
        {
            sub_out = 0.0f;
            carried_out = 0.0f;
        }

        o.punch += punch_out * fade;
        o.sub += (sub_out + carried_out) * fade;

        /* The carried sub was already feeding the models; it is not held off. */
        o.send += ((punch_out + sub_out) * open + carried_out) * fade;
        o.bpf_punch += punch_out * open * fade;

        /* Done once both paths are spent and no handoff is fading. */
        if(sub_done && punch_done && !handoff)
        {
            active = false;
            last_level = 0.0f;
        }
    }
};


static KickVoice kick_voice;
static KickVoice kick_voice_fading[KICK_FADING_SLOTS];

static float kick_punch_gain_smoothed = 1.0f;
static float kick_sub_gain_smoothed = 0.95f;

/* This hit started from silence; see KICK_OLD_ONSET_LEVEL. */
static bool kick_fresh_hit = true;
static float kick_onset_level = 1.0f;

/* KICK_OLD_FINAL_HF_GUARD state. Never reset on a trigger. */
static float final_hf_guard_cutoff = OLD_FINAL_HF_SETTLED_HZ;
static float final_hf_guard_state[3] = {0.0f, 0.0f, 0.0f};


/*
 * K3 / TAIL DELAY as the gap before the sub starts. Latched per hit, so
 * moving K3 changes the next kick rather than stepping the current one.
 */
static float KickSubDelayMs()
{
    if(!tail_delay_enabled || macro_tail_delay <= 0.005f)
        return 0.0f;

    return MacroTailDelayMs(macro_tail_delay);
}


/* A free fading slot, else the one furthest through its fade. */
static KickVoice& FadingSlot()
{
    int slot = 0;

    for(int i = 0; i < KICK_FADING_SLOTS; i++)
    {
        if(!kick_voice_fading[i].active)
            return kick_voice_fading[i];

        if(kick_voice_fading[i].fade_age >
           kick_voice_fading[slot].fade_age)
        {
            slot = i;
        }
    }

    return kick_voice_fading[slot];
}


static void TriggerKickVoice(uint8_t velocity)
{
    float delay_ms = KickSubDelayMs();
    bool sounding = kick_voice.Sounding();

    kick_fresh_hit = !kick_voice.active;

    for(int i = 0; i < KICK_FADING_SLOTS; i++)
    {
        if(kick_voice_fading[i].active)
            kick_fresh_hit = false;
    }

    if(kick_voice.PunchSounding())
    {
        /* The sub is handed off below; the old punch finishes on its own. */
        KickVoice& slot = FadingSlot();
        slot = kick_voice;
        slot.BeginFadeOut(true);
    }

    KickHitParams hit;
    hit.frequency = kick_frequency;
    hit.punch = macro_kick_shape;
    hit.velocity = velocity;
    hit.delay_ms = delay_ms;
    hit.decay_seconds = MacroDecaySeconds(macro_decay);
    hit.sub_attack_ms = macro_tail_attack_ms;
    hit.sweep_time_scale = macro_punch_time_scale;
    hit.tail_mod = macro_tail_mod;
    hit.wave = macro_wave;

    kick_voice.Trigger(hit, sounding);

    kick_age_samples = 0;
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


/*
 * High-pass on the Mackie / Tube RETURN only: two one-poles at 120 Hz.
 * POLE_A = expf(-2*pi*120/48000).
 */
static constexpr float CHARACTER_SUB_PROTECT_POLE_A = 0.98441477f;

static float character_delta_hp_state = 0.0f;
static float character_delta_hp_state_2 = 0.0f;


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
   SOFT SATURATION
   ============================================================ */



/*
 * SIDECHAIN REVERB (CC59) — post-mixer, last thing before the ceiling.
 *
 * Schroeder topology: four parallel combs into two series allpasses. The
 * SEND is high-passed at 250 Hz so only the punch and upper body excite the
 * tank; letting the sub in turns a kick reverb to mud immediately.
 *
 * The return is then ducked by the dry kick's own envelope, so the tail
 * blooms in the gaps rather than smearing over the attack. That is the
 * sidechain: no external key input, the kick keys itself.
 */
struct KickSidechainReverb
{
    static constexpr int C0 = 1116, C1 = 1188, C2 = 1277, C3 = 1356;
    static constexpr int A0 = 556, A1 = 441;

    float comb0[C0] = {}, comb1[C1] = {}, comb2[C2] = {}, comb3[C3] = {};
    float ap0[A0] = {}, ap1[A1] = {};
    int ci0 = 0, ci1 = 0, ci2 = 0, ci3 = 0, ai0 = 0, ai1 = 0;
    float lp0 = 0.0f, lp1 = 0.0f, lp2 = 0.0f, lp3 = 0.0f;

    float send_hp_state = 0.0f;
    /* Set by Trigger(); the tank is emptied once the gain has ramped out. */
    bool clear_pending = false;
    /* MUST default to zero: any non-zero member initialiser moves this whole
     * object (32 KB of comb buffers) out of .bss into .data, i.e. into FLASH.
     * Reset() sets the real starting value. */
    float duck_gain = 0.0f;

    void ClearTank()
    {
        for(int i = 0; i < C0; ++i) comb0[i] = 0.0f;
        for(int i = 0; i < C1; ++i) comb1[i] = 0.0f;
        for(int i = 0; i < C2; ++i) comb2[i] = 0.0f;
        for(int i = 0; i < C3; ++i) comb3[i] = 0.0f;
        for(int i = 0; i < A0; ++i) ap0[i] = 0.0f;
        for(int i = 0; i < A1; ++i) ap1[i] = 0.0f;
        ci0 = ci1 = ci2 = ci3 = ai0 = ai1 = 0;
        lp0 = lp1 = lp2 = lp3 = 0.0f;
    }

    void Reset()
    {
        ClearTank();
        send_hp_state = 0.0f;
        duck_gain = 1.0f;
        clear_pending = false;
    }

    /*
     * Called from the kick Note-On. The tail must not run into the next hit,
     * so the tank is genuinely emptied rather than just turned down. Zeroing
     * the buffers is inaudible here precisely because the gain goes to zero
     * in the same instant; it then blooms back up as the new kick feeds it.
     *
     * send_hp_state is deliberately left alone - resetting it would step the
     * high-pass and inject a transient into the fresh send.
     */
    void Trigger()
    {
        /*
         * Do NOT cut here. Stepping duck_gain to zero in one sample is an
         * amplitude discontinuity in the output - an audible click, and a
         * louder one the longer the decay, because a longer tail leaves more
         * energy standing in the tank. Ramp out over REVERB_DUCK_CUT_MS and
         * empty the tank only once the gain is actually at zero, where the
         * discontinuity really is inaudible.
         */
        clear_pending = true;
    }

    static float Comb(float in, float* buf, int size, int& idx, float& store)
    {
        float out = buf[idx];
        /* Damped feedback: a bare comb rings metallic on a percussive send. */
        store = out * (1.0f - REVERB_DAMPING) + store * REVERB_DAMPING;
        buf[idx] = in + store * REVERB_FEEDBACK;
        if(++idx >= size)
            idx = 0;
        return out;
    }

    static float Allpass(float in, float* buf, int size, int& idx)
    {
        float buffered = buf[idx];
        float out = -in + buffered;
        buf[idx] = in + buffered * 0.5f;
        if(++idx >= size)
            idx = 0;
        return out;
    }

    float Process(float dry, float amount)
    {
        if(amount <= 0.001f)
        {
            /* Track the input so re-enabling does not thump. */
            send_hp_state = dry;
            duck_gain = 1.0f;
            clear_pending = false;
            return dry;
        }

        /* 250 Hz high-pass on the send only. */
        send_hp_state =
            (1.0f - REVERB_SEND_HP_POLE_A) * dry +
            REVERB_SEND_HP_POLE_A * send_hp_state;

        float send = (dry - send_hp_state) * REVERB_SEND_LEVEL;

        float wet =
            Comb(send, comb0, C0, ci0, lp0) +
            Comb(send, comb1, C1, ci1, lp1) +
            Comb(send, comb2, C2, ci2, lp2) +
            Comb(send, comb3, C3, ci3, lp3);

        wet *= 0.25f;

        wet = Allpass(wet, ap0, A0, ai0);
        wet = Allpass(wet, ap1, A1, ai1);

        if(clear_pending)
        {
            /* Fast but finite ramp out, then empty the tank at silence. */
            duck_gain -= REVERB_DUCK_CUT_STEP;

            if(duck_gain <= 0.0f)
            {
                duck_gain = 0.0f;
                ClearTank();
                clear_pending = false;
            }
        }
        else
        {
            duck_gain += (1.0f - duck_gain) * REVERB_DUCK_RECOVER_A;
            if(duck_gain > 1.0f)
                duck_gain = 1.0f;
        }

        if(!(wet == wet))
        {
            Reset();
            return dry;
        }

        return dry + wet * REVERB_RETURN_LEVEL * duck_gain * amount;
    }
};

static KickSidechainReverb kick_reverb;


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
    constexpr float knee  = OUTPUT_CEILING_KNEE;
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
     * Note-Off. K2 owns musical decay through the voice's sub envelope.
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

    uint8_t latched_rate_index = 2;

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

        latched_rate_index = 2;

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
         * CC30 division ladder, as multiples of a quarter note:
         *
         * 1/4, 1/4T, 1/8, 1/8T   where 1/4T = 1/6 and 1/8T = 1/12
         *
         * Must stay in step with STUT_VALUES and StutterRateFromCc, and with
         * the Teensy's STUT_LABELS ladder in src/KickPerformance.cpp.
         */
        static const float fractions[4] =
        {
            1.0f,
            0.666666667f,
            0.5f,
            0.333333333f
        };


        if(rate_index > 3)
            rate_index = 3;


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
         * LOOP_HISTORY_SAMPLES = 43200.
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

/*
 * DO NOT REINTRODUCE AN ONSET BYPASS HERE.
 *
 * This used to hold the kick path DRY for the first 4 ms of every hit
 * (keyed on kick_age_samples, which resets on every trigger) and crossfade
 * into the HPF over the next 12 ms. It assumed the kick starts from silence.
 * Whenever the previous hit was still sounding - DECAY INF, any overlap - it
 * snapped from the filtered signal back to the dry one in a single sample,
 * and even at the 24 Hz minimum cutoff the phase shift makes that a large
 * step. It was the loud click heard as soon as the HPF was engaged.
 *
 * The SVF is primed to the current input on enable and runs continuously, so
 * there is nothing for a bypass to protect against.
 */
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
        float input)
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


        return filtered;
    }
};

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
                logf(current_hz[i] / MACRO_BPF_LOW_HZ) /
                logf(MACRO_BPF_HIGH_HZ / MACRO_BPF_LOW_HZ);
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

            /*
             * The band now reaches down to 85 Hz, onto the kick's own
             * fundamental. A wide filter there just lifts everything the sub
             * and punch are already doing, so tighten it as it descends: down
             * low it should pick out a pitch, not add weight.
             *
             * Squared, so the whole upper range keeps exactly the character
             * it had and this only takes hold once the band is genuinely low.
             */
            float low_sharpen =
                (200.0f - current_hz[i]) /
                (200.0f - MACRO_BPF_LOW_HZ);

            if(low_sharpen < 0.0f)
                low_sharpen = 0.0f;

            if(low_sharpen > 1.0f)
                low_sharpen = 1.0f;


            return_q +=
                low_sharpen *
                low_sharpen *
                3.40f;


            if(return_q > 8.00f)
                return_q = 8.00f;


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
        uint8_t count = macro_bpf_layer_count_latched;

        /*
         * Compensation for the parallel energy each extra layer adds. This
         * is the ONLY layer-count attenuation: the dirty bus used to
         * subtract a second one, which also dimmed Mackie and Tube even
         * though they had gained no energy.
         */
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

        /*
         * Sits far above any musical level, so it never colours the sound.
         * It exists only so a runaway cannot climb to infinity and take the
         * rest of the chain with it.
         */
        return ClampAdded(added, -4.0f, 4.0f);
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

/*
 * Past the base drive, the first stage is already fully saturated, so more
 * drive only squares the same shape a little more: the knob stopped doing
 * anything. Instead, above MACKIE_BRIGHT_FROM the amount reshapes the tone
 * around the clippers, all scaled by one "brightness" that is 0 through the
 * base drive and 1 at full:
 *
 *   pre-emphasis   highs above ~1.2 kHz lifted INTO the first clipper, up
 *                  to +11 dB, so it generates denser, brighter harmonics
 *   presence / air the 1 kHz presence band grows and a 4.2 kHz air band
 *                  joins it, while the 340 Hz body band eases off
 *   second stage   driven harder for more harmonic density
 *   top end        the output low-pass opens from 10 kHz to 16 kHz
 *
 * The bass is not involved: the dry kick never passes through here, and the
 * return is high-passed at 120 Hz before it is added back.
 */
static constexpr float MACKIE_BRIGHT_FROM = 0.25f;
static constexpr float MACKIE_EMPHASIS_GAIN = 2.5f;       /* +11 dB above ~1.2 kHz */
static constexpr float MACKIE_EMPHASIS_LP_A = 0.14543f;    /* 1 - expf(-2pi 1200/48000) */
static constexpr float MACKIE_POST_A_DARK = 0.72990000f;   /* 10 kHz, as before */
static constexpr float MACKIE_POST_A_BRIGHT = 0.87670000f; /* 16 kHz */

struct MacroMackieProcessor
{
    Biquad body_band;
    Biquad presence_band;
    Biquad air_band;
    float emphasis_lp = 0.0f;

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
        air_band.Reset();
        air_band.SetBandpass(4200.0f, 0.70f);
        emphasis_lp = 0.0f;

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
            air_band.Process(0.0f);
            emphasis_lp = input;
            return 0.0f;
        }

        /* 0 through the base drive, 1 at full; see MACKIE_BRIGHT_FROM. */
        float bright =
            SmoothstepAdded(
                Clamp01Added(
                    (amount - MACKIE_BRIGHT_FROM) / (1.0f - MACKIE_BRIGHT_FROM)
                )
            );

        constexpr float pre_a = 0.79210000f; /* MACKIE_PRE_LP_HZ */
        pre_lp_1 += pre_a * (input - pre_lp_1);
        pre_lp_2 += pre_a * (pre_lp_1 - pre_lp_2);

        /* Pre-emphasis into the first clipper. */
        emphasis_lp += MACKIE_EMPHASIS_LP_A * (pre_lp_2 - emphasis_lp);
        float emphasised =
            pre_lp_2 +
            (pre_lp_2 - emphasis_lp) * MACKIE_EMPHASIS_GAIN * bright;

        float accumulated = 0.0f;
        for(int os = 1; os <= 4; ++os)
        {
            float t = static_cast<float>(os) * 0.25f;
            float x = previous_input + (emphasised - previous_input) * t;
            accumulated += Core(x * MACKIE_INTERNAL_GAIN);
        }
        previous_input = emphasised;

        float stage1 = accumulated * 0.25f;

        constexpr float dc_r = 0.99935f;
        float dc_blocked = stage1 - dc_x1 + dc_r * dc_y1;
        dc_x1 = stage1;
        dc_y1 = dc_blocked;

        /* overload -> broad desk EQ boosts -> overload again */
        float body = body_band.Process(dc_blocked);
        float presence = presence_band.Process(dc_blocked);
        float air = air_band.Process(dc_blocked);
        float eq_driven =
            dc_blocked +
            body * 0.62f * (1.0f - 0.4f * bright) +
            presence * (0.25f + 0.75f * bright) +
            air * 0.9f * bright;

        float stage2 = Core(eq_driven * 1.65f * (1.0f + 0.8f * bright));

        float post_a =
            MACKIE_POST_A_DARK +
            (MACKIE_POST_A_BRIGHT - MACKIE_POST_A_DARK) * bright;
        post_lp_1 += post_a * (stage2 - post_lp_1);
        post_lp_2 += post_a * (post_lp_1 - post_lp_2);

        return post_lp_2 * param_mackie_gain;
    }
};

/* ============================================================
   TUBE — TWO-STAGE TRIODE PREAMP
   ============================================================

   Replaces the Sherman VCF-4 model on K5's second slot (CC49 amount,
   CC55 mixer gain, CC50 model switch). A musical model of two cascaded
   common-cathode triode stages, not a circuit simulation. What makes it
   read as a tube rather than another clipper:

       asymmetric transfer     the grid starts conducting on positive
                               swings, so they flatten early and hard;
                               negative swings run into cutoff on a much
                               softer knee. The mismatch is even-order
                               harmonics, the "warm" part.
       blocking bias shift     grid current charges the coupling cap, so
                               a loud hit drags the operating point
                               negative and the stage compresses and
                               sags, recovering over ~60 ms. Level
                               dependent, so the drive responds to the
                               kick's envelope instead of clipping flat.
       two inverting stages    each triode inverts, and the second is
                               biased differently, so the pair does not
                               simply cancel its own asymmetry.
       coupling caps           ~20 Hz high-pass between stages.
       roll-off                a gentle 2-pole top end at ~7.5 kHz, and a
                               low-mid bump at ~180 Hz from the plate load.

   Nonlinear stages run 4x oversampled, as the Mackie does.
   ============================================================ */

/* Stage operating points, as a fraction of the grid swing. */
static constexpr float TUBE_STAGE1_BIAS = -0.18f;
static constexpr float TUBE_STAGE2_BIAS = -0.32f;

/* Input drive into stage 1 at full amount (the amount knob adds more). */
static constexpr float TUBE_INPUT_GAIN = 3.2f;
static constexpr float TUBE_INTERSTAGE_GAIN = 2.4f;

/* Blocking: how much grid current shifts the bias, and how fast it recovers. */
static constexpr float TUBE_BLOCKING_DEPTH = 0.45f;
static constexpr float TUBE_BLOCKING_CHARGE_A = 0.0052f;    /* ~1 ms at the 4x rate */
static constexpr float TUBE_BLOCKING_RECOVER_A = 0.0000868f; /* ~60 ms at the 4x rate */

/*
 * Output staging, and polarity. Negative on purpose: with it positive the
 * return came back out of phase with the dry kick around 100-150 Hz and
 * cancelled 2-3 dB of it (measured with the host harness). Negative, the
 * tube's low harmonics add to the dry kick instead, about +4 dB below 150 Hz.
 */
static constexpr float TUBE_OUTPUT_GAIN = -1.0f;


struct MacroTubeProcessor
{
    Biquad plate_bump;

    float previous_input = 0.0f;
    float pre_lp_1 = 0.0f;
    float pre_lp_2 = 0.0f;

    /* Coupling caps: input, interstage, output. */
    float couple_in = 0.0f;
    float couple_mid = 0.0f;
    float couple_out_x = 0.0f;
    float couple_out_y = 0.0f;

    /* Blocking bias shift, per stage. */
    float block_1 = 0.0f;
    float block_2 = 0.0f;

    float post_lp_1 = 0.0f;
    float post_lp_2 = 0.0f;


    /*
     * One triode stage, inverting. x is the grid swing, bias the operating
     * point. Above the grid-conduction point the curve flattens hard; below
     * it runs into cutoff on a soft knee. Normalised so the output is zero
     * at the operating point, whatever the bias.
     */
    static float Triode(float x, float bias)
    {
        float v = x + bias;

        float plate;

        if(v >= 0.0f)
        {
            /* Grid conduction: early, firm compression. */
            plate = v / (1.0f + 2.2f * v);
        }
        else
        {
            /* Towards cutoff: a much softer knee. */
            float u = -v;
            plate = -u / sqrtf(1.0f + 0.55f * u * u);
        }

        float rest =
            bias >= 0.0f
            ? bias / (1.0f + 2.2f * bias)
            : bias / sqrtf(1.0f + 0.55f * bias * bias);

        return -(plate - rest);
    }


    void Reset()
    {
        plate_bump.Reset();
        plate_bump.SetBandpass(180.0f, 0.70f);

        previous_input = 0.0f;
        pre_lp_1 = pre_lp_2 = 0.0f;
        couple_in = couple_mid = 0.0f;
        couple_out_x = couple_out_y = 0.0f;
        block_1 = block_2 = 0.0f;
        post_lp_1 = post_lp_2 = 0.0f;
    }


    void Trigger() {}


    float Process(float input, float amount)
    {
        if(amount <= CHARACTER_HEAVY_PROCESS_EPSILON)
        {
            previous_input = input;
            pre_lp_1 = pre_lp_2 = input;
            couple_in = input;
            couple_mid = 0.0f;
            couple_out_x = couple_out_y = 0.0f;
            block_1 = block_2 = 0.0f;
            post_lp_1 = post_lp_2 = 0.0f;
            plate_bump.Process(0.0f);
            return 0.0f;
        }

        constexpr float pre_a = 0.79210000f; /* 12 kHz, as the Mackie */
        pre_lp_1 += pre_a * (input - pre_lp_1);
        pre_lp_2 += pre_a * (pre_lp_1 - pre_lp_2);

        /* Input coupling cap, ~20 Hz. */
        constexpr float couple_a = 0.00261f;
        couple_in += couple_a * (pre_lp_2 - couple_in);
        float grid = pre_lp_2 - couple_in;

        float stage2_sum = 0.0f;

        for(int os = 1; os <= 4; ++os)
        {
            float t = static_cast<float>(os) * 0.25f;
            float x =
                (previous_input + (grid - previous_input) * t) *
                TUBE_INPUT_GAIN;

            /* Stage 1, with its bias dragged down by grid current. */
            float bias_1 = TUBE_STAGE1_BIAS - block_1 * TUBE_BLOCKING_DEPTH;
            float s1 = Triode(x, bias_1);

            float grid_current_1 = fmaxf(0.0f, x + bias_1);
            block_1 +=
                (grid_current_1 > block_1 ? TUBE_BLOCKING_CHARGE_A
                                          : TUBE_BLOCKING_RECOVER_A) *
                (grid_current_1 - block_1);

            /* Interstage coupling cap, ~20 Hz, at the oversampled rate. */
            constexpr float mid_a = 0.000654f;
            couple_mid += mid_a * (s1 - couple_mid);
            float x2 = (s1 - couple_mid) * TUBE_INTERSTAGE_GAIN;

            float bias_2 = TUBE_STAGE2_BIAS - block_2 * TUBE_BLOCKING_DEPTH;
            float s2 = Triode(x2, bias_2);

            float grid_current_2 = fmaxf(0.0f, x2 + bias_2);
            block_2 +=
                (grid_current_2 > block_2 ? TUBE_BLOCKING_CHARGE_A
                                          : TUBE_BLOCKING_RECOVER_A) *
                (grid_current_2 - block_2);

            stage2_sum += s2;
        }

        previous_input = grid;

        float stage2 = stage2_sum * 0.25f;

        /* Output coupling cap / DC block. */
        constexpr float dc_r = 0.99935f;
        float coupled = stage2 - couple_out_x + dc_r * couple_out_y;
        couple_out_x = stage2;
        couple_out_y = coupled;

        /* Plate-load low-mid bump. */
        float voiced = coupled + plate_bump.Process(coupled) * 0.35f;

        constexpr float post_a = 0.62500000f; /* ~7.5 kHz */
        post_lp_1 += post_a * (voiced - post_lp_1);
        post_lp_2 += post_a * (post_lp_1 - post_lp_2);

        return post_lp_2 * TUBE_OUTPUT_GAIN * param_tube_gain;
    }
};


struct MacroCharacterProcessor
{
    MacroMackieProcessor mackie;
    MacroTubeProcessor tube;

    float mackie_amount_smoothed = 0.0f;
    float tube_amount_smoothed = 0.0f;

    enum class SwitchState
    {
        STABLE,
        FADE_TO_ZERO,
        FADE_FROM_ZERO
    };

    SwitchState switch_state = SwitchState::STABLE;
    bool active_tube = false;
    bool desired_tube = false;
    float transition_gain = 1.0f;

    static bool AudioValueSafe(float x)
    {
        return (x == x) && fabsf(x) < 8.0f;
    }

    void PrepareMackie() { mackie.Reset(); }
    void PrepareTube() { tube.Reset(); }

    void Reset()
    {
        PrepareMackie();
        PrepareTube();
        mackie_amount_smoothed = 0.0f;
        tube_amount_smoothed = 0.0f;
        active_tube = macro_character_tube;
        desired_tube = active_tube;
        switch_state = SwitchState::STABLE;
        transition_gain = 1.0f;
        character_switch_pending = false;
        character_switch_target_tube = active_tube;
    }

    void Trigger()
    {
        if(active_tube)
            tube.Trigger();
        else
            mackie.Trigger();
    }

    void ConsumeSwitchRequest()
    {
        if(!character_switch_pending)
            return;

        desired_tube = character_switch_target_tube;
        character_switch_pending = false;
        if(desired_tube != active_tube)
            switch_state = SwitchState::FADE_TO_ZERO;
    }

    static float SmoothAmount(float current, float target)
    {
        constexpr float a = 0.99884392f;  // ~18 ms at 48 kHz
        return target + (current - target) * a;
    }

    float CurrentSmoothedAmount() const
    {
        return active_tube ? tube_amount_smoothed : mackie_amount_smoothed;
    }

    float ProcessSelectedWet(float input)
    {
        if(active_tube)
        {
            float target = Clamp01Added(macro_tube_amount);
            tube_amount_smoothed = SmoothAmount(tube_amount_smoothed, target);
            float drive =
                1.0f + tube_amount_smoothed * CHARACTER_AMOUNT_DRIVE_RANGE;
            float wet = tube.Process(input * drive, tube_amount_smoothed);
            if(!AudioValueSafe(wet))
            {
                PrepareTube();
                return 0.0f;
            }
            return wet * tube_amount_smoothed;
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
                active_tube = desired_tube;

                if(active_tube)
                {
                    if(CHARACTER_RESET_ON_MODEL_SWITCH)
                        tube_amount_smoothed = 0.0f;
                    PrepareTube();
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
     * Rolling pre-FX capture for the looper, which lives on the external
     * lane, so this holds the Digitakt return rather than the kick.
     *
     * 43200 samples = 900 ms at 48 kHz. The chop is a live processor and
     * needs no capture, so the looper is the only reader.
     */
    static constexpr uint32_t LOOP_HISTORY_SAMPLES = 43200;
    float loop_history[LOOP_HISTORY_SAMPLES];
    uint32_t loop_history_write = 0;


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
            i < LOOP_HISTORY_SAMPLES;
            ++i)
        {
            loop_history[i] = 0.0f;
        }

        loop_history_write = 0;
    }


    void TriggerKick()
    {
        /*
         * Real audible kick drives the external sidechain envelope using
         * the SAME K6 sweep duration as the kick itself.
         */
        pump.TriggerWithSweepMs(
            MacroKickShapeSweepSeconds(
                macro_kick_shape
            ) *
            1000.0f
        );
    }


    void OnSixteenth()
    {
        /*
         * The kick's own chop still waits for a kick boundary; everything
         * on the external lane lands here instead.
         */
        ServiceExternalQuantizedCommands();
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
                0
            );
        }
    }


    /*
     * External-lane quantization point.
     *
     * The external stutter and the looper both live on the Digitakt
     * return, which has to keep working when the kick channel is silent,
     * so they land on the clock instead of on a kick boundary.
     */
    void ServiceExternalQuantizedCommands()
    {
        uint32_t stutter_command =
            external_stutter_quantized_command;


        if(stutter_command !=
           QUANT_FX_NONE)
        {
            external_stutter_quantized_command =
                QUANT_FX_NONE;

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
                loop_history_write,
                loop_history
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
                0
            );
        }


        /*
         * With no running transport no sixteenth pulse will ever arrive,
         * so the external lane consumes its commands here instead. That
         * is what keeps the Digitakt chop responsive with the sequencer
         * stopped.
         */
        if(!midi_running)
        {
            ServiceExternalQuantizedCommands();

            return;
        }


        /*
         * A force-disable still has to land even while the clock runs.
         */
        if(
            (
                (
                    external_stutter_quantized_command &
                    0xFFu
                )
                ==
                QUANT_FX_FORCE_DISABLE
            )
            ||
            (
                (
                    looper_quantized_command &
                    0xFFu
                )
                ==
                QUANT_FX_FORCE_DISABLE
            )
        )
        {
            ServiceExternalQuantizedCommands();
        }
    }


    float ProcessExternal(float input)
    {
        /*
         * Digitakt path:
         *
         * PUMP + DELAY + LOOPER are external-only.
         * CHOP + HPF + LPF use cheap independent DSP state.
         *
         * The looper reads a frozen capture, so while it is active
         * (including its release fade) the writer stops completely and the
         * captured region becomes immutable. Rolling capture resumes once
         * the looper is fully inactive.
         */
        if(!looper.active)
        {
            loop_history[
                loop_history_write
            ] =
                input;


            loop_history_write++;


            if(loop_history_write >=
               LOOP_HISTORY_SAMPLES)
            {
                loop_history_write = 0;
            }
        }


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
            looper.Process(
                x,
                loop_history
            );


        x =
            external_dj_hpf.Process(
                x
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
         * The chop is a LIVE non-sampling processor, so the kick lane no
         * longer reads the loop capture at all.
         */
        float x =
            stutter.Process(
                input,
                nullptr
            );


        x =
            dj_hpf.Process(
                x
            );


        /*
         * The LPF object existed and was reset, but was never processed here,
         * so CC34 moved a filter that nothing listened to and the control did
         * nothing at all on the kick. Its external twin was always wired.
         */
        x =
            macro_dj_lowpass.Process(
                x
            );


        /*
         * Kick performance chain:
         *
         *     STUTTER -> HPF -> LPF
         *
         * LOOPER, pump and delay remain external-input effects only.
         */
        return x;
    }
};

constexpr uint32_t AddedPerformanceFx::LOOP_HISTORY_SAMPLES;

static AddedPerformanceFx added_performance_fx;


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
     * VELOCITY = SUB MOVEMENT, latched by the voice at trigger:
     *
     *      1 = glide one octave down
     *     64 = flat
     *    127 = glide one octave up
     */

    /*
     * Gate starts now.
     */
    note_gate = true;

    /*
     * Kick starts immediately.
     */
    kick_trigger_pending = true;
}


/* ============================================================
   MIDI NOTE OFF
   ============================================================ */

static void HandleKickNoteOff(
    uint8_t note)
{
    (void)note;


    note_gate = false;
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
     *   16  = 1/4
     *   48  = 1/4T
     *   80  = 1/8
     *   112 = 1/8T
     */
    if(raw <= 32) return 0;
    if(raw <= 64) return 1;
    if(raw <= 96) return 2;
    return 3;
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

    PostStutterCommand(
        MakeQuantFxCommand(
            QUANT_FX_DISABLE,
            0
        )
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
        PostStutterCommand(
            MakeQuantFxCommand(
                QUANT_FX_DISABLE,
                0
            )
        );

        return;
    }


    uint8_t rate =
        StutterRateFromCc(
            raw
        );


    if(was_off)
    {
        PostStutterCommand(
            MakeQuantFxCommand(
                QUANT_FX_ENABLE,
                rate
            )
        );
    }
    else
    {
        /*
         * CC value is now an explicit rate address. If it enters a new
         * division zone, change rate at the next quantization point.
         */
        PostStutterCommand(
            MakeQuantFxCommand(
                QUANT_FX_RATE_CHANGE,
                rate
            )
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


static void RequestClearAllK1FxExceptPump()
{
    if(midi_running)
        fx_reset_pending = true;
    else
        ClearAllK1FxExceptPump();
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
        RequestClearAllK1FxExceptPump();

        macro_fx_long_press_fired = true;

        /*
         * Force pickup on the current page because its stored value may
         * just have been reset to zero.
         */
        MacroFxBeginPickup();
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

        case CC_MACRO_FX_REVERB:
            param_reverb_amount = v * REVERB_AMOUNT_MAX;
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
            bool wanted =
                value >= 64;


            /*
             * With no transport there is no beat to wait for, so honour the
             * button immediately rather than letting it feel dead.
             */
            if(midi_running)
                tail_delay_pending_state = wanted ? 1 : 0;
            else
                tail_delay_enabled = wanted;

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
           K5 — MACKIE/TUBE HAVE SEPARATE AMOUNT CCs
           ==================================================== */

        case CC_MACKIE_AMOUNT:
        {
            macro_mackie_amount = v;

            if(!macro_character_tube)
                macro_character_wet = v;

            return true;
        }

        case CC_TUBE_AMOUNT:
        {
            macro_tube_amount = v;

            if(macro_character_tube)
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
               macro_character_tube)
            {
                macro_character_tube =
                    requested;

                character_switch_target_tube =
                    requested;

                character_switch_pending =
                    true;
            }


            macro_character_wet =
                requested
                ? macro_tube_amount
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
            param_line_gain_target = v * PARAM_LINE_GAIN_MAX;
            return true;

        case CC_MIX_MACKIE_GAIN:
            param_mackie_gain_target = v * PARAM_MACKIE_GAIN_MAX;
            return true;

        case CC_MIX_TUBE_GAIN:
            param_tube_gain_target = v * PARAM_TUBE_GAIN_MAX;
            return true;

        case CC_MIX_BPF_GAIN:
            param_bpf_gain_target = v * PARAM_BPF_GAIN_MAX;
            return true;

        case CC_MIX_SUB_GAIN:
            param_sub_gain = v * PARAM_SUB_GAIN_MAX;
            return true;

        case CC_MIX_PUNCH_GAIN:
            param_punch_gain = v * PARAM_PUNCH_GAIN_MAX;
            return true;

        /* v1.3.0 kick shaping: plain values, latched by the next hit. */
        case CC_TAIL_ATTACK:
            macro_tail_attack_ms =
                TAIL_ATTACK_MIN_MS *
                powf(TAIL_ATTACK_MAX_MS / TAIL_ATTACK_MIN_MS, v);
            return true;

        case CC_PUNCH_SWEEP_TIME:
            macro_punch_time_scale =
                powf(2.0f, (static_cast<float>(value) - 64.0f) / 32.0f);
            return true;

        case CC_WAVE:
            macro_wave = v;
            return true;

        case CC_TAIL_MOD:
            macro_tail_mod = v;
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
                        RequestClearAllK1FxExceptPump();
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
                if(macro_character_tube)
                    macro_tube_amount = v;
                else
                    macro_mackie_amount = v;
                macro_character_wet = v;
            }
            return true;
        }

        /* K5 button: Mackie <-> Tube, press edge only. */
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

                macro_character_tube = !macro_character_tube;

                if(macro_character_tube)
                    macro_tube_amount = live_amount;
                else
                    macro_mackie_amount = live_amount;

                character_switch_target_tube = macro_character_tube;
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
        PostStutterCommand(
            QUANT_FX_FORCE_DISABLE
        );

        looper_quantized_command =
            QUANT_FX_FORCE_DISABLE;

        macro_fx_value_stutter = 0.0f;
        macro_fx_value_looper = 0.0f;


        /*
         * No further quarter pulses will arrive to land a beat-quantized
         * request on, so honour what was waiting instead of swallowing it.
         */
        if(tail_delay_pending_state >= 0)
        {
            tail_delay_enabled =
                tail_delay_pending_state > 0;

            tail_delay_pending_state = -1;
        }


        if(fx_reset_pending)
        {
            fx_reset_pending = false;

            ClearAllK1FxExceptPump();
        }


        hw.SetLed(false);

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
                /* Handled. */
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
    /*
     * RECEIVER ERROR RECOVERY — DO NOT REMOVE.
     *
     * There is no FIFO and no DMA here, and this loop only runs when the
     * control loop gets round to it. A burst arriving while the control loop
     * is busy therefore overruns the single receive register.
     *
     * An overrun latches ORE, and while ORE is set RXNE stops asserting:
     * the loop below sees nothing, the port is deaf for good, and the last
     * Note-On never receives its Note-Off, so the kick drones until the
     * board is power-cycled. ORE is only cleared by writing ICR, which is
     * what this does.
     *
     * Framing and noise errors latch the same way and are cleared here too.
     */
    uint32_t rx_errors =
        USART3->ISR &
        (
            USART_ISR_ORE |
            USART_ISR_FE |
            USART_ISR_NE
        );


    if(rx_errors)
    {
        USART3->ICR =
            USART_ICR_ORECF |
            USART_ICR_FECF |
            USART_ICR_NECF;


        /*
         * Bytes were lost mid-stream, so a remembered running status would
         * reassemble the next bytes into the wrong message. Drop it and
         * wait for a fresh status byte.
         */
        midi_running_status = 0;
    }


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
         * One sine voice, phase reset to zero. A voice still sounding is
         * handed off rather than cut.
         */
        TriggerKickVoice(
            last_velocity
        );


        /*
         * MASTER OUTPUT DE-CLICK ENVELOPE:
         * Note-On -> short attack -> unity hold.
         */
        added_kick_master_envelope.Trigger();


        /* Latch the BPF layer count for this hit; see the declaration. */
        macro_bpf_layer_count_latched = macro_bpf_layer_count;

        kick_reverb.Trigger();

        macro_whole_kick_reverse.Trigger();
        macro_character_processor.Trigger();

        added_performance_fx.TriggerKick();


        /*
         * CHOP / LOOPER ON-OFF is quantized to THIS kick boundary.
         */
        added_performance_fx.OnKickBoundary();
    }



    /*
     * Consume any transport-stop safety commands in the AUDIO thread.
     */
    added_performance_fx.ServiceImmediateSafety();


    if(master_decay_retime_pending)
    {
        master_decay_retime_pending = false;


        /*
         * The sub envelope multiplies its CURRENT level by the new
         * coefficient, so retiming never steps the amplitude.
         */
        kick_voice.SetDecay(
            MacroDecaySeconds(
                macro_decay
            )
        );
    }


    /*
     * Note-gate state is intentionally NOT used to release the master
     * amplitude envelope anymore.
     *
     * Velocity owns the sub movement and K2 owns amplitude decay; Note-Off
     * has no audio effect.
     */


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


        if(tail_delay_pending_state >= 0)
        {
            tail_delay_enabled =
                tail_delay_pending_state > 0;

            tail_delay_pending_state = -1;
        }


        if(fx_reset_pending)
        {
            fx_reset_pending = false;

            ClearAllK1FxExceptPump();
        }


        added_performance_fx.OnQuarter();
    }



    /*
     * Macro-4 BPF layer frequency smoothing/coefficient update.
     */
    macro_bpf_bank.Update();


    /* Slew the mix gains toward their CC targets; see their declarations. */
    param_line_gain    += (param_line_gain_target    - param_line_gain)    * PARAM_GAIN_SLEW;
    param_mackie_gain  += (param_mackie_gain_target  - param_mackie_gain)  * PARAM_GAIN_SLEW;
    param_tube_gain += (param_tube_gain_target - param_tube_gain) * PARAM_GAIN_SLEW;
    param_bpf_gain     += (param_bpf_gain_target     - param_bpf_gain)     * PARAM_GAIN_SLEW;


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
           DRY: PUNCH + SUB FROM THE ONE SINE VOICE
           ==================================================== */

        kick_punch_gain_smoothed +=
            (param_punch_gain - kick_punch_gain_smoothed) *
            KICK_GAIN_SMOOTH_A;

        kick_sub_gain_smoothed +=
            (param_sub_gain - kick_sub_gain_smoothed) *
            KICK_GAIN_SMOOTH_A;


        KickVoiceOut voices;

        kick_voice.Process(voices);

        for(int v = 0; v < KICK_FADING_SLOTS; v++)
        {
            if(kick_voice_fading[v].active)
                kick_voice_fading[v].Process(voices);
        }


        /* The mixer gains set the dry level only. */
        float dry =
            voices.punch * kick_punch_gain_smoothed +
            voices.sub * kick_sub_gain_smoothed;


        /* ====================================================
           WET: DISTORTION (MACKIE / TUBE + BPF)
           ====================================================

           As 3af0e35 had it. The send is the full-band punch + sub before
           the mixer gains, so the fundamental is what drives the models. Only the model's
           RETURN is high-passed, two one-poles at 120 Hz, so its distorted
           sub does not stack on the dry one; the BPF colour is added after.

           Not a steeper filter: a 4th-order Butterworth at 120 Hz is 180
           degrees out at its corner, so the return came back inverted
           around 100-150 Hz and cancelled the dry kick there, 5 dB of upper
           bass lost at full Mackie. Two one-poles are 90 degrees at the
           corner and add instead.
         */

        float wet = 0.0f;

        if(!KICK_BYPASS_WET)
        {
            /* Macro-4 broad BPF EQ is pushed INTO the distortion model. */
            float wet_send =
                voices.send +
                macro_bpf_bank.ProcessDriveFeed(voices.send);


            wet =
                macro_character_processor.ProcessWet(wet_send);


            /* Return-only sub protection. */
            wet =
                HighPassFixedPole(
                    HighPassFixedPole(
                        wet,
                        CHARACTER_SUB_PROTECT_POLE_A,
                        character_delta_hp_state
                    ),
                    CHARACTER_SUB_PROTECT_POLE_A,
                    character_delta_hp_state_2
                );


            /*
             * Macro-4 additive BPF colour, excited by the punch and the
             * return; the sub stays out of it, as before.
             */
            wet +=
                macro_bpf_bank.Process(
                    voices.bpf_punch +
                    wet
                );




            wet *=
                1.0f -
                separation * 0.13f;


            /*
             * The dirty-bus manager's compression (from 28 % of K5: up to
             * 8:1 and an 18 % static trim) is what dulled MID I..III, so it
             * is held off. It still runs, at zero strength, so re-enabling
             * it cannot jump.
             */
            wet =
                character_dirty_bus_manager.Process(
                    wet,
                    0.0f
                );


            float dirty_post_gain =
                DIRTY_POST_GAIN_DRY +
                (
                    DIRTY_POST_GAIN_WET -
                    DIRTY_POST_GAIN_DRY
                )
                *
                macro_character_processor.CurrentSmoothedAmount();


            if(dirty_post_gain < 0.62f)
                dirty_post_gain = 0.62f;


            wet *=
                dirty_post_gain;

        }


        /* ====================================================
           DRY + WET, THEN THE KICK FX
           ==================================================== */

        /* KICK_WET_HEADROOM: see its declaration. */
        if(KICK_WET_HEADROOM)
        {
            float out_gain = param_line_gain * KICK_OUTPUT_LINEAR_GAIN;

            if(out_gain > 0.0001f)
            {
                float room = OUTPUT_CEILING_KNEE / out_gain - fabsf(dry);

                if(room < 0.0f)
                    room = 0.0f;

                /* |wet| stays under room, and is ~unchanged while small. */
                wet = wet / (1.0f + fabsf(wet) / (room + 1.0e-6f));
            }
        }

        float signal =
            dry +
            wet;


        if(!KICK_BYPASS_POST)
        {
            /*
             * MACRO 2 — whole-waveform reverse of the complete kick. External
             * Digitakt audio is deliberately outside this buffer.
             */
            signal =
                macro_whole_kick_reverse.Process(
                    signal
                );


            /*
             * Master de-click envelope, after every kick layer. External
             * passthrough remains outside it.
             */
            signal *=
                added_kick_master_envelope.Process();


            /* KICK_OLD_FINAL_HF_GUARD: see its declaration. */
            if(KICK_OLD_FINAL_HF_GUARD)
            {
                float age_ms =
                    static_cast<float>(kick_age_samples) * 1000.0f /
                    SAMPLE_RATE;

                float target =
                    OLD_FINAL_HF_INITIAL_HZ +
                    (OLD_FINAL_HF_SETTLED_HZ - OLD_FINAL_HF_INITIAL_HZ) *
                    SmoothstepAdded(Clamp01Added(age_ms / OLD_FINAL_HF_OPEN_MS));

                if(target < final_hf_guard_cutoff - OLD_FINAL_HF_CLOSE_STEP_HZ)
                    final_hf_guard_cutoff -= OLD_FINAL_HF_CLOSE_STEP_HZ;
                else
                    final_hf_guard_cutoff = target;

                float a = expf(-TWO_PI * final_hf_guard_cutoff / SAMPLE_RATE);

                for(int p = 0; p < 3; p++)
                {
                    final_hf_guard_state[p] =
                        (1.0f - a) * signal +
                        a * final_hf_guard_state[p];

                    signal = final_hf_guard_state[p];
                }
            }
        }


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
         * LINEAR OUTPUT HEADROOM.
         *
         * No nonlinear final limiter here: character/filter combinations
         * should not suddenly enter a different transfer curve at 0.92.
         */
        /* KICK_OLD_ONSET_LEVEL: see its declaration. */
        if(KICK_OLD_ONSET_LEVEL)
        {
            float target = 1.0f;

            if(kick_fresh_hit)
            {
                float age_ms =
                    static_cast<float>(kick_age_samples) * 1000.0f /
                    SAMPLE_RATE;

                target =
                    OLD_ONSET_LEVEL +
                    (1.0f - OLD_ONSET_LEVEL) *
                    SmoothstepAdded(
                        Clamp01Added(
                            (age_ms - OLD_ONSET_HOLD_MS) / OLD_ONSET_RELEASE_MS
                        )
                    );
            }

            if(target < kick_onset_level - OLD_ONSET_FALL_STEP)
                kick_onset_level -= OLD_ONSET_FALL_STEP;
            else
                kick_onset_level = target;
        }


        kick_output *=
            param_line_gain *
            kick_onset_level;


        if(KICK_BYPASS_POST)
        {
            kick_output *=
                KICK_OUTPUT_LINEAR_GAIN;
        }
        else
        {
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


            /* Sidechain reverb: last thing before the ceiling, so the tank
             * hears the finished kick and the ceiling still bounds the sum. */
            kick_output =
                kick_reverb.Process(
                    kick_output,
                    param_reverb_amount
                );
        }


        /*
         * LAST-RESORT NON-FINITE GUARD.
         *
         * A NaN or an infinity does not fade: it is stored by the next
         * filter it reaches and every later sample multiplies against it,
         * so the voice stays silent until the board is power-cycled. That
         * presents as the engine dying rather than glitching, which is the
         * worst possible failure on stage.
         *
         * The stages above are bounded, so reaching this should be
         * impossible. If it ever does, mute this sample and reset the
         * state that can hold one, turning a dead engine into a blip.
         */
        if(!(kick_output == kick_output) ||
           fabsf(kick_output) > 1000.0f)
        {
            kick_output = 0.0f;

            audio_panic_pending = true;
        }


        kick_output =
            OutputCeiling(
                kick_output
            );


        if(KICK_DAC_KEEPALIVE)
            kick_output += KICK_DAC_KEEPALIVE_OFFSET;


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

/*
 * Every piece of DSP state that can hold a value across samples.
 *
 * Used both at startup and by the non-finite recovery, so the recovery can
 * never miss a stage the way a hand-picked list does: a NaN parked in one
 * untouched filter keeps poisoning the output and the engine stays dead
 * until the board is power-cycled.
 */
static void ResetAudioDspState()
{
    kick_voice.Reset();

    for(int i = 0; i < KICK_FADING_SLOTS; i++)
        kick_voice_fading[i].Reset();


    character_delta_hp_state = 0.0f;
    character_delta_hp_state_2 = 0.0f;

    final_hf_guard_cutoff = OLD_FINAL_HF_SETTLED_HZ;

    for(int p = 0; p < 3; p++)
        final_hf_guard_state[p] = 0.0f;

    kick_onset_level = 1.0f;

    added_performance_fx.Reset();
    added_kick_master_envelope.Reset();

    macro_bpf_bank.Reset();
    macro_character_processor.Reset();
    character_dirty_bus_manager.Reset();
    kick_reverb.Reset();
    macro_whole_kick_reverse.Reset();
}


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
       DSP INITIAL STATE
       -------------------------------------------------------- */

    kick_frequency = 55.0f;


    separation = 0.35f;


    final_hf_low_state = 0.0f;
    final_hf_envelope = 0.0f;
    final_hf_gain = 1.0f;



    ResetAudioDspState();


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
    external_stutter_quantized_command = QUANT_FX_NONE;
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
    macro_tube_amount = 0.0f;
    macro_character_wet = 0.0f;

    character_switch_target_tube =
        macro_character_tube;

    character_switch_pending = false;

    character_button_down = false;

    macro_fx_button_down = false;
    macro_fx_long_press_fired = false;

    emergency_b1_down = false;
    emergency_b3_down = false;
    emergency_combo_timing = false;
    emergency_combo_start_time = 0;


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
         */
        ServiceMidi();


        /*
         * NON-FINITE RECOVERY.
         *
         * The audio thread has muted itself and asked for a rebuild. Doing
         * it here keeps the 43200-sample capture clear out of the audio
         * block. The result is a dropout instead of an engine that stays
         * dead until the board is power-cycled.
         */
        if(audio_panic_pending)
        {
            audio_panic_pending = false;

            ResetAudioDspState();
        }


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


    }
}