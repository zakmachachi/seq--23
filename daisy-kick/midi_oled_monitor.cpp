#include "daisy_seed.h"
#include "stm32h7xx_hal.h"

#include <math.h>
#include <stdint.h>
#include "midi_rx_queue.h"
#include "midi_pulse_output.h"
#include "erosion_fx.h"
#include "external_pitch_fx.h"

using namespace daisy;


/* ============================================================
   HARDWARE
   ============================================================ */

DaisySeed hw;
UART_HandleTypeDef midi_uart;
static MidiRxQueue midi_rx;
static volatile uint32_t midi_uart_errors = 0;
static volatile uint32_t audio_max_cycles = 0, audio_overruns = 0;


// Eurorack interface logic outputs (3.3 V): D2/header pin 3 and D3/pin 4.
// Use external protected/level-shifted trigger drivers where 5 V is required.
static constexpr int EURO_KICK_GPIO = 2, EURO_CLOCK_GPIO = 3;
static constexpr uint32_t EURO_CLOCK_DIVIDER = 1; // 1=24 PPQN, 6=sixteenths
static_assert(EURO_CLOCK_DIVIDER>0, "Clock divider must be nonzero");
static GPIO euro_kick_gpio, euro_clock_gpio;
static MidiPulseOutput euro_kick_pulse, euro_clock_pulse;
static bool euro_gpio_ready=false, euro_kick_high=false, euro_clock_high=false;
static uint32_t euro_clock_phase=0;

static void InitEuroOutputs()
{
    euro_kick_gpio.Init(hw.GetPin(EURO_KICK_GPIO),GPIO::Mode::OUTPUT,GPIO::Pull::PULLDOWN);
    euro_clock_gpio.Init(hw.GetPin(EURO_CLOCK_GPIO),GPIO::Mode::OUTPUT,GPIO::Pull::PULLDOWN);
    euro_kick_gpio.Write(false);euro_clock_gpio.Write(false);
    euro_gpio_ready=true;
}
static void ResetEuroOutputs()
{
    euro_kick_pulse.Reset();euro_clock_pulse.Reset();
    euro_kick_high=euro_clock_high=false;
    euro_clock_phase=0;
    if(euro_gpio_ready){euro_kick_gpio.Write(false);euro_clock_gpio.Write(false);}
}
static void ServiceEuroOutputs(uint32_t block_samples)
{
    // 48 kHz: 5 ms trigger / 1 ms low; 1 ms clock / >=1 callback low.
    bool kick=euro_kick_pulse.Advance(block_samples,240,48);
    bool clock=euro_clock_pulse.Advance(block_samples,48,16);
    if(euro_gpio_ready && kick!=euro_kick_high)euro_kick_gpio.Write(kick);
    if(euro_gpio_ready && clock!=euro_clock_high)euro_clock_gpio.Write(clock);
    euro_kick_high=kick;euro_clock_high=clock;
}

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

   Deterministic kick generator with a clean low end.

   Performance routing:

       Digitakt / external IN1 -> external FX -> mixer2
       real-kick + ghost-clock pump on external input
       MIDI-clock dotted delay on external input
       MIDI-clock 1/16 stutter on kick + external lanes
       quantized looper on external input
       DJ high-pass on external input only
       chromatically tuned kick with a high-passed character return

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


/* Both outputs carry mixer2's mono mix: kick FX + external FX, then glue.
 * Independent branch trims reserve headroom before the shared output trim.
 * The external HPF stays in its own branch and never filters the generator.
 */
static constexpr float KICK_OUTPUT_LINEAR_GAIN = 0.06f;
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
static volatile float param_line_gain    = 1.00f;   /* CC53, linear output trim 0..1 */
static volatile float param_mackie_gain  = 0.65f;   /* CC54, max 1.25 */
static volatile float param_tube_gain    = 0.60f;   /* CC55, max 1.25 */
static volatile float param_bpf_gain     = 0.70f;   /* CC56, pre-drive BPF blend 0..1 */
static volatile float param_sub_gain     = 1.00f;   /* CC57, clean low-shelf boost 0..15 dB endpoint */
static volatile float param_punch_gain   = 1.00f;   /* CC58, clean LPF lane gain 0..1 */
static volatile float param_reverb_amount = 0.0f;   /* CC36, FX page  */

static constexpr float PARAM_LINE_GAIN_MAX    = 1.0f;
static constexpr float PARAM_MACKIE_GAIN_MAX  = 1.25f;
static constexpr float PARAM_TUBE_GAIN_MAX    = 1.25f;
static constexpr float PARAM_BPF_GAIN_MAX     = 1.0f;
static constexpr float PARAM_SUB_GAIN_MAX     = 1.00f;
// PUNCH is the clean-lane fader. It never changes oscillator or dirty drive.

/*
 * CC wrote these directly and they are read per sample, so every message
 * stepped the gain - audible as a tick while turning. CC now writes *_target;
 * audio slews to it once per block. SUB and PUNCH are not listed: the kick
 * voice already slews them per sample (KICK_GAIN_SMOOTH_A).
 */
static volatile float param_line_gain_target    = 1.00f;
static volatile float param_mackie_gain_target  = 0.65f;
static volatile float param_tube_gain_target = 0.60f;
static volatile float param_bpf_gain_target     = 0.70f;
static constexpr float PARAM_GAIN_SLEW = 0.06f;

/* Reverb control is normalized 0..1; dry remains at unity. */
static constexpr float REVERB_AMOUNT_MAX = 1.0f;

/* ============================================================
   MUSICAL MACKIE / TUBE CHARACTER
   ============================================================

   Body/attack splits into clean LPF and serial mid-boost/drive lanes.
   SUB boosts the clean body with a low shelf and never enters drive.
   A matched 240 Hz LR4 crossover separates clean body from dirty harmonics.
   Mackie runs EQ and nonlinear stages at 4x with anti-alias FIR filtering;
   the clean path compensates its fixed 24-sample conversion latency.
   Mixer1 -> kick FX / gate / reverb; mixer2 adds external FX, then gentle
   compression and linear output trim. Both outputs carry the mono mix.
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

/*
 * KICK-DESIGN ADVANCED PAGE.
 *
 * The LIVE instrument no longer needs these controls to make a good kick:
 *   MIDI Note     = settled/body pitch
 *   Note velocity / CC77 = bipolar tail-pitch excursion, neutral at 64
 *   KICK SHAPE    = sweep DEPTH + transient/harmonic character
 *
 * CC77 carries exact bipolar tail pitch (64 neutral); CC78 controls sweep
 * duration independently of both SHAPE and amplitude DECAY.
 */
static constexpr uint8_t CC_KICK_TAIL_PITCH = 77;
static constexpr uint8_t CC_KICK_SWEEP_TIME  = 78;

/* WAVE: the body morphs from the sine to a phase-derived saw, latched per hit. */
static constexpr uint8_t CC_WAVE = 64;

/* TMOD (Menu 1 pot 3) sets the resettable tail LFO rate. Velocity supplies
 * signed pitch depth; zero TMOD leaves a one-shot glide. BELLY sets the
 * earliest tail-gate start, with no action when the gate is off.
 */
static constexpr uint8_t CC_KICK_TAIL_MOD = 79;
static constexpr uint8_t CC_KICK_BELLY = 61;


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
 *   CC77 TAIL PITCH             next hit: 0=-12 st, 64=0, 127=+12 st
 *   CC78 SWEEP TIME TRIM        optional, 70~=neutral
 *   CC64 WAVE                   0=sine, 127=saw harmonics
 *   CC79 TMOD RATE              0 one-shot; 1..127 = .125..16 Hz
 *   CC61 BELLY                  64 = 1x punch window; 0 = 0.35x, 127 = 2x
 *
 * Note velocity is bipolar tail pitch; CC77 carries the full 0..127 range.
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
static constexpr uint8_t CC_MACRO_FX_BITCRUSH      = 37;
static volatile float bitcrush_target = 0.f;
static constexpr uint8_t CC_EROSION_AMOUNT=38, CC_EROSION_FREQUENCY=39;
static volatile float erosion_amount=0.f, erosion_frequency=64.f/127.f;
// FX route bits 0..3 add kick: pump, reverb, bitcrush, erosion.
// Bit 4 disables external reverb (INT-only); other effects keep EXT enabled.
static volatile uint8_t fx_internal_routes=14;
static ExternalPitchFx external_pitch_fx;
static volatile float external_pitch_ratio=1.f;
static uint16_t fx_bar_reset_mask=0;
static ErosionFx erosion_fx, external_erosion_fx;
static float pump_internal_mix=0.f;

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
 * DECAY (CC40) changes are also handed to the audio thread rather than
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


/* DECAY (CC40): true lifetime of the complete generated body. */
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


/* TAIL DELAY AMOUNT/STATE (CC42/43): a deliberate kick-break-bass gate. */
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
 * Layer count LATCHED at Note-On, exactly as KICK SHAPE (CC51) already is.
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
    231.033796623f,
    544.371585997f,
    1282.671314641f
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


/*
 * KICK SHAPE (CC51): MASTER PUNCH CHARACTER.
 *
 * The front-panel relationship is intentionally drum-machine-like:
 *
 *   MIDI Note     = settled/body pitch
 *   PITCH/velocity= signed tail excursion; 64 neutral, extremes +/-12 semitones
 *   KICK SHAPE    = initial pitch sweep depth/time, attack and harmonics
 *
 * This mirrors the useful separation found on dedicated kick machines: base
 * pitch, pitch-envelope amount, and pitch-envelope time are different jobs.
 * SHAPE=0 is a bass tone: attack sweep depth and extra attack are zero.
 * SHAPE rises through round/909/Jomox/hardcore into the long laser region.
 */
static volatile float macro_kick_shape = 0.50f;

/*
 * Optional advanced trims. 0.55 is neutral. They are deliberately narrow so
 * entering this page is never required just to get a good kick.
 */
static volatile float kick_sweep_time  = 64.f / 127.f;

/* WAVE (CC64): 0 = the body's own sine, 1 = full phase-locked saw harmonics. */
static volatile float macro_wave = 0.0f;

/* TMOD (CC79) rate defaults off; BELLY (CC61) defaults to 64. */
static volatile uint8_t kick_tail_mod_cc = 0; // CC79 TMOD rate, zero = one-shot tail glide
static volatile uint8_t kick_tail_pitch_cc = 64;
static volatile bool kick_tail_pitch_pending = false;
static volatile uint8_t kick_belly_cc = 64;


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

static bool ENABLE_KICK_MASTER_GATE_ENVELOPE = false;

static volatile float kick_master_attack_ms = 0.15f;
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
 * DECAY (CC40) — lifetime of the WHOLE generated kick body.
 *
 * This envelope is calculated before the body splits into clean and
 * Mackie/Tube paths.  Therefore shortening DECAY shortens BOTH the clean
 * bass and the material available to the parallel distorted tail.  It is not
 * a wet-return control.
 *
 * 0.00  ~45 ms    genuinely short/snappy freetekno drum
 * 0.25  ~122 ms   tight 909-ish body
 * 0.50  ~329 ms   classic useful kick
 * 0.75  ~889 ms   kick-bass territory
 * 1.00  ~2.40 s   long finite bass tail
 */
static float MacroDecaySeconds(float x)
{
    x = Clamp01Added(x);

    /* Time to about -30 dB after the initial one-sub-cycle body hold. */
    return 0.045f * powf(53.3333333f, x);
}


static bool MacroDecayInfinite(float x)
{
    (void)x;
    return false;
}


/*
 * Retained for the legacy master-envelope helper. The new kick core does not
 * use Note-Off to determine its decay, but keeping a finite mapping makes the
 * surrounding controller/performance code safe if that helper is re-enabled.
 */
static float MacroMasterReleaseMs(float x)
{
    return MacroDecaySeconds(x) * 1000.0f;
}


/*
 * SWEEP (CC78) pitch duration also used by the external pump timing.
 * Keep this numerically matched to KickSweepTimeMs() below so external
 * sidechain movement follows the audible kick anatomy.
 */
static float KickSweepTimeMs(float shape);
static float MacroKickSweepSeconds(float x)
{
    return KickSweepTimeMs(x) * .001f;
}
/* TAIL DELAY is a gate, never an audio delay: gap length after BELLY,
 * from zero to an eighth note. The power law gives finer short-gap control.
 */
static float MacroTailDelayMs(float x)
{
    x = Clamp01Added(x);


    /* Short gaps at low settings; a full eighth-note break at maximum. */
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

static uint8_t last_velocity = 64;
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
static volatile float kick_frequency = 52.0f;

/*
 * MIDI NOTE = settled/body pitch.
 *
 * The old v3 fixed-drum mode disabled an important performance dimension.
 * This is ON again by default.  The note selects only the frequency the kick
 * settles to; it does NOT alter DECAY, Tail Delay, character amount, or the
 * deterministic phase reset.
 *
 * 30..120 Hz is intentionally wider than the 45..70 Hz "sweet spot": it lets
 * the instrument move from deep sub kicks to short high/tom/laser material
 * without allowing an accidental note to put the sustained body at hundreds
 * of hertz.
 *
 * Body amplitude, sweep compensation and body waveshape are note-invariant.
 * Adjacent MIDI notes therefore leave the digital gain structure essentially
 * unchanged. The final 15 Hz housekeeping filter is nearly flat above 30 Hz.
 *
 * The PA still needs its cabinet-specific protection HPF.
 */
static constexpr bool KICK_USE_MIDI_NOTE_FOR_TUNING = true;
static constexpr float KICK_FIXED_FREQUENCY_HZ = 52.0f;
static constexpr float KICK_CHROMATIC_MIN_HZ = 30.0f;
static constexpr float KICK_CHROMATIC_MAX_HZ = 120.0f;


/*
 * Gate-derived temporal separation.
 *
 * This is calculated from the actual MIDI gate length.
 */
static volatile float separation = 0.50f;


/*
 * Per-hit age: zero at each trigger; used by the optional legacy onset/HF shaping.
 */
static uint32_t kick_age_samples = 0;


/*
 * Final >8 kHz dynamics state.
 */
static float final_hf_low_state = 0.0f;
static float final_hf_envelope = 0.0f;
static float final_hf_gain = 1.0f;


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

static constexpr int KICK_PUNCH_PROFILE_STAGE = 0;

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
      linear per sample (-60 dB exponential at the DECAY time), which empties in
      about 60 % of it. Offset by the silence floor so it lands on zero.
      Off, the sub is (1 - u)^2 and lasts the full DECAY time, about 3-5 dB
      more low end.
   ============================================================ */

static constexpr bool KICK_OLD_WET_ONSET      = false;
static constexpr bool KICK_OLD_SUB_DECAY      = false;
static constexpr float OLD_SUB_DECAY_LINEARITY = 0.55f;
static constexpr bool KICK_OLD_FINAL_HF_GUARD = false;
static constexpr bool KICK_OLD_ONSET_LEVEL    = false;

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
   PERFORMANCE PITCH / SHAPE MAPPINGS
   ============================================================ */

// Velocity is a bipolar tail-pitch control. Square-law depth gives fine
// microtonal steps near 64, reaching exactly -/+12 semitones at 0/127.
static float TailPitchSemitones(uint8_t value)
{
    float x = value <= 64 ? (float(value)-64.f)/64.f : (float(value)-64.f)/63.f;
    return 12.f * x * fabsf(x);
}
static float TailModHz(uint8_t value)
{
    if(value == 0) return 0.f;
    return .125f * powf(128.f, float(value-1)/126.f); // .125..16 Hz
}
static float ShapeStartRatio(float shape)
{
    float x = Clamp01Added(shape);
    float semitones = x <= .5f ? 28.f * SmoothstepAdded(x*2.f)
        : 28.f + 20.f * SmoothstepAdded((x-.5f)*2.f);
    return powf(2.f, semitones/12.f);
}
// SHAPE controls depth/attack character. CC78 alone sets pitch timing.
static float KickSweepTimeMs(float x)
{
    x = Clamp01Added(x);
    return 4.f * powf(60.f, x); // 4..240 ms, logarithmic resolution
}

/* BELLY: scale on the punch window's times. 64 = 1x. */
static float KickBellyScale(uint8_t cc)
{
    if(cc <= 64)
        return 0.35f + 0.65f * static_cast<float>(cc) / 64.0f;

    return 1.0f + static_cast<float>(cc - 64) / 63.0f;
}


/*
 * Sweep-level compensation. A high-frequency start needs headroom, while the
 * settled LF body should be allowed to become the loudest/weightiest part of
 * the kick. This solves the previous "punch loud, sub weak" balance without
 * simply slamming the final limiter.
 */
static float SweepAmplitudeCompensation(float instantaneous_ratio)
{
    instantaneous_ratio = fmaxf(1.0f, instantaneous_ratio);

    float g = 1.0f / sqrtf(1.0f + 0.18f * (instantaneous_ratio - 1.0f));
    return ClampAdded(g, 0.52f, 1.0f);
}


/*
 * Jomox/909-family clean body waveshape.
 *
 * The oscillator remains ONE phase-coherent body oscillator.  Shape moves it
 * continuously from a sine toward a rounded/parabolic drum waveform rather
 * than layering independent harmonic oscillators.
 *
 * The parabolic transform is:
 *     sign(s) * (2|s| - |s|^2)
 *
 * A tiny zero-mean even component is then allowed at harder Shape values.
 * Most importantly, the normalization below preserves the FUNDAMENTAL
 * coefficient, not the waveform peak.  Turning Shape therefore adds body
 * character without making the 50-ish-Hz authority disappear.
 */
static float DrumBodyWaveshape(float sine_sample,
                               float morph,
                               float asymmetry)
{
    morph = Clamp01Added(morph);
    asymmetry = ClampAdded(asymmetry, 0.0f, 0.08f);

    float a = fabsf(sine_sample);
    float parabolic =
        (sine_sample >= 0.0f ? 1.0f : -1.0f) *
        (2.0f * a - a * a);

    float y =
        sine_sample +
        morph * (parabolic - sine_sample);

    /*
     * sin^2 - 1/2 is zero-mean and contributes a controlled 2nd harmonic
     * without a DC step.
     */
    y +=
        asymmetry *
        (sine_sample * sine_sample - 0.5f);

    /*
     * Fundamental coefficient of the parabolic waveform is ~1.151173637.
     * Divide by the interpolated coefficient so f0 stays almost invariant
     * through the entire Shape macro.
     */
    float fundamental_norm =
        1.0f /
        (1.0f + 0.151173637f * morph);

    return y * fundamental_norm;
}


/* ============================================================
   KICK GENERATOR: ONE SWEPT BODY
   ============================================================
   A resettable oscillator supplies the attack and settled note. WAVE is
   phase-derived; SUB is a clean low shelf downstream, not another oscillator.
   The original pulse/noise attack and raised-cosine onset are retained.
 */
static constexpr float KICK_GAIN_SMOOTH_A = 0.00208117f;
static constexpr float KICK_BODY_INTERNAL_LEVEL = 0.90f;
static constexpr float KICK_MIN_AUDIO_ENV = 0.00001f;
static constexpr float KICK_DECAY_COEFF_SMOOTH = 0.0025f;

static inline float Decay60Coefficient(float seconds)
{
    return expf(-6.907755f / (fmaxf(seconds, 0.0001f) * SAMPLE_RATE));
}
static inline float BodyDecayCoefficient(float seconds)
{
    return expf(-3.453877639f / (fmaxf(seconds, 0.0001f) * SAMPLE_RATE));
}
static inline float RaisedCosine01(float t)
{
    return 0.5f - 0.5f * cosf(Clamp01Added(t) * PI);
}
static inline uint32_t MsToSamples(float ms)
{
    return static_cast<uint32_t>(fmaxf(ms, 0.0f) * .001f * SAMPLE_RATE);
}
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

struct KickVoiceOut
{
    float punch = 0.0f; // complete swept body
    float gate = 1.0f;  // mixer1 gate, before reverb
};

struct KickVoice
{
    bool active = false;
    uint32_t age = 0;
    float phase = 0.0f; // body phase; no octave oscillator
    float body_phase = 0.0f;
    float base_hz = KICK_FIXED_FREQUENCY_HZ;
    float tail_semitones = 0.f, tail_rate_hz = 0.f, tail_lfo_phase = 0.f;
    float tail_glide_samples = SAMPLE_RATE * .15f;
    uint32_t tail_start_samples = 0;
    float instantaneous_hz = KICK_FIXED_FREQUENCY_HZ;
    float shape = .5f;
    float pitch_env = 0.0f, pitch_coeff = 0.0f, pitch_depth = 0.0f;
    bool curve_active = false;
    float curve_g = 1.0f, curve_u_step = 0.0f, curve_end_u = 1.0f;
    float body_env = 0.0f, decay_coeff = 1.0f, decay_coeff_target = 1.0f;
    uint32_t body_hold_samples = 1;
    float attack_env = 0.0f, attack_coeff = 0.0f, attack_level = 0.0f;
    float attack_noise_mix = 0.0f, attack_noise_band_a = .2f;
    uint32_t attack_pulse_width_samples = 1;
    uint32_t noise_state = 0x5A17C9E3u;
    float noise_lp = 0.0f, noise_band_lp = 0.0f;
    float body_shape_morph = 0.0f, body_shape_asymmetry = 0.0f;
    float wave_morph = 0.0f;
    uint32_t onset_samples = 384;
    bool tail_gate_active = false;
    uint32_t tail_gate_start_samples = 0, tail_hold_samples = 0;
    uint32_t tail_rise_samples = 144; // 3 ms endpoint-smooth edges

    void Reset() { *this = KickVoice(); }
    void SetDecay(float seconds)
    {
        decay_coeff_target = BodyDecayCoefficient(ClampAdded(seconds, .040f, 3.50f));
        if(!active) decay_coeff = decay_coeff_target;
    }
    static float ShapeMap(float x, float a, float b, float c)
    {
        x = Clamp01Added(x);
        return x <= .5f ? a + (b-a) * SmoothstepAdded(x*2.0f)
                       : b + (c-b) * SmoothstepAdded((x-.5f)*2.0f);
    }
    void Trigger(float frequency, float kick_shape, float sweep_depth_control,
                 float sweep_time_control, uint8_t velocity, float character_delay_ms,
                 float decay_seconds, float wave, uint8_t curve_cc, uint8_t belly_cc)
    {
        active = true;
        age = 0;
        phase = body_phase = 0.0f;
        noise_state = 0x5A17C9E3u;
        noise_lp = noise_band_lp = 0.0f;
        base_hz = ClampAdded(frequency, KICK_CHROMATIC_MIN_HZ, KICK_CHROMATIC_MAX_HZ);
        shape = Clamp01Added(kick_shape);
        wave_morph = Clamp01Added(wave);
        // Two complete body cycles at full envelope even on the shortest kick.
        // No sidechain or handoff delays the clean body.
        body_hold_samples = static_cast<uint32_t>(ceilf(2.f * SAMPLE_RATE / base_hz));
        (void)sweep_depth_control; // CC77 now transports full-resolution tail pitch.
        float ratio = ShapeStartRatio(shape);
        tail_semitones = TailPitchSemitones(velocity);
        tail_rate_hz = TailModHz(curve_cc);
        tail_lfo_phase = 0.f;
        tail_glide_samples = SAMPLE_RATE * KickSweepTimeMs(sweep_time_control) * .001f;
        onset_samples = MsToSamples(8.0f + (.5f-8.0f) * SmoothstepAdded(shape / .35f));
        pitch_depth = fmaxf(0.0f, fminf(base_hz * ratio, 3000.0f) / base_hz - 1.0f);
        pitch_env = pitch_depth > .00001f ? 1.0f : 0.0f;
        float sweep_ms = KickSweepTimeMs(sweep_time_control);
        pitch_coeff = Decay60Coefficient(sweep_ms * .001f);
        curve_g = 1.f;
        curve_active = false;
        curve_u_step = 1.0f / (sweep_ms * .001f * SAMPLE_RATE);
        curve_end_u = powf(11.512925f / 6.907755f, 1.0f / curve_g);
        // Pitch timing is independent of DECAY and of the amplitude hold.
        float settle_u = pitch_depth > .05f
            ? logf(pitch_depth / .05f) / 6.907755f : 0.0f;
        tail_start_samples = MsToSamples(sweep_ms * settle_u);
        // Retain two base body cycles, independent of SHAPE, SWEEP and TUNE.
        // Very short decay + long sweep can intentionally make a laser.
        body_env = 1.0f;
        SetDecay(decay_seconds);
        decay_coeff = decay_coeff_target;
        body_shape_morph = ShapeMap(shape, 0.0f, .44f, .88f);
        body_shape_asymmetry = ShapeMap(shape, 0.0f, .012f, .045f);
        attack_level = ShapeMap(shape, 0.0f, .070f, .165f);
        attack_noise_mix = ShapeMap(shape, 0.0f, .14f, .42f);
        attack_pulse_width_samples = MsToSamples(ShapeMap(shape, 4.5f, 1.6f, .42f));
        if(attack_pulse_width_samples < 8) attack_pulse_width_samples = 8;
        attack_noise_band_a = 1.0f - expf(-TWO_PI * ShapeMap(shape, 3200.f, 6500.f, 10500.f) / SAMPLE_RATE);
        attack_env = attack_level > 0.0f ? 1.0f : 0.0f;
        attack_coeff = Decay60Coefficient(ShapeMap(shape, 5.5f, 2.6f, 1.15f) * .001f);
        float belly = KickBellyScale(belly_cc);
        tail_gate_start_samples = MsToSamples(50.0f * belly);
        if(tail_gate_start_samples < body_hold_samples)
            tail_gate_start_samples = body_hold_samples;
        // TAIL DELAY means gap LENGTH, after BELLY, not an absolute return
        // deadline that can occur before the punch finishes. Phase continues.
        tail_gate_active = character_delay_ms > .05f;
        tail_rise_samples = MsToSamples(3.0f);
        tail_hold_samples = tail_gate_start_samples + tail_rise_samples + MsToSamples(character_delay_ms);
    }
    float NextNoise()
    {
        uint32_t x = noise_state;
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;
        noise_state = x;
        return (static_cast<int32_t>(x >> 9) - 0x00400000) * (1.0f / 4194304.0f);
    }
    float TailGate() const
    {
        if(!tail_gate_active || age <= tail_gate_start_samples) return 1.0f;
        if(age < tail_gate_start_samples + tail_rise_samples)
            return 1.0f - RaisedCosine01(float(age-tail_gate_start_samples) / tail_rise_samples);
        if(age < tail_hold_samples) return 0.0f;
        return RaisedCosine01(float(age-tail_hold_samples) / tail_rise_samples);
    }
    void Process(KickVoiceOut& o)
    {
        if(!active) return;
        decay_coeff += (decay_coeff_target-decay_coeff) * KICK_DECAY_COEFF_SMOOTH;
        float sweep_env = pitch_env;
        if(curve_active)
        {
            float u = age * curve_u_step;
            sweep_env = u >= curve_end_u ? 0.0f : expf(-6.907755f * powf(u, curve_g));
        }
        float ratio = 1.0f + pitch_depth * sweep_env;
        float tail_position = 0.f;
        if(age >= tail_start_samples)
        {
            tail_position = tail_rate_hz > 0.f
                ? .5f - .5f*cosf(TWO_PI * tail_lfo_phase)
                : RaisedCosine01(float(age-tail_start_samples) / tail_glide_samples);
            tail_lfo_phase += tail_rate_hz / SAMPLE_RATE;
            tail_lfo_phase -= floorf(tail_lfo_phase);
        }
        float frequency = base_hz * ratio * powf(2.f, tail_semitones * tail_position / 12.f);
        instantaneous_hz = frequency;
        float sine = sinf(body_phase * TWO_PI);
        float body_wave = DrumBodyWaveshape(sine, body_shape_morph, body_shape_asymmetry);
        if(wave_morph > 0.0f)
        {
            // Single phase-derived, anti-aliased saw; subtract its own
            // fundamental so WAVE cannot double/cancel the pitched body.
            float q = body_phase + .5f;
            if(q >= 1.0f) q -= 1.0f;
            float harmonics = PolyBlepSaw(q, frequency / SAMPLE_RATE) * (PI * .5f) - sine;
            body_wave += harmonics * wave_morph;
        }
        float pulse = 0.0f;
        if(age < attack_pulse_width_samples)
        {
            float x = float(age) / attack_pulse_width_samples;
            pulse = sinf(TWO_PI*x) * sinf(PI*x);
        }
        float noise = NextNoise();
        noise_lp += .10f * (noise-noise_lp);
        noise_band_lp += attack_noise_band_a * (noise-noise_lp-noise_band_lp);
        float attack = ((1.0f-attack_noise_mix)*pulse + attack_noise_mix*noise_band_lp)
                       * attack_env * attack_level;
        // Eight ms at the bass endpoint, smoothly reaching .5 ms for kicks.
        float onset = RaisedCosine01(float(age) / onset_samples);
        o.punch = (body_wave * body_env * KICK_BODY_INTERNAL_LEVEL * SweepAmplitudeCompensation(ratio)
                   + attack) * onset;
        o.gate = TailGate();
        phase += frequency / SAMPLE_RATE;
        phase -= floorf(phase);
        body_phase = phase;
        pitch_env *= pitch_coeff;
        if(age >= body_hold_samples) body_env *= decay_coeff;
        attack_env *= attack_coeff;
        ++age;
        if(body_env < KICK_MIN_AUDIO_ENV && attack_env < KICK_MIN_AUDIO_ENV)
        {
            active = false;
            body_env = attack_env = 0.0f;
        }
    }
};


static KickVoice kick_voice;

static float kick_punch_gain_smoothed = 1.0f;
static float kick_sub_gain_smoothed = 1.0f;

/* Retained for surrounding legacy code; the new core never varies onset by history. */
static bool kick_fresh_hit = true;
static float kick_onset_level = 1.0f;

/* Legacy guard state retained because performance code still references it. */
static float final_hf_guard_cutoff = OLD_FINAL_HF_SETTLED_HZ;
static float final_hf_guard_state[3] = {0.0f, 0.0f, 0.0f};


/*
 * TAIL DELAY AMOUNT/STATE never delays audio. This returns only the requested
 * silent gap length applied after mixer1, before the reverb.
 */
static float TailGateGapMs()
{
    if(!tail_delay_enabled || macro_tail_delay <= 0.005f)
        return 0.0f;

    return MacroTailDelayMs(macro_tail_delay);
}


static void TriggerKickVoice(uint8_t velocity)
{
    /*
     * Hard deterministic retrigger. The previous hit is intentionally NOT
     * copied into fading slots or handed off. Starting the new oscillator at
     * a zero crossing, together with the finite output bridge, preserves
     * continuity. The swept body supplies the transient.
     */
    kick_fresh_hit = true;

    kick_voice.Trigger(
        kick_frequency,
        macro_kick_shape,
        .55f, // retained unused argument for historical host voice probes
        kick_sweep_time,
        velocity,
        TailGateGapMs(),
        MacroDecaySeconds(macro_decay),
        macro_wave,
        kick_tail_mod_cc,
        kick_belly_cc
    );

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


    void SetLowShelf(float frequency, float gain_db)
    {
        // RBJ low shelf, slope S=1. Gains are normalized by a0.
        float A=powf(10.f,gain_db/40.f);
        float w=TWO_PI*frequency/SAMPLE_RATE, c=cosf(w), sn=sinf(w);
        float beta=sqrtf(2.f*A)*sn;
        float a0=(A+1.f)+(A-1.f)*c+beta;
        b0=A*((A+1.f)-(A-1.f)*c+beta)/a0;
        b1=2.f*A*((A-1.f)-(A+1.f)*c)/a0;
        b2=A*((A+1.f)-(A-1.f)*c-beta)/a0;
        a1=-2.f*((A-1.f)+(A+1.f)*c)/a0;
        a2=((A+1.f)+(A-1.f)*c-beta)/a0;
    }

    void SetPeak(float frequency, float gain_db, float rate = SAMPLE_RATE)
    {
        // RBJ peaking EQ, fixed two-octave bandwidth like the 8-bus low mid.
        float w = TWO_PI * frequency / rate, sn = sinf(w), cs = cosf(w);
        float A = powf(10.f, gain_db / 40.f);
        float alpha = sn * sinhf(.69314718f * w / sn);
        float a0 = 1.f + alpha/A;
        b0=(1.f+alpha*A)/a0; b1=-2.f*cs/a0; b2=(1.f-alpha*A)/a0;
        a1=b1; a2=(1.f-alpha/A)/a0;
    }

    void SetLowpass(float frequency, float q, float rate = SAMPLE_RATE)
    {
        float w0 = TWO_PI * frequency / rate;
        float c = cosf(w0), alpha = sinf(w0) / (2.0f*q);
        float a0 = 1.0f + alpha;
        b0 = (1.0f-c) * .5f / a0;
        b1 = 2.0f*b0; b2 = b0;
        a1 = -2.0f*c/a0; a2 = (1.0f-alpha)/a0;
    }

    void SetHighpass(float frequency,
                     float q)
    {
        frequency = ClampAdded(frequency, 5.0f, 10000.0f);
        q = ClampAdded(q, 0.3f, 8.0f);

        float w0 = TWO_PI * frequency / SAMPLE_RATE;
        float c = cosf(w0);
        float s = sinf(w0);
        float alpha = s / (2.0f * q);
        float a0 = 1.0f + alpha;
        float hp = (1.0f + c) * 0.5f;

        b0 = hp / a0;
        b1 = -(1.0f + c) / a0;
        b2 = hp / a0;
        a1 = -2.0f * c / a0;
        a2 = (1.0f - alpha) / a0;
    }
};


// Matched fourth-order Linkwitz-Riley crossover. For identical linear
// inputs LP+HP is flat in magnitude with matching phase at every frequency.
// Distortion/BPF changes the dirty waveform: it is not a perfect crossover
// reconstruction, but the eight-pole return's extra rotations are gone.
struct KickCrossover
{
    Biquad stages[2];
    void Reset(bool highpass)
    {
        for(auto& stage : stages)
        {
            stage.Reset();
            if(highpass) stage.SetHighpass(240.f, .70710678f);
            else stage.SetLowpass(240.f, .70710678f);
        }
    }
    float Process(float x)
    {
        return stages[1].Process(stages[0].Process(x));
    }
};
static KickCrossover clean_lowpass, character_highpass;

struct CleanBassShelf
{
    Biquad eq;
    void Reset(){eq.Reset();eq.SetLowShelf(120.f,15.f);}
    float Process(float x,float amount)
    {
        float boosted=eq.Process(x);
        return x+Clamp01Added(amount)*(boosted-x);
    }
};
static CleanBassShelf clean_bass_shelf;


// Gentle bus compression, no makeup gain, no saturation. A 30 ms detector
// attack preserves the onset; 150 ms release avoids following bass cycles.
// It is continuous across triggers because the external bus shares it.
struct MixGlue
{
    float envelope = 0.f;
    float gain = 1.f;
    void Reset() { envelope = 0.f; gain = 1.f; }
    float Process(float input)
    {
        float level = fabsf(input);
        float a = level > envelope ? .99930580f : .99986112f;
        envelope = level + a * (envelope-level);
        // 1.25:1 above -12 dBFS, capped at 2 dB reduction.
        gain = envelope > .25f ? fmaxf(.79432823f, powf(.25f/envelope, .2f)) : 1.f;
        return input * gain;
    }
};
static MixGlue mix_glue;
static constexpr float MIX_OUTPUT_TRIM = .8f;

// -0.26 dB at 30 Hz, approximately unity throughout the kick body.
static constexpr float FINAL_INFRASONIC_HPF_HZ = 15.0f;
static constexpr float FINAL_INFRASONIC_HPF_Q = .70710678f;
static Biquad final_infrasonic_hpf;

// One output continuity bridge, after all reset filters. It expires exactly
// after 2 ms (8 ms at SHAPE zero); no old oscillator remains beneath a hit.
struct KickOutputBridge
{
    float last = 0.0f, previous = 0.0f, residual = 0.0f, slope = 0.0f;
    uint32_t age = 96, length = 96;
    void Reset() { *this = KickOutputBridge(); }
    void Trigger()
    {
        length = kick_voice.onset_samples > 96 ? kick_voice.onset_samples : 96;
        residual = last;
        slope = last-previous;
        age = 0;
    }
    float Process(float x)
    {
        if(age == 0) residual -= x;
        if(age < length)
        {
            float t = float(age) / length;
            x += residual * (1.0f-SmoothstepAdded(t));
            // Preserve the outgoing slope briefly without extrapolating a
            // high-frequency transient throughout the whole bridge.
            if(age < 12)
            {
                float r = 1.0f-float(age)/12.0f;
                x += slope * float(age) * r*r*r;
            }
            ++age;
        }
        previous = last;
        last = x;
        return x;
    }
};
static KickOutputBridge kick_output_bridge;

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
 * SIDECHAIN REVERB (CC36) — after the kick gate, before mixer2/glue.
 *
 * Schroeder topology: four parallel combs into two series allpasses. The
 * SEND is high-passed at 250 Hz so only the punch and upper body excite the
 * tank; letting the sub in turns a kick reverb to mud immediately.
 *
 * The return is then ducked by the dry kick's own envelope, so the tail
 * blooms in the gaps rather than smearing over the attack. That is the
 * sidechain: no external key input, the kick keys itself.
 */
// Long delay lives in SDRAM, not the nearly-full internal SRAM. Initialized
// explicitly after hw.Init(); never cleared or allocated on a kick trigger.
static constexpr int REVERB_DELAY_SIZE = 48002;
static float DSY_SDRAM_BSS reverb_delay_buffer[REVERB_DELAY_SIZE];
static Biquad reverb_return_hp;
// Outside the tank object to avoid putting its zero-filled arrays in FLASH.
static Biquad reverb_echo_hp, reverb_echo_lp;
static float DSY_SDRAM_BSS external_reverb_delay_buffer[REVERB_DELAY_SIZE];
static Biquad external_reverb_return_hp, external_reverb_echo_hp, external_reverb_echo_lp;
static constexpr float REVERB_ECHO_QUARTER_NOTES = .5f;
static constexpr float REVERB_ECHO_RETURN = .72f; // +4.08 dB vs previous .45

struct KickSidechainReverb
{
    static constexpr int C0=1116,C1=1188,C2=1277,C3=1356,A0=556,A1=441;
    float comb0[C0]={},comb1[C1]={},comb2[C2]={},comb3[C3]={};
    float ap0[A0]={},ap1[A1]={};
    int ci0=0,ci1=0,ci2=0,ci3=0,ai0=0,ai1=0,write=0;
    float lp0=0,lp1=0,lp2=0,lp3=0,send_lp=0,echo_lp=0;
    float amount_smooth=0,detector=0,duck_gain=0;
    float tap=0,next_tap=0,tap_fade=0;
    uint32_t duck_hold=0,clock_divider=0;
    bool changing_tap=false;

    float* delay_buffer=nullptr;
    Biquad *return_hp=nullptr,*echo_hp=nullptr,*echo_filter=nullptr;
    void Reset(bool external=false)
    {
        delay_buffer=external?external_reverb_delay_buffer:reverb_delay_buffer;
        return_hp=external?&external_reverb_return_hp:&reverb_return_hp;
        echo_hp=external?&external_reverb_echo_hp:&reverb_echo_hp;
        echo_filter=external?&external_reverb_echo_lp:&reverb_echo_lp;
        for(float& x:comb0)x=0;
        for(float& x:comb1)x=0;
        for(float& x:comb2)x=0;
        for(float& x:comb3)x=0;
        for(float& x:ap0)x=0;
        for(float& x:ap1)x=0;
        for(int i=0;i<REVERB_DELAY_SIZE;++i)delay_buffer[i]=0;
        ci0=ci1=ci2=ci3=ai0=ai1=write=0;
        lp0=lp1=lp2=lp3=send_lp=echo_lp=0;
        amount_smooth=detector=0;duck_gain=1;
        duck_hold=clock_divider=0;changing_tap=false;tap_fade=0;
        tap=next_tap=ClampAdded(perf_quarter_note_ms*(SAMPLE_RATE*.001f)*REVERB_ECHO_QUARTER_NOTES,48.f,48000.f);
        return_hp->Reset();return_hp->SetHighpass(180.f,.70710678f);
        echo_hp->Reset();echo_hp->SetHighpass(240.f,.70710678f);
        echo_filter->Reset();echo_filter->SetLowpass(2800.f,.70710678f);
    }
    void Trigger(){duck_hold=720;} // 15 ms hold; never clear the tank.

    static float Comb(float in,float* buf,int size,int& idx,float& store,float fb)
    {
        float out=buf[idx];
        store=out*.55f+store*.45f;
        buf[idx]=in+store*fb;
        if(++idx>=size)idx=0;
        return out;
    }
    static float Allpass(float in,float* buf,int size,int& idx)
    {
        float buffered=buf[idx],out=buffered-in;
        buf[idx]=in+buffered*.5f;
        if(++idx>=size)idx=0;
        return out;
    }
    float ReadTap(float delay) const
    {
        // Wrap integer indices, not a negative fractional float: adding the
        // buffer length to a tiny negative value can round to SIZE itself.
        int whole=int(delay);
        float fraction=delay-whole;
        int i=write-whole;
        if(i<0)i+=REVERB_DELAY_SIZE;
        int previous=i-1;
        if(previous<0)previous+=REVERB_DELAY_SIZE;
        return delay_buffer[i]
            +(delay_buffer[previous]-delay_buffer[i])*fraction;
    }
    float Process(float dry,float amount)
    {
        amount_smooth+=(Clamp01Added(amount)-amount_smooth)*.0006942034f; // 30 ms
        if(amount==0.f && amount_smooth<1e-7f)amount_smooth=0.f;
        const float x=amount_smooth;
        const float echo_mix=SmoothstepAdded(Clamp01Added((x-.65f)/.35f));
        // Update at 1 kHz; hysteresis rejects clock jitter. Fixed read heads
        // crossfade over 50 ms, rather than pitching a moving delay line.
        if(++clock_divider>=48)
        {
            clock_divider=0;
            float wanted=ClampAdded(perf_quarter_note_ms*(SAMPLE_RATE*.001f)*REVERB_ECHO_QUARTER_NOTES,48.f,48000.f);
            if(!changing_tap && fabsf(wanted-tap)>fmaxf(24.f,tap*.01f))
            {next_tap=wanted;tap_fade=0;changing_tap=true;}
        }
        float echo=ReadTap(tap);
        if(changing_tap)
        {
            echo+=(ReadTap(next_tap)-echo)*SmoothstepAdded(tap_fade);
            tap_fade+=1.f/2400.f;
            if(tap_fade>=1.f){tap=next_tap;changing_tap=false;}
        }
        // Feedback stays below unity. Damping and HP keep repetitions dark
        // and out of the clean bass; the dry path is never filtered here.
        send_lp+=(dry-send_lp)*.03851f; // original ~300 Hz send HP
        float send=(dry-send_lp)*x*.65f;
        // Filter each audible repeat AND its feedback, not only the send.
        echo_lp=echo_filter->Process(echo_hp->Process(echo));
        delay_buffer[write]=send+echo_lp*(.35f+.25f*echo_mix);
        if(++write==REVERB_DELAY_SIZE)write=0;
        float feed=send+echo_lp*echo_mix*.25f;
        float fb=.86f+.07f*x;
        float wet=.25f*(Comb(feed,comb0,C0,ci0,lp0,fb)+Comb(feed,comb1,C1,ci1,lp1,fb)
                       +Comb(feed,comb2,C2,ci2,lp2,fb)+Comb(feed,comb3,C3,ci3,lp3,fb));
        wet=Allpass(wet,ap0,A0,ai0);wet=Allpass(wet,ap1,A1,ai1);
        wet=return_hp->Process(wet*.70f+echo_lp*echo_mix*REVERB_ECHO_RETURN);
        float level=fabsf(dry);
        detector+=(level-detector)*(level>detector?.0103626f:.000347162f);
        float target=1.f/(1.f+30.f*detector);
        if(duck_hold){--duck_hold;target=fminf(target,.18f);}
        duck_gain+=(target-duck_gain)*(target<duck_gain?.0103626f:.0002083116f);
        return dry+wet*duck_gain*x;
    }
};


static KickSidechainReverb kick_reverb;
static KickSidechainReverb DSY_SDRAM_BSS external_reverb;


/* Last-resort DAC ceiling, unity below .93. Normal full-mixer settings
 * are verified below its knee: it is not used as mastering compression.
 */
static inline float OutputCeiling(float x)
{
    constexpr float knee  = 0.93f;
    constexpr float limit = 0.985f;
    constexpr float range = limit - knee;

    float magnitude = fabsf(x);
    if(magnitude <= knee)
        return x;

    float over = magnitude - knee;
    float shaped = knee + range * (over / (over + range));

    return x < 0.0f ? -shaped : shaped;
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
     * Note-Off. DECAY (CC40) owns musical decay through the voice body envelope.
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
             * moving DECAY after the note has already been released.
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
             * Release() starts from the CURRENT value, so changing DECAY
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
         *     uses the current SWEEP (CC78) duration.
         *
         * Ghost kick:
         *     uses the SAME SWEEP-derived dummy duration.
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
            MacroKickSweepSeconds(
                kick_sweep_time
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
   EXTERNAL-INPUT DJ HIGH-PASS
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

// One pre-distortion filter bank. With no layers it bypasses; otherwise
// BPF crossfades from the full kick to the mean of the selected bands.
// It cannot create a second post-distortion return or bypass the dirty HPF.
// The panel retains BPF as its label, but these are serial mid BOOSTS,
// never replacement band-pass audio. Frequency CC laws stay 85..3200 Hz.
struct MacroBpfBank
{
    Biquad drive_filters[3], oversampled_filters[3];
    float current_hz[3] = {330.f,700.f,1450.f};
    float configured_hz[3] = {}, configured_boost[3] = {-1.f,-1.f,-1.f};
    unsigned update_samples = 0;
    uint32_t coefficient_updates = 0;
    void Reset()
    {
        for(int i=0;i<3;++i)
        {
            drive_filters[i].Reset(); oversampled_filters[i].Reset();
            current_hz[i]=macro_bpf_target_hz[i];
        }
        Update(48);
    }
    void Update(unsigned samples=8)
    {
        update_samples += samples;
        if(update_samples < 48) return; // 1 kHz control rate, not 6 kHz
        update_samples = 0;
        for(int i=0;i<3;++i)
        {
            float target=macro_bpf_target_hz[i];
            current_hz[i]+=(target-current_hz[i])*.18126925f; // 5 ms
            if(fabsf(target-current_hz[i]) < target*.0001f) current_hz[i]=target;
            float boost=15.f*Clamp01Added(param_bpf_gain);
            if(fabsf(current_hz[i]-configured_hz[i]) < current_hz[i]*.0001f
               && fabsf(boost-configured_boost[i]) < .001f) continue;
            drive_filters[i].SetPeak(current_hz[i],boost);
            oversampled_filters[i].SetPeak(current_hz[i],boost,SAMPLE_RATE*4.f);
            configured_hz[i]=current_hz[i]; configured_boost[i]=boost;
            ++coefficient_updates;
        }
    }
    float ProcessDriveFeed(float x)
    {
        for(int i=0;i<macro_bpf_layer_count_latched;++i) x=drive_filters[i].Process(x);
        return x;
    }
};
static MacroBpfBank macro_bpf_bank;

// Symmetric 97-tap anti-imaging/anti-alias FIR at 192 kHz. Coefficients are
// generated by test/daisy_kick_low_end/design_oversampling.py. Each filter
// delays 12 base-rate samples; the round trip delays exactly 24 samples.
#include "mackie_fir.h"
struct MackieOversampling
{
    float input_history[50] = {}, output_history[194] = {};
    int input_pos=0, output_pos=0;
    void Reset(){ *this=MackieOversampling(); }
    void Push(float x){ if(--input_pos<0)input_pos=24; input_history[input_pos]=input_history[input_pos+25]=x; }
    float Upsample(int phase)
    {
        float y=0.f;
        for(int k=phase,j=0;k<97;k+=4,++j)
            y+=4.f*MACKIE_FIR[k]*input_history[input_pos+j];
        return y;
    }
    float Downsample(float x, bool emit)
    {
        if(--output_pos<0)output_pos=96;
        output_history[output_pos]=output_history[output_pos+97]=x;
        if(!emit)return 0.f;
        float y=0.f;
        for(int k=0;k<97;++k)y+=MACKIE_FIR[k]*output_history[output_pos+k];
        return y;
    }
};
struct CharacterAlignment
{
    float history[24] = {}; int pos=0;
    void Reset(){ *this=CharacterAlignment(); }
    float Process(float x){float y=history[pos];history[pos]=x;if(++pos==24)pos=0;return y;}
};
static CharacterAlignment clean_alignment;

// A cascade of one to three desk-like channels. Every EQ and nonlinear
// stage runs at 192 kHz; there is no base-rate second clipper. Each channel
// has bounded smooth asymmetric rails and DC coupling. No parallel stacks.
struct MacroMackieProcessor
{
    MackieOversampling os;
    Biquad output_lowpass;
    float dc_x[3]={},dc_y[3]={};
    static float Core(float x)
    {
        float shaped=x+.020f*x*fabsf(x);
        float rail=shaped>=0.f ? 1.f : .955f;
        float u=shaped/rail;
        return rail*u/sqrtf(1.f+u*u);
    }
    void Reset()
    {
        os.Reset(); output_lowpass.Reset();
        output_lowpass.SetLowpass(12000.f,.70710678f,SAMPLE_RATE*4.f);
        for(int i=0;i<3;++i){dc_x[i]=dc_y[i]=0.f;macro_bpf_bank.oversampled_filters[i].Reset();}
    }
    float Process(float input,float amount)
    {
        os.Push(input);
        float result=0.f;
        int count=macro_bpf_layer_count_latched;
        if(count<1)count=1;
        for(int phase=0;phase<4;++phase)
        {
            float x=os.Upsample(phase);
            for(int channel=0;channel<count;++channel)
            {
                if(channel<macro_bpf_layer_count_latched)
                    x=macro_bpf_bank.oversampled_filters[channel].Process(x);
                float drive=channel==0 ? MACKIE_INTERNAL_GAIN : 1.f+2.f*amount;
                x=Core(x*drive);
                float y=x-dc_x[channel]+.99983639f*dc_y[channel]; // 5 Hz @192k
                dc_x[channel]=x; dc_y[channel]=y; x=y;
            }
            x=output_lowpass.Process(x);
            float y=os.Downsample(x,phase==0);
            if(phase==0)result=y;
        }
        return result*param_mackie_gain;
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
       coupling caps           ~5 Hz high-pass between stages; their corner
                               removes infrasonic content.
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
 * Two inverting triode stages give a positive-polarity return. Preserve
 * their net polarity before the common post-distortion high-pass.
 */
static constexpr float TUBE_OUTPUT_GAIN = 1.0f;


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

        /* Input coupling cap, ~5 Hz. */
        constexpr float couple_a = 0.00065428f;
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

            /* Interstage coupling cap, ~5 Hz, at the oversampled rate. */
            constexpr float mid_a = 0.000163611f;
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
    CharacterAlignment tube_alignment;
    void PrepareTube() { tube.Reset(); tube_alignment.Reset(); }

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
        /*
         * The character state is reset on every kick for sample-repeatability.
         * One finite bridge after the final output filter preserves continuity;
         * distortion receives the body/attack only; SUB never enters this path.
         */
        if(active_tube)
            PrepareTube();
        else
            PrepareMackie();
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
            float wet = tube_alignment.Process(tube.Process(
                macro_bpf_bank.ProcessDriveFeed(input) * drive, tube_amount_smoothed));
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

// Quantizer thresholds stay fixed. Interpolate adjacent integer bit depths
// instead of moving the thresholds with the knob (which causes zipper noise).
// Linear blend of time-aligned signals adds no equal-power gain bump.
struct WetBitcrusher
{
    float amount=0.f;
    void Reset(){ amount=0.f; }
    float Process(float x, float target=bitcrush_target)
    {
        amount+=(Clamp01Added(target)-amount)*.0004165799f; // 50 ms
        if(amount<.000001f && target==0.f){amount=0.f;return x;}
        // Reach audible low resolutions early; the last part becomes a
        // coarse, gated quantizer instead of stopping at two bits.
        float remaining=1.f-amount;
        float bits=1.f+15.f*remaining*remaining*remaining*remaining;
        float wet=1.f-remaining*remaining;
        int lower=static_cast<int>(bits);
        float fraction=bits-lower;
        float levels=static_cast<float>(1u << (lower-1));
        float coarse=roundf(x*levels)/levels;
        float fine=roundf(x*(2.f*levels))/(2.f*levels);
        float crushed=coarse+(fine-coarse)*fraction;
        return x+(crushed-x)*wet;
    }
};
static WetBitcrusher wet_bitcrusher, external_bitcrusher;




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
     * The reversed hit has run out while reverse is on: stay silent until
     * the next kick rather than fading the live hit's tail back in under it
     * (an 8 ms swell of a full-level tail, heard as a thump).
     */
    bool reverse_finished = false;


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
        reverse_finished = false;

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


        reverse_finished = false;

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

            /*
             * Go straight to the reversed bank. Crossfading into it from the
             * live signal let ~8 ms of the NEW forward hit (its attack and
             * sweep) through in front of every reverse kick: a click. The
             * bridge starts on the exact last output sample instead, so
             * nothing steps whatever was sounding before.
             */
            bank_handoff_active = true;
            bank_handoff_from = last_output_sample;
            bank_handoff_pos = 0;
            wet.wet = 1.0f;
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


        if(!want_reverse && reverse_bass_enabled && reverse_finished)
        {
            /* Already ~0 after the end-edge fade; settle the rest to silence. */
            last_reversed_sample *= 0.995f;
            last_output_sample = last_reversed_sample;
            return last_reversed_sample;
        }


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
        {
            playing = false;
            reverse_finished = true;
        }


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
     * Kick-side stutter; the looper is used by ProcessExternal().
     */
    AddedStutter stutter;
    AddedQuantizedLooper looper;

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
         * the SAME SWEEP (CC78) duration law as the kick itself.
         */
        pump.TriggerWithSweepMs(
            MacroKickSweepSeconds(
                kick_sweep_time
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
                MacroKickSweepSeconds(
                    kick_sweep_time
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


    float ProcessMaster(float input){ return input; } // All repeat/filter slots are EXT-only.

};

constexpr uint32_t AddedPerformanceFx::LOOP_HISTORY_SAMPLES;

static AddedPerformanceFx added_performance_fx;


/* ============================================================
   MIDI UART INITIALIZATION
   ============================================================ */

#if defined(__arm__)
// Higher-level parsing stays in the foreground. This ISR only drains bytes.
// Linker alias routes the startup vector here, independently of libDaisy.
extern "C" void KickMidiRxIrq()
{
    uint32_t status=USART3->ISR;
    if(status & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE))
    {
        USART3->ICR=USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
        ++midi_uart_errors;
        midi_rx.Discontinuity();
    }
    while(USART3->ISR & USART_ISR_RXNE_RXFNE)
        midi_rx.Push(static_cast<uint8_t>(USART3->RDR));
}
#endif

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
#if defined(__arm__)
    // FIFO provides extra margin if another interrupt is briefly running.
    HAL_UARTEx_EnableFifoMode(&midi_uart);
    USART3->ICR=USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    USART3->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
    USART3->CR3 |= USART_CR3_EIE;
    HAL_NVIC_EnableIRQ(USART3_IRQn);
#endif

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

    if(velocity == 0)
    {
        HandleKickNoteOff(note);
        return;
    }

    /*
     * MIDI NOTE is the nominal kick pitch.
     *
     * Velocity controls bipolar tail-pitch excursion around this note.
     * SHAPE controls initial sweep depth/time and character. CC77 carries
     * an exact 0..127 tail value; CC78 sets sweep duration.
     */
    if(KICK_USE_MIDI_NOTE_FOR_TUNING)
    {
        float f = MidiNoteToFrequency(note);
        kick_frequency = ClampAdded(
            f,
            KICK_CHROMATIC_MIN_HZ,
            KICK_CHROMATIC_MAX_HZ
        );
    }
    else
    {
        kick_frequency = KICK_FIXED_FREQUENCY_HZ;
    }

    note_gate = true;
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
    bitcrush_target = 0.f;
    erosion_amount = 0.f;
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
        case 90: fx_bar_reset_mask=(fx_bar_reset_mask&~127u)|(value&127); return true;
        case 91: fx_bar_reset_mask=(fx_bar_reset_mask&127u)|((value&7u)<<7); return true;
        case 93: external_pitch_ratio=powf(2.f,(int(value)-64)/(value<64?64.f:63.f)); return true;
        case 92: fx_internal_routes=(value&31u)|((value&16u)?2u:0u); return true;
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

        case CC_EROSION_AMOUNT: erosion_amount=v; return true;
        case CC_EROSION_FREQUENCY: erosion_frequency=v; return true;
        case CC_MACRO_FX_BITCRUSH:
            bitcrush_target = v;
            return true;

        case CC_MACRO_FX_REVERB:
            param_reverb_amount = v * REVERB_AMOUNT_MAX;
            return true;


        /* ====================================================
           DECAY / REVERSE — ABSOLUTE STATE
           ==================================================== */

        case CC_DECAY_ABSOLUTE:
        {
            macro_decay_target = v;

            /* DECAY (CC40) controls the complete body lifetime; it never changes only the wet level. */

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
           TAIL DELAY — ABSOLUTE INTERNAL-SIDECHAIN AMOUNT + STATE
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
           KICK SHAPE / PUMP — ABSOLUTE STATE
           ==================================================== */

        case CC_KICK_SHAPE_ABSOLUTE:
        {
            macro_kick_shape = v;
            return true;
        }

        /* ====================================================
           PER-HIT TAIL PITCH AND SWEEP TIME
           ==================================================== */

        case CC_KICK_TAIL_PITCH:
        {
            kick_tail_pitch_cc = value;
            kick_tail_pitch_pending = true;
            return true;
        }

        case CC_KICK_SWEEP_TIME:
        {
            kick_sweep_time = v;
            return true;
        }

        /* Plain value, latched by the next hit. */
        case CC_WAVE:
        {
            macro_wave = v;
            return true;
        }

        case CC_KICK_TAIL_MOD:
        {
            kick_tail_mod_cc = value;
            return true;
        }

        case CC_KICK_BELLY:
        {
            kick_belly_cc = value;
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
            param_punch_gain = v;
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
        euro_clock_phase=0;

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
        // Forward received timing clocks even when the sender is stopped.
        // No free-running oscillator invents ticks after input clock stops.
        if(euro_clock_phase==0)euro_clock_pulse.Request();
        euro_clock_phase=(euro_clock_phase+1)%EURO_CLOCK_DIVIDER;
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
        if(midi_running && perf_clock_pulse_count%96==0 && fx_bar_reset_mask){
            const uint16_t resets=fx_bar_reset_mask;fx_bar_reset_mask=0;
            const uint8_t cc[]={30,31,32,33,34,35,36,37,38,93};
            for(uint8_t i=0;i<10;++i)if(resets&(1u<<i))HandleSixMacroCC(cc[i],i==9?64:0);
        }


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


    uint8_t type=midi_running_status & 0xF0;
    uint8_t channel=midi_running_status & 0x0F;
    // ALL channel messages must consume their data, including ignored ones.
    // Otherwise pitch bend / aftertouch running status overflows midi_data[2].
    uint8_t length=(type==0xC0 || type==0xD0) ? 1 : 2;
    if(midi_data_count>=length) midi_data_count=0;
    midi_data[midi_data_count++]=byte & 0x7F;
    if(midi_data_count<length) return;
    midi_data_count=0;

    /*
     * --------------------------------------------------------
     * CHANNEL 15 CONTROL CHANGE = ADDED PERFORMANCE FX
     * --------------------------------------------------------
     */
    if(type == 0xB0)
    {
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
                if(velocity>0)euro_kick_pulse.Request();
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
    // A 1024-byte interrupt-fed queue retains 327 ms of full-rate MIDI.
    // A gap marker invalidates both partial messages and running status.
    uint16_t received;
    unsigned budget=256;
    while(!kick_trigger_pending && budget-- && midi_rx.Pop(received))
    {
        if(received & MidiRxQueue::GAP)
        {
            midi_running_status=0; midi_data_count=0;
            kick_tail_pitch_pending=false;
        }
        ProcessMidiByte(static_cast<uint8_t>(received));
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
#if defined(__arm__)
    uint32_t cycle_start=DWT->CYCCNT;
#endif
    ServiceEuroOutputs(static_cast<uint32_t>(size));


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
         * Deterministic monophonic voice: fixed phase/envelope reset.
         * No previous-hit phase or bass trajectory is carried forward.
         */
        TriggerKickVoice(kick_tail_pitch_pending ? kick_tail_pitch_cc : last_velocity);
        kick_tail_pitch_pending = false;


        /*
         * MASTER OUTPUT DE-CLICK ENVELOPE:
         * Note-On -> short attack -> unity hold.
         */
        added_kick_master_envelope.Trigger();


        /* Latch the BPF layer count for this hit; see the declaration. */
        macro_bpf_layer_count_latched = macro_bpf_layer_count;

        // All generator/character/filter history resets. Continuity belongs
        // to a single finite bridge after the final filter, not bass layers.
        kick_output_bridge.Trigger();
        final_infrasonic_hpf.Reset();
        character_highpass.Reset(true);
        clean_lowpass.Reset(false);
        clean_bass_shelf.Reset();
        clean_alignment.Reset();
        macro_bpf_bank.Reset();

        kick_reverb.Trigger();
        external_reverb.Trigger();

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
         * DECAY (CC40) changes the CURRENT body's decay coefficient. Because
         * the clean and distorted paths are derived only after this envelope,
         * the same decay law governs both. Retiming never steps amplitude.
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
     * MIDI note owns settled pitch; KICK SHAPE owns the pitch envelope;
     * DECAY (CC40) owns body lifetime. Note-Off has no audio effect.
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
    /* Slew the mix gains toward their CC targets; see their declarations. */
    param_line_gain    += (param_line_gain_target    - param_line_gain)    * PARAM_GAIN_SLEW;
    param_mackie_gain  += (param_mackie_gain_target  - param_mackie_gain)  * PARAM_GAIN_SLEW;
    param_tube_gain += (param_tube_gain_target - param_tube_gain) * PARAM_GAIN_SLEW;
    param_bpf_gain     += (param_bpf_gain_target     - param_bpf_gain)     * PARAM_GAIN_SLEW;
    macro_bpf_bank.Update(static_cast<unsigned>(size));


    /*
     * Slew DECAY toward its CC target once per block. Stepping the decay
     * coefficient straight from each MIDI message zippers while DECAY turns.
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
           GENERATOR: SWEPT BODY + CLEAN BASS SHELF
           ==================================================== */

        kick_punch_gain_smoothed +=
            (param_punch_gain - kick_punch_gain_smoothed) *
            KICK_GAIN_SMOOTH_A;

        kick_sub_gain_smoothed +=
            (param_sub_gain - kick_sub_gain_smoothed) *
            KICK_GAIN_SMOOTH_A;


        KickVoiceOut voices;

        kick_voice.Process(voices);

        // One kick feeds clean and dirty lanes. SUB now boosts existing lows
        // in the clean lane; it cannot add a new pitch or drive the clipper.
        float dry = clean_alignment.Process(
            clean_bass_shelf.Process(clean_lowpass.Process(voices.punch),
                                     kick_sub_gain_smoothed) * kick_punch_gain_smoothed);
        float wet = 0.0f;
        if(!KICK_BYPASS_WET)
        {
            float wet_send = voices.punch;
            wet = macro_character_processor.ProcessWet(wet_send);
            wet = wet_bitcrusher.Process(wet,(fx_internal_routes&4)?bitcrush_target:0.f);
            wet = character_highpass.Process(wet);
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


        kick_output = final_infrasonic_hpf.Process(kick_output);
        if(KICK_BYPASS_POST)
        {
            kick_output *= KICK_OUTPUT_LINEAR_GAIN * voices.gate;
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
             * Repeat and filter slots are locked to the external input.
             */
            kick_output =
                added_performance_fx.ProcessMaster(
                    kick_output
                );


            kick_output *=
                KICK_OUTPUT_LINEAR_GAIN;


            // The gate chops mixer1 before reverb. An enabled tank can ring
            // into the break; with reverb off the break remains silent.
            kick_output *= voices.gate;
            /* Reverb follows the chopped generated-kick bus. */
            kick_output =
                kick_reverb.Process(
                    kick_output,
                    (fx_internal_routes&2)?param_reverb_amount:0.f
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


        kick_output = kick_output_bridge.Process(kick_output);



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

           The two buses meet only at mixer2, before the shared glue/output.
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


        // Mixer2: generated kick FX plus external-only performance FX.
        // Both DAC channels carry the same mono mix.
        pump_internal_mix+=(((fx_internal_routes&1)?1.f:0.f)-pump_internal_mix)*.0006942034f;
        kick_output*=1.f+(added_performance_fx.pump.gain-1.f)*pump_internal_mix;
        external_output=external_pitch_fx.Process(external_output,external_pitch_ratio);
        external_output=external_reverb.Process(external_output,(fx_internal_routes&16)?0.f:param_reverb_amount);
        external_output=external_bitcrusher.Process(external_output,bitcrush_target);
        float mix=erosion_fx.Process(kick_output,(fx_internal_routes&8)?erosion_amount:0.f,erosion_frequency)
                 +external_erosion_fx.Process(external_output,erosion_amount,erosion_frequency);
        float mixed_output = OutputCeiling(mix_glue.Process(mix * MIX_OUTPUT_TRIM));
        out[KICK_OUTPUT_CHANNEL][i] = mixed_output;
        out[EXTERNAL_OUTPUT_CHANNEL][i] = mixed_output;

        /*
         * Advance the shared per-hit anatomy clock once per audio frame.
         */
        kick_age_samples++;
    }
#if defined(__arm__)
    uint32_t elapsed=DWT->CYCCNT-cycle_start;
    if(elapsed>audio_max_cycles) audio_max_cycles=elapsed;
    if(elapsed>uint32_t(SystemCoreClock / 48000u * size)) ++audio_overruns;
#endif

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
    ResetEuroOutputs();
    kick_voice.Reset();
    mix_glue.Reset();
    wet_bitcrusher.Reset();
    external_bitcrusher.Reset();
    erosion_fx.Reset();
    external_erosion_fx.Reset();

    character_highpass.Reset(true);
    clean_lowpass.Reset(false);
    clean_bass_shelf.Reset();
    clean_alignment.Reset();

    final_infrasonic_hpf.Reset();
    final_infrasonic_hpf.SetHighpass(
        FINAL_INFRASONIC_HPF_HZ,
        FINAL_INFRASONIC_HPF_Q
    );

    kick_output_bridge.Reset();

    final_hf_guard_cutoff = OLD_FINAL_HF_SETTLED_HZ;

    for(int p = 0; p < 3; p++)
        final_hf_guard_state[p] = 0.0f;

    kick_onset_level = 1.0f;

    added_performance_fx.Reset();
    added_kick_master_envelope.Reset();

    macro_bpf_bank.Reset();
    macro_character_processor.Reset();
    kick_reverb.Reset();
    external_reverb.Reset(true);
    external_pitch_fx.Reset();
    external_pitch_ratio=1.f;
    macro_whole_kick_reverse.Reset();
}


int main(void)
{
    /* --------------------------------------------------------
       DAISY
       -------------------------------------------------------- */

    hw.Configure();

    hw.Init();
    InitEuroOutputs();


    /*
     * 8-sample blocks still give sub-millisecond trigger latency while
     * providing more scheduling margin as FX are added. The previous
     * 4-sample block made audio overruns easier to hear as digital ticks.
     */
    hw.SetAudioBlockSize(16); // 0.33 ms; amortizes per-block work
#if defined(__arm__)
    // Avoid data-dependent slow paths as IIR tails enter denormal range.
    __set_FPSCR(__get_FPSCR() | (1u << 24));
    FPU->FPDSCR |= (1u << 24);
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif


    /* --------------------------------------------------------
       MIDI
       -------------------------------------------------------- */

    InitMidiUart();


    /* --------------------------------------------------------
       DSP INITIAL STATE
       -------------------------------------------------------- */

    kick_frequency = 52.0f;


    separation = 0.0f;


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
    bitcrush_target = 0.f;
    erosion_amount = 0.f;

    stutter_quantized_command = QUANT_FX_NONE;
    external_stutter_quantized_command = QUANT_FX_NONE;
    looper_quantized_command = QUANT_FX_NONE;
    master_decay_retime_pending = false;

    macro_fx_value_hpf = 0.0f;
    macro_fx_value_lpf = 0.0f;



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

            hw.StopAudio();
            ResetAudioDspState();
            hw.StartAudio(AudioCallback);
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
