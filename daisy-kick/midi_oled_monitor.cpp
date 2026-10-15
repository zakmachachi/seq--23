#include "daisy_seed.h"
#include "stm32h7xx_hal.h"

#include <math.h>
#include <stdint.h>
#include "midi_rx_queue.h"
#include "midi_pulse_output.h"
#include "erosion_fx.h"
#include "external_pitch_fx.h"
#include "../include/FixedDelayTap.h"
#include "../include/KickShapeModel.h"
#include "effects/HeldFxEngine.h"
#include "../include/SoftSynthProtocol.h"

using namespace daisy;


/* ============================================================
   HARDWARE
   ============================================================ */

DaisySeed hw;
UART_HandleTypeDef midi_uart;
static MidiRxQueue midi_rx;
static volatile uint32_t midi_uart_errors = 0;
static volatile uint32_t audio_max_cycles = 0, audio_overruns = 0;

// USB serial diagnostics (Seed micro-USB, 1 line per second). Measures what
// the codec input and output actually carry on hardware, which the desktop
// harness cannot. Accumulated in the audio callback, printed by main().
static constexpr bool KICK_USB_DIAGNOSTICS = false;
static constexpr size_t AUDIO_BLOCK_SIZE = 2; // see SetAudioBlockSize
struct DiagStats
{
    double sum[2], sum_sq[2];
    float peak[2], out_peak;
    uint32_t samples, max_cycles;
};
static DiagStats diag_accum, diag_snapshot;
static volatile bool diag_snapshot_ready = false;


// Eurorack interface logic outputs (3.3 V): D12/header pin 13 and D3/pin 4.
// Use external protected/level-shifted trigger drivers where 5 V is required.
static constexpr int EURO_KICK_GPIO = 12, EURO_CLOCK_GPIO = 3;
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
static constexpr float EXTERNAL_INPUT_1_GAIN = 0.00f;
// Input 2 is intentionally quarantined: leave the ADC channel disconnected
// without allowing its floating/noisy pin to enter the external bus.
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

/* Level restored after the glue, so its threshold and the kick's internal
 * balance are unchanged. +6 dB: the all-max mixer grid peaks at .72 and a
 * full-scale input sine plus an all-max kick at .90, both below the .93
 * ceiling knee. A louder line output lifts the kick above the DAC noise
 * floor instead of making the amp do it.
 * The external branch is trimmed by the same factor, so its input-to-output
 * gain is unchanged.
 */
static constexpr float OUTPUT_MAKEUP_GAIN = 2.0f;
static constexpr float EXTERNAL_OUTPUT_LINEAR_GAIN = 1.0f / OUTPUT_MAKEUP_GAIN;

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
static volatile float param_line_gain    = 1.00f;   /* CC53, bipolar +/-15 dB output gain */
static volatile float param_mackie_gain  = 0.65f;   /* CC54, max 1.25 */
static volatile float param_tube_gain    = 0.60f;   /* CC55, max 1.25 */
static volatile float param_bpf_gain     = 0.70f;   /* CC56, pre-drive BPF blend 0..1 */
static volatile float param_sub_gain     = 1.00f;   /* CC57, clean low-shelf boost 0..15 dB endpoint */
static volatile float param_punch_gain   = 1.00f;   /* CC58, clean LPF lane gain 0..1 */
static volatile float param_reverb_amount = 0.0f;   /* CC36, FX page  */

/* Function + Menu-2 knob 1 (CC53): bipolar output trim, 0 dB at centre. */
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



/* MIDI-clock fallback if no external clock has been received yet. */
static constexpr float DEFAULT_BPM = 180.0f;
static volatile float perf_quarter_note_ms =
    60000.0f / DEFAULT_BPM;

static uint32_t perf_clock_last_ms = 0;
static uint32_t perf_clock_pulse_count = 0;

static volatile bool perf_sixteenth_pending = false;
static volatile bool perf_quarter_pending = false;


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

/* HARMONICS: phase-locked VCO waveshaping, latched per hit. */
static constexpr uint8_t CC_HARMONICS = 64; // protocol-compatible CC number

/* TMOD (Menu 1 pot 5) sets the resettable tail LFO rate. Velocity supplies
 * signed pitch depth; zero TMOD holds the selected note pitch static. BELLY sets the
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
 *   CC42 AMP AHD HOLD           sweep time..one whole note
 *   CC43 FIXED TAIL GAP STATE   0=OFF, 127=ON; 32 ms gap
 *   CC44 BPF CENTRE             85 Hz..3.2 kHz
 *   CC45/46 legacy BPF centres  retained, not used by the raw path
 *   CC47 BPF SEND               absolute OFF / 1/3 / 2/3 / FULL
 *   CC48 MACKIE AMOUNT
 *   CC49 TUBE AMOUNT
 *   CC50 CHARACTER MODEL        0=MACKIE, 127=TUBE
 *   CC51 KICK SHAPE
 *   CC52 PUMP STATE             0=OFF, 127=ON
 *   CC77 TAIL PITCH             next hit: 0=-12 st, 64=0, 127=+12 st
 *   CC78 SWEEP TIME             absolute 4..120 ms macro; centre 58 ms
 *   CC64 HARMONICS             0=existing sine/body, 127=parabolic VCO enrichment
 *   CC79 TMOD RATE              0 static; 1..127 = .02..16 Hz
 *   CC61 BELLY                  64 = 1x punch window; 0 = 0.35x, 127 = 2x
 *
 * Note velocity is bipolar tail pitch; CC77 carries the full 0..127 range.
 *
 * CC100 is retained ONLY for the K1 UI/long-hold reset gesture.
 * Old CC20/21..25/101..105 are consumed and ignored, except the physical
 * K4/K5 compatibility CCs 23/24/103/104.
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
static constexpr uint8_t CC_KICK_TEMP_MUTE = 60;
static constexpr uint8_t CC_KICK_TEMP_LEVEL = 66;
static constexpr uint8_t CC_KICK_PERFORMANCE_HPF = 62;
static constexpr uint8_t CC_KICK_TEMP_DECAY = 63;
static constexpr uint8_t CC_KICK_DUB_ECHO = 65;
static constexpr uint8_t CC_KICK_GROUND_MACHINE = 70;
static volatile float bitcrush_target = 0.f;
static constexpr uint8_t CC_EROSION_AMOUNT=38, CC_EROSION_FREQUENCY=39;
static volatile float erosion_amount=0.f, erosion_frequency=64.f/127.f;
// FX route bits 0..3 add kick: pump, reverb, bitcrush, erosion.
// Bit 4 disables external reverb (INT-only); other effects keep EXT enabled.
static volatile uint8_t fx_internal_routes=14;
static ExternalPitchFx external_pitch_fx;
static volatile float external_pitch_ratio=1.f;
static heldfx::Control held_fx_control;
// Zero-initialized SDRAM storage; no large initialized-data image in FLASH.
static heldfx::Engine DSY_SDRAM_BSS held_fx_engine;
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
static constexpr uint8_t CC_MIX_EXT_KICK_MODE      = 59;
static constexpr uint8_t CC_KICK_TAIL_START        = 67;
static constexpr uint8_t CC_KICK_COMP_AMOUNT       = 68;
static constexpr uint8_t CC_KICK_COMP_ENABLE       = 69;
static constexpr uint8_t CC_KICK_CUTOFF_TIME       = 71;
static constexpr uint8_t CC_KICK_CUTOFF_ENABLE     = 72;

// EXT KICK is a volatile source switch. A short audio-rate fade prevents a
// discontinuity when the external signal replaces the synthesized voice.
static volatile bool external_kick_mode_target = false;
static float external_kick_mode_mix = 0.0f;
static constexpr float EXT_KICK_MODE_SMOOTH_A = 0.99791884f; // ~10 ms @48 kHz


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

/* Legacy six-macro addresses; only the K4/K5 ones still act. */
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
static volatile bool kick_temp_mute_target = false;
static volatile bool kick_temp_level_target = false;
static volatile bool kick_dub_echo_target = false;
static volatile bool kick_ground_machine_target = false;
static volatile bool kick_ground_decay_override = false;
static float kick_temp_level_gain = 1.f;


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
 * Raised by the audio thread when the kick output stops being a finite
 * number. The recovery itself runs in the control loop: it clears the
 * 43200-sample loop capture, which is far too long to sit inside an audio
 * block.
 */
static volatile bool audio_panic_pending = false;


/* DECAY (CC40): the target is snapshotted when a new hit starts. */
static volatile float macro_decay_target = 0.50f;
static bool reverse_bass_enabled = false;


/*
 * REVERSE SWITCHING POLICY
 *
 * Both directions are committed on the next MIDI kick. The capture and
 * playback window therefore follows the sequencer clock and never changes
 * halfway through a hit.
 */
static constexpr bool ALLOW_REVERSE_ENABLE_MID_KICK = false;

static volatile bool reverse_enable_pending = false;
static volatile bool reverse_disable_pending = false;


/* CC42 controls AHD hold; CC43 toggles the fixed 32 ms tail gate. */
static volatile float macro_amp_hold = 0.0f;
static volatile float macro_tail_start = 64.0f / 127.0f;
static volatile float kick_comp_amount = 0.0f;
static volatile bool kick_comp_enabled = false;
static volatile float kick_cutoff_time = 64.0f / 127.0f;
static volatile bool kick_cutoff_enabled = false;
static bool tail_delay_enabled = false;

/*
 * Beat-quantized requests. The control loop records what was asked for and
 * the audio thread applies it on the next quarter-note pulse, so the tail
 * delay and the K1 reset land on the beat instead of wherever the button
 * happened to be pressed. -1 means nothing is waiting.
 */
static volatile int8_t tail_delay_pending_state = -1;
static volatile bool fx_reset_pending = false;


/* Active BPF layer count: 0=bypass, 1..3=the first one to three layers. */
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

/* HARMONICS (CC64): 0 preserves the existing sine/body waveform. */
static volatile float macro_harmonics = 0.0f;

/* TMOD (CC79) rate defaults off; BELLY (CC61) defaults to 64. */
static volatile uint8_t kick_tail_mod_cc = 0; // CC79 rate; zero = no modulation
static volatile uint8_t kick_tail_pitch_cc = 64;
static volatile bool kick_tail_pitch_pending = false;
static volatile uint8_t kick_belly_cc = 64;
// Optional high-passed progression voice, configured by the sequencer before
// each MIDI-15 trigger. Values 0..100 are reserved for this layer protocol.
static volatile uint8_t kick_overlay_mode = 0;
static volatile uint8_t kick_overlay_note = 33;
static volatile uint8_t kick_overlay_decay = 64;
static volatile uint8_t kick_overlay_shape = 64;
static volatile uint8_t kick_overlay_sweep = 64;


/*
 * Button 6 is a dedicated pump performance toggle.
 * Pump depth/recovery is edited from the PUMP page of Macro 1.
 */
static bool macro_pump_enabled = false;


/* Physical controller K4 button edge latch (K4/K5 compatibility CCs). */
static bool legacy_bpf_button_down = false;


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


/* ============================================================
   MUSICAL MACRO MAPPINGS
   ============================================================ */

/* DECAY remains the 909 amplitude-envelope control. Its midpoint is close
 * to the measured 909 reference; the range extends from a short drum to a
 * pitched kick tails up to 2.4 seconds. */
static float MacroDecaySeconds(float x)
{
    x = Clamp01Added(x);
    /* T30 after the 60 ms body plateau: 40 ms..2.4 s. SHAPE=0 can use CC40=127
     * for gate-held sustain; matching note-off still gives a smooth release. */
    return 0.040f * powf(60.0f, x);
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
static volatile bool note_gate = false;
static uint8_t active_kick_note = 255;
static uint8_t kick_note_holds[128] = {};
static volatile bool kick_temp_decay_active = false;


/* ============================================================
   MIDI → AUDIO EVENTS
   ============================================================ */

static volatile bool kick_trigger_pending = false;
// Teensy stages a complete next-hit packet in CC 110..114 and commits it with
// CC 115. The MIDI note is then only a trigger edge; pitch and synthesis state
// are captured here and consumed together by the audio callback.
static volatile uint8_t kick_preset_stage[5] = {33,64,64,64,64};
static volatile uint8_t kick_preset_next[5]  = {33,64,64,64,64};
static volatile bool kick_preset_next_valid = true; // safe default before first packet
static volatile uint8_t kick_pending_preset[5] = {33,64,64,64,64};
static volatile bool kick_pending_preset_valid = false;


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
       skipping reverse, performance FX, headroom and
       reverb. With this and KICK_BYPASS_WET, Out 1 is the bare sine.
   ============================================================ */

static constexpr bool KICK_DAC_KEEPALIVE = false;
static constexpr bool KICK_BYPASS_WET = false;
static constexpr bool KICK_BYPASS_POST = false;

static constexpr float KICK_DAC_KEEPALIVE_OFFSET = 1.0f / 1048576.0f;


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
static inline float TailPitchRatio(float semitones)
{
    // Fifth-order exp approximation keeps per-sample pitch modulation cheap;
    // its error stays below about 0.02% across the full +/-12-semitone range.
    const float x = semitones * 0.057762265f;
    return 1.0f + x * (1.0f + x * (0.5f + x * (0.16666667f +
           x * (0.04166667f + x * 0.00833333f))));
}
static float TailModHz(uint8_t value)
{
    if(value == 0) return 0.f;
    // Start near DC so the first few knob values introduce a slow, gentle
    // wobble instead of jumping from static pitch to an audible fast LFO.
    const float x = float(value - 1) / 126.f;
    return .02f * powf(800.f, x * x); // .02..16 Hz
}
static float OutputGainFromCC(uint8_t value)
{
    const float db = value <= 64
        ? -15.0f * (64.0f - value) / 64.0f
        :  21.0f * (value - 64.0f) / 63.0f;
    return powf(10.0f, db / 20.0f);
}
static float ShapeStartRatio(float shape)
{
    float x = Clamp01Added(shape);
    /* Keep the measured 909 sweep at the centre, add finer control near a
     * steady bass note, and reserve the top end for laser-range excursions. */
    float semitones = x <= .5f ? 25.5f * powf(x * 2.f, 1.8f)
        : 25.5f + 22.5f * SmoothstepAdded((x-.5f)*2.f);
    return powf(2.f, semitones/12.f);
}
// The center is the useful 50..65 ms 909-like zone; the ends retain short
// laser and long falling-kick settings with finer control around the center.
static float KickSweepTimeMs(float x)
{
    x = Clamp01Added(x);
    if(x <= .25f)
        return 4.f * powf(5.f, x * 4.f); // 4..20 ms
    if(x <= .5f)
        return 20.f * powf(2.9f, (x - .25f) * 4.f); // 20..58 ms
    return 58.f * powf(120.f / 58.f, (x - .5f) * 2.f); // 58..120 ms
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
static float DrumBodyFundamentalNorm(float morph)
{
    return 1.0f / (1.0f + 0.151173637f * Clamp01Added(morph));
}

static float DrumBodyWaveshape(float sine_sample,
                               float morph,
                               float asymmetry,
                               float fundamental_norm)
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
     * through the entire Shape macro (DrumBodyFundamentalNorm, per hit).
     */
    return y * fundamental_norm;
}


/* ============================================================
   KICK GENERATOR: ONE SWEPT BODY
   ============================================================
   A resettable oscillator supplies the attack and settled note. HARMONICS is
   phase-derived; SUB is a clean low shelf downstream, not another oscillator.
   The original pulse/noise attack and raised-cosine onset are retained.
 */
static constexpr float KICK_GAIN_SMOOTH_A = 0.00208117f;
static constexpr float KICK_BODY_INTERNAL_LEVEL = 0.55f;
static constexpr float KICK_MIN_AUDIO_ENV = 0.00001f;
static constexpr float KICK_DECAY_COEFF_SMOOTH = 0.0025f;
static constexpr float RAW_909_SOURCE_SMOOTH_A = 0.004158f; // 5 ms source switch
static constexpr float RAW_909_BODY_HOLD_MS = 60.0f;
static constexpr float RAW_909_SWEEP_TAU_SCALE = 0.36f;
static constexpr float RAW_909_ATTACK_HARMONIC_COEFF = 0.9965338f; // 6 ms
static constexpr uint32_t RAW_909_RETRIGGER_SAMPLES = 240; // 5 ms two-voice handoff

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
// The settled value, so finished ramps skip the cosine bit-identically.
static const float RAISED_COSINE_END = RaisedCosine01(1.0f);
static inline uint32_t MsToSamples(float ms)
{
    return static_cast<uint32_t>(fmaxf(ms, 0.0f) * .001f * SAMPLE_RATE);
}
static inline float HarmonicPartialFade(float frequency, float multiple)
{
    // Fade each phase-locked partial out smoothly before Nyquist. This avoids
    // the bright, aliased edge of a discontinuous saw at high kick pitches.
    const float cycles_per_sample = frequency * multiple / SAMPLE_RATE;
    return SmoothstepAdded((0.48f - cycles_per_sample) / 0.06f);
}

struct KickVoiceOut
{
    float punch = 0.0f;
    float gate = 1.0f; // retained for the bypassed legacy path below
};

struct KickVoice
{
    bool active = false;
    bool sustained = false;
    bool infinite_sustain = false;
    uint32_t age = 0;
    float phase = 0.5f; // deterministic negative-going zero crossing
    float base_hz = KICK_FIXED_FREQUENCY_HZ;
    float instantaneous_hz = KICK_FIXED_FREQUENCY_HZ;
    float sweep_start_ratio = 1.0f, sweep_peak_ratio = 1.0f;
    float sweep_decay_coeff = 1.0f, sweep_tail = 1.0f;
    float attack_harmonic_env = 1.0f;
    float harmonics_amount = 0.0f;
    float character_mix = 1.0f, character_mix_target = 1.0f;
    float sustain_hz = KICK_FIXED_FREQUENCY_HZ;
    float tail_pitch_semitones = 0.0f, tail_lfo_phase = 0.0f;
    float tail_lfo_increment = 0.0f;
    uint32_t tail_glide_samples = 1;
    uint32_t sweep_rise_samples = 1;
    uint32_t body_hold_samples = 1;
    float body_env = 0.0f, decay_coeff = 1.0f, decay_coeff_target = 1.0f;
    float last_output = 0.0f;
    uint32_t onset_samples = RAW_909_RETRIGGER_SAMPLES; // legacy bridge compatibility

    void Reset() { *this = KickVoice(); }
    void SetDecay(float seconds, bool infinite=false)
    {
        infinite_sustain = sustained && infinite;
        decay_coeff_target = infinite_sustain ? 1.0f : BodyDecayCoefficient(ClampAdded(seconds, .0212f, 2.4f));
        if(!active) decay_coeff = decay_coeff_target;
    }
    void Trigger(float frequency, float kick_shape, float sweep_time_control,
                 float decay_seconds, uint8_t tune, uint8_t tmod, float wave)
    {
        const bool was_active = active;
        const float old_phase = phase;
        const float old_hz = instantaneous_hz;
        const float old_env = body_env;
        const float old_character = character_mix;
        const float character = SmoothstepAdded(Clamp01Added(kick_shape) * 5.0f);
        active = true;
        age = 0;
        base_hz = ClampAdded(frequency, KICK_CHROMATIC_MIN_HZ, KICK_CHROMATIC_MAX_HZ);
        character_mix_target = character;
        if(kick_shape <= 0.000001f)
        {
            // Shape zero is legato bass: carry oscillator phase, glide note
            // changes, and preserve the decay state. Only DECAY=127 holds the
            // level while MIDI gate is on; matching note-off releases smoothly.
            sustained = true;
            phase = was_active ? old_phase : 0.5f;
            sustain_hz = was_active ? old_hz : base_hz;
            instantaneous_hz = sustain_hz;
            character_mix = was_active ? old_character : 0.0f;
            body_env = was_active ? old_env : 1.0f;
            infinite_sustain = macro_decay_target >= 0.99999f;
            decay_coeff_target = decay_coeff = infinite_sustain ? 1.0f : BodyDecayCoefficient(ClampAdded(decay_seconds,.0212f,2.4f));
            sweep_start_ratio = sweep_peak_ratio = sweep_tail = 1.0f;
            tail_pitch_semitones = 0.0f;
            tail_lfo_increment = 0.0f;
            tail_glide_samples = 1;
            attack_harmonic_env = 0.0f;
            harmonics_amount = 0.0f;
            return;
        }
        sustained = false;
        phase = 0.5f;
        sweep_peak_ratio = ShapeStartRatio(kick_shape);
        sweep_start_ratio = 1.0f + 0.28f * (sweep_peak_ratio - 1.0f);

        /* A tune-50 reference begins near 100 Hz, peaks around 214 Hz in
         * ~5 ms, then falls exponentially to its ~50 Hz body. At the centre
         * SWEEP position, the measured decay constant is about 11 ms. */
        float sweep_ms = KickSweepTimeMs(sweep_time_control);
        float tau_ms = fmaxf(0.5f, sweep_ms * RAW_909_SWEEP_TAU_SCALE);
        sweep_decay_coeff = expf(-1.0f / (tau_ms * .001f * SAMPLE_RATE));
        sweep_tail = 1.0f;
        attack_harmonic_env = character;
        harmonics_amount = Clamp01Added(wave) * character;
        tail_pitch_semitones = TailPitchSemitones(tune) * character;
        tail_lfo_phase = 0.0f;
        tail_lfo_increment = TailModHz(tmod) / SAMPLE_RATE;
        sweep_rise_samples = MsToSamples(fmaxf(0.5f, sweep_ms * 0.15f));
        if(sweep_rise_samples < 1) sweep_rise_samples = 1;
        body_hold_samples = MsToSamples(RAW_909_BODY_HOLD_MS);
        tail_glide_samples = MsToSamples(fmaxf(80.0f, sweep_ms));
        if(tail_glide_samples < 1) tail_glide_samples = 1;

        body_env = 1.0f;
        SetDecay(decay_seconds);
        decay_coeff = decay_coeff_target;
    }
    void Process(KickVoiceOut& o)
    {
        if(!active) return;
        decay_coeff += (decay_coeff_target-decay_coeff) * KICK_DECAY_COEFF_SMOOTH;
        character_mix += (character_mix_target-character_mix) * 0.00104112f;

        float ratio;
        if(sustained)
        {
            sustain_hz += (base_hz-sustain_hz) * 0.000694203f; // ~30 ms note glide
            ratio = 1.0f;
        }
        else if(age < sweep_rise_samples)
        {
            float rise = SmoothstepAdded(float(age) / float(sweep_rise_samples));
            ratio = sweep_start_ratio + (sweep_peak_ratio - sweep_start_ratio) * rise;
        }
        else
        {
            ratio = 1.0f + (sweep_peak_ratio - 1.0f) * sweep_tail;
        }
        float tail_pitch = 0.0f;
        if(!sustained && age > body_hold_samples)
        {
            if(tail_lfo_increment > 0.0f)
            {
                const float p = tail_lfo_phase;
                const float tri = p < 0.25f ? 4.0f*p
                    : p < 0.75f ? 2.0f-4.0f*p : 4.0f*p-4.0f;
                tail_pitch = tail_pitch_semitones * tri;
                tail_lfo_phase += tail_lfo_increment;
                tail_lfo_phase -= floorf(tail_lfo_phase);
            }
        }
        float frequency = sustained
            ? sustain_hz
            : base_hz * ratio * TailPitchRatio(tail_pitch);
        instantaneous_hz = frequency;

        /* The reference's strike is a short, phase-locked 3rd/5th/7th
         * harmonic shoulder; its settled body retains only small 2nd/3rd
         * partials. This is one oscillator waveform, not a distortion lane. */
        const float fundamental = sinf(phase * TWO_PI);
        const float h = harmonics_amount * character_mix;
        float waveform;
        if(attack_harmonic_env < 0.0001f && h < 0.0001f)
        {
            // The transient partials are inaudible now. Keep the settled 2nd
            // and 3rd body partials but skip the strike-only 5th/7th partials
            // and their normalization for the remainder of long decays.
            waveform = fundamental + character_mix * (
                  0.037f * sinf(phase * TWO_PI * 2.0f)
                - 0.05f * sinf(phase * TWO_PI * 3.0f));
        }
        else
        {
        const float body_wave = fundamental + character_mix * (
              0.037f * sinf(phase * TWO_PI * 2.0f)
            + (-0.05f - 0.25f * attack_harmonic_env) * sinf(phase * TWO_PI * 3.0f)
            + 0.22f * attack_harmonic_env * sinf(phase * TWO_PI * 5.0f)
            - 0.05f * attack_harmonic_env * sinf(phase * TWO_PI * 7.0f));
        // HARMONICS reshapes this one oscillator. At zero this is bit-exactly
        // the existing body waveform; higher settings add phase-locked 2nd,
        // 3rd and 5th partials, progressing from round body to a harder,
        // parabolic kick character. There are no detuned voices to beat.
        const float h2 = 0.62f * h * HarmonicPartialFade(frequency, 2.0f);
        const float h3 = 0.26f * h * h * HarmonicPartialFade(frequency, 3.0f);
        const float h5 = 0.12f * h * h * h * HarmonicPartialFade(frequency, 5.0f);
        const float enriched = body_wave
            + h2 * sinf(phase * TWO_PI * 2.0f)
            - h3 * sinf(phase * TWO_PI * 3.0f)
            + h5 * sinf(phase * TWO_PI * 5.0f);
        const float added_energy = 0.125f * h2 * h2
            + 0.5f * h3 * h3 + 0.5f * h5 * h5;
        waveform = enriched / sqrtf(1.0f + added_energy);
        }
        attack_harmonic_env *= RAW_909_ATTACK_HARMONIC_COEFF;
        float sample = waveform * body_env * KICK_BODY_INTERNAL_LEVEL;

        o.punch = sample;
        o.gate = 1.0f;
        phase += frequency / SAMPLE_RATE;
        phase -= floorf(phase);
        if(age >= sweep_rise_samples) sweep_tail *= sweep_decay_coeff;
        if(sustained && note_gate && infinite_sustain)
            body_env += (1.0f-body_env) * 0.00041658f;
        else if(sustained)
            body_env *= infinite_sustain ? BodyDecayCoefficient(.040f) : decay_coeff;
        else if(age >= body_hold_samples)
        {
            const float effective_decay = 1.0f - character_mix * (1.0f-decay_coeff);
            body_env *= effective_decay;
        }
        ++age;
        last_output = sample;
        if(body_env < KICK_MIN_AUDIO_ENV)
        {
            active = false;
            body_env = 0.0f;
        }
    }

};

static KickVoice kick_voice;
static KickVoice kick_voice_previous;
static uint32_t kick_voice_handoff_age = RAW_909_RETRIGGER_SAMPLES;
static KickVoice kick_overlay_voice;
static KickVoice kick_overlay_previous;
static uint32_t kick_overlay_handoff_age = RAW_909_RETRIGGER_SAMPLES;
static uint32_t kick_overlay_attack_age = RAW_909_RETRIGGER_SAMPLES;
static float kick_overlay_hpf_x1=0.f,kick_overlay_hpf_y1=0.f;
static constexpr float KICK_OVERLAY_HPF_COEFF = 0.97091f; // 225 Hz @ 48 kHz
static constexpr float KICK_OVERLAY_EXTRA_GAIN = 2.0f;     // +6 dB before HPF
static constexpr uint32_t KICK_OVERLAY_MACKIE_FLUSH_SAMPLES = 480u; // 10 ms
static uint32_t kick_overlay_mackie_flush_samples = 0;

static float kick_punch_gain_smoothed = 1.0f;
static float kick_sub_gain_smoothed = 1.0f;


static void TriggerKickVoice(uint8_t velocity)
{
    const bool continuous_bass = macro_kick_shape <= 0.000001f;
    const float hit_decay = kick_ground_decay_override ? 0.323f
        : (kick_temp_decay_active ? 0.0f : macro_decay_target);
    const float hit_decay_seconds = MacroDecaySeconds(hit_decay);
    // Keep the outgoing-hit handoff brief, then stop it so long-decay hits
    // cannot continue phasing against later notes.
    if(continuous_bass)
    {
        // SHAPE zero is legato: the voice itself glides to the next note and
        // must not be copied/retriggered as a second kick oscillator.
        kick_voice_previous.active = false;
        kick_voice_handoff_age = RAW_909_RETRIGGER_SAMPLES;
    }
    else if(kick_voice.active)
    {
        kick_voice_previous = kick_voice;
        kick_voice_handoff_age = 0;
    }
    else
    {
        kick_voice_previous.active = false;
        kick_voice_handoff_age = RAW_909_RETRIGGER_SAMPLES;
    }
    const uint8_t tune = kick_tail_pitch_pending ? kick_tail_pitch_cc : velocity;
    kick_voice.Trigger(
        kick_frequency,
        macro_kick_shape,
        kick_sweep_time,
        hit_decay_seconds,
        tune,
        kick_tail_mod_cc,
        macro_harmonics
    );
    kick_voice.SetDecay(hit_decay_seconds,
                        !kick_temp_decay_active && hit_decay >= 0.99999f);
    kick_tail_pitch_pending = false;

    if(kick_overlay_voice.active){
        kick_overlay_previous=kick_overlay_voice;
        kick_overlay_handoff_age=0;
    }else if(!kick_overlay_previous.active){
        kick_overlay_handoff_age=RAW_909_RETRIGGER_SAMPLES;
    }
    kick_overlay_voice.active=false;
    if(kick_overlay_mode){
        // The parallel hit is an independent one-shot even when SHAPE is at
        // its sustained-bass endpoint; never inherit a free-running phase.
        kick_overlay_voice.Trigger(
            MidiNoteToFrequency(kick_overlay_note),
            float(kick_overlay_shape)/127.f,
            float(kick_overlay_sweep)/127.f,
            MacroDecaySeconds(float(kick_overlay_decay)/127.f),
            64,0,macro_harmonics
        );
        kick_overlay_voice.SetDecay(MacroDecaySeconds(float(kick_overlay_decay)/127.f));
        kick_overlay_attack_age=0;
    }else{
        // With no new layer, the previous overlay fades to silence over the
        // same short handoff and cannot ring into the next current kick.
        kick_overlay_voice.active=false;
        kick_overlay_attack_age=RAW_909_RETRIGGER_SAMPLES;
    }
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


        if(q > 20.0f)
            q = 20.0f;


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

// First-order, smoothed HPF for the temporary kick gestures. Cutoff and wet
// blend move continuously, so kick-boundary CC updates cannot click.
struct KickPerformanceHpf
{
    float cutoff=0.f,target=0.f,input1=0.f,output1=0.f;
    void Reset(){cutoff=target=input1=output1=0.f;}
    void Set(uint8_t value){
        target=value?140.f*powf(800.f/140.f,float(value-1u)/126.f):0.f;
    }
    float Process(float x){
        cutoff+=(target-cutoff)*.0015f;
        // Bilinear one-pole coefficient avoids a per-sample expf on the Seed.
        const float omega=TWO_PI*cutoff/SAMPLE_RATE;
        const float alpha=1.f/(1.f+omega);
        const float filtered=alpha*(output1+x-input1);
        input1=x;output1=filtered;
        const float blend=SmoothstepAdded(ClampAdded(cutoff/30.f,0.f,1.f));
        return x+(filtered-x)*blend;
    }
};
static KickPerformanceHpf kick_performance_hpf;


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
    // Per hit: the 240 Hz coefficients are fixed, only history is cleared.
    void ClearState(){ for(auto& stage : stages) stage.Reset(); }
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
    void ClearState(){eq.Reset();}
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

// Two-sample audio blocks keep callback ripple ultrasonic, but any residual
// 24 kHz DMA/supply component must still be removed before the DAC.
static Biquad final_ultrasonic_lowpass_a, final_ultrasonic_lowpass_b;

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
static constexpr float DUB_ECHO_FEEDBACK = 1.04f; // deliberately grows over successive repeats

struct KickSidechainReverb
{
    static constexpr int C0=1116,C1=1188,C2=1277,C3=1356,A0=556,A1=441;
    float comb0[C0]={},comb1[C1]={},comb2[C2]={},comb3[C3]={};
    float ap0[A0]={},ap1[A1]={};
    int ci0=0,ci1=0,ci2=0,ci3=0,ai0=0,ai1=0,write=0;
    float lp0=0,lp1=0,lp2=0,lp3=0,send_lp=0,echo_lp=0;
    float amount_smooth=0,detector=0,duck_gain=0;
    float tap=0,next_tap=0,tap_fade=0;
    uint32_t duck_hold=0,clock_divider=0,idle_samples=0,dub_tail_samples=0,dub_tail_total_samples=0;
    bool changing_tap=false,dub_was_active=false;
    // At zero amount the wet is muted; after the tank has rung below
    // -120 dB (combs .86, echo .35 per repeat of up to 1.2 s) stop it.
    static constexpr uint32_t IDLE_AFTER_SAMPLES=15u*48000u;

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
        duck_hold=clock_divider=idle_samples=dub_tail_samples=dub_tail_total_samples=0;changing_tap=false;dub_was_active=false;tap_fade=0;
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
    float Process(float dry,float amount,bool dub=false,bool allow_dub_tail=true)
    {
        if(!allow_dub_tail){dub_tail_samples=dub_tail_total_samples=0;dub_was_active=false;}
        else if(dub){
            if(!dub_was_active){
                const float three_steps=perf_quarter_note_ms*.75f;
                dub_tail_total_samples=uint32_t(ClampAdded(three_steps*.001f*SAMPLE_RATE,4800.f,72000.f));
                dub_tail_samples=dub_tail_total_samples;
            }
            dub_was_active=true;
        }else{
            dub_was_active=false;
            if(dub_tail_samples)--dub_tail_samples;
        }
        const bool releasing_dub=!dub&&dub_tail_samples>0;
        const bool dub_path=allow_dub_tail&&(dub||dub_tail_samples>0);
        const float dub_fade=dub?1.f:(dub_tail_total_samples?float(dub_tail_samples)/dub_tail_total_samples:0.f);
        const float amount_target=dub_path?fmaxf(amount,.90f*dub_fade):amount;
        amount_smooth+=(Clamp01Added(amount_target)-amount_smooth)*.0006942034f; // 30 ms
        if(amount==0.f && amount_smooth<1e-7f)amount_smooth=0.f;
        const float x=amount_smooth;
        const float echo_mix=dub_path?1.f:SmoothstepAdded(Clamp01Added((x-.65f)/.35f));
        // Update at 1 kHz; hysteresis rejects clock jitter. Fixed read heads
        // crossfade over 50 ms, rather than pitching a moving delay line.
        if(++clock_divider>=48)
        {
            clock_divider=0;
            float ratio=dub_path?.75f:REVERB_ECHO_QUARTER_NOTES;
            float wanted=ClampAdded(perf_quarter_note_ms*(SAMPLE_RATE*.001f)*ratio,48.f,48000.f);
            if(!changing_tap && fabsf(wanted-tap)>fmaxf(24.f,tap*.01f))
            {next_tap=wanted;tap_fade=0;changing_tap=true;}
        }
        if(x==0.f && idle_samples>=IDLE_AFTER_SAMPLES)
        {
            send_lp+=(dry-send_lp)*.03851f;
            UpdateDuck(dry);
            // Keep walking the SDRAM echo line (read, rewrite unchanged).
            // Measured on hardware: when nothing accesses SDRAM, an audible
            // whine appears on the codec output (likely its periodic
            // refresh, no longer broken up by traffic). One access per
            // sample per tank, as the stopped tank used to do, prevents it.
            volatile float* line=delay_buffer;
            line[write]=line[write];
            if(++write==REVERB_DELAY_SIZE)write=0;
            return dry;
        }
        idle_samples=x==0.f?idle_samples+1:0;
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
        float send=releasing_dub?0.f:(dry-send_lp)*x*.65f;
        // Filter each audible repeat AND its feedback, not only the send.
        echo_lp=echo_filter->Process(echo_hp->Process(echo));
        const float dub_feedback=dub?DUB_ECHO_FEEDBACK:(.65f+.39f*dub_fade);
        float delay_write=send+echo_lp*(dub_path?dub_feedback:(.35f+.25f*echo_mix));
        if(dub_path){
            // Above-unity feedback builds the dub repeats. A smooth rational
            // ceiling bounds the stored loop level before it can run away.
            const float magnitude=fabsf(delay_write);
            if(magnitude>1.2f){
                const float excess=magnitude-1.2f;
                delay_write=copysignf(1.2f+.8f*(excess/(.8f+excess)),delay_write);
            }
        }
        delay_buffer[write]=delay_write;
        if(++write==REVERB_DELAY_SIZE)write=0;
        float feed=send+echo_lp*echo_mix*.25f;
        float fb=.86f+.07f*x;
        float wet=.25f*(Comb(feed,comb0,C0,ci0,lp0,fb)+Comb(feed,comb1,C1,ci1,lp1,fb)
                       +Comb(feed,comb2,C2,ci2,lp2,fb)+Comb(feed,comb3,C3,ci3,lp3,fb));
        wet=Allpass(wet,ap0,A0,ai0);wet=Allpass(wet,ap1,A1,ai1);
        wet=return_hp->Process(wet*.70f+echo_lp*echo_mix*REVERB_ECHO_RETURN);
        UpdateDuck(dry);
        return dry+wet*duck_gain*x;
    }
    void UpdateDuck(float dry)
    {
        float level=fabsf(dry);
        detector+=(level-detector)*(level>detector?.0103626f:.000347162f);
        float target=1.f/(1.f+30.f*detector);
        if(duck_hold){--duck_hold;target=fminf(target,.18f);}
        duck_gain+=(target-duck_gain)*(target<duck_gain?.0103626f:.0002083116f);
    }
};


static KickSidechainReverb kick_reverb;
static KickSidechainReverb external_reverb;


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
        cached_macro = cached_quarter_ms = -1.0f;
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


    float cached_macro = -1.0f, cached_quarter_ms = -1.0f;
    float minimum_gain = 1.0f, attack_a = 0.0f, release_a = 0.0f;
    uint32_t attack_samples = 0;

    void Configure(float macro, float quarter_note_ms)
    {
        cached_macro = macro;
        cached_quarter_ms = quarter_note_ms;

        float depth = 0.22f + 0.74f * powf(macro, 0.82f);
        minimum_gain = 1.0f - depth;

        float attack_ms = 2.5f + macro * 1.5f;
        attack_a = expf(-5.0f / (SAMPLE_RATE * attack_ms / 1000.0f));
        attack_samples = static_cast<uint32_t>(
            SAMPLE_RATE * (attack_ms + 0.8f) / 1000.0f);

        float release_fraction = 0.10f + 0.58f * powf(macro, 1.22f);
        float release_ms = quarter_note_ms * release_fraction;
        if(release_ms < 32.0f)
            release_ms = 32.0f;
        release_a = expf(-5.0f / (SAMPLE_RATE * release_ms / 1000.0f));
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


        // Everything below depends only on the macro and the tempo: derive
        // it when either changes, not on every sample.
        if(macro != cached_macro || quarter_note_ms != cached_quarter_ms)
            Configure(macro, quarter_note_ms);


        if(attacking)
        {
            /*
             * Smooth duck onset.
             */
            gain =
                minimum_gain +
                (
                    gain -
                    minimum_gain
                ) *
                attack_a;


            attack_age++;


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
            gain =
                1.0f +
                (
                    gain -
                    1.0f
                ) *
                release_a;
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

// The long delay and loop histories live in SDRAM (read sequentially,
// cache-friendly), keeping internal SRAM free. Cleared by Reset() after
// hw.Init(); SDRAM is not zeroed at boot.
static float DSY_SDRAM_BSS clocked_delay_buffer[40800];
// Written on every sample, so it stays in internal SRAM: constant SDRAM
// traffic is a suspected source of audible noise on the analogue output.
static float loop_history_buffer[43200];

struct AddedClockedDelay
{
    static constexpr uint32_t MAX_SAMPLES = 40800;
    float* buffer = clocked_delay_buffer;

    uint32_t write = 0;
    uint32_t valid_written = 0;

    float macro_smoothed = 0.0f;
    float cached_quarter_ms = -1.0f, cached_m = -1.0f, feedback = 0.18f;
    uint32_t delay_samples = 0;
    FixedDelayTap tap;
    float send_gain=0,send_lp=0,send_dc=0,return_lp=0,return_dc=0;

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

        cached_quarter_ms = cached_m = -1.0f;

        wet.Reset();tap.Reset();
        send_gain=send_lp=send_dc=return_lp=return_dc=0;
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
            valid_written = 0;tap.Reset();
            send_gain=send_lp=send_dc=return_lp=return_dc=0;
            macro_smoothed = 0.0f;
            return input;
        }


        if(quarter_note_ms != cached_quarter_ms)
        {
            cached_quarter_ms = quarter_note_ms;
            float delay_ms =
                ClampAdded(quarter_note_ms * 1.5f, 20.0f, 840.0f);
            delay_samples =
                static_cast<uint32_t>(delay_ms * SAMPLE_RATE / 1000.0f);
            if(delay_samples >= MAX_SAMPLES)
                delay_samples = MAX_SAMPLES - 1;
        }


        float delayed = tap.Process(buffer, MAX_SAMPLES, write, valid_written,
                                    float(delay_samples));
        // Band-limit the return, including the feedback path. Fixed heads
        // avoid pitch modulation when MIDI-clock estimates change.
        return_dc += .019443f*(delayed-return_dc); // ~150 Hz
        return_lp += .692136f*((delayed-return_dc)-return_lp); // ~9 kHz
        delayed=return_lp;

        float m =
            Clamp01Added(
                macro_smoothed
            );


        if(m != cached_m)
        {
            cached_m = m;
            feedback = 0.18f + 0.60f * powf(m, 1.25f);
        }


        float user_wet =
            0.10f +
            0.66f *
            SmoothstepAdded(
                m
            );


        // Fade the send in from silence so enabling on a nonzero input
        // does not record a sharp artificial edge one delay-time later.
        send_gain += .004158f*((requested?1.f:0.f)-send_gain);
        send_dc += .019443f*(input-send_dc);
        send_lp += .692136f*((input-send_dc)-send_lp);
        buffer[write] = ClampAdded(send_lp*send_gain + delayed*feedback, -8.f, 8.f);

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


/*
 * Search backwards around the nominal capture position for a seam whose
 * beginning/end values and slopes agree better.
 *
 * This does NOT alter the requested clock length; it shifts the entire
 * capture window in time by at most a few milliseconds.
 */
static constexpr uint32_t LOOPER_SEAM_SEARCH_SAMPLES = 160;


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
 * (keyed on a per-hit age counter reset on every trigger) and crossfade
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
   MACRO 4 — THREE INDEPENDENT MACKIE-MID BPF SENDS
   ============================================================

   The selected bands are true parallel band-pass channels. Each is tuned
   independently, runs at full send, and all selected outputs are summed;
   there is no serial filtering, averaging, or layer-count attenuation.
   A Mackie 8-Bus sweep offers up to +15 dB of mid boost. That maximum
   channel gain is applied to each band before the Mackie character stage.
   ============================================================ */
struct MacroBpfBank
{
    Biquad drive_filters[3];
    float current_hz[3] = {330.f,700.f,1450.f};
    float configured_hz[3] = {};
    float current_gain_db[3] = {}, configured_gain_db[3] = {};
    unsigned update_samples = 0;
    uint32_t coefficient_updates = 0;
    static constexpr float MACKIE_MAX_MID_GAIN_DB = 15.f;
    void Reset()
    {
        for(int i=0;i<3;++i)
        {
            drive_filters[i].Reset();
            current_hz[i]=macro_bpf_target_hz[i];
            current_gain_db[i]=configured_gain_db[i]=0.f;
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
            float gain_target=i<macro_bpf_layer_count_latched
                ? MACKIE_MAX_MID_GAIN_DB*Clamp01Added(param_bpf_gain) : 0.f;
            current_gain_db[i]+=(gain_target-current_gain_db[i])*.04877058f; // 20 ms
            if(fabsf(gain_target-current_gain_db[i])<.001f)current_gain_db[i]=gain_target;
            if(fabsf(current_hz[i]-configured_hz[i])<current_hz[i]*.0001f &&
               fabsf(current_gain_db[i]-configured_gain_db[i])<.001f)continue;
            // Broad two-octave peaking EQ, like a mixer sweep mid, not a
            // narrow resonator. Cascaded bell boosts avoid parallel BPF phase
            // cancellation while preserving the old three-frequency UI.
            drive_filters[i].SetPeak(current_hz[i],current_gain_db[i]);
            configured_hz[i]=current_hz[i];
            configured_gain_db[i]=current_gain_db[i];
            ++coefficient_updates;
        }
    }
    float ProcessDriveFeed(float input)
    {
        float shaped=input;
        for(int i=0;i<3;++i)
        {
            // Run every section continuously; its gain slews to/from unity.
            shaped=drive_filters[i].Process(shaped);
        }
        return isfinite(shaped) ? shaped : input;
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
            y+=MACKIE_FIR[k]*input_history[input_pos+j];
        return 4.f*y; // interpolation gain once; power-of-two scaling is exact

    }
    float Downsample(float x, bool emit)
    {
        if(--output_pos<0)output_pos=96;
        output_history[output_pos]=output_history[output_pos+97]=x;
        if(!emit)return 0.f;
        // Symmetric taps: 49 multiplies instead of 97.
        const float* h=output_history+output_pos;
        float y=MACKIE_FIR[48]*h[48];
        for(int k=0;k<48;++k)y+=MACKIE_FIR[k]*(h[k]+h[96-k]);
        return y;
    }
};
struct CharacterAlignment
{
    float history[48] = {}; int pos=0;
    void Reset(){ *this=CharacterAlignment(); }
    float Process(float x){float y=history[pos];history[pos]=x;if(++pos==48)pos=0;return y;}
};
static CharacterAlignment clean_alignment;

// Mackie channel/preamp character, based on the earlier project version that
// sounded closer to the external desk: input rolloff, asymmetrical overload,
// broad low-mid/presence coloration, a second driven stage and output rolloff.
// Both nonlinear stages are oversampled and FIR-decimated to keep aliasing low.
struct MacroMackieProcessor
{
    MackieOversampling os,os2;
    Biquad body_band,presence_band;
    float pre_lp_1=0.f,pre_lp_2=0.f,dc_x1=0.f,dc_y1=0.f,post_lp_1=0.f,post_lp_2=0.f;
    bool idle=true;
    static float Core(float x)
    {
        float shaped=x+.020f*x*fabsf(x);
        const float rail=shaped>=0.f?1.f:.955f;
        float u=fabsf(shaped)/rail;
        return (shaped>=0.f?1.f:-1.f)*rail*(u/sqrtf(1.f+u*u));
    }
    void Reset()
    {
        os.Reset();
        os2.Reset();
        body_band.Reset();presence_band.Reset();
        body_band.SetBandpass(340.f,.78f);
        presence_band.SetBandpass(1050.f,.92f);
        pre_lp_1=pre_lp_2=dc_x1=dc_y1=post_lp_1=post_lp_2=0.f;
        idle=true;
    }
    // Prime the input interpolator while bypassed. Downstream state is cleared
    // only after the wet blend has faded away, so the next engage is silent and
    // costs almost nothing while Mackie is not selected.
    void Idle(float input)
    {
        constexpr float pre_a=.79210000f;
        pre_lp_1+=pre_a*(input-pre_lp_1);
        pre_lp_2+=pre_a*(pre_lp_1-pre_lp_2);
        os.Push(pre_lp_2);
        if(!idle){
            os2.Reset();body_band.Reset();presence_band.Reset();
            dc_x1=dc_y1=post_lp_1=post_lp_2=0.f;
            body_band.SetBandpass(340.f,.78f);
            presence_band.SetBandpass(1050.f,.92f);
            idle=true;
        }
    }
    float Process(float input,float amount)
    {
        idle=false;
        constexpr float pre_a=.79210000f,post_a=.72990000f;
        pre_lp_1+=pre_a*(input-pre_lp_1);
        pre_lp_2+=pre_a*(pre_lp_1-pre_lp_2);
        os.Push(pre_lp_2);
        float first=0.f;
        const float drive=1.f+(MACKIE_INTERNAL_GAIN-1.f)*Clamp01Added(amount);
        for(int phase=0;phase<4;++phase)
        {
            float x=os.Upsample(phase);
            x=Core(x*drive);
            float y=os.Downsample(x,phase==0);
            if(phase==0)first=y;
        }
        constexpr float dc_r=.99935f;
        float dc=first-dc_x1+dc_r*dc_y1;
        dc_x1=first;dc_y1=dc;
        float body=body_band.Process(dc);
        float presence=presence_band.Process(dc);
        float eq=dc+body*.62f+presence*.25f;
        os2.Push(eq);
        float second=0.f;
        for(int phase=0;phase<4;++phase)
        {
            float y=os2.Downsample(Core(os2.Upsample(phase)*1.65f),phase==0);
            if(phase==0)second=y;
        }
        post_lp_1+=post_a*(second-post_lp_1);
        post_lp_2+=post_a*(post_lp_1-post_lp_2);
        float output=post_lp_2*param_mackie_gain;
        return isfinite(output)?output:0.f;
    }
};

// Step 6/8's previous/cue voice has its own Mackie state. It is mixed after
// the main character stage so it cannot drive or alter the current kick's
// distortion path.
static MacroMackieProcessor kick_overlay_mackie;

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
// The output trim cannot undo distortion made before it, so keep the Tube
// stages below the harsh digital-sounding overload region.
static constexpr float TUBE_INPUT_GAIN = 1.45f;
static constexpr float TUBE_INTERSTAGE_GAIN = 1.25f;

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
    float previous_input = 0.0f;
    /* Blocking bias shift, per stage. */
    float block_1 = 0.0f;
    float block_2 = 0.0f;


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
        previous_input = 0.0f;
        block_1 = block_2 = 0.0f;
    }


    float Process(float input, float amount)
    {
        if(amount <= CHARACTER_HEAVY_PROCESS_EPSILON)
        {
            previous_input = input;
            block_1 = block_2 = 0.0f;
            return 0.0f;
        }

        float stage2_sum = 0.0f;
        const float drive_1=1.f+(TUBE_INPUT_GAIN-1.f)*Clamp01Added(amount);
        const float drive_2=1.f+(TUBE_INTERSTAGE_GAIN-1.f)*Clamp01Added(amount);

        for(int os = 1; os <= 4; ++os)
        {
            float t = static_cast<float>(os) * 0.25f;
            float x =
                (previous_input + (input - previous_input) * t) * drive_1;

            /* Stage 1, with its bias dragged down by grid current. */
            float bias_1 = TUBE_STAGE1_BIAS - block_1 * TUBE_BLOCKING_DEPTH;
            float s1 = Triode(x, bias_1);

            float grid_current_1 = fmaxf(0.0f, x + bias_1);
            block_1 +=
                (grid_current_1 > block_1 ? TUBE_BLOCKING_CHARGE_A
                                          : TUBE_BLOCKING_RECOVER_A) *
                (grid_current_1 - block_1);

            float x2 = s1 * drive_2;

            float bias_2 = TUBE_STAGE2_BIAS - block_2 * TUBE_BLOCKING_DEPTH;
            float s2 = Triode(x2, bias_2);

            float grid_current_2 = fmaxf(0.0f, x2 + bias_2);
            block_2 +=
                (grid_current_2 > block_2 ? TUBE_BLOCKING_CHARGE_A
                                          : TUBE_BLOCKING_RECOVER_A) *
                (grid_current_2 - block_2);

            stage2_sum += s2;
        }

        previous_input = input;
        return stage2_sum * 0.25f * TUBE_OUTPUT_GAIN;
    }
};


struct MacroCharacterProcessor
{
    static constexpr float MAX_PARALLEL_BLEND = 0.65f;
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
        // A Mackie channel is a continuous analogue path: preserve its filter,
        // bias and converter history across kick boundaries to avoid clicks.
        if(active_tube)
            PrepareTube();
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

    float ProcessSelectedWet(float input)
    {
        if(active_tube)
        {
            float target = Clamp01Added(macro_tube_amount);
            tube_amount_smoothed = SmoothAmount(tube_amount_smoothed, target);
            if(tube_amount_smoothed <= CHARACTER_HEAVY_PROCESS_EPSILON)
            {
                tube.Process(input, 0.0f);
                return input;
            }
            float wet = tube_alignment.Process(
                tube.Process(input, tube_amount_smoothed));
            if(!AudioValueSafe(wet))
            {
                PrepareTube();
                return input;
            }
            return wet;
        }

        float target = Clamp01Added(macro_mackie_amount);
        mackie_amount_smoothed = SmoothAmount(mackie_amount_smoothed, target);
        if(mackie_amount_smoothed <= CHARACTER_HEAVY_PROCESS_EPSILON)
        {
            mackie_amount_smoothed = 0.0f;
            mackie.Idle(input);
            return input;
        }
        float wet = mackie.Process(input, mackie_amount_smoothed);
        if(!AudioValueSafe(wet))
        {
            PrepareMackie();
            return input;
        }
        return wet;
    }

    float ProcessWet(float dry_input, float character_input)
    {
        ConsumeSwitchRequest();
        float colored = ProcessSelectedWet(character_input);
        float amount = active_tube ? tube_amount_smoothed : mackie_amount_smoothed;
        /* Keep at least 35% phase-aligned dry body at the extreme setting;
         * this preserves the kick's fundamental and leaves output headroom. */
        float blend = amount * MAX_PARALLEL_BLEND;
        float selected = dry_input + (colored - dry_input) * blend;
        constexpr float transition_step = 1.0f / (SAMPLE_RATE * 0.010f);

        if(switch_state == SwitchState::FADE_TO_ZERO)
        {
            transition_gain -= transition_step;
            if(transition_gain <= 0.0f)
            {
                transition_gain = 0.0f;
                active_tube = desired_tube;

                if(active_tube)
                    PrepareTube();
                else
                    PrepareMackie();

                switch_state = SwitchState::FADE_FROM_ZERO;
                return dry_input;
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

        float output = dry_input + (selected - dry_input) * transition_gain;
        return AudioValueSafe(output) ? output : dry_input;
    }

    float ProcessWet(float input) { return ProcessWet(input, input); }
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
        // Broad usable range: ~9.4 bits at 30%, 4 bits at full, with
        // a retained dry component so quiet tails never gate completely.
        float bits=16.f-12.f*sqrtf(amount);
        float wet=.8f*amount*amount*(3.f-2.f*amount);
        int lower=static_cast<int>(bits);
        float fraction=bits-lower;
        float levels=static_cast<float>(1u << (lower-1));
        // Powers of two: the reciprocal multiply is exact, no divides.
        float step=static_cast<float>(1.0 / (1u << (lower-1)));
        float coarse=roundf(x*levels)*step;
        float fine=roundf(x*(2.f*levels))*(.5f*step);
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
    float* loop_history = loop_history_buffer;
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

    active_kick_note=note;
    if(kick_note_holds[note]<255)++kick_note_holds[note];
    note_gate=true;

    if(kick_preset_next_valid)
    {
        for(uint8_t i=0;i<5;++i)kick_pending_preset[i]=kick_preset_next[i];
        kick_pending_preset_valid=true;
    }
    else
    {
        // Backward compatibility with older controllers that use note pitch.
        if(KICK_USE_MIDI_NOTE_FOR_TUNING)
            kick_frequency=ClampAdded(MidiNoteToFrequency(note),
                                      KICK_CHROMATIC_MIN_HZ,
                                      KICK_CHROMATIC_MAX_HZ);
        else
            kick_frequency=KICK_FIXED_FREQUENCY_HZ;
        kick_pending_preset_valid=false;
    }

    kick_trigger_pending = true;
}


/* ============================================================
   MIDI NOTE OFF
   ============================================================ */

static void HandleKickNoteOff(
    uint8_t note)
{
    if(kick_note_holds[note])--kick_note_holds[note];
    note_gate=false;
    for(uint8_t n=0;n<128;++n)if(kick_note_holds[n]){note_gate=true;break;}
    if(note==active_kick_note && !kick_note_holds[note])active_kick_note=255;
}


/* ============================================================
   ADDED PERFORMANCE CC HANDLER
   ============================================================ */

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


    macro_fx_value_lpf =
        v;
}


static void SetMacroPump(float v)
{
    v =
        Clamp01Added(
            v
        );


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
    }
}


static bool HandleSixMacroCC(
    uint8_t cc,
    uint8_t value)
{
    if(held_fx_control.Handle(cc,value,System::GetNow()))return true;
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

        case CC_KICK_TEMP_MUTE:
            kick_temp_mute_target=value>=64;
            return true;

        case CC_KICK_TEMP_LEVEL:
            kick_temp_level_target=value>=64;
            return true;

        case CC_KICK_TEMP_DECAY:
            kick_temp_decay_active=value>=64;
            return true;

        case CC_KICK_DUB_ECHO:
            kick_dub_echo_target=value>=64;
            return true;

        case CC_KICK_GROUND_MACHINE:
            kick_ground_machine_target=value>=64;
            return true;

        case CC_KICK_PERFORMANCE_HPF:
            kick_performance_hpf.Set(value);
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

        case CC_KICK_COMP_AMOUNT:
            kick_comp_amount = v;
            return true;

        case CC_KICK_COMP_ENABLE:
            kick_comp_enabled = value >= 64;
            return true;

        case CC_KICK_TAIL_START:
            macro_tail_start = v;
            return true;

        case CC_KICK_CUTOFF_TIME:
            kick_cutoff_time = v;
            return true;

        case CC_KICK_CUTOFF_ENABLE:
            kick_cutoff_enabled = value >= 64;
            return true;


        /* ====================================================
           DECAY / REVERSE — ABSOLUTE STATE
           ==================================================== */

        case CC_DECAY_ABSOLUTE:
        {
            macro_decay_target = v;
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
           AMP HOLD / FIXED TAIL GAP — ABSOLUTE AMOUNT + STATE
           ==================================================== */

        case CC_TAIL_DELAY_ABSOLUTE:
        {
            macro_amp_hold = v;
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

        // Next-hit kick packet. Values are staged individually, then become
        // visible together only when COMMIT arrives. Note-On consumes that
        // immutable snapshot in the audio callback.
        case softsynth::KICK_NEXT_PITCH: kick_preset_stage[0]=value; return true;
        case softsynth::KICK_NEXT_DECAY: kick_preset_stage[1]=value; return true;
        case softsynth::KICK_NEXT_SHAPE: kick_preset_stage[2]=value; return true;
        case softsynth::KICK_NEXT_SWEEP: kick_preset_stage[3]=value; return true;
        case softsynth::KICK_NEXT_TAIL: kick_preset_stage[4]=value; return true;
        case softsynth::KICK_NEXT_COMMIT:
            for(uint8_t i=0;i<5;++i)kick_preset_next[i]=kick_preset_stage[i];
            kick_preset_next_valid=true;
            return true;

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
        case CC_HARMONICS:
        {
            macro_harmonics = v;
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

        case 95: case 96: case 97: case 98: case 99:
            kick_overlay_mode=0; return true; // step 6/8 overlay route disabled


        /* ====================================================
           MIX PAGE — FUNCTION + MENU2 on the Teensy
           ==================================================== */

        case CC_MIX_LINE_GAIN:
            param_line_gain_target = OutputGainFromCC(value);
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

        case CC_MIX_EXT_KICK_MODE:
            external_kick_mode_target = value >= 64;
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
           LEGACY ADDRESSES
           ==================================================== */

        /* Old banked K1 value: consumed and ignored. */
        case CC_MACRO_FX_VALUE_LEGACY:
            return true;

        /* K4 edits the newest/current BPF layer centre. */
        case CC_MACRO_BPF_FREQUENCY_LEGACY:
        {
            uint8_t index = macro_bpf_layer_count == 0
                ? 0 : static_cast<uint8_t>(macro_bpf_layer_count - 1);
            if(index > 2) index = 2;
            macro_bpf_target_hz[index] = MacroBpfFrequencyHz(v);
            return true;
        }

        /* K4 button: cycle 0 / 1 / 2 / 3 active BPF layers. */
        case CC_BUTTON_BPF_LAYERS_LEGACY:
        {
            bool down = value >= 64;
            if(down && !legacy_bpf_button_down)
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
            if(macro_character_tube)
                macro_tube_amount = v;
            else
                macro_mackie_amount = v;
            macro_character_wet = v;
            return true;
        }

        /* K5 button: Mackie <-> Tube, press edge only. */
        case CC_BUTTON_CHARACTER_LEGACY:
        {
            bool down = value >= 64;
            if(down && !character_button_down)
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
            return true;
        }


        default:
            return false;
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
        note_gate = false;
        active_kick_note = 255;
        memset(kick_note_holds,0,sizeof(kick_note_holds));

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
            if(cc==120 || cc==123){
                note_gate=false;
                active_kick_note=255;
                memset(kick_note_holds,0,sizeof(kick_note_holds));
                return;
            }
            HandleSixMacroCC(
                cc,
                value
            );
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
    unsigned budget=32; // bounded foreground work; audio IRQ owns voice state
    while(!kick_trigger_pending && budget-- && midi_rx.Pop(received))
    {
        if(received & MidiRxQueue::GAP)
        {
            midi_running_status=0; midi_data_count=0;
            kick_tail_pitch_pending=false;
            note_gate=false;active_kick_note=255;
            memset(kick_note_holds,0,sizeof(kick_note_holds));
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

    /* Raw 909 path: source -> optional three-band BPF / character model ->
     * clocked reverse -> tail gate -> routed internal reverb -> AHD output
     * envelope. Standalone
     * HPF/LPF, crossover, protected bands and master FX stay out of this path. */
    static bool raw_was_external = false;
    static uint32_t raw_kick_age_samples = UINT32_MAX;
    static uint32_t amp_hold_samples_for_hit = 0;
    static float amp_env_gain = 0.0f;
    static float amp_env_trigger_start_gain = 0.0f;
    static float tail_start_smooth = 64.0f / 127.0f;
    static float tail_enable_smooth = 0.0f;
    static float tail_pump_smooth = 1.0f;
    static float compressor_amount_smooth = 0.0f;
    static float compressor_mix = 0.0f;
    static float compressor_input_level = 0.0f;
    static float compressor_output_level = 0.0f;
    static float compressor_makeup = 1.0f;
    static float raw_bpf_gain_smooth = 0.70f;
    // 0 idle, 1 crackle, 2 dead-air pause, 3 held scream, 4 release tail.
    static uint8_t ground_machine_stage = 0;
    static uint32_t ground_machine_stage_samples = 0;
    static float ground_machine_sine = 0.0f;
    static float ground_machine_cosine = 1.0f;
    static float ground_machine_osc_phase = 0.0f;
    static float ground_machine_scream_gain = 0.0f;
    static float ground_machine_kick_gain = 1.0f;
    static float ground_machine_kick_detector = 0.0f;
    static float ground_machine_comp_level = 0.0f;
    static bool ground_machine_kick_started = false;
    static bool ground_machine_scream_started = false;
    static uint32_t ground_machine_rng = 0x9173a5c1u;
    static float ground_feedback[32] = {};
    static uint8_t ground_feedback_pos = 0;
    if(external_kick_mode_target && !raw_was_external)
    {
        kick_voice.Reset();
        kick_voice_previous.Reset();
        kick_voice_handoff_age = RAW_909_RETRIGGER_SAMPLES;
        kick_overlay_voice.Reset();kick_overlay_previous.Reset();
        kick_overlay_handoff_age=kick_overlay_attack_age=RAW_909_RETRIGGER_SAMPLES;
        kick_overlay_mackie.Reset();
        kick_overlay_mackie_flush_samples=0;
        kick_overlay_hpf_x1=kick_overlay_hpf_y1=0.f;
    }
    raw_was_external = external_kick_mode_target;

    const bool quarter_edge = perf_quarter_pending;
    held_fx_engine.BeginBlock(held_fx_control, System::GetNow(), midi_running,
        kick_trigger_pending, quarter_edge, perf_quarter_note_ms,
        static_cast<unsigned>(size));
    if(held_fx_engine.ratchetTrigger || held_fx_engine.retriggerTrigger)
        kick_trigger_pending = true;

    // Step 9 re-rolls the BPF on each clocked retrigger; it does not capture audio.
    static bool raw_kick_bpf_override = false;
    static uint8_t raw_kick_bpf_saved_count = 0;
    static float raw_kick_bpf_saved_hz[3] = {};
    const bool raw_held_kick_bpf = held_fx_engine.KickBpfForKick(kick_trigger_pending);
    if(held_fx_engine.KickBpfEngaged() && !raw_kick_bpf_override)
    {
        raw_kick_bpf_saved_count = macro_bpf_layer_count;
        for(int band = 0; band < 3; ++band)
            raw_kick_bpf_saved_hz[band] = macro_bpf_target_hz[band];
        raw_kick_bpf_override = true;
    }
    else if(!held_fx_engine.KickBpfEngaged() && raw_kick_bpf_override)
    {
        macro_bpf_layer_count = raw_kick_bpf_saved_count;
        for(int band = 0; band < 3; ++band)
            macro_bpf_target_hz[band] = raw_kick_bpf_saved_hz[band];
        raw_kick_bpf_override = false;
    }
    if(raw_held_kick_bpf)
    {
        macro_bpf_layer_count = 3;
        for(int band = 0; band < 3; ++band)
            macro_bpf_target_hz[band] = held_fx_engine.kickBpfHz[band];
    }
    if(external_kick_mode_target && !held_fx_engine.KickBpfEngaged())
        macro_bpf_layer_count_latched = macro_bpf_layer_count;

    if(quarter_edge)
    {
        added_performance_fx.OnQuarter();
        perf_quarter_pending = false;
        perf_sixteenth_pending = false;
        if(tail_delay_pending_state >= 0)
        {
            tail_delay_enabled = tail_delay_pending_state > 0;
            tail_delay_pending_state = -1;
        }
    }

    if(kick_trigger_pending)
    {
        kick_trigger_pending = false;
        if(kick_ground_machine_target && ground_machine_stage!=1 &&
           ground_machine_stage!=2 && ground_machine_stage!=3)
        {
            ground_machine_stage = 1;
            ground_machine_stage_samples = 0;
            ground_machine_scream_gain = 0.0f;
            ground_machine_kick_gain = 0.0f;
            ground_machine_kick_started = false;
            ground_machine_scream_started = false;
            kick_voice.active=false;kick_voice_previous.active=false;
        }
        else if(!kick_ground_machine_target && ground_machine_stage!=0 &&
                ground_machine_stage!=4)
        {
            ground_machine_stage = 4;
            ground_machine_stage_samples = 0;
        }
        if(ground_machine_stage==3 && kick_ground_machine_target &&
           ground_machine_stage_samples>=uint32_t(4.0f*SAMPLE_RATE))
            ground_machine_kick_started = true;
        kick_ground_decay_override = ground_machine_stage==3 && ground_machine_kick_started;
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
        if(kick_pending_preset_valid)
        {
            const uint8_t pitch=kick_pending_preset[0];
            kick_frequency=ClampAdded(MidiNoteToFrequency(pitch),
                                      KICK_CHROMATIC_MIN_HZ,
                                      KICK_CHROMATIC_MAX_HZ);
            macro_decay_target=float(kick_pending_preset[1])/127.f;
            macro_kick_shape=float(kick_pending_preset[2])/127.f;
            kick_sweep_time=float(kick_pending_preset[3])/127.f;
            kick_tail_pitch_cc=kick_pending_preset[4];
            kick_tail_pitch_pending=true;
            kick_pending_preset_valid=false;
        }
        // Pad 1 suppresses each new generated hit at its trigger boundary.
        // Keep the existing voice/tails running so muting a hit never chops
        // the decay that was already in progress.
        const bool suppress_new_hit = (kick_temp_mute_target ||
                                       ground_machine_stage==1 ||
                                       ground_machine_stage==2 ||
                                       (ground_machine_stage==3 && !ground_machine_kick_started)) &&
                                      !external_kick_mode_target;
        if(!external_kick_mode_target && !suppress_new_hit)
            TriggerKickVoice(last_velocity);
        macro_bpf_layer_count_latched = raw_held_kick_bpf ? 3 : macro_bpf_layer_count;
        if(!suppress_new_hit){
            added_performance_fx.TriggerKick();
            macro_whole_kick_reverse.Trigger();
            kick_reverb.Trigger();
            amp_env_trigger_start_gain = amp_env_gain;
            raw_kick_age_samples = 0;
            const float sweep_ms = KickSweepTimeMs(kick_sweep_time);
            const float whole_note_ms = perf_quarter_note_ms*4.0f;
            const uint8_t hold_cc = static_cast<uint8_t>(
                Clamp01Added(macro_amp_hold)*127.f + 0.5f);
            const float hold_ms = kickdaisy::ampHoldMs(hold_cc,sweep_ms,whole_note_ms);
            amp_hold_samples_for_hit = static_cast<uint32_t>(hold_ms*0.001f*SAMPLE_RATE);
        }
    }

    raw_bpf_gain_smooth +=
        (param_bpf_gain_target - raw_bpf_gain_smooth) * PARAM_GAIN_SLEW;
    param_bpf_gain = raw_bpf_gain_smooth;
    param_line_gain +=
        (param_line_gain_target - param_line_gain) * PARAM_GAIN_SLEW;
    macro_bpf_bank.Update(static_cast<unsigned>(size));

    for(size_t i = 0; i < size; ++i)
    {
        external_kick_mode_mix +=
            ((external_kick_mode_target ? 1.0f : 0.0f) - external_kick_mode_mix) *
            RAW_909_SOURCE_SMOOTH_A;

        float external = in[EXTERNAL_INPUT_CHANNEL_1][i] * EXTERNAL_INPUT_1_GAIN;
        if(!std::isfinite(external)) external = 0.0f;
        const float external_pumped = added_performance_fx.pump.Process(
            external, perf_quarter_note_ms);
        KickVoiceOut voice, previous_voice;
        kick_voice_previous.Process(previous_voice);
        kick_voice.Process(voice);
        if(kick_voice_handoff_age < RAW_909_RETRIGGER_SAMPLES)
        {
            const float t = float(kick_voice_handoff_age) /
                            float(RAW_909_RETRIGGER_SAMPLES);
            const float fade = SmoothstepAdded(t);
            voice.punch = previous_voice.punch * (1.0f - fade) +
                          voice.punch * fade;
            if(++kick_voice_handoff_age >= RAW_909_RETRIGGER_SAMPLES)
                kick_voice_previous.active = false;
        }
        KickVoiceOut overlay, overlay_previous;
        bool overlay_layer_active =
            kick_overlay_voice.active || kick_overlay_previous.active;
        kick_overlay_previous.Process(overlay_previous);
        kick_overlay_voice.Process(overlay);
        float overlaySample=overlay.punch;
        if(kick_overlay_handoff_age < RAW_909_RETRIGGER_SAMPLES){
            float t=float(kick_overlay_handoff_age)/float(RAW_909_RETRIGGER_SAMPLES);
            float fade=SmoothstepAdded(t);
            overlaySample=overlay_previous.punch*(1.f-fade)+overlaySample*fade;
            if(++kick_overlay_handoff_age>=RAW_909_RETRIGGER_SAMPLES)
                kick_overlay_previous.active=false;
        }
        if(kick_overlay_attack_age < 96u){
            overlaySample*=SmoothstepAdded(float(kick_overlay_attack_age)/96.f);
            ++kick_overlay_attack_age;
        }
        float raw = voice.punch +
                    (external_pumped - voice.punch) * external_kick_mode_mix;
        if(!std::isfinite(raw)) raw = 0.0f;

        /* BPF changes only the character send; the fundamental stays dry. */
        float dry = clean_alignment.Process(raw);
        float character_send = macro_bpf_bank.ProcessDriveFeed(raw);
        float output_sample = macro_character_processor.ProcessWet(dry, character_send);

        output_sample = held_fx_engine.ProcessKick(output_sample);
        output_sample = macro_whole_kick_reverse.Process(output_sample);

        // Tempo-locked hard gate. Smoothstep edges reach exact silence without
        // discontinuities; the short bounded slew also protects retriggers or
        // a state change that lands while the gate is closed. Reverb follows
        // the gate; the final AHD envelope then cuts the complete kick bus.
        tail_start_smooth += (Clamp01Added(macro_tail_start) - tail_start_smooth) * 0.001f;
        tail_enable_smooth = tail_delay_enabled ? 1.0f : 0.0f;
        float gate_target = 1.0f;
        if(raw_kick_age_samples != UINT32_MAX && tail_enable_smooth > 0.5f)
        {
            const float age = static_cast<float>(raw_kick_age_samples) / SAMPLE_RATE;
            // Map the full control to the audible 0..200 ms range. Preserve
            // the established centre setting (CC64 = 130 ms), with finer
            // resolution throughout the useful range.
            const float tail_start_value = tail_start_smooth * 127.0f;
            const float start_ms = tail_start_value <= 64.0f
                ? tail_start_value * (130.0f / 64.0f)
                : 130.0f + (tail_start_value - 64.0f) * (70.0f / 63.0f);
            const float start_seconds = start_ms * 0.001f;
            const float elapsed = age - start_seconds;
            const float edge = kick_cutoff_enabled
                ? (0.002f + 0.058f * Clamp01Added(kick_cutoff_time))
                : 0.003f;
            const float reopen_edge = 0.003f;
            constexpr float gap = 0.032f;
            if(elapsed >= 0.0f && elapsed < edge)
                gate_target = 1.0f - SmoothstepAdded(elapsed / edge);
            else if(elapsed >= edge && elapsed < edge + gap)
                gate_target = 0.0f;
            else if(elapsed >= edge + gap && elapsed < edge + gap + reopen_edge)
                gate_target = SmoothstepAdded((elapsed - edge - gap) / reopen_edge);
        }
        // Bound gain movement to a 3 ms edge if a hit restarts the gate phase
        // or the quantized enable state changes while it is closed.
        const float gate_step = 1.0f / (0.003f * SAMPLE_RATE);
        const float gate_delta = gate_target - tail_pump_smooth;
        tail_pump_smooth += ClampAdded(gate_delta, -gate_step, gate_step);
        const float pump=tail_pump_smooth;
        output_sample *= pump;
        // Apply the AHD to the dry kick/effect send. Reverb and delay receive
        // the shaped signal, so their tails are not abruptly cut by the amp
        // envelope itself.
        float amp_env_target = 0.0f;
        if(raw_kick_age_samples != UINT32_MAX)
        {
            constexpr uint32_t attack_samples = uint32_t(0.005f*SAMPLE_RATE);
            constexpr uint32_t release_samples = uint32_t(0.020f*SAMPLE_RATE);
            const uint32_t age = raw_kick_age_samples;
            if(age < attack_samples)
                amp_env_target = amp_env_trigger_start_gain +
                    (1.0f-amp_env_trigger_start_gain) *
                    SmoothstepAdded(float(age)/attack_samples);
            else if(age < attack_samples+amp_hold_samples_for_hit)
                amp_env_target = 1.0f;
            else if(age < attack_samples+amp_hold_samples_for_hit+release_samples)
                amp_env_target = 1.0f-SmoothstepAdded(
                    float(age-attack_samples-amp_hold_samples_for_hit)/release_samples);
        }
        const float amp_env_step=1.0f/(0.001f*SAMPLE_RATE);
        amp_env_gain += ClampAdded(amp_env_target-amp_env_gain,
                                   -amp_env_step,amp_env_step);
        output_sample *= amp_env_gain;
        float kick_reverb_amount=(fx_internal_routes & 2) ? param_reverb_amount : 0.0f;
        const bool dub_echo_active=kick_dub_echo_target && !external_kick_mode_target;
        if(dub_echo_active)kick_reverb_amount=fmaxf(kick_reverb_amount,.90f);
        output_sample = kick_reverb.Process(output_sample,kick_reverb_amount,dub_echo_active,
                                            !external_kick_mode_target);
        output_sample = kick_performance_hpf.Process(output_sample);
        float ground = 0.0f;
        const bool ground_transition = ground_machine_stage==1 || ground_machine_stage==2;
        if(ground_machine_stage!=0)
        {
            ++ground_machine_stage_samples;
            const float old_sine=ground_machine_sine;
            ground_machine_sine=old_sine*0.9999786f+ground_machine_cosine*0.0065449f;
            ground_machine_cosine=ground_machine_cosine*0.9999786f-old_sine*0.0065449f;
            ground_machine_osc_phase+=50.0f/SAMPLE_RATE;
            if(ground_machine_osc_phase>=1.0f)ground_machine_osc_phase-=1.0f;
            if(ground_machine_stage==1)
            {
                // Brief sharp static crackle, isolated from the old kick tail.
                ground_machine_rng ^= ground_machine_rng << 13;
                ground_machine_rng ^= ground_machine_rng >> 17;
                ground_machine_rng ^= ground_machine_rng << 5;
                const float noise=float(int32_t(ground_machine_rng))/2147483648.0f;
                ground=ClampAdded(5.0f*(noise*.22f+ground_machine_sine*.42f),-1.0f,1.0f);
                if(ground_machine_stage_samples>=uint32_t(SAMPLE_RATE*.012f))
                { ground_machine_stage=2;ground_machine_stage_samples=0; }
            }
            else if(ground_machine_stage==2)
            {
                // A full half-second break before the electrical scream.
                ground=0.0f;
                if(ground_machine_stage_samples>=uint32_t(SAMPLE_RATE*.5f))
                {
                    ground_machine_stage=3;ground_machine_stage_samples=0;
                    ground_machine_kick_gain=0.0f;ground_machine_kick_started=false;
                    ground_machine_scream_started=true;
                    ground_machine_comp_level=0.0f;
                    kick_ground_decay_override=true;
                    for(uint8_t j=0;j<32;++j)ground_feedback[j]=0.0f;
                    ground_feedback_pos=0;
                }
            }
            else if(ground_machine_stage==3 || ground_machine_stage==4)
            {
                const uint8_t read=uint8_t((ground_feedback_pos+1u)&31u);
                const float feedback=ground_feedback[read];
                const float decay=ground_machine_stage==4
                    ? SmoothstepAdded(1.0f-fminf(1.0f,float(ground_machine_stage_samples)/(0.8f*SAMPLE_RATE)))
                    : 1.0f;
                // A 50 Hz saw contains the 50/100/150/200 Hz mains harmonics
                // through the bright upper range. Resonant feedback and hard
                // clipping turn it into a loud, metallic electrical roar.
                const float saw=2.0f*ground_machine_osc_phase-1.0f;
                ground_machine_rng ^= ground_machine_rng << 13;
                ground_machine_rng ^= ground_machine_rng >> 17;
                ground_machine_rng ^= ground_machine_rng << 5;
                const float fuzz=float(int32_t(ground_machine_rng))/2147483648.0f*.025f;
                const float mains=saw*.72f+ground_machine_sine*.22f+fuzz;
                const float detector=fabsf(mains);
                ground_machine_comp_level+=(detector-ground_machine_comp_level)*
                    (detector>ground_machine_comp_level?.025f:.00025f);
                // Near-infinite compression with strong makeup, followed by
                // a hard drive ceiling: the scream stays full and aggressive.
                const float comp_gain=ClampAdded(.96f/(ground_machine_comp_level+.001f),.25f,8.0f);
                const float feedback_gain=1.08f-.30f*(1.0f-decay);
                const float drive=(mains+feedback*feedback_gain)*comp_gain*8.0f;
                ground=ClampAdded(drive,-1.0f,1.0f);
                ground_feedback[ground_feedback_pos]=ground;
                ground_feedback_pos=uint8_t((ground_feedback_pos+1u)&31u);
                ground_machine_scream_gain=ground_machine_scream_started?decay:0.0f;
                if(ground_machine_stage==4 && ground_machine_stage_samples>=uint32_t(SAMPLE_RATE*.8f))
                {
                    ground_machine_stage=0;ground_machine_stage_samples=0;
                    ground_machine_scream_gain=0.0f;ground_machine_kick_gain=1.0f;
                    ground_machine_scream_started=false;
                    kick_ground_decay_override=false;
                }
            }
        }
        if(ground_transition) output_sample=0.0f;
        if(ground_machine_stage==1 || ground_machine_stage==2)
            ground_machine_kick_gain=0.0f;
        else if(ground_machine_stage==3)
        {
            if(ground_machine_kick_started)
                ground_machine_kick_gain=fminf(1.0f,ground_machine_kick_gain+1.0f/(4.0f*SAMPLE_RATE));
            else ground_machine_kick_gain=0.0f;
        }
        else if(ground_machine_stage==4)
            ground_machine_kick_gain=fminf(1.0f,ground_machine_kick_gain+1.0f/(2.0f*SAMPLE_RATE));
        else ground_machine_kick_gain=1.0f;
        const float kick_level=fabsf(output_sample);
        ground_machine_kick_detector+=(kick_level-ground_machine_kick_detector)*
            (kick_level>ground_machine_kick_detector?.012f:.00022f);
        const float scream_duck=ground_machine_stage>=3
            ?1.0f-.85f*Clamp01Added(ground_machine_kick_detector*2.5f):1.0f;
        // Send the saturated SFX almost at full scale; only the kick sidechain
        // and release envelope pull it down. The final output ceiling protects
        // the converter while preserving the dense, overdriven level.
        const float ground_scream_output=ground*ground_machine_scream_gain*scream_duck*.96f;
        const float level_target=(kick_temp_level_target && !external_kick_mode_target)?.12f:1.f;
        const float level_step=(level_target<kick_temp_level_gain)
            ?(.88f/(2.0f*SAMPLE_RATE))   // two-second fade down
            :(.88f/(.5f*SAMPLE_RATE));   // half-second recovery
        if(kick_temp_level_gain<level_target)
            kick_temp_level_gain=fminf(level_target,kick_temp_level_gain+level_step);
        else if(kick_temp_level_gain>level_target)
            kick_temp_level_gain=fmaxf(level_target,kick_temp_level_gain-level_step);
        output_sample*=kick_temp_level_gain;
        // Keep the added voice out of the current kick's distortion and shared
        // effect path. Process it through its independent Mackie/boost/HPF
        // chain, then sum near the output before the common safety ceiling.
        float overlay_high = 0.0f;
        if(overlay_layer_active)
            kick_overlay_mackie_flush_samples = KICK_OVERLAY_MACKIE_FLUSH_SAMPLES;
        else if(kick_overlay_mackie_flush_samples > 0)
            --kick_overlay_mackie_flush_samples;
        if(overlay_layer_active || kick_overlay_mackie_flush_samples > 0)
        {
            const float overlay_mackie =
                kick_overlay_mackie.Process(overlaySample, 1.0f);
            const float boosted_overlay = overlay_mackie * KICK_OVERLAY_EXTRA_GAIN;
            const float filtered_overlay = boosted_overlay - kick_overlay_hpf_x1 +
                                           KICK_OVERLAY_HPF_COEFF * kick_overlay_hpf_y1;
            kick_overlay_hpf_x1 = boosted_overlay;
            kick_overlay_hpf_y1 = filtered_overlay;
            const float flush_gain = overlay_layer_active
                ? 1.0f
                : SmoothstepAdded(float(kick_overlay_mackie_flush_samples) /
                                  KICK_OVERLAY_MACKIE_FLUSH_SAMPLES);
            overlay_high = filtered_overlay * flush_gain;
        }
        else
        {
            kick_overlay_mackie.Idle(0.f);
            kick_overlay_hpf_x1 = kick_overlay_hpf_y1 = 0.f;
        }
        output_sample += overlay_high;
        // Optional end-of-kick tanh compressor. Its tracked makeup ratio
        // keeps average level close to the clean, click-safe bypass.
        // Keep the detector/makeup estimate warm while bypassed, so toggling
        // K3 does not expose an uncompensated first hit.
        const float comp_amount_target = Clamp01Added(kick_comp_amount);
        compressor_amount_smooth +=
            (comp_amount_target - compressor_amount_smooth) * 0.001f;
        const float comp_target = kick_comp_enabled && comp_amount_target > 0.001f
            ? 1.0f : 0.0f;
        compressor_mix += (comp_target - compressor_mix) * 0.002f;
        if(compressor_mix > 0.0001f || comp_amount_target > 0.001f)
        {
            const float comp_drive = 1.0f + 8.0f * compressor_amount_smooth;
            const float compressed = tanhf(output_sample * comp_drive);
            const float abs_input = fabsf(output_sample);
            const float abs_output = fabsf(compressed);
            const float in_coeff = abs_input > compressor_input_level ? 0.0069f : 0.000208f;
            const float out_coeff = abs_output > compressor_output_level ? 0.0069f : 0.000208f;
            compressor_input_level += (abs_input - compressor_input_level) * in_coeff;
            compressor_output_level += (abs_output - compressor_output_level) * out_coeff;
            float makeup_target = 1.0f;
            if(compressor_amount_smooth > 0.001f && compressor_output_level > 0.0001f)
                makeup_target = ClampAdded(compressor_input_level / compressor_output_level, 0.25f, 4.0f);
            compressor_makeup += (makeup_target - compressor_makeup) * 0.005f;
            if(compressor_mix > 0.0f)
                output_sample += (compressed * compressor_makeup - output_sample) * compressor_mix;
        }
        else
            compressor_makeup += (1.0f - compressor_makeup) * 0.001f;
        output_sample*=ground_machine_kick_gain;
        output_sample+=ground_scream_output;
        // EXT IN remains a live passthrough when EXT KICK is off. Its pump is
        // the same clocked sidechain, but no kick-only FX touches this lane.
        output_sample += external_pumped * (1.0f - external_kick_mode_mix) *
                         (ground_transition?0.0f:1.0f);
        output_sample *= param_line_gain;
        output_sample = OutputCeiling(output_sample);
        if(!std::isfinite(output_sample)) output_sample = 0.0f;
        out[KICK_OUTPUT_CHANNEL][i] = output_sample;
        out[EXTERNAL_OUTPUT_CHANNEL][i] = output_sample;

        if(raw_kick_age_samples != UINT32_MAX && raw_kick_age_samples < UINT32_MAX - 1)
            ++raw_kick_age_samples;

#if defined(__arm__)
        if(KICK_USB_DIAGNOSTICS)
        {
            for(int c = 0; c < 2; ++c)
            {
                float x = in[c][i];
                diag_accum.sum[c] += x;
                diag_accum.sum_sq[c] += double(x) * x;
                if(fabsf(x) > diag_accum.peak[c]) diag_accum.peak[c] = fabsf(x);
            }
            if(fabsf(raw) > diag_accum.out_peak)
                diag_accum.out_peak = fabsf(raw);
        }
#endif
    }

#if defined(__arm__)
    uint32_t raw_elapsed = DWT->CYCCNT - cycle_start;
    if(raw_elapsed > audio_max_cycles) audio_max_cycles = raw_elapsed;
    uint32_t raw_budget = uint32_t(SystemCoreClock / 48000u * size);
    if(raw_elapsed > raw_budget) ++audio_overruns;
    if(KICK_USB_DIAGNOSTICS)
    {
        if(raw_elapsed > diag_accum.max_cycles) diag_accum.max_cycles = raw_elapsed;
        diag_accum.samples += size;
        if(diag_accum.samples >= 48000u && !diag_snapshot_ready)
        {
            diag_snapshot = diag_accum;
            diag_accum = DiagStats{};
            diag_snapshot_ready = true;
        }
    }
#endif
    return;

    // Legacy processing is intentionally unreachable during the raw-909
    // phase of the redesign. Keep its code below for the later FX rebuild.
    held_fx_engine.BeginBlock(held_fx_control,System::GetNow(),midi_running,
        kick_trigger_pending,perf_quarter_pending,perf_quarter_note_ms,unsigned(size));
    if(held_fx_engine.ratchetTrigger || held_fx_engine.retriggerTrigger)kick_trigger_pending=true;

    // Held pads 1/9 temporarily take over the generated kick's three BPF
    // layers. The choice is made only at a kick boundary, so no filter
    // coefficient changes occur halfway through a hit; step 9 re-rolls here
    // on every base or clocked retrigger while held.
    static bool kick_bpf_override = false;
    static uint8_t kick_bpf_saved_count = 0;
    static float kick_bpf_saved_hz[3] = {};
    const bool held_kick_bpf = held_fx_engine.KickBpfForKick(kick_trigger_pending);
    if(held_fx_engine.KickBpfEngaged() && !kick_bpf_override){
        kick_bpf_saved_count=macro_bpf_layer_count;
        for(int i=0;i<3;++i)kick_bpf_saved_hz[i]=macro_bpf_target_hz[i];
        kick_bpf_override=true;
    }else if(!held_fx_engine.KickBpfEngaged() && kick_bpf_override){
        macro_bpf_layer_count=kick_bpf_saved_count;
        for(int i=0;i<3;++i)macro_bpf_target_hz[i]=kick_bpf_saved_hz[i];
        kick_bpf_override=false;
    }
    if(held_kick_bpf){
        macro_bpf_layer_count = 3;
        for(int i=0;i<3;++i) macro_bpf_target_hz[i]=held_fx_engine.kickBpfHz[i];
    }


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


        /* Latch the BPF layer count for this hit; see the declaration. */
        macro_bpf_layer_count_latched = held_kick_bpf ? 3 : macro_bpf_layer_count;

        // All generator/character/filter history resets. Continuity belongs
        // to a single finite bridge after the final filter, not bass layers.
        // In EXT KICK the input is continuous audio. Resetting the filters or
        // nonlinear history on each MIDI note would inject clicks into it.
        if(!external_kick_mode_target && external_kick_mode_mix < 0.001f)
        {
            kick_output_bridge.Trigger();
            final_infrasonic_hpf.Reset();
            character_highpass.ClearState();
            clean_lowpass.ClearState();
            clean_bass_shelf.ClearState();
            clean_alignment.Reset();
            macro_bpf_bank.Reset();
            kick_reverb.Trigger();
            external_reverb.Trigger();
            macro_whole_kick_reverse.Trigger();
            macro_character_processor.Trigger();
        }

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


    /* SHAPE=0 is legato: MIDI note-on changes pitch without resetting phase
     * or the envelope, and only a matching note-off closes that note's gate. */


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

        external_kick_mode_mix +=
            ((external_kick_mode_target ? 1.0f : 0.0f) - external_kick_mode_mix) *
            EXT_KICK_MODE_SMOOTH_A;
        float external_input_raw =
            in[EXTERNAL_INPUT_CHANNEL_1][i] * EXTERNAL_INPUT_1_GAIN;
        if(!std::isfinite(external_input_raw)) external_input_raw = 0.0f;


        KickVoiceOut voices;

        kick_voice.Process(voices);

        // EXT KICK replaces the oscillator at the common source point, so
        // clean SUB/punch and BPF+Mackie/Tube receive the same external kick.
        const float kick_source =
            voices.punch + (external_input_raw - voices.punch) * external_kick_mode_mix;
        // External kick audio already has its own envelope; do not mute it
        // simply because no MIDI note is currently holding the synth gate.
        const float kick_gate =
            voices.gate + (1.0f - voices.gate) * external_kick_mode_mix;

        // One kick feeds clean and dirty lanes. SUB now boosts existing lows
        // in the clean lane; it cannot add a new pitch or drive the clipper.
        float dry = clean_alignment.Process(
            clean_bass_shelf.Process(clean_lowpass.Process(kick_source),
                                     kick_sub_gain_smoothed) * kick_punch_gain_smoothed);
        float wet = 0.0f;
        if(!KICK_BYPASS_WET)
        {
            float wet_send = kick_source;
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
        kick_output *=
            param_line_gain;


        kick_output = final_infrasonic_hpf.Process(kick_output);
        if(KICK_BYPASS_POST)
        {
            kick_output *= KICK_OUTPUT_LINEAR_GAIN * kick_gate;
        }
        else
        {
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
            kick_output *= kick_gate;
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
               kick HF management

           It has its OWN independent external processing state:
               PUMP
               dotted external delay
               STUTTER / CHOP
               DJ HPF
               DJ LPF

           The two buses meet only at mixer2, before the shared glue/output.
         */

        // Input 1 is the external audio return; input 2 remains quarantined
        // because its jack may float and reintroduce the supply hum.
        float external_output = external_input_raw;

#if defined(__arm__)
        if(KICK_USB_DIAGNOSTICS)
        {
            for(int c = 0; c < 2; ++c)
            {
                float x = in[c][i];
                diag_accum.sum[c] += x;
                diag_accum.sum_sq[c] += double(x) * x;
                if(fabsf(x) > diag_accum.peak[c]) diag_accum.peak[c] = fabsf(x);
            }
        }
#endif
        // Grounding is handled at the hardware. Always pass legitimate quiet
        // input and tails; the former automatic noise gate was not a user FX.
        if(!std::isfinite(external_output))external_output=0.f;

        // Hand the already processed kick bus to the external FX lane when
        // enabled. Undo its branch trim before the normal external return
        // trim below, so the source level remains the kick bus level.
        const float external_trim =
            EXTERNAL_RETURN_GAIN * EXTERNAL_OUTPUT_LINEAR_GAIN;
        const float processed_kick_send =
            external_trim > 0.000001f ? kick_output / external_trim : 0.0f;
        external_output +=
            (processed_kick_send - external_output) * external_kick_mode_mix;


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
        external_output=external_erosion_fx.Process(external_output,erosion_amount,erosion_frequency);
        external_output=held_fx_engine.Process(external_output,&kick_output);
        // In EXT KICK the processed kick now lives in the external FX lane;
        // fade the original kick bus out to prevent doubling it at the mix.
        kick_output *= 1.0f - external_kick_mode_mix;
        float mix=erosion_fx.Process(kick_output,(fx_internal_routes&8)?erosion_amount:0.f,erosion_frequency)
                 +external_output;
        float mixed_output = mix_glue.Process(mix * MIX_OUTPUT_TRIM) * OUTPUT_MAKEUP_GAIN;
        mixed_output = final_ultrasonic_lowpass_b.Process(
            final_ultrasonic_lowpass_a.Process(mixed_output));
        mixed_output = OutputCeiling(mixed_output);
        out[KICK_OUTPUT_CHANNEL][i] = mixed_output;
        out[EXTERNAL_OUTPUT_CHANNEL][i] = mixed_output;
#if defined(__arm__)
        if(KICK_USB_DIAGNOSTICS)
        {
            if(fabsf(mixed_output) > diag_accum.out_peak)
                diag_accum.out_peak = fabsf(mixed_output);
        }
#endif
    }
#if defined(__arm__)
    uint32_t elapsed=DWT->CYCCNT-cycle_start;
    if(elapsed>audio_max_cycles) audio_max_cycles=elapsed;
    if(elapsed>uint32_t(SystemCoreClock / 48000u * size)) ++audio_overruns;
    // Main-loop MIDI parsing needs CPU between audio callbacks. Shed the held
    // effect after repeated near-deadline blocks, without waiting for a MIDI
    // release that a saturated CPU may never get time to parse.
    static unsigned held_overload_blocks=0;
    uint32_t budget=uint32_t(SystemCoreClock / 48000u * size);
    if(held_fx_engine.active && elapsed>budget*9u/10u){
        if(++held_overload_blocks>=3){held_fx_engine.Suspend();held_overload_blocks=0;}
    }else if(held_overload_blocks)--held_overload_blocks;
    if(KICK_USB_DIAGNOSTICS)
    {
        if(elapsed > diag_accum.max_cycles) diag_accum.max_cycles = elapsed;
        diag_accum.samples += size;
        if(diag_accum.samples >= 48000u && !diag_snapshot_ready)
        {
            diag_snapshot = diag_accum;
            diag_accum = DiagStats{};
            diag_snapshot_ready = true;
        }
    }
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
    kick_overlay_voice.Reset();kick_overlay_previous.Reset();
    kick_overlay_handoff_age=kick_overlay_attack_age=RAW_909_RETRIGGER_SAMPLES;
    kick_overlay_hpf_x1=kick_overlay_hpf_y1=0.f;
    kick_overlay_mode=0;kick_overlay_note=33;kick_overlay_decay=64;
    kick_overlay_shape=64;kick_overlay_sweep=64;
    kick_performance_hpf.Reset();
    kick_overlay_mackie.Reset();
    kick_overlay_mackie_flush_samples=0;
    kick_temp_mute_target=false;
    kick_temp_level_target=false;kick_dub_echo_target=false;kick_temp_level_gain=1.f;
    kick_voice_previous.Reset();
    kick_voice_handoff_age = RAW_909_RETRIGGER_SAMPLES;
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
    final_ultrasonic_lowpass_a.Reset();
    final_ultrasonic_lowpass_a.SetLowpass(18000.0f, .70710678f);
    final_ultrasonic_lowpass_b.Reset();
    final_ultrasonic_lowpass_b.SetLowpass(18000.0f, .70710678f);

    kick_output_bridge.Reset();

    added_performance_fx.Reset();

    macro_bpf_bank.Reset();
    clean_alignment.Reset();
    macro_character_processor.Reset();
    kick_reverb.Reset();
    external_reverb.Reset(true);
    external_pitch_fx.Reset();
    held_fx_control.Handle(heldfx::SELECT,0,System::GetNow());
    held_fx_control.Handle(heldfx::LAYERS,0,System::GetNow());
    held_fx_engine.Reset();
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
     * Per-block CPU bursts repeat at 48 kHz / block size and couple into the
     * analogue output as supply ripple. A hardware recording showed the
     * whine at exactly 6000 Hz (+25 dB above the noise floor) with 8-sample
     * blocks, and users heard it at 3 kHz with 16. Two-sample blocks move
     * the ripple to 24 kHz: above hearing and the codec's passband.
     */
    hw.SetAudioBlockSize(AUDIO_BLOCK_SIZE);
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

#if defined(__arm__)
    if(KICK_USB_DIAGNOSTICS)
        hw.StartLog(false); // never waits for a host: plays without USB
#endif
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
#if defined(__arm__)
        if(KICK_USB_DIAGNOSTICS && diag_snapshot_ready)
        {
            DiagStats d = diag_snapshot;
            diag_snapshot_ready = false;
            // Integers only: micro-full-scale for levels, permille for CPU.
            auto micro = [](double v) { return int(v * 1e6); };
            double n = d.samples ? double(d.samples) : 1.0;
            int dc[2], ac[2];
            for(int c = 0; c < 2; ++c)
            {
                double mean = d.sum[c] / n;
                double var = d.sum_sq[c] / n - mean * mean;
                dc[c] = micro(mean);
                ac[c] = micro(var > 0.0 ? sqrt(var) : 0.0);
            }
            uint32_t budget = SystemCoreClock / 48000u * AUDIO_BLOCK_SIZE;
            hw.PrintLine("diag t=%ds blk2 in1 dc=%d rms=%d pk=%d | in2 dc=%d rms=%d pk=%d"
                         " | out_pk=%d | cpu_max=%d/1000 overruns=%d"
                         " uart_err=%d",
                         int(System::GetNow() / 1000u),
                         dc[0], ac[0], micro(d.peak[0]), dc[1], ac[1], micro(d.peak[1]),
                         micro(d.out_peak),
                         int(d.max_cycles * 1000ull / budget), int(audio_overruns),
                         int(midi_uart_errors));
        }
#endif


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
