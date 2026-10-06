#ifndef KICK_SHAPE_MODEL_H
#define KICK_SHAPE_MODEL_H
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// The Daisy kick as the Teensy shows it: the generator's pitch, amplitude and
// gate laws, drawn before distortion, output filtering and performance FX.
// Header-only and free of Arduino types for desktop previews. Changes in
// the Daisy's KickVoice (daisy-kick/midi_oled_monitor.cpp) must be mirrored
// here, or the screens stop showing what is being played.

namespace kickdaisy {

inline float clamp01(float x){ return x < 0 ? 0 : (x > 1 ? 1 : x); }
inline float smooth(float x){ x = clamp01(x); return x * x * (3.f - 2.f * x); }
inline float raisedCosine(float x){ return .5f - .5f * cosf(clamp01(x) * 3.14159265f); }
inline float polyBlepSaw(float phase, float dt){
  float y = 2.f * phase - 1.f;
  if (phase < dt){ float t = phase / dt; y -= t + t - t * t - 1.f; }
  else if (phase > 1.f - dt){ float t = (phase - 1.f) / dt; y -= t * t + t + t + 1.f; }
  return y;
}

// KickVoice::ShapeMap: round / punch / hard values across SHAPE.
inline float shapeMap(float x, float a, float b, float c){
  x = clamp01(x);
  return x <= .5f ? a + (b - a) * smooth(x * 2.f) : b + (c - b) * smooth((x - .5f) * 2.f);
}

// MIDI note = the settled body pitch, 30..120 Hz.
inline float noteHz(uint8_t note){
  float f = 440.f * powf(2.f, ((int)note - 69) / 12.f);
  return f < 30.f ? 30.f : (f > 120.f ? 120.f : f);
}

inline float bpfHz(uint8_t value){return 85.f*powf(3200.f/85.f,value/127.f);}
inline float tailPitchSemitones(uint8_t value){
  float x=value<=64 ? (float(value)-64.f)/64.f : (float(value)-64.f)/63.f;
  return 12.f*x*fabsf(x);
}
inline float tailModHz(uint8_t value){return value ? .125f*powf(128.f,float(value-1)/126.f) : 0.f;}
inline float shapeStartRatio(float x){
  x=clamp01(x);
  float semitones=x<=.5f ? 28.f*smooth(x*2.f) : 28.f+20.f*smooth((x-.5f)*2.f);
  return powf(2.f,semitones/12.f);
}

// CC78 directly controls pitch sweep duration, independent of SHAPE/DECAY.
inline float shapeSweepMs(float position){ return 4.f*powf(60.f,clamp01(position)); }
inline uint8_t sweepTimeCC(uint8_t knob){ return knob > 127 ? 127 : knob; }
inline float sweepMs(uint8_t shape, uint8_t sweepKnob){
  (void)shape;
  return shapeSweepMs(sweepTimeCC(sweepKnob)/127.f);
}

// KickCurveExponent: the sweep falls as exp(-6.9 u^g), u = t / sweep time.
inline float curveExponent(uint8_t cc){
  return cc <= 64 ? .5f + .5f * cc / 64.f : 1.f + 1.5f * (cc - 64) / 63.f;
}

// KickBellyScale: scale on the punch window's times, 64 = 1x.
inline float bellyScale(uint8_t cc){
  return cc <= 64 ? .35f + .65f * cc / 64.f : 1.f + (cc - 64) / 63.f;
}

// MacroDecaySeconds: ~30 dB down this long after a one-sub-cycle hold. No INF.
inline float decaySeconds(uint8_t decay){
  float s = .045f * powf(53.3333333f, decay / 127.f);
  return s < .04f ? .04f : (s > 3.5f ? 3.5f : s);
}

// MacroTailDelayMs: silent gap length, starting after BELLY and a 3 ms fall.
inline float tailDelayMs(uint8_t amount, uint32_t bpm){
  float quarter = 60000.f / (bpm ? bpm : 120);
  return quarter * .5f * powf(amount / 127.f, 1.7f);
}

} // namespace kickdaisy

struct KickShapeInputs {
  uint8_t note = 33;          // MIDI note (channel root): the body pitch
  uint8_t shape = 64;         // K6 SHAPE: pitch depth and attack character
  uint8_t sweepTime = 64;     // Menu 1 K3 SWEEP TIME, 4..240 ms
  uint8_t velocity = 64;      // TUNE: bipolar tail pitch; 64 neutral
  uint8_t wave = 0;           // Menu 1 WAVE: sine -> one phase-derived saw
  uint8_t decay = 64;         // K2 DECAY
  bool tailOn = false;        // B3
  uint8_t tailAmount = 0;     // K3
  uint8_t punch = 64;         // mix page PUNCH: clean low-pass lane gain 0..1
  uint8_t sub = 75;           // mix page SUB: clean bass shelf, 0 = flat
  uint8_t tmod = 0;           // TMOD: rate; 0 one-shot, 1..127 .125..16 Hz
  uint8_t belly = 64;         // mix page BELLY: punch window scale, 64 = 1x
  uint32_t bpm = 120;
  int32_t elapsedMs = -1;     // since this channel's last hit; < 0 = idle

  // Everything that changes the picture (the cursor's elapsedMs does not).
  bool sameShape(const KickShapeInputs& o) const {
    return note == o.note && shape == o.shape && sweepTime == o.sweepTime &&
           velocity == o.velocity && wave == o.wave && decay == o.decay &&
           tailOn == o.tailOn && tailAmount == o.tailAmount && punch == o.punch && sub == o.sub &&
           tmod == o.tmod && belly == o.belly && bpm == o.bpm;
  }
};

// A scope preview of the clean generator and bass shelf. Attack noise, crossover,
// distortion, final output filtering and performance effects are omitted.
struct KickShape {
  float f0 = 55, depth = 0, sweepMs = 28, decayS = .33f;
  float morph = 0, asym = 0, punchStart = 50;
  float holdMs = 0, riseMs = 3.f, wave = 0, lift = 1, curveG = 1, sub = 0;
  float onsetMs = 8.f, bodyHoldMs = 0.f;
  float tailSemitones=0.f, tailRate=0.f, tailStartMs=0.f, tailGlideMs=150.f;
  bool tailActive = false;

  void compute(const KickShapeInputs& in){
    using namespace kickdaisy;
    float s = in.shape / 127.f;
    f0 = noteHz(in.note);
    float ratio = shapeStartRatio(s);
    tailSemitones=tailPitchSemitones(in.velocity);
    tailRate=tailModHz(in.tmod);
    onsetMs = 8.f + (.5f-8.f) * smooth(s / .35f);
    if (f0 * ratio > 3000.f) ratio = 3000.f / f0;
    depth = ratio - 1.f;
    sweepMs = kickdaisy::sweepMs(in.shape, in.sweepTime);
    decayS = decaySeconds(in.decay);
    morph = shapeMap(s, 0.f, .44f, .88f);
    asym = shapeMap(s, 0.f, .012f, .045f);
    curveG = 1.f;
    float settleU = depth > .05f ? powf(logf(depth / .05f) / 6.907755f, 1.f / curveG) : 0.f;
    tailStartMs=sweepMs*settleU;
    bodyHoldMs=2000.f/f0;
    tailGlideMs=sweepMs;
    float bellyK = bellyScale(in.belly);
    punchStart = 50.f * bellyK;
    if (punchStart < bodyHoldMs) punchStart = bodyHoldMs;
    curveG = 1.f;
    lift = in.punch / 127.f;
    holdMs = in.tailOn && in.tailAmount > 0 ? tailDelayMs(in.tailAmount, in.bpm) : 0.f;
    tailActive = holdMs > .05f;
    wave = in.wave / 127.f;
    sub = in.sub / 127.f;
  }

  // Shared amplitude: settle the sweep, hold one sub cycle, then decay.
  float envelope(float ms) const {
    float age = ms - bodyHoldMs;
    return age <= 0 ? 1.f : expf(-3.453877639f * age / (decayS * 1000.f));
  }
  float level(float ms) const {
    return envelope(ms) * kickdaisy::raisedCosine(ms / onsetMs);
  }
  float gate(float ms) const {
    if (!tailActive || ms <= punchStart) return 1.f;
    if (ms < punchStart + riseMs) return 1.f - kickdaisy::raisedCosine((ms - punchStart) / riseMs);
    float reopen = punchStart + riseMs + holdMs;
    return ms < reopen ? 0.f : kickdaisy::raisedCosine((ms - reopen) / riseMs);
  }
};

// WAVE as a picture, w x h pixels at (x, y), in the style of the Elektron
// Digitone 2 / Digitakt 2 wave displays: the waveform itself bends rather than
// two shapes blending. Phase distortion (the Casio CZ sine-to-saw trick): the
// rising half of the cycle is stretched and the fall squeezed, so the sine
// leans over and sharpens into a saw with a vertical drop. A visual indicator
// only; the knee moves on a square-root curve so small settings show clearly.
template <class D>
void drawWaveIcon(D& d, int x, int y, int w, int h, uint8_t wave, uint16_t white){
  float m = sqrtf(wave / 127.f);
  float knee = .5f + .47f * m;           // where the peak sits in the cycle
  int prev = -1;
  for (int i = 0; i < w; i++){
    float ph = (float)i / (w - 1);       // one cycle
    float warped =
      ph < knee ? .5f * ph / knee
                : .5f + .5f * (ph - knee) / (1.f - knee);
    float v = -cosf(warped * 6.2831853f);  // -1 at the ends, +1 at the knee
    int yy = y + (int)lroundf((1.f - v) * .5f * (h - 1));
    if (prev >= 0) d.drawLine(x + i - 1, prev, x + i, yy, white);
    prev = yy;
  }
}

// CURVE as a picture, w x h pixels at (x, y): the pitch falling from the top
// left to the note at the bottom right, exp(-6.9 u^g) as the Daisy has it.
template <class D>
void drawCurveIcon(D& d, int x, int y, int w, int h, uint8_t curve, uint16_t white){
  float g = kickdaisy::curveExponent(curve);
  int prev = -1;
  for (int i = 0; i < w; i++){
    float u = (float)i / (w - 1);
    float v = expf(-6.907755f * powf(u, g));     // 1 at the left, ~0 at the right
    int yy = y + (int)lroundf((1.f - v) * (h - 1));
    if (prev >= 0) d.drawLine(x + i - 1, prev, x + i, yy, white);
    prev = yy;
  }
}

// Screen 2's time axis: linear, so the sweep reads as it sounds, tight cycles
// at the front opening out onto the body. The first 150 ms of the hit, or
// with TAIL DELAY on, long enough to show the tail all the way back.
inline float kickViewMs(const KickShape& k){
  float ms = k.tailActive ? k.punchStart + k.holdMs + 2.f * k.riseMs + 30.f : 150.f;
  return ms < 150.f ? 150.f : ms;
}

// Draws the 128x64 clean-generator preview, as a scope.
// While a hit is sounding, the part already played is solid, the rest is
// dimmed, and a cursor runs across in real time. D is any Adafruit_GFX-like
// display with clearDisplay, setTextSize, setTextColor, setCursor,
// print(const char*), drawPixel and drawFastVLine.
template <class D>
void drawKickShape(D& d, const KickShapeInputs& in, uint16_t white, uint16_t inverse){
  (void)inverse;
  KickShape k; k.compute(in);
  d.clearDisplay();
  d.setTextSize(1);
  d.setTextColor(white);

  // Header: selected body note + sub Hz on the left, sweep on the right.
  static const char* const NOTE_NAMES[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  char text[16];
  if (in.sub) snprintf(text, sizeof(text), "%s%d %.0fHz", NOTE_NAMES[in.note % 12], (int)(in.note / 12) - 1, k.f0);
  else snprintf(text, sizeof(text), "%s%d S-", NOTE_NAMES[in.note % 12], (int)(in.note / 12) - 1);
  d.setCursor(0, 0); d.print(text);
  if (k.depth > .005f) snprintf(text, sizeof(text), "%.1fx %.0fms", 1.f + k.depth, k.sweepMs);
  else snprintf(text, sizeof(text), "NO SWEEP");
  d.setCursor(128 - 6 * (int)strlen(text), 0); d.print(text);

  // The body, rendered at 12 kHz into per-column min / max. Only when an
  // input changes: the cursor animating over it costs nothing.
  const int mid = 37, half = 26;
  static int8_t lo[128], hi[128];
  static KickShapeInputs last;
  static bool cached = false;
  const float viewMs = kickViewMs(k);
  if (!cached || !in.sameShape(last)){
  cached = true; last = in;
  for (int x = 0; x < 128; x++){ lo[x] = 127; hi[x] = -128; }
  const float sr = 12000.f;
  float phase = 0, bodyPhase = 0;
  // Fixed RBJ +15 dB / 120 Hz shelf at the preview sample rate.
  const float A=powf(10.f,15.f/40.f), w=6.2831853f*120.f/sr;
  const float c=cosf(w), beta=sqrtf(2.f*A)*sinf(w);
  const float a0=(A+1.f)+(A-1.f)*c+beta;
  const float b0=A*((A+1.f)-(A-1.f)*c+beta)/a0;
  const float b1=2.f*A*((A-1.f)-(A+1.f)*c)/a0;
  const float b2=A*((A+1.f)-(A-1.f)*c-beta)/a0;
  const float a1=-2.f*((A-1.f)+(A+1.f)*c)/a0;
  const float a2=((A+1.f)+(A-1.f)*c-beta)/a0;
  float z1=0.f,z2=0.f;
  int prevX = 0, prevY = 0;
  // Past this the sweep is under 1e-5 (the Daisy's curve_end_u).
  const float uEnd = powf(11.512925f / 6.907755f, 1.f / k.curveG);
  const int n = (int)(viewMs * .001f * sr);
  for (int i = 0; i < n; i++){
    float ms = i * 1000.f / sr;
    float u = ms / k.sweepMs;
    float pitchEnv = u >= uEnd ? 0.f : expf(-6.907755f * powf(u, k.curveG));
    float ratioNow = 1.f + k.depth * pitchEnv;
    float tailAge=fmaxf(0.f,ms-k.tailStartMs);
    float motion=k.tailRate>0.f ? .5f-.5f*cosf(6.2831853f*k.tailRate*tailAge*.001f)
                              : kickdaisy::raisedCosine(tailAge/k.tailGlideMs);
    float f = k.f0 * ratioNow * powf(2.f,k.tailSemitones*motion/12.f);

    float s1 = sinf(bodyPhase * 6.2831853f);
    float a = fabsf(s1);
    float para = (s1 >= 0 ? 1.f : -1.f) * (2.f * a - a * a);
    float body = (s1 + k.morph * (para - s1) + k.asym * (s1 * s1 - .5f)) / (1.f + .151173637f * k.morph);
    if (k.wave > 0){
      float q = bodyPhase + .5f; q -= floorf(q);
      float h = kickdaisy::polyBlepSaw(q, f / sr) * 1.57079633f - s1;
      body += h * k.wave;
    }
    float comp = 1.f / sqrtf(1.f + .18f * (ratioNow - 1.f));
    if (comp < .52f) comp = .52f;
    float dry = .9f * body * comp * k.level(ms);
    float bass = b0*dry+z1;
    z1=b1*dry-a1*bass+z2; z2=b2*dry-a2*bass;
    float y=(dry+k.sub*(bass-dry))*k.lift*k.gate(ms);
    int yy = mid - (int)lroundf((y > 4.5f ? 4.5f : (y < -4.5f ? -4.5f : y)) * half / 4.5f);
    int x = (int)(127.f * ms / viewMs); if (x > 127) x = 127;

    // Connect to the previous sample across any columns it skipped.
    if (i == 0){ prevX = x; prevY = yy; }
    for (int cx = prevX; cx <= x; cx++){
      int cy = x == prevX ? yy : prevY + (yy - prevY) * (cx - prevX) / (x - prevX);
      int y0 = cx == prevX ? prevY : cy;
      if (cy < lo[cx]) lo[cx] = (int8_t)cy;
      if (cy > hi[cx]) hi[cx] = (int8_t)cy;
      if (y0 < lo[cx]) lo[cx] = (int8_t)y0;
      if (y0 > hi[cx]) hi[cx] = (int8_t)y0;
    }
    prevX = x; prevY = yy;

    phase += f / sr; phase -= floorf(phase);
    bodyPhase = phase;
  }
  }

  // Played part solid, the rest dimmed, and the cursor, while it sounds.
  bool playing = in.elapsedMs >= 0 && in.elapsedMs < (int32_t)viewMs;
  int cursor = playing ? (int)(127.f * in.elapsedMs / viewMs) : 128;
  for (int x = 0; x < 128; x++){
    if (hi[x] < lo[x]) continue;
    if (x <= cursor) d.drawFastVLine(x, lo[x], hi[x] - lo[x] + 1, white);
    else for (int y = lo[x]; y <= hi[x]; y++) if (((x + y) & 1) == 0) d.drawPixel(x, y, white);
  }
  if (playing) for (int y = 10; y < 64; y += 2) d.drawPixel(cursor, y, white);
}
#endif
