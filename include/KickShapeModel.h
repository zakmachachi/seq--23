#ifndef KICK_SHAPE_MODEL_H
#define KICK_SHAPE_MODEL_H
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// The Daisy kick as the Teensy shows it: the Daisy's own laws, and screen 2's
// picture of the kick drawn from them. Header-only and free of Arduino types
// so a desktop preview renders the exact same picture. Anything changed in
// the Daisy's KickVoice (daisy-kick/midi_oled_monitor.cpp) must be mirrored
// here, or the screens stop showing what is being played.

namespace kickdaisy {

inline float clamp01(float x){ return x < 0 ? 0 : (x > 1 ? 1 : x); }
inline float smooth(float x){ x = clamp01(x); return x * x * (3.f - 2.f * x); }

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

// PitchMacroStartRatio: velocity is the sweep DEPTH, 1x at 1 to 16x at 127.
inline float depthRatio(uint8_t velocity){
  float p = clamp01(((velocity < 1 ? 1 : velocity) - 1) / 126.f);
  return powf(2.f, 48.f * powf(p, .9f) / 12.f);
}

// ShapeSweepTimeMs: SHAPE owns the sweep TIME.
inline float shapeSweepMs(float shape){
  shape = clamp01(shape);
  if (shape <= .25f) return .7f + (11.f - .7f) * smooth(shape / .25f);
  if (shape <= .50f) return 11.f + (28.f - 11.f) * smooth((shape - .25f) / .25f);
  if (shape <= .75f) return 28.f + (55.f - 28.f) * smooth((shape - .5f) / .25f);
  return 55.f + (135.f - 55.f) * smooth((shape - .75f) / .25f);
}

// Menu 1 SWEEP TIME knob -> CC78. The knob keeps 64 as its neutral (and its
// detent); the Daisy's neutral is 70, so each half maps onto its own side.
inline uint8_t sweepTimeCC(uint8_t knob){
  if (knob > 127) knob = 127;
  return knob <= 64 ? (uint8_t)((knob * 70 + 32) / 64)
                    : (uint8_t)(70 + ((knob - 64) * 57 + 31) / 63);
}

// SweepTimeTrim, on the CC78 value: 0.65x .. 1x at 70 .. 1.55x.
inline float sweepTimeTrim(uint8_t cc){
  float x = clamp01(cc / 127.f);
  return x <= .55f ? .65f + .35f * (x / .55f) : 1.f + .55f * ((x - .55f) / .45f);
}

// The sweep's length, SHAPE and the SWEEP TIME knob together.
inline float sweepMs(uint8_t shape, uint8_t sweepKnob){
  float ms = shapeSweepMs(shape / 127.f) * sweepTimeTrim(sweepTimeCC(sweepKnob));
  return ms < .5f ? .5f : (ms > 220.f ? 220.f : ms);
}

// MacroDecaySeconds: the body is ~30 dB down at this time. No INF.
inline float decaySeconds(uint8_t decay){
  float s = .045f * powf(53.3333333f, decay / 127.f);
  return s < .04f ? .04f : (s > 3.5f ? 3.5f : s);
}

// MacroTailDelayMs: how long TAIL DELAY holds the tail down.
inline float tailDelayMs(uint8_t amount, uint32_t bpm){
  float quarter = 60000.f / (bpm ? bpm : 120);
  return quarter * .5f * powf(amount / 127.f, 1.7f);
}

} // namespace kickdaisy

struct KickShapeInputs {
  uint8_t note = 33;          // MIDI note (channel root): the body pitch
  uint8_t shape = 64;         // K6 SHAPE: sweep time and character
  uint8_t sweepTime = 64;     // Menu 1 SWEEP TIME knob, 64 = neutral
  uint8_t velocity = 100;     // Menu 1 DEPTH: the sweep's start
  uint8_t wave = 0;           // Menu 1 WAVE: sine -> supersaw
  uint8_t decay = 64;         // K2 DECAY
  bool tailOn = false;        // B3
  uint8_t tailAmount = 0;     // K3
  uint32_t bpm = 120;
  int32_t elapsedMs = -1;     // since this channel's last hit; < 0 = idle
};

// The kick's body, sample by sample, as KickVoice::Process renders it (the
// attack click and the output stage are left out: neither changes the shape).
struct KickShape {
  float f0 = 55, depth = 0, sweepMs = 28, decayS = .33f;
  float morph = 0, asym = 0, belly = 0, bellyStart = 0, bellyPeak = 1, bellyEnd = 2;
  float tailStart = 14, tailEnd = 34, holdMs = 0, riseMs = 2.5f, wave = 0;
  bool tailActive = false;

  void compute(const KickShapeInputs& in){
    using namespace kickdaisy;
    float s = in.shape / 127.f;
    f0 = noteHz(in.note);
    float ratio = s <= .0025f ? 1.f : depthRatio(in.velocity);
    if (f0 * ratio > 3200.f) ratio = 3200.f / f0;
    depth = ratio - 1.f;
    sweepMs = kickdaisy::sweepMs(in.shape, in.sweepTime);
    decayS = decaySeconds(in.decay);
    morph = shapeMap(s, 0.f, .44f, .88f);
    asym = shapeMap(s, 0.f, .012f, .045f);
    belly = shapeMap(s, 0.f, .16f, .11f);
    bellyStart = shapeMap(s, 18.f, 12.f, 8.f);
    bellyPeak = shapeMap(s, 72.f, 48.f, 36.f);
    bellyEnd = shapeMap(s, 230.f, 170.f, 130.f);
    tailStart = shapeMap(s, 10.f, 14.f, 18.f);
    tailEnd = shapeMap(s, 28.f, 34.f, 44.f);
    holdMs = in.tailOn && in.tailAmount > 0 ? tailDelayMs(in.tailAmount, in.bpm) : 0.f;
    tailActive = holdMs > .05f;
    float quarter = 60000.f / (in.bpm ? in.bpm : 120);
    float d01 = clamp01(holdMs / (quarter * .5f));
    riseMs = 2.5f + 24.f * sqrtf(d01);
    wave = in.wave / 127.f;
  }

  float tailWindow(float ms) const { return kickdaisy::smooth((ms - tailStart) / (tailEnd - tailStart)); }

  // Body level (no waveform) t ms after the hit: decay, sweep compensation,
  // the belly contour and TAIL DELAY's duck.
  float level(float ms, float ratioNow) const {
    float env = expf(-3.453877639f * ms / (decayS * 1000.f));
    float comp = 1.f / sqrtf(1.f + .18f * (ratioNow - 1.f));
    comp = comp < .52f ? .52f : (comp > 1.f ? 1.f : comp);
    float b = 1.f;
    if (belly > 0 && ms > bellyStart && ms < bellyEnd)
      b += belly * (ms < bellyPeak ? kickdaisy::smooth((ms - bellyStart) / (bellyPeak - bellyStart))
                                   : 1.f - kickdaisy::smooth((ms - bellyPeak) / (bellyEnd - bellyPeak)));
    float tw = tailWindow(ms);
    float gate = !tailActive ? 1.f : (ms <= holdMs ? 0.f : kickdaisy::smooth((ms - holdMs) / riseMs));
    return env * comp * b * ((1.f - tw) + tw * gate);
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

// Screen 2's time axis: linear, so the sweep reads as it sounds, tight cycles
// at the front opening out onto the body. The first 150 ms of the hit, or
// with TAIL DELAY on, long enough to show the tail coming back.
inline float kickViewMs(const KickShape& k){
  float ms = k.tailActive ? k.holdMs + 50.f : 150.f;
  return ms < 150.f ? 150.f : (ms > 400.f ? 400.f : ms);
}

// Draws the 128x64 kick view: the waveform the Daisy plays, as a scope.
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

  // Header: the body pitch on the left, the sweep on the right.
  static const char* const NOTE_NAMES[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  char text[16];
  snprintf(text, sizeof(text), "%s%d %.0fHz", NOTE_NAMES[in.note % 12], (int)(in.note / 12) - 1, k.f0);
  d.setCursor(0, 0); d.print(text);
  if (k.depth > .005f) snprintf(text, sizeof(text), "%.1fx %.0fms", 1.f + k.depth, k.sweepMs);
  else snprintf(text, sizeof(text), "NO SWEEP");
  d.setCursor(128 - 6 * (int)strlen(text), 0); d.print(text);

  // The body, rendered at 24 kHz into per-column min / max.
  const int mid = 37, half = 26;
  int8_t lo[128], hi[128];
  for (int x = 0; x < 128; x++){ lo[x] = 127; hi[x] = -128; }
  const float sr = 24000.f;
  const float pitchCoef = expf(-6.907755f / (k.sweepMs * .001f * sr));
  const float hpA = 1.f - expf(-6.2831853f * 250.f / sr);   // WAVE_BODY_HP_A at this rate
  float phase = 0, pitchEnv = 1, hpLp = 0;
  int prevX = 0, prevY = 0;
  const float viewMs = kickViewMs(k);
  const int n = (int)(viewMs * .001f * sr);
  for (int i = 0; i < n; i++){
    float ms = i * 1000.f / sr;
    float ratioNow = 1.f + k.depth * pitchEnv;
    float f = k.f0 * ratioNow;
    f = f < 18.f ? 18.f : (f > 3000.f ? 3000.f : f);

    float s1 = sinf(phase * 6.2831853f);
    float a = fabsf(s1);
    float para = (s1 >= 0 ? 1.f : -1.f) * (2.f * a - a * a);
    float body = (s1 + k.morph * (para - s1) + k.asym * (s1 * s1 - .5f)) / (1.f + .151173637f * k.morph);
    if (k.wave > 0){
      float q = phase + .5f; q -= floorf(q);
      float h = (2.f * q - 1.f - .63661977f * s1) / .63661977f;
      hpLp += hpA * (h - hpLp);
      body += (h - hpLp) * k.wave * (.25f + .75f * k.tailWindow(ms));
    }
    float y = body * k.level(ms, ratioNow);
    int yy = mid - (int)lroundf((y > 1.2f ? 1.2f : (y < -1.2f ? -1.2f : y)) * half / 1.2f);
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
    pitchEnv *= pitchCoef;
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
