#ifndef KICK_SHAPE_MODEL_H
#define KICK_SHAPE_MODEL_H
#include <math.h>
#include <stdint.h>
#include <stdio.h>

// The Daisy kick as screen 2 draws it: pitch and level over time, computed
// with the same formulas the Daisy's KickVoice uses. Header-only and free of
// Arduino types so a desktop preview can render the exact same picture.
// Anything changed in the Daisy's KickVoice / KickHitParams must be mirrored
// here, or the screen stops showing what is being played.

struct KickShapeInputs {
  uint8_t note = 33;          // MIDI note (channel root)
  uint8_t shape = 64;         // K6 SHAPE: punch sweep depth
  uint8_t sweepTime = 64;     // Menu 1 SWEEP: 64 = SHAPE's own time
  uint8_t velocity = 64;      // Menu 1 PITCH
  uint8_t tailMod = 0;        // Menu 1 TMOD: wobble intensity
  uint8_t wave = 0;           // Menu 1 WAVE: sine -> supersaw
  uint8_t decay = 64;         // K2 DECAY
  bool tailOn = false;        // B3
  uint8_t tailAmount = 0;     // K3
  uint8_t tailAttack = 0;     // mix page pot 3
  uint32_t bpm = 120;
};

struct KickShape {
  float f0 = 55, ratio = 1, sweepMs = 88, holdMs = 36, endMs = 82;
  float decayS = .5f, delayMs = 0, pitchSt = 0, attackMs = 8, totalMs = 300;
  float gapHoldMs = 82;   // TAIL DELAY: full through the punch, then a gap
  float wobbleDepth = 0, wobbleHz = 2, wobbleIrregular = 0;
  bool inf = false;

  static float smooth(float x){
    x = x < 0 ? 0 : (x > 1 ? 1 : x);
    return x * x * (3.f - 2.f * x);
  }
  static float threePoint(float x, float a, float b, float c){
    return x <= .5f ? a + (b - a) * smooth(x / .5f) : b + (c - b) * smooth((x - .5f) / .5f);
  }

  void compute(const KickShapeInputs& in){
    f0 = 440.f * powf(2.f, ((int)in.note - 69) / 12.f);
    f0 = f0 < 25.f ? 25.f : (f0 > 130.f ? 130.f : f0);
    float p = in.shape / 127.f;
    ratio = powf(30.f, powf(p, 1.35f));
    if (f0 * ratio > 3000.f) ratio = 3000.f / f0;
    float scale = powf(2.f, ((int)in.sweepTime - 64) / 32.f);
    sweepMs = (p <= .5f ? 20.f + 68.f * (p / .5f) : 88.f + 22.f * ((p - .5f) / .5f)) * scale;
    holdMs = threePoint(p, 20.f, 36.f, 115.f) * scale;
    endMs = threePoint(p, 52.f, 82.f, 165.f) * scale;
    float x = in.decay / 127.f;
    inf = x >= .99f;
    decayS = inf ? 2.f : .035f * powf(171.428571f, x);
    delayMs = 0;
    if (in.tailOn && in.tailAmount > 0){
      float quarter = 60000.f / (in.bpm ? in.bpm : 120);
      delayMs = quarter * .5f * powf(in.tailAmount / 127.f, 1.7f);
    }
    gapHoldMs = endMs;
    uint8_t v = in.velocity < 1 ? 1 : in.velocity;
    pitchSt = v <= 64 ? -12.f * (1.f - (v - 1) / 63.f) : 12.f * (v - 64) / 63.f;
    float intensity = in.tailMod / 127.f;
    wobbleDepth = 2.f * intensity;
    wobbleHz = 2.f + 10.f * intensity * intensity;
    wobbleIrregular = .6f * intensity;
    attackMs = 8.f * powf(60.f / 8.f, in.tailAttack / 127.f);
    // The old blended sub decay empties in ~61 % of the DECAY time.
    float subMs = inf ? 1200.f : .61f * decayS * 1000.f;
    float end = subMs > delayMs + 60.f ? subMs : delayMs + 60.f;
    if (end < endMs) end = endMs;
    totalMs = end < 150.f ? 150.f : (end > 1200.f ? 1200.f : end);
  }

  // TAIL DELAY: the whole kick's level, t ms after the hit (the Daisy's
  // KickVoice::GapLevel).
  float gapLevel(float t) const {
    if (delayMs <= 0 || t < gapHoldMs) return 1.f;
    float down = 1.f - smooth((t - gapHoldMs) / 10.f);
    float up = t < delayMs ? 0.f : smooth((t - delayMs) / attackMs);
    return 1.f - (1.f - down) * (1.f - up);
  }

  // Pitch relative to the note, in semitones, t ms after the hit.
  float pitchSemitones(float t) const {
    float st = 0;
    {
      float ts = t;
      st += pitchSt * smooth(ts / 115.f);
      if (wobbleDepth > 0){
        float c = wobbleHz * ts / 1000.f;
        float shape = (sinf(c * 6.2831853f) + wobbleIrregular * sinf(c * 1.73f * 6.2831853f + 1.1f)) /
                      (1.f + wobbleIrregular);
        st += wobbleDepth * smooth(ts / 40.f) * shape;
      }
    }
    float base = f0 * powf(2.f, st / 12.f);
    base = base < 12.f ? 12.f : (base > 440.f ? 440.f : base);
    float f = base * (1.f + (ratio - 1.f) * expf(-6.9078f * t / sweepMs));
    return 12.f * log2f(f / f0);
  }

  // Punch and sub levels, 0..1, t ms after the hit (the old anatomy).
  void levels(float t, float& punch, float& sub) const {
    float a = t <= holdMs ? 0.f : smooth((t - holdMs) / (endMs - holdMs));
    float g = gapLevel(t);
    punch = cosf(a * 1.5707963f) * g;
    float env = inf ? 1.f : 1.177f * expf(-3.1085f * t / (decayS * 1000.f)) - .177f;
    if (env < 0) env = 0;
    sub = sinf(a * 1.5707963f) * env * smooth(t / 6.f) * g;
  }
};

// Pitch display scale: exact semitones near the note, compressed above +14 so
// a laser punch (up to ~+59) still fits while PITCH and the TMOD wobble keep
// their resolution. Returns "display semitones", -14..+30.
inline float kickShapeDisplaySt(float st){
  if (st > 14.f) st = 14.f + (st - 14.f) * .35f;
  if (st > 30.f) st = 30.f;
  if (st < -14.f) st = -14.f;
  return st;
}

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

// Draws the 128x64 kick view. D is any Adafruit_GFX-like display with
// clearDisplay, setTextSize, setTextColor, setCursor, print(const char*),
// drawPixel, drawLine, drawFastHLine, drawFastVLine and fillTriangle, and
// the colour constants passed in (white, inverse).
template <class D>
void drawKickShape(D& d, const KickShapeInputs& in, uint16_t white, uint16_t inverse){
  KickShape k; k.compute(in);
  d.clearDisplay();
  d.setTextSize(1);
  d.setTextColor(white);

  // Header: the kick channel's Menu 1 knobs, in pot order, in fixed
  // six-character columns so they never run into each other.
  char text[12];
  drawWaveIcon(d, 2, 0, 24, 9, in.wave, white);
  snprintf(text, sizeof(text), "SWP%.1f", powf(2.f, ((int)in.sweepTime - 64) / 32.f));
  d.setCursor(46, 0); d.print(text);
  snprintf(text, sizeof(text), "TMD%3d", (int)(in.tailMod * 100 + 63) / 127);
  d.setCursor(92, 0); d.print(text);

  // Pitch panel, y 11..35: the note is a dotted line, the curve is the punch
  // sweep, the PITCH glide and the TMOD wobble.
  const int pTop = 11, pBot = 35;
  auto pitchY = [&](float st){
    float v = kickShapeDisplaySt(st);           // -14..+30
    return pBot - (int)lroundf((v + 14.f) / 44.f * (pBot - pTop));
  };
  int noteY = pitchY(0);
  for (int x = 0; x < 128; x += 3) d.drawPixel(x, noteY, white);

  // Level panel, y 40..62: punch as a 50 % hatch, the sub solid in front of
  // it, and one outline over the top of the two.
  const int aTop = 40, aBot = 62;
  int prevPitch = -1, prevTop = -1;
  for (int x = 0; x < 128; x++){
    float t = x / 127.f * k.totalMs;
    int y = pitchY(k.pitchSemitones(t));
    if (prevPitch >= 0) d.drawLine(x - 1, prevPitch, x, y, white);
    prevPitch = y;

    float punch, sub; k.levels(t, punch, sub);
    int yp = aBot - (int)lroundf(punch * (aBot - aTop));
    int ys = aBot - (int)lroundf(sub * (aBot - aTop));
    if (punch > .01f)
      for (int yy = yp; yy <= aBot; yy++)
        if (((x + yy) & 1) == 0) d.drawPixel(x, yy, white);
    if (sub > .01f) d.drawFastVLine(x, ys, aBot - ys + 1, white);

    int top = yp < ys ? yp : ys;
    if (punch <= .01f && sub <= .01f) top = aBot + 1;   // nothing sounding
    if (prevTop >= 0 && top <= aBot && prevTop <= aBot) d.drawLine(x - 1, prevTop, x, top, white);
    else if (top <= aBot) d.drawPixel(x, top, white);
    prevTop = top;
  }

  // Baseline under the panel, with a gap every 100 ms.
  d.drawFastHLine(0, 63, 128, white);
  for (float ms = 100; ms < k.totalMs; ms += 100){
    int x = (int)lroundf(ms / k.totalMs * 127.f);
    d.drawPixel(x, 63, inverse);
  }

  // Where the kick comes back, when the tail is delayed.
  if (k.delayMs > 0){
    int x = (int)lroundf(k.delayMs / k.totalMs * 127.f);
    d.fillTriangle(x - 2, 36, x + 2, 36, x, 38, white);
  }
}
#endif
