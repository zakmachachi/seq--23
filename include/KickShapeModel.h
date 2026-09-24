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
  uint8_t tailOffset = 64;    // FUNCTION + K3
  uint8_t tailAttack = 0;     // FUNCTION + B3, then K3
  uint32_t bpm = 120;
};

struct KickShape {
  float f0 = 55, ratio = 1, sweepMs = 88, holdMs = 36, endMs = 82;
  float decayS = .5f, delayMs = 0, pitchSt = 0, attackMs = 6, totalMs = 300;
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
    if (in.tailOn){
      float quarter = 60000.f / (in.bpm ? in.bpm : 120);
      if (in.tailAmount > 0) delayMs = quarter * .5f * powf(in.tailAmount / 127.f, 1.7f);
      delayMs += ((int)in.tailOffset - 64) * .5f;
      if (delayMs < 0) delayMs = 0;
    }
    uint8_t v = in.velocity < 1 ? 1 : in.velocity;
    pitchSt = v <= 64 ? -12.f * (1.f - (v - 1) / 63.f) : 12.f * (v - 64) / 63.f;
    float intensity = in.tailMod / 127.f;
    wobbleDepth = 2.f * intensity;
    wobbleHz = 2.f + 10.f * intensity * intensity;
    wobbleIrregular = .6f * intensity;
    attackMs = 6.f * powf(10.f, in.tailAttack / 127.f);
    // The old blended sub decay empties in ~61 % of the DECAY time.
    float subMs = inf ? 1200.f : .61f * decayS * 1000.f;
    float end = delayMs + subMs;
    if (end < endMs) end = endMs;
    totalMs = end < 150.f ? 150.f : (end > 1200.f ? 1200.f : end);
  }

  // Pitch relative to the note, in semitones, t ms after the hit.
  float pitchSemitones(float t) const {
    float st = 0;
    if (t > delayMs){
      float ts = t - delayMs;
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
    punch = cosf(a * 1.5707963f);
    sub = 0;
    if (t >= delayMs){
      float ts = t - delayMs;
      float env = inf ? 1.f : 1.177f * expf(-3.1085f * ts / (decayS * 1000.f)) - .177f;
      if (env < 0) env = 0;
      sub = sinf(a * 1.5707963f) * env * smooth(ts / attackMs);
    }
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
  snprintf(text, sizeof(text), "WAV%3d", (int)(in.wave * 100 + 63) / 127);
  d.setCursor(0, 0); d.print(text);
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

  // Where the sub starts, when the tail is delayed.
  if (k.delayMs > 0){
    int x = (int)lroundf(k.delayMs / k.totalMs * 127.f);
    d.fillTriangle(x - 2, 36, x + 2, 36, x, 38, white);
  }
}
#endif
