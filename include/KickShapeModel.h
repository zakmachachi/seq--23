#ifndef KICK_SHAPE_MODEL_H
#define KICK_SHAPE_MODEL_H
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Raw 909 voice preview. The generator's pitch and amplitude laws are drawn
// without any output filtering, gain stages, distortion or FX. Keep this in
// step with KickVoice in daisy-kick/midi_oled_monitor.cpp.

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
inline float tailModHz(uint8_t value){
  if(!value)return 0.f;
  float x=float(value-1)/126.f;
  return .02f*powf(800.f,x*x);
}
inline float shapeStartRatio(float x){
  x=clamp01(x);
  float semitones=x<=.5f ? 25.5f*powf(x*2.f,1.8f) : 25.5f+22.5f*smooth((x-.5f)*2.f);
  return powf(2.f,semitones/12.f);
}

// CC78 is centred on the measured 909 sweep, with the useful kick range
// concentrated below centre and the long laser falls at the top.
inline float shapeSweepMs(float position){
  float x=clamp01(position);
  if(x<=.25f) return 4.f*powf(5.f,x*4.f);
  if(x<=.5f) return 20.f*powf(2.9f,(x-.25f)*4.f);
  return 58.f*powf(120.f/58.f,(x-.5f)*2.f);
}
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

// MacroDecaySeconds: ~30 dB down this long after the 60 ms 909 body plateau;
// the upper range extends to 2.4 s for rolling, sustained bass-note kicks.
inline float decaySeconds(uint8_t decay){
  float s = .040f * powf(60.f, decay / 127.f);
  return s < .040f ? .040f : (s > 2.400f ? 2.400f : s);
}

// MacroTailDelayMs: silent gap length, starting after BELLY and a 3 ms fall.
// The expanded macro range reaches 2.4 s so long-tail kicks can create gaps
// beyond an eighth note without depending on the current tempo.
inline float tailDelayMs(uint8_t amount, uint32_t bpm){
  (void)bpm;
  return 2400.f * powf(amount / 127.f, 1.7f);
}

// CC42: reserve a small range for sweep-length holds, concentrate most knob
// positions across the useful 90..400 ms zone, then leave the top for long
// whole-note holds. The same mapping drives the controller readout and Seed.
inline float ampHoldMs(uint8_t amount, float sweepMs, float wholeNoteMs){
  const float low = fmaxf(sweepMs,90.f);
  const float middle = fmaxf(low,400.f);
  const float whole = fmaxf(wholeNoteMs,middle);
  const float x = float(amount) / 127.f;
  const float first = 8.f / 127.f;
  const float last = 116.f / 127.f;
  if(x <= first)
    return sweepMs + (low-sweepMs) * (x/first);
  if(x <= last)
    return low + (middle-low) * ((x-first)/(last-first));
  return middle + (whole-middle) * ((x-last)/(1.f-last));
}

} // namespace kickdaisy

struct KickShapeInputs {
  uint8_t note = 33;          // MIDI note (channel root): the body pitch
  uint8_t shape = 64;         // SHAPE: pitch-sweep depth
  uint8_t sweepTime = 64;     // Menu 1 K3 SWEEP, 4..100 ms
  uint8_t velocity = 64;      // retained for patch/layout compatibility
  uint8_t wave = 0;           // retained for patch/layout compatibility
  uint8_t decay = 64;         // K2 DECAY
  bool tailOn = false;        // retained for patch/layout compatibility
  uint8_t tailAmount = 0;     // retained for patch/layout compatibility
  uint8_t punch = 127;        // fixed internal voice level
  uint8_t sub = 0;            // no separate sub layer
  uint8_t tmod = 0;           // TMOD: rate; 0 one-shot, 1..127 .125..16 Hz
  uint8_t belly = 64;         // retained for patch/layout compatibility
  uint32_t bpm = 120;
  int32_t elapsedMs = -1;     // since this channel's last hit; < 0 = idle

  // Everything that changes the picture (the cursor's elapsedMs does not).
  // KickShape::compute() and kickViewMs() read only these four; comparing
  // HARM/TUNE/TMOD/tempo too re-simulated the whole kick on every frame
  // while one of those knobs was turning, for an identical picture.
  bool sameShape(const KickShapeInputs& o) const {
    return note == o.note && shape == o.shape && sweepTime == o.sweepTime &&
           decay == o.decay;
  }
};

// A scope preview of the raw oscillator and its short harmonic attack shoulder.
struct KickShape {
  float f0 = 55, depth = 0, sweepMs = 28, decayS = .127f;
  float peakRatio = 1, startRatio = 1, riseMs = 4.7f, bodyHoldMs = 60.f;

  void compute(const KickShapeInputs& in){
    using namespace kickdaisy;
    f0 = noteHz(in.note);
    peakRatio = shapeStartRatio(in.shape / 127.f);
    startRatio = 1.f + .28f * (peakRatio - 1.f);
    depth = peakRatio - 1.f;
    sweepMs = kickdaisy::sweepMs(in.shape, in.sweepTime);
    decayS = decaySeconds(in.decay);
    riseMs = fmaxf(.5f, sweepMs * .15f);
  }

  float pitchHz(float ms) const {
    float tauMs = fmaxf(.5f, sweepMs * .36f);
    float ratio;
    if (ms < riseMs) {
      float u = kickdaisy::clamp01(ms / riseMs);
      float r = u*u*(3.f-2.f*u);
      ratio = startRatio + (peakRatio-startRatio)*r;
    } else {
      ratio = 1.f + depth * expf(-(ms-riseMs) / tauMs);
    }
    return f0 * ratio;
  }
  float envelope(float ms) const {
    float age = ms - bodyHoldMs;
    return age <= 0 ? 1.f : expf(-3.453877639f * age / (decayS * 1000.f));
  }
};

// HARMONICS as a picture, w x h pixels at (x, y). It starts at the existing
// sine and smoothly adds phase-locked partials toward a fuller, harder kick.
template <class D>
void drawWaveIcon(D& d, int x, int y, int w, int h, uint8_t wave, uint16_t white){
  const float m = wave / 127.f;
  const float h2 = .62f * m;
  const float h3 = .26f * m * m;
  const float h5 = .12f * m * m * m;
  int prev = -1;
  for (int i = 0; i < w; i++){
    float ph = (float)i / (w - 1);       // one cycle
    const float fundamental = sinf(ph * 6.2831853f);
    float v = fundamental
      + h2 * sinf(ph * 12.5663706f)
      - h3 * sinf(ph * 18.8495559f)
      + h5 * sinf(ph * 31.4159265f);
    v *= 1.f / sqrtf(1.f + .125f*h2*h2 + .5f*h3*h3 + .5f*h5*h5);
    v = fmaxf(-1.f, fminf(1.f, v));
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
  float ms = k.bodyHoldMs + 2.1f * k.decayS * 1000.f;
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

  // Header: selected body note and settled oscillator frequency, sweep right.
  static const char* const NOTE_NAMES[] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  char text[16];
  snprintf(text, sizeof(text), "%s%d %.0fHz", NOTE_NAMES[in.note % 12], (int)(in.note / 12) - 1, k.f0);
  d.setCursor(0, 0); d.print(text);
  if (k.depth > .005f) snprintf(text, sizeof(text), "%.1fx %.0fms", 1.f + k.depth, k.sweepMs);
  else snprintf(text, sizeof(text), "NO SWEEP");
  d.setCursor(128 - 6 * (int)strlen(text), 0); d.print(text);

  // Raw oscillator scope: no crossover, EQ shelf, distortion, gain macro,
  // compressor, output filter or FX is drawn because none is in this mode.
  const int mid = 37, half = 26;
  static int8_t lo[128], hi[128];
  static KickShapeInputs last;
  static bool cached = false;
  const float viewMs = kickViewMs(k);
  if (!cached || !in.sameShape(last)){
    cached = true; last = in;
    for (int x = 0; x < 128; x++){ lo[x] = 127; hi[x] = -128; }
    // At most 4096 samples (32 per column) for the min/max scope. A fixed
    // 12 kHz made a long decay (up to ~5 s on screen) cost ~60k samples.
    const float sr = fminf(12000.f, 4096.f / (viewMs * .001f));
    float phase = .5f;
    int prevX = 0, prevY = 0;
    const int n = (int)(viewMs * .001f * sr);
    for (int i = 0; i < n; i++){
      float ms = i * 1000.f / sr;
      float f = k.pitchHz(ms);
      float attack = expf(-ms / 6.f);
      // sin(2..7 theta) from one sine/cosine pair: sin((m+1)t) =
      // 2cos(t)sin(mt) - sin((m-1)t). Same waveform, two calls instead of five.
      const float th = phase * 6.2831853f;
      const float s1 = sinf(th), c2 = 2.f * cosf(th);
      const float s2 = c2 * s1;
      const float s3 = c2 * s2 - s1;
      const float s4 = c2 * s3 - s2;
      const float s5 = c2 * s4 - s3;
      const float s6 = c2 * s5 - s4;
      const float s7 = c2 * s6 - s5;
      float body = s1 + .037f*s2
                   + (-.05f-.25f*attack)*s3
                   + .22f*attack*s5
                   - .05f*attack*s7;
      float y = .55f * k.envelope(ms) * body;
      int yy = mid - (int)lroundf(y * half);
      int x = (int)(127.f * ms / viewMs); if (x > 127) x = 127;
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
