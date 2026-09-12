#include "KickMixer.h"
#include <math.h>
#include <stdio.h>

namespace {
constexpr uint8_t CC[] = {53,54,55,56,57,58};
const char* const SHORT_NAMES[] = {"LINE","MACK","SHRM","BPF","SUB","PUNCH"};
const char* const LONG_NAMES[] = {"LINE OUT","MACKIE","SHERMAN","BPF MIX","SUB","PUNCH VOL"};
constexpr float PI_F = 3.14159265358979323846f;
constexpr int LEFT = 4, RIGHT = 123;
void label(Adafruit_SH1106G& d, int x, int y, const char* text, uint8_t size = 1){
  d.setTextSize(size); d.setCursor(x,y); d.print(text);
}
}

void KickMixer::begin(SendCC send, void* context){
  midi_.send = send; midi_.context = context;
  if (midi_.initialized) return;
  midi_.initialized = true;
  snapshot();
  flushMidi();
}
void KickMixer::queue(uint8_t cc, uint8_t value){
  for (uint8_t i = 0; i < CONTROL_COUNT; ++i){
    if (CC[i] != cc) continue;
    midi_.value[i] = value;
    midi_.pending[i] = true;
    return;
  }
}
void KickMixer::flushMidi(){
  if (!midi_.send) return;
  for (uint8_t i = 0; i < CONTROL_COUNT; ++i){
    if (!midi_.pending[i]) continue;
    if (!midi_.send(midi_.context, CC[i], midi_.value[i])) return;
    midi_.pending[i] = false;
  }
}
void KickMixer::snapshot(){
  for (uint8_t c = 0; c < CONTROL_COUNT; ++c) queue(CC[c], value_[c]);
}
void KickMixer::saveTo(SavedState& out) const {
  for (uint8_t c = 0; c < CONTROL_COUNT; ++c) out.value[c] = value_[c];
}
void KickMixer::restoreFrom(const SavedState& in){
  for (uint8_t c = 0; c < CONTROL_COUNT; ++c)
    value_[c] = in.value[c] <= 127 ? in.value[c] : value_[c];
  // The Daisy keeps no state of its own, so a restore re-sends the whole page
  // through the pending array; service() retries whatever the UART refused.
  snapshot();
  flushMidi();
  dirty_ = true;
}
void KickMixer::setActive(bool active){
  if (active_ == active) return;
  active_ = active;
  for (auto& down : buttonDown_) down = false;
  if (active){
    // Values are session state; only the angle reference is re-seeded.
    for (auto& pot : physical_) pot.initialized = false;
    dirty_ = true;
    rendered_ = false;
  }
}
void KickMixer::focusControl(uint8_t knob){
  focusKnob_ = knob;
  dirty_ = true;
}
void KickMixer::buttonEdge(uint8_t button, bool pressed, uint32_t now){
  (void)now;
  if (!active_ || button >= 6) return;
  if (pressed == buttonDown_[button]) return;
  buttonDown_[button] = pressed;
  if (!pressed) return;
  // No pot button carries a mix function; they only move the sticky focus.
  focusControl(button);
}
void KickMixer::service(uint32_t now){
  (void)now;
  flushMidi();
}
void KickMixer::sampleAngle(uint8_t knob, float angle){
  if (!active_ || knob >= 6) return;
  PhysicalPot& pot = physical_[knob];
  if (!pot.initialized){
    pot.initialized = true;
    pot.angle = angle;
    pot.accumulated = 0;
    return; // The current physical angle is only a movement reference.
  }
  float delta = angle - pot.angle;
  pot.angle = angle;
  if (delta > PI_F) delta -= 2.f * PI_F;
  if (delta < -PI_F) delta += 2.f * PI_F;
  // Clockwise increases; a full revolution spans 128 value units. Unwrap
  // angle differences so crossing the atan2 seam continues in the same direction.
  float movement = -delta * (128.f / (2.f * PI_F));
  uint8_t value = value_[knob];
  // At a limit discard any outward fractional remainder on reversal, so
  // reversing responds without unwinding overshoot.
  if ((value == 127 && movement < 0 && pot.accumulated > 0) ||
      (value == 0 && movement > 0 && pot.accumulated < 0))
    pot.accumulated = 0;
  pot.accumulated += movement;
  // Net movement deadband rejects ADC jitter without a low-pass filter's
  // delayed direction reversal. Small deliberate movements accumulate.
  if (fabsf(pot.accumulated) < 2.f) return;
  int ticks = (int)pot.accumulated;
  pot.accumulated -= ticks;
  adjust(knob,ticks);
  value = value_[knob];
  if (value == 0 || value == 127) pot.accumulated = 0;
}
void KickMixer::adjust(uint8_t knob, int delta){
  if (!active_ || knob >= 6 || delta == 0) return;
  focusControl(knob);
  uint8_t current = value_[knob];
  uint8_t next = (uint8_t)constrain((int)current + delta,0,127);
  if (next == current) return; // No wrap and no repeated MIDI at the limits.
  value_[knob] = next;
  queue(CC[knob],next);
  flushMidi();
}

uint8_t KickMixer::percent(uint8_t v){ return ((unsigned)v * 100 + 63) / 127; }

void KickMixer::drawOverview(Adafruit_SH1106G& d){
  d.clearDisplay(); d.setTextWrap(false); d.setTextColor(SH110X_WHITE);
  for (uint8_t k = 0; k < 6; ++k){
    int x = (k % 3) * 43, y = (k / 3) * 32;
    char value[12];
    snprintf(value,sizeof(value),"%u%%",percent(value_[k]));
    if (focusKnob_ == k) d.drawRect(x,y,k % 3 == 2 ? 42 : 43,32,SH110X_WHITE);
    label(d,x+3,y+3,SHORT_NAMES[k]); label(d,x+3,y+13,value);
  }
  d.setTextWrap(true); d.display();
}

void KickMixer::drawFocus(Adafruit_SH1106G& d){
  d.clearDisplay(); d.setTextWrap(false); d.setTextColor(SH110X_WHITE);
  label(d,0,0,"MIX");
  uint8_t v = value_[focusKnob_];
  char value[16];
  snprintf(value,sizeof(value),"%u%%",percent(v));
  label(d,0,13,LONG_NAMES[focusKnob_]);
  label(d,0,26,value,3);
  d.drawRect(LEFT,52,RIGHT-LEFT+1,10,SH110X_WHITE);
  int w = (RIGHT-LEFT-1)*v/127;
  if (w) d.fillRect(LEFT+1,53,w,8,SH110X_WHITE);
  d.setTextWrap(true); d.display();
}

void KickMixer::render(Adafruit_SH1106G& overview, Adafruit_SH1106G* focus, uint32_t now){
  if (!active_) return;
  if (!dirty_ || (rendered_ && (uint32_t)(now-lastFrameMs_) < 40)) return;
  drawOverview(overview);
  if (focus) drawFocus(*focus);
  dirty_ = false; rendered_ = true; lastFrameMs_ = now;
}
