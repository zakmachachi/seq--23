#include "KickMixer.h"
#include <stdio.h>

namespace {
constexpr uint8_t CC_EXT_KICK_MODE = 59;
void label(Adafruit_SH1106G& d, int x, int y, const char* text, uint8_t size = 1){
  d.setTextSize(size); d.setCursor(x,y); d.print(text);
}
}

void KickMixer::begin(SendCC send, void* context){
  send_ = send;
  context_ = context;
  sendSource();
  sendOutputGain();
}

void KickMixer::sendSource(){
  if (send_) send_(context_, CC_EXT_KICK_MODE, externalKickMode_ ? 127 : 0);
}

void KickMixer::sendOutputGain(){
  gainPending_ = true;
  if (send_ && send_(context_, 53, legacy_values_[LINE])) gainPending_ = false;
}

void KickMixer::saveTo(SavedState& out) const {
  for (uint8_t i=0; i<CONTROL_COUNT; ++i) out.value[i] = legacy_values_[i];
}

void KickMixer::restoreFrom(const SavedState& in){
  for (uint8_t i=0; i<CONTROL_COUNT; ++i)
    if (in.value[i] <= 127) legacy_values_[i] = in.value[i];
  sendOutputGain();
  dirty_ = true;
}

void KickMixer::setActive(bool active){
  if (active_ == active) return;
  active_ = active;
  dirty_ = true;
  rendered_ = false;
  potInitialized_ = false;
  potAccumulated_ = 0.f;
  if (active_){ sendSource(); sendOutputGain(); }
}

void KickMixer::sampleAngle(uint8_t knob, float angle){
  if (!active_ || knob != 0) return;
  if (!potInitialized_){
    potInitialized_ = true; potAngle_ = angle; potAccumulated_ = 0.f; return;
  }
  float delta = angle-potAngle_; potAngle_ = angle;
  if (delta > 3.14159265f) delta -= 6.2831853f;
  if (delta < -3.14159265f) delta += 6.2831853f;
  if (buttonDown_){ potAccumulated_ = 0.f; return; }
  potAccumulated_ += -delta * (128.f/6.2831853f);
  if (fabsf(potAccumulated_) < 2.f) return;
  int ticks = (int)potAccumulated_;
  potAccumulated_ -= ticks;
  int next = (int)legacy_values_[LINE] + ticks;
  if (next < 0) next = 0;
  if (next > 127) next = 127;
  if (next == legacy_values_[LINE]) return;
  legacy_values_[LINE] = (uint8_t)next;
  sendOutputGain();
  dirty_ = true;
}

void KickMixer::buttonEdge(uint8_t button, bool pressed, uint32_t now){
  (void)now;
  if (button == 0) buttonDown_ = pressed;
  if (!active_ || !pressed || button != 0) return;
  externalKickMode_ = !externalKickMode_;
  sendSource();
  dirty_ = true;
}

void KickMixer::service(uint32_t now){
  (void)now;
  if (gainPending_ && send_ && send_(context_, 53, legacy_values_[LINE]))
    gainPending_ = false;
}

void KickMixer::draw(Adafruit_SH1106G& d, bool focus){
  d.clearDisplay();
  d.setTextWrap(false);
  d.setTextColor(SH110X_WHITE);
  const uint8_t cc = legacy_values_[LINE];
  const float db = cc <= 64 ? -15.f*(64.f-cc)/64.f : 15.f*(cc-64.f)/63.f;
  char gain[16]; snprintf(gain,sizeof(gain),"%+.1f dB",db);
  label(d,0,0,"RAW 909 / OUT GAIN");
  label(d,0,16,"EXT KICK");
  label(d,59,16,externalKickMode_ ? "ON" : "OFF");
  label(d,0,29,gain,2);
  label(d,0,53,focus ? "TURN K1: GAIN  PUSH: EXT" : "K1 GAIN / PUSH EXT KICK");
  d.setTextWrap(true);
  d.display();
}

void KickMixer::render(Adafruit_SH1106G& overview, Adafruit_SH1106G* focus, uint32_t now){
  if (!active_) return;
  // Keep full-frame OLED I2C traffic at the same 10 FPS ceiling as the other
  // performance pages.
  if (!dirty_ || (rendered_ && uint32_t(now-lastFrameMs_) < 100)) return;
  draw(overview,false);
  if (focus) draw(*focus,true);
  dirty_ = false;
  rendered_ = true;
  lastFrameMs_ = now;
}
