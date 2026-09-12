#include "KickPerformance.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {
constexpr uint8_t CC[] = {30,31,32,33,34,35,40,41,42,43,44,45,46,47,48,49,50,51,52};
constexpr uint8_t PARAM_CC[] = {30,31,32,33,34,35,40,42,44,45,46,48,49,51};
constexpr uint8_t STUT_VALUES[] = {10,28,46,64,82,100,118};
constexpr uint8_t LOOP_VALUES[] = {13,38,63,88,114};
constexpr uint8_t COUNT_VALUES[] = {0,42,85,127};
const char* const FX_NAMES[] = {"STUT","LOOP","DLY","HPF","LPF","PUMP"};
constexpr float PI_F = 3.14159265358979323846f;
constexpr int LEFT = 4, RIGHT = 123, GRAPH_TOP = 36, GRAPH_BOTTOM = 48;
int marker(uint8_t value){ return LEFT + (int)value * (RIGHT - LEFT) / 127; }
void label(Adafruit_SH1106G& d, int x, int y, const char* text, uint8_t size = 1){
  d.setTextSize(size); d.setCursor(x,y); d.print(text);
}
}

void KickPerformance::begin(SendCC send, void* context){
  midi_.send = send; midi_.context = context;
  if (midi_.initialized) return;
  midi_.initialized = true;
  snapshot();
  flushMidi();
}

KickPerformance::Parameter KickPerformance::assignment(uint8_t knob) const {
  switch (knob){
    case 0: return (Parameter)state_.selectedFx;
    case 1: return DECAY;
    case 2: return TAIL;
    case 3: return (Parameter)(BPF1 + state_.editedBpfLayer);
    case 4: return state_.selectedCharacterModel ? SHERMAN : MACKIE;
    default: return SHAPE;
  }
}
uint8_t& KickPerformance::position(Parameter p){
  switch (p){
    case STUT: return state_.stutter.position;
    case LOOP: return state_.looper.position;
    case DELAY: return state_.delay;
    case HPF: return state_.hpf;
    case LPF: return state_.lpf;
    case PUMP: return state_.pumpAmount;
    case DECAY: return state_.decay;
    case TAIL: return state_.tailDelayAmount;
    case BPF1: case BPF2: case BPF3: return state_.bpfFrequencyValue[p - BPF1];
    case MACKIE: return state_.mackieAmount;
    case SHERMAN: return state_.shermanAmount;
    default: return state_.kickShape;
  }
}
const uint8_t& KickPerformance::position(Parameter p) const {
  return const_cast<KickPerformance*>(this)->position(p);
}
uint8_t KickPerformance::outputValue(Parameter p) const {
  if (p == STUT) return state_.stutter.on ? STUT_VALUES[state_.stutter.division] : 0;
  if (p == LOOP) return state_.looper.on ? LOOP_VALUES[state_.looper.division] : 0;
  return position(p);
}
void KickPerformance::queue(uint8_t cc, uint8_t value){
  for (uint8_t i = 0; i < 19; ++i){
    if (CC[i] != cc) continue;
    midi_.value[i] = value;
    midi_.pending[i] = true;
    return;
  }
}
void KickPerformance::flushMidi(){
  if (!midi_.send) return;
  for (uint8_t i = 0; i < 19; ++i){
    if (!midi_.pending[i]) continue;
    if (!midi_.send(midi_.context, CC[i], midi_.value[i])) return;
    midi_.pending[i] = false;
  }
}
void KickPerformance::snapshot(){
  for (uint8_t p = 0; p < PARAM_COUNT; ++p) queue(PARAM_CC[p], outputValue((Parameter)p));
  queue(41, state_.reverseEnabled ? 127 : 0);
  queue(43, state_.tailDelayEnabled ? 127 : 0);
  queue(47, COUNT_VALUES[state_.bpfLayerCount]);
  queue(50, state_.selectedCharacterModel ? 127 : 0);
  queue(52, state_.pumpEnabled ? 127 : 0);
}
void KickPerformance::setActive(bool active){
  if (active_ == active) return;
  active_ = active;
  // Leaving a mode is not a reset. Cancel unfinished local button gestures.
  for (auto& button : buttons_) button = ButtonState{};
  if (active){
    for (auto& pot : physical_) pot.initialized = false;
    dirty_ = true;
    rendered_ = false;
  }
}
void KickPerformance::focusControl(uint8_t knob){
  focus_.knob = knob;
  // A new deliberate action may dismiss an older transient status.
  focus_.resetOverlay = false;
  dirty_ = true;
}
void KickPerformance::reassign(uint8_t knob){
  // Seed a fresh angle on the next scan; never carry motion into a new target.
  physical_[knob] = PhysicalPot{};
  focusControl(knob);
}
void KickPerformance::buttonEdge(uint8_t button, bool pressed, uint32_t now){
  if (!active_ || button >= 6) return;
  ButtonState& b = buttons_[button];
  if (pressed == b.down) return;
  if (pressed){
    b.down = true; b.longFired = false; b.pressedMs = now;
    focusControl(button);
    if (button == 0) return;
    switch (button){
      case 1:
        state_.reverseEnabled = !state_.reverseEnabled;
        queue(41, state_.reverseEnabled ? 127 : 0); break;
      case 2:
        state_.tailDelayEnabled = !state_.tailDelayEnabled;
        queue(43, state_.tailDelayEnabled ? 127 : 0); break;
      case 3: {
        uint8_t previous = state_.editedBpfLayer;
        state_.bpfLayerCount = (state_.bpfLayerCount + 1) % 4;
        state_.editedBpfLayer = state_.bpfLayerCount ? state_.bpfLayerCount - 1 : 0;
        queue(47, COUNT_VALUES[state_.bpfLayerCount]);
        if (previous != state_.editedBpfLayer) reassign(3);
        break;
      }
      case 4:
        state_.selectedCharacterModel = !state_.selectedCharacterModel;
        queue(50, state_.selectedCharacterModel ? 127 : 0);
        reassign(4); break;
      case 5:
        state_.pumpEnabled = !state_.pumpEnabled;
        queue(52, state_.pumpEnabled ? 127 : 0); break;
    }
  } else {
    if (button == 0 && !b.longFired){
      if ((uint32_t)(now - b.pressedMs) >= 500){
        b.longFired = true; resetFx(now);
      } else {
        state_.selectedFx = (state_.selectedFx + 1) % 6;
        reassign(0);
      }
    }
    b.down = false;
  }
  flushMidi();
}
void KickPerformance::resetFx(uint32_t now){
  int8_t stutPrevious = state_.stutter.previousStart;
  int8_t loopPrevious = state_.looper.previousStart;
  state_.stutter = RepeatState{}; state_.looper = RepeatState{};
  state_.stutter.previousStart = stutPrevious;
  state_.looper.previousStart = loopPrevious;
  state_.delay = state_.hpf = state_.lpf = 0;
  for (uint8_t p = STUT; p <= LPF; ++p){
    queue(PARAM_CC[p], 0);
  }
  physical_[0] = PhysicalPot{};
  focus_.knob = 0;
  focus_.resetOverlay = true; focus_.resetMs = now;
  dirty_ = true;
}
void KickPerformance::service(uint32_t now){
  if (active_ && buttons_[0].down && !buttons_[0].longFired &&
      (uint32_t)(now - buttons_[0].pressedMs) >= 500){
    buttons_[0].longFired = true;
    resetFx(now);
  }
  if (focus_.resetOverlay && (uint32_t)(now - focus_.resetMs) >= 700){
    focus_.resetOverlay = false; dirty_ = true;
  }
  flushMidi();
}

void KickPerformance::sampleAngle(uint8_t knob, float angle){
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
  if (knob == 0 && buttons_[0].down){
    pot.accumulated = 0;
    return;
  }
  uint8_t value = position(assignment(knob));
  // At a limit discard any outward fractional remainder on reversal. There
  // is no accumulated overshoot to unwind, even after many extra revolutions.
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
  value = position(assignment(knob));
  if (value == 0 || value == 127) pot.accumulated = 0;
}
void KickPerformance::adjust(uint8_t knob, int delta){
  if (!active_ || knob >= 6 || delta == 0) return;
  focusControl(knob);
  Parameter p = assignment(knob);
  uint8_t current = position(p);
  uint8_t next = (uint8_t)constrain((int)current + delta,0,127);
  if (next == current) return; // No wrap and no repeated MIDI at the limits.
  if (p == STUT) updateRepeat(state_.stutter,false,next);
  else if (p == LOOP) updateRepeat(state_.looper,true,next);
  else {
    position(p) = next;
    queue(PARAM_CC[p],next);
  }
  flushMidi();
}
uint8_t KickPerformance::randomStart(bool loop, int8_t previous){
  static const uint8_t stutWeights[] = {50,25,12,7,3,2,1};
  static const uint8_t loopWeights[] = {55,25,12,6,2};
  const uint8_t* weights = loop ? loopWeights : stutWeights;
  uint8_t count = loop ? 5 : 7, pick = 0;
  for (uint8_t attempt = 0; attempt < 2; ++attempt){
    long draw = random(0,100);
    for (pick = 0; pick + 1 < count; ++pick){
      if (draw < weights[pick]) break;
      draw -= weights[pick];
    }
    if ((int8_t)pick != previous) return pick;
  }
  return pick + 1 < count ? pick + 1 : pick - 1;
}
void KickPerformance::updateRepeat(RepeatState& r, bool loop, uint8_t value){
  uint8_t cc = loop ? 31 : 30;
  const uint8_t* canonical = loop ? LOOP_VALUES : STUT_VALUES;
  r.position = value;
  if (value <= 3){
    bool wasOn = r.on;
    int8_t previous = r.previousStart;
    r = RepeatState{}; r.position = value; r.previousStart = previous;
    if (wasOn) queue(cc,0);
    return;
  }
  if (!r.on){
    r.randomStart = randomStart(loop, r.previousStart);
    r.previousStart = r.randomStart;
    r.division = r.randomStart;
    r.direction = r.randomStart <= 3 ? 1 : -1;
    r.activationPosition = value; r.on = true;
    queue(cc,canonical[r.division]);
    return;
  }
  int last = loop ? 4 : 6;
  int steps = r.direction > 0 ? last - r.randomStart : r.randomStart;
  int range = 127 - r.activationPosition;
  if (steps == 0 || range <= 0) return;
  int offset = ((int)r.division - r.randomStart) * r.direction;
  int original = offset;
  // Two-count Schmitt hysteresis around each evenly spaced boundary.
  while (offset < steps){
    int boundary = r.activationPosition + ((offset + 1) * range + steps - 1) / steps;
    int threshold = boundary + 2;
    if (threshold > 127) threshold = 127;
    if (value < threshold) break;
    ++offset;
  }
  while (offset > 0){
    int boundary = r.activationPosition + (offset * range + steps - 1) / steps;
    if ((int)value > boundary - 2) break;
    --offset;
  }
  if (offset != original){
    r.division = r.randomStart + r.direction * offset;
    queue(cc,canonical[r.division]);
  }
}

uint8_t KickPerformance::percent(uint8_t v){ return ((unsigned)v * 100 + 63) / 127; }
float KickPerformance::smoothstep(float x){ return x*x*(3.f-2.f*x); }
float KickPerformance::frequency(Parameter p, uint8_t v){
  float x = v / 127.f;
  if (p == HPF) return 30.f * powf(11500.f/30.f,x);
  if (p == LPF) return 18000.f * powf(120.f/18000.f,x);
  return 140.f * powf(3200.f/140.f,x);
}
float KickPerformance::releaseMs(uint8_t v){ return 8.f * powf(375.f,v/127.f); }
void KickPerformance::shapeValues(uint8_t v, float& ratio, float& seconds){
  float x = v/127.f;
  if (x <= 0.5f){
    float t = smoothstep(x*2.f);
    ratio = 12.f + (5.7f-12.f)*t;
    seconds = 0.008f + (0.069f-0.008f)*t;
  } else {
    float t = smoothstep((x-0.5f)*2.f);
    ratio = 5.7f + (1.05f-5.7f)*t;
    seconds = 0.069f + (0.320f-0.069f)*t;
  }
}
void KickPerformance::frequencyText(char* out, size_t size, float hz, bool compact){
  if (hz < 1000) snprintf(out,size,compact ? "%.0fH" : "%.0f Hz",hz);
  else snprintf(out,size,compact ? "%.1fK" : "%.1f kHz",hz/1000.f);
}
void KickPerformance::valueText(Parameter p, char* out, size_t size, bool compact) const {
  uint8_t v = position(p);
  if (p == STUT || p == LOOP){
    const RepeatState& r = p == STUT ? state_.stutter : state_.looper;
    if (!r.on) snprintf(out,size,"OFF");
    else snprintf(out,size,"1/%u",2u << r.division);
  } else if (p == HPF || p == LPF){
    if (v == 0) snprintf(out,size,"OFF");
    else frequencyText(out,size,frequency(p,v),compact);
  } else if (p == DECAY){
    if (v >= 126) snprintf(out,size,"INF");
    else {
      float ms = releaseMs(v);
      if (ms < 1000) snprintf(out,size,compact ? "%.0fM" : "%.0f ms",ms);
      else snprintf(out,size,compact ? "%.1fS" : "%.1f s",ms/1000.f);
    }
  } else if (p == TAIL) snprintf(out,size,compact ? "%.1fS" : "%.2f STEP",2.f*v/127.f);
  else if (p >= BPF1 && p <= BPF3) frequencyText(out,size,frequency(p,v),compact);
  else if (p == DELAY && v == 0) snprintf(out,size,"OFF");
  else snprintf(out,size,"%u%%",percent(v));
}

void KickPerformance::drawOverview(Adafruit_SH1106G& d){
  d.clearDisplay(); d.setTextWrap(false); d.setTextColor(SH110X_WHITE);
  for (uint8_t k = 0; k < 6; ++k){
    int x = (k % 3) * 43, y = (k / 3) * 32;
    Parameter p = assignment(k);
    char title[12], value[16], secondary[12] = {};
    if (k == 0) snprintf(title,sizeof(title),"%s",FX_NAMES[state_.selectedFx]);
    else if (k == 1){
      snprintf(title,sizeof(title),"DECAY");
      snprintf(secondary,sizeof(secondary),"R %s",state_.reverseEnabled ? "ON" : "OFF");
    } else if (k == 2){
      snprintf(title,sizeof(title),"TAIL");
      snprintf(secondary,sizeof(secondary),"%s",state_.tailDelayEnabled ? "ON" : "OFF");
    } else if (k == 3){
      snprintf(title,sizeof(title),"BPF %u",state_.bpfLayerCount);
      snprintf(secondary,sizeof(secondary),"%sL%u",state_.bpfLayerCount ? "EDIT " : "NXT ",state_.editedBpfLayer+1);
    } else if (k == 4) snprintf(title,sizeof(title),"%s",state_.selectedCharacterModel ? "SHRM" : "MACK");
    else {
      snprintf(title,sizeof(title),"SHAPE");
      snprintf(secondary,sizeof(secondary),"P %s",state_.pumpEnabled ? "ON" : "OFF");
    }
    valueText(p,value,sizeof(value),true);
    if (focus_.knob == k) d.drawRect(x,y,k % 3 == 2 ? 42 : 43,32,SH110X_WHITE);
    label(d,x+3,y+3,title); label(d,x+3,y+13,value); label(d,x+3,y+23,secondary);
  }
  d.setTextWrap(true); d.display();
}

void KickPerformance::drawVisualization(Adafruit_SH1106G& d, Parameter p){
  uint8_t v = position(p);
  float x = v/127.f;
  if (p == STUT || p == LOOP){
    const RepeatState& r = p == STUT ? state_.stutter : state_.looper;
    uint8_t count = p == STUT ? 7 : 5;
    for (uint8_t i = 0; i < count; ++i){
      int tx = 2 + (i % 4)*32, ty = 33 + (i / 4)*9;
      char text[8]; snprintf(text,sizeof(text),"1/%u",2u << i);
      if (r.on && r.division == i){
        d.fillRect(tx,ty,31,8,SH110X_WHITE); d.setTextColor(SH110X_BLACK);
      }
      label(d,tx,ty,text); d.setTextColor(SH110X_WHITE);
    }
    return;
  }
  if (p >= BPF1 && p <= BPF3){
    // All bands share the same logarithmic x-axis. Separate label lanes keep
    // coincident stored frequencies readable on a monochrome panel.
    d.drawFastHLine(LEFT,GRAPH_BOTTOM,RIGHT-LEFT+1,SH110X_WHITE);
    for (uint8_t i = 0; i < 3; ++i){
      int mx = marker(state_.bpfFrequencyValue[i]), y = 25+i*8;
      bool enabled = i < state_.bpfLayerCount;
      for (int py = y+7; py < GRAPH_BOTTOM; py += enabled ? 1 : 3)
        d.drawPixel(mx,py,SH110X_WHITE);
      char text[4]; snprintf(text,sizeof(text),"L%u",i+1);
      int tx = constrain(mx-6,3,113);
      if (i == state_.editedBpfLayer) d.drawRect(tx-1,y-1,14,9,SH110X_WHITE);
      if (enabled){
        d.fillRect(tx,y,12,7,SH110X_WHITE); d.setTextColor(SH110X_BLACK);
      }
      label(d,tx,y,text); d.setTextColor(SH110X_WHITE);
    }
    return;
  }
  if (p == HPF || p == LPF){
    int cutoff = p == HPF ? marker(v) : marker(127-v);
    // Zero is bypass; LPF's filled spectrum shrinks toward the left.
    if (v == 0) cutoff = p == HPF ? LEFT : RIGHT;
    for (int px = LEFT; px <= RIGHT; px += 4){
      bool pass = p == HPF ? px >= cutoff : px <= cutoff;
      if (pass) d.drawFastVLine(px,GRAPH_TOP+3,12,SH110X_WHITE);
      else d.drawPixel(px,GRAPH_BOTTOM,SH110X_WHITE);
    }
    d.drawFastHLine(LEFT,GRAPH_BOTTOM,RIGHT-LEFT+1,SH110X_WHITE);
    d.fillTriangle(cutoff-2,GRAPH_TOP,cutoff+2,GRAPH_TOP,cutoff,GRAPH_TOP+3,SH110X_WHITE);
    return;
  }
  if (p == DELAY || p == MACKIE || p == SHERMAN){
    d.drawRect(LEFT,GRAPH_TOP+4,RIGHT-LEFT+1,10,SH110X_WHITE);
    int w = (RIGHT-LEFT-2)*v/127;
    if (w) d.fillRect(LEFT+1,GRAPH_TOP+5,w,8,SH110X_WHITE);
    if (p != DELAY){
      const uint8_t marks[] = {25,48,73};
      for (uint8_t m : marks){
        int px = LEFT+(RIGHT-LEFT)*m/100;
        d.drawFastVLine(px,GRAPH_TOP+2,14,SH110X_WHITE);
        d.drawFastVLine(px,GRAPH_TOP+5,8,SH110X_BLACK);
      }
    }
    return;
  }
  int previousY = GRAPH_BOTTOM;
  for (int px = LEFT; px <= RIGHT; ++px){
    float t = (float)(px-LEFT)/(RIGHT-LEFT), amplitude = 0;
    if (p == PUMP){
      float depth = v ? (22.f+74.f*powf(x,0.82f))/100.f : 0.f;
      float dip = t < .15f ? 0 : t < .23f ? (t-.15f)/.08f : expf(-(t-.23f)*6.f);
      amplitude = 1.f-depth*dip;
    } else if (p == DECAY){
      amplitude = v >= 126 ? .9f : expf(-t/(.025f+.7f*x*x));
    } else if (p == TAIL){
      float start = .08f+.65f*x;
      amplitude = t < .015f ? 1.f : t < start ? 0.f : .8f*expf(-(t-start)*12.f);
    } else if (p == SHAPE){
      float ratio,seconds; shapeValues(v,ratio,seconds);
      // Keep the smallest displacement visible on the 13-pixel plot.
      float height = .17f + .83f*(ratio-1.f)/11.f;
      amplitude = height*expf(-t/(.025f+seconds*2.f));
    }
    int y = GRAPH_BOTTOM-(int)(amplitude*(GRAPH_BOTTOM-GRAPH_TOP));
    if (px > LEFT) d.drawLine(px-1,previousY,px,y,SH110X_WHITE);
    previousY = y;
  }
}

void KickPerformance::drawFocus(Adafruit_SH1106G& d, uint32_t bpm){
  d.clearDisplay(); d.setTextWrap(false); d.setTextColor(SH110X_WHITE);
  if (focus_.resetOverlay){
    label(d,16,19,"FX RESET",2); label(d,13,45,"STUT..LPF OFF");
    d.setTextWrap(true); d.display(); return;
  }
  Parameter p = assignment(focus_.knob);
  uint8_t v = position(p); float x = v/127.f;
  char title[22], value[22], footer[22] = {}, extra[22] = {};
  if (p <= PUMP) snprintf(title,sizeof(title),"%s",FX_NAMES[p]);
  else if (p == DECAY) snprintf(title,sizeof(title),"DECAY");
  else if (p == TAIL) snprintf(title,sizeof(title),"TAIL DELAY");
  else if (p <= BPF3) snprintf(title,sizeof(title),"BPF %u EDIT L%u",state_.bpfLayerCount,state_.editedBpfLayer+1);
  else if (p == MACKIE || p == SHERMAN) snprintf(title,sizeof(title),"%s",p == MACKIE ? "MACKIE" : "SHERMAN");
  else snprintf(title,sizeof(title),"SHAPE");
  valueText(p,value,sizeof(value),false);
  if (p == STUT || p == LOOP){
    const RepeatState& r = p == STUT ? state_.stutter : state_.looper;
    if (r.on) snprintf(footer,sizeof(footer),"START 1/%u > %s",2u << r.randomStart,r.direction > 0 ? "FAST" : "SLOW");
    else snprintf(footer,sizeof(footer),"UP = RANDOM RATE");
  } else if (p == DELAY){
    unsigned wet = v ? (unsigned)(10.f+66.f*smoothstep(x)+.5f) : 0;
    unsigned fb = v ? (unsigned)(18.f+60.f*powf(x,1.25f)+.5f) : 0;
    snprintf(extra,sizeof(extra),"3/8"); snprintf(footer,sizeof(footer),"WET %u%%  FB %u%%",wet,fb);
  } else if (p == HPF || p == LPF){
    snprintf(footer,sizeof(footer),p == HPF ? "30H - 11.5K   %u%%" : "120H - 18K    %u%%",percent(v));
  } else if (p == PUMP){
    unsigned depth = v ? (unsigned)(22.f+74.f*powf(x,.82f)+.5f) : 0;
    snprintf(footer,sizeof(footer),"DEPTH %u%%  %s",depth,state_.pumpEnabled ? "ON" : "OFF");
  } else if (p == DECAY) snprintf(footer,sizeof(footer),"REV %s",state_.reverseEnabled ? "ON" : "OFF");
  else if (p == TAIL){
    unsigned ms = (unsigned)(60000.f/(bpm ? bpm : 120)*.5f*x+.5f);
    snprintf(footer,sizeof(footer),"%u ms  %s",ms,state_.tailDelayEnabled ? "ON" : "OFF");
  } else if (p <= BPF3) snprintf(footer,sizeof(footer),"140Hz ----- 3.2kHz");
  else if (p == MACKIE || p == SHERMAN){
    unsigned pct = percent(v);
    snprintf(extra,sizeof(extra),"%s",v == 0 ? "DRY" : pct <= 25 ? "BASE DRIVE" : pct <= 48 ? "MID I" : pct < 73 ? "MID II" : "MID III");
    snprintf(footer,sizeof(footer),"%s %u%%",p == MACKIE ? "SHERMAN" : "MACKIE",percent(p == MACKIE ? state_.shermanAmount : state_.mackieAmount));
  } else {
    float ratio,seconds; shapeValues(v,ratio,seconds);
    snprintf(extra,sizeof(extra),"%.2fx  %.0f ms",ratio,seconds*1000.f);
    snprintf(footer,sizeof(footer),"PUMP %s",state_.pumpEnabled ? "ON" : "OFF");
  }
  label(d,0,0,title);
  label(d,0,11,value,(p >= BPF1 && p <= BPF3) || strlen(value) > 10 ? 1 : 2);
  if (extra[0]) label(d,2,28,extra);
  drawVisualization(d,p);
  label(d,2,56,footer);
  d.setTextWrap(true); d.display();
}

void KickPerformance::render(Adafruit_SH1106G& overview, Adafruit_SH1106G* focus,
                             uint32_t bpm, uint32_t now){
  if (!active_) return;
  if (focus_.knob == 2 && bpm != renderedBpm_) dirty_ = true;
  if (!dirty_ || (rendered_ && (uint32_t)(now-lastFrameMs_) < 40)) return;
  drawOverview(overview);
  if (focus) drawFocus(*focus,bpm);
  dirty_ = false; rendered_ = true; lastFrameMs_ = now; renderedBpm_ = bpm;
}
