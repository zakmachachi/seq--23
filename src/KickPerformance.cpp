#include "KickPerformance.h"
#include "KickShapeModel.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {
constexpr uint8_t CC[] = {90,91,92,93,30,31,32,33,34,35,36,40,41,42,43,44,45,46,47,48,49,50,51,52,37,38,39};
constexpr uint8_t CC_SLOTS = sizeof(CC);
constexpr uint8_t PARAM_CC[] = {30,31,32,33,34,35,36,40,42,44,45,46,48,49,51,37,38,39,93};
static_assert(sizeof(PARAM_CC)==KickPerformance::PARAM_COUNT, "Missing parameter CC");
constexpr uint8_t STUT_VALUES[] = {16,48,80,112};
constexpr uint8_t LOOP_VALUES[] = {13,38,63,88,114};
constexpr uint8_t COUNT_VALUES[] = {0,42,85,127};
// The stutter ladder alternates straight and triplet across 1/4..1/8T, so its
// labels can no longer be derived as 2<<index the way the looper's still are.
const char* const STUT_LABELS[] = {"1/4","1/4T","1/8","1/8T"};
constexpr uint8_t STUT_DIVISIONS = sizeof(STUT_LABELS)/sizeof(STUT_LABELS[0]);
constexpr uint8_t LOOP_DIVISIONS = sizeof(LOOP_VALUES);
void divisionLabel(char* out, size_t size, bool loop, uint8_t division){
  if (loop) snprintf(out,size,"1/%u",2u << division);
  else snprintf(out,size,"%s",STUT_LABELS[division]);
}
const char* const FX_NAMES[] = {"STUT","LOOP","DLY","HPF","LPF","PUMP","REV","BITCRUSH","EROSION","PITCH"};
constexpr KickPerformance::Parameter FX_PARAMS[] = {KickPerformance::STUT, KickPerformance::LOOP, KickPerformance::DELAY, KickPerformance::HPF, KickPerformance::LPF, KickPerformance::PUMP, KickPerformance::REVERB, KickPerformance::BITCRUSH, KickPerformance::EROSION, KickPerformance::PITCH};
constexpr uint8_t FX_PAGES = sizeof(FX_NAMES)/sizeof(FX_NAMES[0]);
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
  if(fxMenu_) return knob==5 && functionHeld_ ? EROSION_FREQ : fxAssignment(knob);
  switch (knob){
    case 0: return PARAM_COUNT; // Effects now live on the dedicated FX menu.
    case 1: return DECAY;
    case 2: return TAIL;
    case 3: return (Parameter)(BPF1 + state_.editedBpfLayer);
    case 4: return state_.selectedCharacterModel ? TUBE : MACKIE;
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
    case REVERB: return state_.reverb;
    case BITCRUSH: return bitcrush_;
    case EROSION: return erosion_;
    case EROSION_FREQ: return erosionFrequency_;
    case PITCH: return pitch_;
    case DECAY: return state_.decay;
    case TAIL: return state_.tailDelayAmount;
    case BPF1: case BPF2: case BPF3: return state_.bpfFrequencyValue[p - BPF1];
    case MACKIE: return state_.mackieAmount;
    case TUBE: return state_.tubeAmount;
    default: return state_.kickShape;
  }
}
const uint8_t& KickPerformance::position(Parameter p) const {
  return const_cast<KickPerformance*>(this)->position(p);
}
KickPerformance::Parameter KickPerformance::focusedParameter() const {
  return assignment(focus_.knob);
}
KickPerformance::Parameter KickPerformance::takeUserEditedParameter(){
  Parameter p = (Parameter)userEdited_;
  userEdited_ = PARAM_COUNT;
  return p;
}
uint8_t KickPerformance::parameterValue(Parameter p) const { return position(p); }
void KickPerformance::setParameterValue(Parameter p, uint8_t value){
  if (!parameterIsLaneable(p) || resetPending(p)) return;
  if (value > 127) value = 127;
  if(p==PUMP && value && !state_.pumpEnabled){state_.pumpEnabled=true;queue(52,127);}
  if (position(p) == value) return;
  position(p) = value;
  queue(PARAM_CC[p], value);
  flushMidi();
  dirty_ = true;
}
uint8_t KickPerformance::outputValue(Parameter p) const {
  if (p == STUT) return state_.stutter.on ? STUT_VALUES[state_.stutter.division] : 0;
  if (p == LOOP) return state_.looper.on ? LOOP_VALUES[state_.looper.division] : 0;
  return position(p);
}
void KickPerformance::queue(uint8_t cc, uint8_t value){
  for (uint8_t i = 0; i < CC_SLOTS; ++i){
    if (CC[i] != cc) continue;
    midi_.value[i] = value;
    midi_.pending[i] = true;
    return;
  }
}
void KickPerformance::flushMidi(){
  if (!midi_.send) return;
  for (uint8_t i = 0; i < CC_SLOTS; ++i){
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
  queue(92,fx_.routes);
  queueResetMask();
}
void KickPerformance::saveTo(ControllerState& out) const { out = state_; }
void KickPerformance::restoreFrom(const ControllerState& in){
  resetMask_=0;
  state_ = in;
  // A corrupted or partially written image must not index the canonical rate
  // tables out of bounds, so every restored field is re-bounded here.
  clampRepeat(state_.stutter,STUT_DIVISIONS);
  clampRepeat(state_.looper,LOOP_DIVISIONS);
  if (state_.selectedFx >= FX_PAGES) state_.selectedFx = STUT;
  if (state_.bpfLayerCount > 3) state_.bpfLayerCount = 0;
  if (state_.editedBpfLayer > 2) state_.editedBpfLayer = 0;
  for (uint8_t p = 0; p < PARAM_COUNT; ++p){
    uint8_t& v = position((Parameter)p);
    if (v > 127) v = 0;
  }
  state_.pumpEnabled=state_.pumpAmount>0;
  // The Daisy keeps no state of its own, so a restore re-sends every CC
  // through the pending array; service() retries whatever the UART refused.
  snapshot();
  flushMidi();
  dirty_ = true;
}
void KickPerformance::clampRepeat(RepeatState& r, uint8_t divisions){
  if (r.division >= divisions) r.division = 0;
  if (r.randomStart >= divisions) r.randomStart = 0;
  if (r.previousStart >= (int8_t)divisions) r.previousStart = -1;
  r.direction = r.direction < 0 ? -1 : 1;
  if (r.position > 127) r.position = 0;
  if (r.activationPosition > 127) r.activationPosition = 0;
  if (r.position <= 3) r.on = false;
}
void KickPerformance::setActive(bool active){
  if (active_ == active) return;
  active_ = active;
  // Leaving a mode is not a reset. Cancel unfinished local button gestures.
  for (auto& button : buttons_) button = ButtonState{};
  if (active){
    lastTouchMs_=now_;
    for (auto& pot : physical_) pot.initialized = false;
    dirty_ = true;
    rendered_ = false;
  }
}
void KickPerformance::focusControl(uint8_t knob){
  lastTouchMs_=now_;
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
  now_=now;
  ButtonState& b = buttons_[button];
  if(fxMenu_){
    if(pressed==b.down)return;
    if(pressed){b.down=true;b.longFired=false;b.pressedMs=now;focusControl(button);}
    else {
      if(!b.longFired && uint32_t(now-b.pressedMs)>=1000)requestReset(button);
      else if(!b.longFired){
        if(button<2){
          if(button==0)fx_.repeat=(fx_.repeat+1)%4;else fx_.filter^=1;
          reassign(button);
        }else {
          if(button==3){ // Reverb: EXT -> EXT+INT -> INT -> EXT.
            if(fx_.routes&16)fx_.routes&=~18u;
            else if(fx_.routes&2)fx_.routes|=16u;
            else fx_.routes|=2u;
          }else fx_.routes^=1u<<(button-2);
          queue(92,fx_.routes);
        }
      }
      b.down=false;dirty_=true;
    }
    flushMidi();return;
  }
  if(button==0)return;

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
        if(assignment(3)==EROSION_FREQ)break;
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
      case 5: break;
    }
  } else {
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
  state_.delay = state_.hpf = state_.lpf = state_.reverb = 0;
  for (uint8_t p = STUT; p <= LPF; ++p){
    queue(PARAM_CC[p], 0);
  }
  // PUMP sits between LPF and REVERB in page order but keeps its amount/enable.
  queue(PARAM_CC[REVERB], 0);
  bitcrush_ = 0; queue(PARAM_CC[BITCRUSH], 0);
  erosion_=0;queue(PARAM_CC[EROSION],0);
  physical_[0] = PhysicalPot{};
  focus_.knob = 0;
  focus_.resetOverlay = true; focus_.resetMs = now;
  dirty_ = true;
}
void KickPerformance::service(uint32_t now){
  now_=now;
  if(active_ && fxMenu_)for(uint8_t k=0;k<6;++k){
    if(buttons_[k].down && !buttons_[k].longFired && uint32_t(now-buttons_[k].pressedMs)>=1000){
      requestReset(k);buttons_[k].longFired=true;
    }
  }
  if (focus_.resetOverlay && (uint32_t)(now - focus_.resetMs) >= 700){
    focus_.resetOverlay = false; dirty_ = true;
  }
  flushMidi();
}

void KickPerformance::sampleAngle(uint8_t knob, float angle){
  if (!active_ || knob >= 6 || assignment(knob)==PARAM_COUNT) return;
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
  if (buttons_[knob].down){
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
  if (!active_ || knob >= 6 || delta == 0 || assignment(knob)==PARAM_COUNT) return;
  focusControl(knob);
  Parameter p = assignment(knob);
  if(resetPending(p)){
    for(uint8_t i=0;i<FX_PAGES;++i)if(FX_PARAMS[i]==p)resetMask_&=~(1u<<i);
    queueResetMask();
  }
  uint8_t current = position(p);
  uint8_t next = (uint8_t)constrain((int)current + delta,0,127);
  if (next == current) return; // No wrap and no repeated MIDI at the limits.
  userEdited_ = (uint8_t)p; // Physical move only; lane playback must not set this.
  if (p == STUT) updateRepeat(state_.stutter,false,next);
  else if (p == LOOP) updateRepeat(state_.looper,true,next);
  else {
    position(p) = next;
    queue(PARAM_CC[p],next);
    if(p==PUMP){state_.pumpEnabled=true;queue(52,127);}
  }
  flushMidi();
}
uint8_t KickPerformance::randomStart(int8_t previous){
  static const uint8_t weights[] = {55,25,12,6,2};
  uint8_t count = LOOP_DIVISIONS, pick = 0;
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
    // Stutter always opens at the slowest division and sweeps up; only the
    // looper still draws a weighted random starting rate.
    r.randomStart = loop ? randomStart(r.previousStart) : 0;
    r.previousStart = r.randomStart;
    r.division = r.randomStart;
    r.direction = r.randomStart <= 3 ? 1 : -1;
    r.activationPosition = value; r.on = true;
    queue(cc,canonical[r.division]);
    return;
  }
  int last = (loop ? LOOP_DIVISIONS : STUT_DIVISIONS) - 1;
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
  // Must track MACRO_BPF_LOW_HZ / MACRO_BPF_HIGH_HZ in the Daisy firmware,
  // or the displayed frequency is not the one being filtered.
  return kickdaisy::bpfHz(v);
}
// The Daisy's MacroDecaySeconds: the body is ~30 dB down at this time.
float KickPerformance::releaseMs(uint8_t v){ return kickdaisy::decaySeconds(v) * 1000.f; }
void KickPerformance::frequencyText(char* out, size_t size, float hz, bool compact){
  if (hz < 1000) snprintf(out,size,compact ? "%.0fH" : "%.0f Hz",hz);
  else snprintf(out,size,compact ? "%.1fK" : "%.1f kHz",hz/1000.f);
}
void KickPerformance::valueText(Parameter p, char* out, size_t size, bool compact) const {
  uint8_t v = position(p);
  if(p==PITCH){snprintf(out,size,"%+.1f%s",12.f*(int(v)-64)/(v<64?64.f:63.f),compact?"S":" ST");return;}
  if (p == STUT || p == LOOP){
    const RepeatState& r = p == STUT ? state_.stutter : state_.looper;
    if (!r.on) snprintf(out,size,"OFF");
    else divisionLabel(out,size,p == LOOP,r.division);
  } else if (p == HPF || p == LPF){
    if (v == 0) snprintf(out,size,"OFF");
    else frequencyText(out,size,frequency(p,v),compact);
  } else if (p == DECAY){
    float ms = releaseMs(v);
    if (ms < 1000) snprintf(out,size,compact ? "%.0fM" : "%.0f ms",ms);
    else snprintf(out,size,compact ? "%.1fS" : "%.1f s",ms/1000.f);
  } else if (p == TAIL) snprintf(out,size,compact ? "%.1fS" : "%.2f STEP",2.f*v/127.f);
  else if(p==EROSION_FREQ) frequencyText(out,size,80.f*powf(150.f,v/127.f),compact);
  else if (p >= BPF1 && p <= BPF3) frequencyText(out,size,frequency(p,v),compact);
  else if ((p == DELAY || p == REVERB || p == BITCRUSH || p == EROSION) && v == 0) snprintf(out,size,"OFF");
  else snprintf(out,size,"%u%%",percent(v));
}

void KickPerformance::drawOverview(Adafruit_SH1106G& d){
  if(fxMenu_){drawFxOverview(d);return;}
  d.clearDisplay(); d.setTextWrap(false); d.setTextColor(SH110X_WHITE);
  for (uint8_t k = 0; k < 6; ++k){
    int x = (k % 3) * 43, y = (k / 3) * 32;
    Parameter p = assignment(k);
    char title[12], value[16], secondary[12] = {};
    if (k == 0) snprintf(title,sizeof(title),"FX >");
    else if (k == 1){
      snprintf(title,sizeof(title),"DECAY");
      snprintf(secondary,sizeof(secondary),"R %s",state_.reverseEnabled ? "ON" : "OFF");
    } else if (k == 2){
      snprintf(title,sizeof(title),"TAIL");
      snprintf(secondary,sizeof(secondary),"%s",state_.tailDelayEnabled ? "ON" : "OFF");
    } else if (k == 3 && p==EROSION_FREQ){
      snprintf(title,sizeof(title),"ERO FREQ");
    } else if (k == 3){
      snprintf(title,sizeof(title),"BPF %u",state_.bpfLayerCount);
      snprintf(secondary,sizeof(secondary),"%sL%u",state_.bpfLayerCount ? "EDIT " : "NXT ",state_.editedBpfLayer+1);
    } else if (k == 4) snprintf(title,sizeof(title),"%s",state_.selectedCharacterModel ? "TUBE" : "MACK");
    else {
      snprintf(title,sizeof(title),"SHAPE");
      snprintf(secondary,sizeof(secondary),"DEPTH");
    }
    if(p==PARAM_COUNT)snprintf(value,sizeof(value),"MENU3");else valueText(p,value,sizeof(value),true);
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
    uint8_t count = p == STUT ? STUT_DIVISIONS : LOOP_DIVISIONS;
    for (uint8_t i = 0; i < count; ++i){
      int tx = 2 + (i % 4)*32, ty = 33 + (i / 4)*9;
      char text[8]; divisionLabel(text,sizeof(text),p == LOOP,i);
      if (r.on && r.division == i){
        d.fillRect(tx,ty,31,8,SH110X_WHITE); d.setTextColor(SH110X_BLACK);
      }
      label(d,tx,ty,text); d.setTextColor(SH110X_WHITE);
    }
    return;
  }
  if(p==EROSION_FREQ){
    d.drawFastHLine(LEFT,GRAPH_BOTTOM,RIGHT-LEFT+1,SH110X_WHITE);
    int centre=marker(v);
    d.drawFastVLine(centre,GRAPH_TOP,GRAPH_BOTTOM-GRAPH_TOP+1,SH110X_WHITE);
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
  if (p == DELAY || p == REVERB || p == MACKIE || p == TUBE || p == BITCRUSH || p == EROSION){
    d.drawRect(LEFT,GRAPH_TOP+4,RIGHT-LEFT+1,10,SH110X_WHITE);
    int w = (RIGHT-LEFT-2)*v/127;
    if (w) d.fillRect(LEFT+1,GRAPH_TOP+5,w,8,SH110X_WHITE);
    if (p == MACKIE || p == TUBE){
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
      amplitude = expf(-t/(.025f+.7f*x*x));
    } else if (p == TAIL){
      float start = .08f+.65f*x;
      amplitude = t < .015f ? 1.f : t < start ? 0.f : .8f*expf(-(t-start)*12.f);
    } else if (p == SHAPE){
      // Illustrate depth only; actual time is set by Menu 1 SWEEP.
      amplitude = (log2f(kickdaisy::shapeStartRatio(x))/4.f)*expf(-t*8.f);
    }
    int y = GRAPH_BOTTOM-(int)(amplitude*(GRAPH_BOTTOM-GRAPH_TOP));
    if (px > LEFT) d.drawLine(px-1,previousY,px,y,SH110X_WHITE);
    previousY = y;
  }
}

void KickPerformance::drawFocus(Adafruit_SH1106G& d, uint32_t bpm){
  d.clearDisplay(); d.setTextWrap(false); d.setTextColor(SH110X_WHITE);
  if(fxMenu_){drawFxFocus(d,idleOverview(now_));return;}
  if (focus_.resetOverlay){
    label(d,16,19,"FX RESET",2); label(d,7,45,"FX OFF / PUMP KEPT");
    d.setTextWrap(true); d.display(); return;
  }
  Parameter p = assignment(focus_.knob);
  if(p==PARAM_COUNT){label(d,10,12,"FX ON MENU 3");label(d,4,36,"SIX EFFECT CONTROLS");d.display();return;}
  uint8_t v = position(p); float x = v/127.f;
  char title[22], value[22], footer[22] = {}, extra[22] = {};
  if(p==EROSION)snprintf(title,sizeof(title),"EROSION");
  else if(p==EROSION_FREQ)snprintf(title,sizeof(title),"EROSION FREQUENCY");
  else if (p == BITCRUSH) snprintf(title,sizeof(title),"BITCRUSH");
  else if (p <= REVERB) snprintf(title,sizeof(title),"%s",FX_NAMES[p]);
  else if (p == DECAY) snprintf(title,sizeof(title),"DECAY");
  else if (p == TAIL) snprintf(title,sizeof(title),"TAIL DELAY");
  else if (p <= BPF3) snprintf(title,sizeof(title),"BPF %u EDIT L%u",state_.bpfLayerCount,state_.editedBpfLayer+1);
  else if (p == MACKIE || p == TUBE) snprintf(title,sizeof(title),"%s",p == MACKIE ? "MACKIE" : "TUBE");
  else snprintf(title,sizeof(title),"SHAPE");
  valueText(p,value,sizeof(value),false);
  if (p == STUT || p == LOOP){
    const RepeatState& r = p == STUT ? state_.stutter : state_.looper;
    if (r.on){
      char start[8]; divisionLabel(start,sizeof(start),p == LOOP,r.randomStart);
      snprintf(footer,sizeof(footer),"START %s > %s",start,r.direction > 0 ? "FAST" : "SLOW");
    }
    else snprintf(footer,sizeof(footer),p == LOOP ? "UP = RANDOM RATE" : "UP = 1/4 > 1/8T");
  } else if (p == DELAY){
    unsigned wet = v ? (unsigned)(10.f+66.f*smoothstep(x)+.5f) : 0;
    unsigned fb = v ? (unsigned)(18.f+60.f*powf(x,1.25f)+.5f) : 0;
    snprintf(extra,sizeof(extra),"3/8"); snprintf(footer,sizeof(footer),"WET %u%%  FB %u%%",wet,fb);
  } else if (p == HPF || p == LPF){
    snprintf(footer,sizeof(footer),p == HPF ? "30H - 11.5K   %u%%" : "120H - 18K    %u%%",percent(v));
  } else if (p == PUMP){
    unsigned depth = v ? (unsigned)(22.f+74.f*powf(x,.82f)+.5f) : 0;
    snprintf(footer,sizeof(footer),"DEPTH %u%%  %s",depth,state_.pumpEnabled ? "ON" : "OFF");
  } else if (p == REVERB){
    unsigned echo=(unsigned)(100.f*smoothstep(fmaxf(0.f,fminf(1.f,(x-.65f)/.35f)))+.5f);
    snprintf(footer,sizeof(footer),"REV %u%% ECHO %u%%",percent(v),echo);
  } else if(p==EROSION){
    snprintf(footer,sizeof(footer),"NOISE %u%%  %.2fms",percent(v),x*x);
  } else if (p == BITCRUSH){
    snprintf(footer,sizeof(footer),"%.1f BIT  WET %u%%",16.f-14.f*x,percent(v));
  } else if (p == DECAY) snprintf(footer,sizeof(footer),"REV %s",state_.reverseEnabled ? "ON" : "OFF");
  else if (p == TAIL){
    unsigned ms = (unsigned)(60000.f/(bpm ? bpm : 120)*.5f*x+.5f);
    snprintf(footer,sizeof(footer),"%u ms  %s",ms,state_.tailDelayEnabled ? "ON" : "OFF");
  } else if (p <= BPF3) snprintf(footer,sizeof(footer),"85Hz ------ 3.2kHz");
  else if (p == MACKIE || p == TUBE){
    unsigned pct = percent(v);
    snprintf(extra,sizeof(extra),"%s",v == 0 ? "DRY" : pct <= 25 ? "BASE DRIVE" : pct <= 48 ? "MID I" : pct < 73 ? "MID II" : "MID III");
    snprintf(footer,sizeof(footer),"%s %u%%",p == MACKIE ? "TUBE" : "MACKIE",percent(p == MACKIE ? state_.tubeAmount : state_.mackieAmount));
  } else {
    snprintf(extra,sizeof(extra),"DEPTH %.1f ST",12.f*log2f(kickdaisy::shapeStartRatio(x)));
    snprintf(footer,sizeof(footer),"TIME: MENU1 SWEEP");
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
  now_=now;
  if(fxMenu_ && idleOverview(now))dirty_=true;
  if (focus_.knob == 2 && bpm != renderedBpm_) dirty_ = true;
  if (!dirty_ || (rendered_ && (uint32_t)(now-lastFrameMs_) < 40)) return;
  drawOverview(overview);
  if (focus) drawFocus(*focus,bpm);
  dirty_ = false; rendered_ = true; lastFrameMs_ = now; renderedBpm_ = bpm;
}


KickPerformance::Parameter KickPerformance::fxAssignment(uint8_t knob) const {
  static constexpr Parameter repeat[]={DELAY,LOOP,STUT,PITCH};
  static constexpr Parameter fixed[]={PUMP,REVERB,BITCRUSH,EROSION};
  if(knob==0)return repeat[fx_.repeat];
  if(knob==1)return fx_.filter?LPF:HPF;
  return fixed[knob-2];
}
void KickPerformance::setFxMenu(bool enabled){
  if(fxMenu_==enabled)return;
  fxMenu_=enabled;functionHeld_=false;
  for(auto& p:physical_)p=PhysicalPot{};
  for(auto& b:buttons_)b=ButtonState{};
  lastTouchMs_=now_;dirty_=true;
}
void KickPerformance::setFunctionHeld(bool held){
  if(functionHeld_==held)return;
  functionHeld_=held;physical_[5]=PhysicalPot{};
  if(fxMenu_ && held)focusControl(5);
  dirty_=true;
}
void KickPerformance::restoreFx(const FxState& state){
  fx_=state;if(fx_.repeat>3)fx_.repeat=0;if(fx_.filter>1)fx_.filter=0;fx_.routes&=31;if(fx_.routes&16)fx_.routes|=2;
  resetMask_=0;queueResetMask();queue(92,fx_.routes);
  for(auto& p:physical_)p=PhysicalPot{};
  dirty_=true;flushMidi();
}
void KickPerformance::queueResetMask(){queue(90,resetMask_&127);queue(91,(resetMask_>>7)&7);}
bool KickPerformance::resetPending(Parameter p) const {
  for(uint8_t i=0;i<FX_PAGES;++i)if(FX_PARAMS[i]==p)return resetMask_&(1u<<i);
  return false;
}
uint32_t KickPerformance::takeResetParameters(){auto result=resetParameters_;resetParameters_=0;return result;}
void KickPerformance::zeroEffect(Parameter p,bool send){
  for(uint8_t i=0;i<FX_PAGES;++i)if(FX_PARAMS[i]==p)resetMask_&=~(1u<<i);
  if(p==STUT)state_.stutter=RepeatState{};
  else if(p==LOOP)state_.looper=RepeatState{};
  else position(p)=p==PITCH?64:0;
  if(send){resetParameters_|=1ul<<p;queueResetMask();queue(PARAM_CC[p],p==PITCH?64:0);}
  dirty_=true;
}
void KickPerformance::requestReset(uint8_t knob){
  Parameter p=fxAssignment(knob); // FN+EROSION still resets amount, never frequency.
  resetParameters_|=1ul<<p;
  for(uint8_t i=0;i<FX_PAGES;++i)if(FX_PARAMS[i]==p)resetMask_|=1u<<i;
  if(!running_)zeroEffect(p,true);else queueResetMask();
  lastTouchMs_=now_;dirty_=true;
}
void KickPerformance::transport(bool running,uint32_t bar){
  // Seed commits at 96 clocks. Mirror and resend idempotent zeros as a
  // backstop if a queued mask missed the edge under UART backpressure.
  if(resetMask_ && ((!running) || (running && bar!=bar_))){
    for(uint8_t i=0;i<FX_PAGES;++i)if(resetMask_&(1u<<i))zeroEffect(FX_PARAMS[i],true);
  }
  running_=running;bar_=bar;
}
void KickPerformance::drawFxOverview(Adafruit_SH1106G& d){
  d.clearDisplay();d.setTextWrap(false);d.setTextColor(SH110X_WHITE);
  for(uint8_t k=0;k<6;++k){
    int x=k%3*43,y=k/3*32;Parameter p=fxAssignment(k);
    const char* names[]={"DLY","LOOP","STUT","HPF","LPF","PUMP","REV","CRSH","ERO","PIT"};
    int index=p==DELAY?0:p==LOOP?1:p==STUT?2:p==HPF?3:p==LPF?4:p==PUMP?5:p==REVERB?6:p==BITCRUSH?7:p==PITCH?9:8;
    char text[8];snprintf(text,sizeof(text),"%u%s",k+1,names[index]);
    if(k==focus_.knob)d.drawRect(x,y,k==2||k==5?42:43,32,SH110X_WHITE);
    label(d,x+2,y+3,text);
    if(p==PITCH)snprintf(text,sizeof(text),"%+d",int(roundf(12.f*(int(pitch_)-64)/(pitch_<64?64.f:63.f))));else snprintf(text,sizeof(text),"%u%%",percent(position(p)));label(d,x+2,y+13,text);
    label(d,x+29,y+13,k==3 && (fx_.routes&16)?"I":k>=2 && (fx_.routes&(1u<<(k-2)))?"+":"E");
    d.drawRect(x+3,y+24,34,4,SH110X_WHITE);
    d.fillRect(x+4,y+25,32*position(p)/127,2,SH110X_WHITE);
    if(resetPending(p))label(d,x+30,y+3,"*");
  }
  d.setTextWrap(true);d.display();
}
void KickPerformance::drawFxFocus(Adafruit_SH1106G& d,bool idle){
  d.clearDisplay();d.setTextWrap(false);d.setTextColor(SH110X_WHITE);
  if(idle){
    label(d,0,0,"E:EXT I:INT +:BOTH");
    // Animated routing overview; heights are control amounts, not audio meters.
    for(uint8_t k=0;k<6;++k){
      int x=4+k*21,h=2+position(fxAssignment(k))*28/127;
      d.drawRect(x,14,16,34,SH110X_WHITE);d.fillRect(x+2,46-h,12,h,SH110X_WHITE);
      char route=k==3 && (fx_.routes&16)?'I':k>=2 && (fx_.routes&(1u<<(k-2)))?'+':'E';
      char n[3]={char('1'+k),route,0};label(d,x+2,50,n);
      if(resetPending(fxAssignment(k)))label(d,x+4,17,"*");
    }
    d.drawPixel(2+(now_/70)%124,62,SH110X_WHITE);
  }else{
    Parameter p=assignment(focus_.knob);
    const char* name=p==PITCH?"PITCH":p==EROSION_FREQ?"EROSION FREQ":p==EROSION?"EROSION":p==BITCRUSH?"BITCRUSH":FX_NAMES[p];
    label(d,0,0,name);
    label(d,103,0,focus_.knob==3 && (fx_.routes&16)?"INT":focus_.knob>=2&&(fx_.routes&(1u<<(focus_.knob-2)))?"E+I":"EXT");
    char value[22];valueText(p,value,sizeof(value),false);label(d,2,12,value,2);
    if(p==EROSION || p==EROSION_FREQ){
      int centre=marker(erosionFrequency_);
      for(int px=4;px<=123;++px){
        float distance=(px-centre)/(6.f+erosion_*.10f);
        float env=expf(-distance*distance*.5f);
        int h=int(env*(8.f+erosion_*.07f));
        d.drawFastVLine(px,49-h,1+h,SH110X_WHITE);
      }
      d.drawFastHLine(4,50,120,SH110X_WHITE);
      d.drawFastVLine(centre,30,22,SH110X_WHITE);
      label(d,2,56,functionHeld_?"FREQ:80Hz --- 12kHz":"FN + TURN = FREQUENCY");
    }else{
      if(p==PITCH){
        d.drawFastHLine(4,43,120,SH110X_WHITE);
        d.drawFastVLine(marker(64),36,15,SH110X_WHITE);
        d.fillTriangle(marker(pitch_)-3,33,marker(pitch_)+3,33,marker(pitch_),39,SH110X_WHITE);
      }else if(p==DELAY || p==REVERB){
        for(int px=4;px<124;++px){
          float t=(px-4)/119.f;
          float env=p==DELAY?powf(.25f+.65f*position(p)/127.f,int(t*7)):expf(-t/(.08f+.6f*position(p)/127.f));
          int h=int(18*env);
          if(p==REVERB || (px-4)%17<2)d.drawFastVLine(px,51-h,h,SH110X_WHITE);
        }
      }else if(p==BITCRUSH){
        float a=position(p)/127.f,levels=1.f+15.f*powf(1.f-a,4.f);int previous=44;
        for(int px=4;px<124;++px){int y=44-int(roundf(sinf((px-4)*.13f)*levels)/levels*9.f);d.drawLine(px-1,previous,px,y,SH110X_WHITE);previous=y;}
      }else drawVisualization(d,p);
      label(d,2,56,resetPending(fxAssignment(focus_.knob))?"ZERO AT BAR END *":focus_.knob<2?"CLICK:NEXT HOLD:ZERO":"CLICK:ROUTE HOLD:ZERO");
    }
    if(resetPending(fxAssignment(focus_.knob)))label(d,118,24,"*");
  }
  d.setTextWrap(true);d.display();
}
