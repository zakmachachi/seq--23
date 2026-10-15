#include "KickMixer.h"
#include <stdio.h>

namespace {
constexpr uint8_t CC_KICK_GAIN = 53;
constexpr uint8_t CC_REVERB = 36;
constexpr uint8_t CC_TAIL_START = 67;
constexpr uint8_t CC_COMP_AMOUNT = 68;
constexpr uint8_t CC_COMP_ENABLE = 69;
constexpr uint8_t CC_ACID_DELAY_RATE = 108;
constexpr uint8_t CC_CUTOFF_TIME = 71;
constexpr uint8_t CC_CUTOFF_ENABLE = 72;

int rateIndex(uint8_t value){
  static const uint8_t points[7]={0,21,42,64,85,106,127};
  int best=0, distance=128;
  for(int i=0;i<7;++i){
    int d=abs((int)value-(int)points[i]);
    if(d<distance){distance=d;best=i;}
  }
  return best;
}
const char* delayRateName(uint8_t value){
  static const char* names[7]={"1/16","1/8","1/4","DOTTED","1/2","3/4","1 BAR"};
  return names[rateIndex(value)];
}
}

uint8_t KickMixer::ccFor(Control control){
  static const uint8_t cc[CONTROL_COUNT]={CC_TAIL_START,CC_ACID_DELAY_RATE,
      CC_COMP_AMOUNT,CC_KICK_GAIN,CC_CUTOFF_TIME,CC_REVERB};
  return cc[control];
}

void KickMixer::begin(SendCC send, void* context){
  send_=send; context_=context;
  if(!send_)return;
  for(uint8_t i=0;i<CONTROL_COUNT;++i) send_(context_,ccFor((Control)i),value_[i]);
  send_(context_,CC_COMP_ENABLE,compressorEnabled_?127:0);
  send_(context_,CC_CUTOFF_ENABLE,cutoffEnabled_?127:0);
}

void KickMixer::send(Control control){
  pending_[control]=true;
  if(send_ && send_(context_,ccFor(control),value_[control])) pending_[control]=false;
}

void KickMixer::setActive(bool active){
  if(active_==active)return;
  active_=active;dirty_=true;rendered_=false;
  for(uint8_t i=0;i<CONTROL_COUNT;++i){
    potInitialized_[i]=false;potAccumulated_[i]=0.f;
    if(active_)send((Control)i);
  }
  if(active_){
    enablePending_=true;
    if(send_&&send_(context_,CC_COMP_ENABLE,compressorEnabled_?127:0))enablePending_=false;
    cutoffEnablePending_=true;
    if(send_&&send_(context_,CC_CUTOFF_ENABLE,cutoffEnabled_?127:0))cutoffEnablePending_=false;
  }
}

void KickMixer::sampleAngle(uint8_t knob,float angle){
  if(!active_ || knob>=CONTROL_COUNT)return;
  if(!potInitialized_[knob]){
    potInitialized_[knob]=true;potAngle_[knob]=angle;potAccumulated_[knob]=0.f;return;
  }
  float delta=angle-potAngle_[knob];potAngle_[knob]=angle;
  if(delta>3.14159265f)delta-=6.2831853f;
  if(delta< -3.14159265f)delta+=6.2831853f;
  if(buttonDown_[knob]){potAccumulated_[knob]=0.f;return;}
  potAccumulated_[knob]+=-delta*(128.f/6.2831853f);
  if(fabsf(potAccumulated_[knob])<2.f)return;
  int ticks=(int)potAccumulated_[knob];potAccumulated_[knob]-=ticks;
  int next=(int)value_[knob]+ticks;
  if(next<0)next=0;
  if(next>127)next=127;
  if(next==value_[knob])return;
  value_[knob]=(uint8_t)next;send((Control)knob);dirty_=true;
}

void KickMixer::buttonEdge(uint8_t button,bool pressed,uint32_t now){
  (void)now;
  if(button>=CONTROL_COUNT)return;
  buttonDown_[button]=pressed;
  if(!active_ || !pressed)return;
  if(button==COMP_AMOUNT){
    compressorEnabled_=!compressorEnabled_;
    enablePending_=true;
    if(send_ && send_(context_,CC_COMP_ENABLE,compressorEnabled_?127:0))
      enablePending_=false;
    dirty_=true;
  } else if(button==TAIL_HOLD){
    cutoffEnabled_=!cutoffEnabled_;
    cutoffEnablePending_=true;
    if(send_&&send_(context_,CC_CUTOFF_ENABLE,cutoffEnabled_?127:0))cutoffEnablePending_=false;
    dirty_=true;
  }
}

void KickMixer::service(uint32_t now){
  (void)now;
  for(uint8_t i=0;i<CONTROL_COUNT;++i)
    if(pending_[i] && send_ && send_(context_,ccFor((Control)i),value_[i]))
      pending_[i]=false;
  if(enablePending_&&send_&&send_(context_,CC_COMP_ENABLE,compressorEnabled_?127:0))
    enablePending_=false;
  if(cutoffEnablePending_&&send_&&send_(context_,CC_CUTOFF_ENABLE,cutoffEnabled_?127:0))
    cutoffEnablePending_=false;
}

void KickMixer::saveTo(SavedState& out)const{
  for(uint8_t i=0;i<CONTROL_COUNT;++i)out.value[i]=value_[i];
  if(compressorEnabled_)out.value[KICK_GAIN]|=0x80;
  if(cutoffEnabled_)out.value[TAIL_HOLD]|=0x80;
}

void KickMixer::restoreFrom(const SavedState& in){
  compressorEnabled_=(in.value[KICK_GAIN]&0x80)!=0;
  cutoffEnabled_=(in.value[TAIL_HOLD]&0x80)!=0;
  for(uint8_t i=0;i<CONTROL_COUNT;++i){
    uint8_t value=in.value[i];
    if(i==KICK_GAIN||i==TAIL_HOLD)value&=0x7f;
    if(value<=127)value_[i]=value;
  }
  for(uint8_t i=0;i<CONTROL_COUNT;++i)send((Control)i);
  enablePending_=true;
  if(send_&&send_(context_,CC_COMP_ENABLE,compressorEnabled_?127:0))enablePending_=false;
  cutoffEnablePending_=true;
  if(send_&&send_(context_,CC_CUTOFF_ENABLE,cutoffEnabled_?127:0))cutoffEnablePending_=false;
  dirty_=true;
}

void KickMixer::draw(Adafruit_SH1106G& d,bool focus){
  static const char* names[CONTROL_COUNT]={"GATE START","ACID RATE","TANH AMT","KICK GAIN","CUT RAMP","REVERB"};
  d.clearDisplay();d.setTextWrap(false);d.setTextColor(SH110X_WHITE);
  d.setTextSize(1);d.setCursor(0,0);d.print(focus?"KICK BACK MENU 2":"KICK END STAGE");
  for(uint8_t i=0;i<CONTROL_COUNT;++i){
    uint8_t y=8+i*8;
    d.setCursor(0,y);d.print(i+1);d.print(' ');d.print(names[i]);d.setCursor(83,y);
    if(i==TAIL_START){
      const uint8_t value=value_[i];
      const uint16_t ms=value<=64 ? (uint16_t)value*130/64
                                  : 130+(uint16_t)(value-64)*70/63;
      d.print(ms);d.print("ms");
    }
    else if(i==ACID_DELAY_RATE)d.print(delayRateName(value_[i]));
    else if(i==COMP_AMOUNT){d.print((uint16_t)value_[i]*100/127);d.print('%');}
    else if(i==KICK_GAIN){
      float db=value_[i]<=64?-15.f*(64.f-value_[i])/64.f:21.f*(value_[i]-64.f)/63.f;
      char text[10];snprintf(text,sizeof(text),"%+.1fdB",db);d.print(text);
    }else if(i==TAIL_HOLD){d.print((uint16_t)(2.f+58.f*value_[i]/127.f));d.print("ms");}
    else {d.print((uint16_t)value_[i]*100/127);d.print('%');}
  }
  d.setCursor(0,57);
  if(focus){d.print(compressorEnabled_?"3 CMP:ON":"3 CMP:OFF");d.print(" 5 CUT:");d.print(cutoffEnabled_?"ON":"OFF");}
  else d.print("K3 COMP / K5 CUTOFF");
  d.setTextWrap(true);d.display();
}

void KickMixer::render(Adafruit_SH1106G& overview,Adafruit_SH1106G* focus,uint32_t now){
  if(!active_ || (!dirty_ && rendered_ && uint32_t(now-lastFrameMs_)<100))return;
  draw(overview,false);if(focus)draw(*focus,true);
  dirty_=false;rendered_=true;lastFrameMs_=now;
}
