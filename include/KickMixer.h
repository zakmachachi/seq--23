#ifndef KICK_MIXER_H
#define KICK_MIXER_H
#include <Arduino.h>
#include <Adafruit_SH110X.h>

// Kick back menu: one sound parameter per physical knob. K3 sets compressor
// amount and its button toggles the compressor through its level-matched path.
class KickMixer {
public:
  enum Control : uint8_t { TAIL_START, ACID_DELAY_RATE, COMP_AMOUNT,
                           KICK_GAIN, TAIL_HOLD, REVERB,
                           CONTROL_COUNT,
                           // Legacy names keep the alternate sequencer copy source-compatible.
                           LINE=KICK_GAIN, DIST=COMP_AMOUNT, BELLY=TAIL_START,
                           BPF=ACID_DELAY_RATE, SUB=TAIL_HOLD, PUNCH=REVERB };
  struct SavedState {
    uint8_t value[CONTROL_COUNT] = {64,64,64,64,64,0};
  };
  using SendCC = bool (*)(void*, uint8_t, uint8_t);
  void begin(SendCC send, void* context);
  void setActive(bool active);
  void invalidate(){dirty_=true;}
  void sampleAngle(uint8_t knob, float angle);
  void buttonEdge(uint8_t button, bool pressed, uint32_t now);
  void service(uint32_t now);
  void render(Adafruit_SH1106G& overview, Adafruit_SH1106G* focus, uint32_t now);
  void saveTo(SavedState& out) const;
  void restoreFrom(const SavedState& in);
  bool compressorEnabled() const { return compressorEnabled_; }
  uint8_t belly() const { return 64; }
  uint8_t punch() const { return 127; }
  uint8_t sub() const { return 0; }
private:
  SendCC send_ = nullptr;
  void* context_ = nullptr;
  uint8_t value_[CONTROL_COUNT] = {64,64,64,64,64,0};
  bool compressorEnabled_ = false;
  bool cutoffEnabled_ = false;
  bool active_ = false, dirty_ = true, rendered_ = false;
  bool pending_[CONTROL_COUNT] = {};
  bool enablePending_ = false;
  bool cutoffEnablePending_ = false;
  bool buttonDown_[CONTROL_COUNT] = {};
  bool potInitialized_[CONTROL_COUNT] = {};
  float potAngle_[CONTROL_COUNT] = {}, potAccumulated_[CONTROL_COUNT] = {};
  uint32_t lastFrameMs_ = 0;
  static uint8_t ccFor(Control control);
  void send(Control control);
  void draw(Adafruit_SH1106G& display, bool focus);
};
#endif
