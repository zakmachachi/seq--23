#ifndef KICK_MIXER_H
#define KICK_MIXER_H
#include <Arduino.h>
#include <Adafruit_SH110X.h>

// Raw-909 back menu 2: K1 turns output gain and pushes EXT KICK source mode.
// The six-byte patch payload stays in place for EEPROM compatibility.
class KickMixer {
public:
  enum Control : uint8_t { LINE, DIST, BELLY, BPF, SUB, PUNCH, CONTROL_COUNT };
  struct SavedState {
    uint8_t value[CONTROL_COUNT] = {64,65,64,64,75,64};
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
  uint8_t belly() const { return 64; }
  uint8_t punch() const { return 127; }
  uint8_t sub() const { return 0; }
  bool externalKickMode() const { return externalKickMode_; }
  void restoreFrom(const SavedState& in);
private:
  SendCC send_ = nullptr;
  void* context_ = nullptr;
  uint8_t legacy_values_[CONTROL_COUNT] = {64,65,64,64,75,64};
  bool externalKickMode_ = false;
  bool active_ = false, dirty_ = true, rendered_ = false;
  bool gainPending_ = false, buttonDown_ = false, potInitialized_ = false;
  float potAngle_ = 0.f, potAccumulated_ = 0.f;
  uint32_t lastFrameMs_ = 0;
  void sendSource();
  void sendOutputGain();
  void draw(Adafruit_SH1106G& display, bool focus);
};
#endif
