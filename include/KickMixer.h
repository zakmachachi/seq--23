#ifndef KICK_MIXER_H
#define KICK_MIXER_H
#include <Arduino.h>
#include <Adafruit_SH110X.h>

// Output/mix stage of the Daisy kick. Same ownership split as KickPerformance:
// state, physical input, focus and outgoing MIDI are separate.
class KickMixer {
public:
  enum Control : uint8_t { LINE, MACK, SHRM, BPF, SUB, COMP, CONTROL_COUNT };
  using SendCC = bool (*)(void*, uint8_t, uint8_t);
  void begin(SendCC send, void* context);
  void setActive(bool active);
  void sampleAngle(uint8_t knob, float angle); // Existing dual-track ADC angle.
  void buttonEdge(uint8_t button, bool pressed, uint32_t now);
  void service(uint32_t now);
  void render(Adafruit_SH1106G& overview, Adafruit_SH1106G* focus, uint32_t now);
private:
  struct PhysicalPot {
    bool initialized = false;
    float angle = 0;
    float accumulated = 0; // Signed relative movement in MIDI-value units.
  };
  struct MidiState {
    SendCC send = nullptr;
    void* context = nullptr;
    bool initialized = false;
    bool pending[CONTROL_COUNT + 1] = {};
    uint8_t value[CONTROL_COUNT + 1] = {};
  };
  uint8_t value_[CONTROL_COUNT] = {89,65,80,64,75,0};
  bool limiter_ = false;
  PhysicalPot physical_[6];
  bool buttonDown_[6] = {};
  uint8_t focusKnob_ = 0;
  bool focusLimiter_ = false;
  MidiState midi_;
  bool active_ = false, dirty_ = true, rendered_ = false;
  uint32_t lastFrameMs_ = 0;

  void queue(uint8_t cc, uint8_t value);
  void flushMidi();
  void snapshot();
  void focusControl(uint8_t knob, bool limiter);
  void adjust(uint8_t knob, int delta);
  static uint8_t percent(uint8_t value);
  void drawOverview(Adafruit_SH1106G& display);
  void drawFocus(Adafruit_SH1106G& display);
};
#endif
