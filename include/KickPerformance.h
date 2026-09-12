#ifndef KICK_PERFORMANCE_H
#define KICK_PERFORMANCE_H
#include <Arduino.h>
#include <Adafruit_SH110X.h>

// Controller state, physical input, focus and outgoing MIDI have separate owners.
class KickPerformance {
public:
  enum Parameter : uint8_t {
    STUT, LOOP, DELAY, HPF, LPF, PUMP, DECAY, TAIL,
    BPF1, BPF2, BPF3, MACKIE, SHERMAN, SHAPE, PARAM_COUNT
  };
  struct RepeatState {
    uint8_t position = 0;
    bool on = false;
    uint8_t division = 0, randomStart = 0, activationPosition = 0;
    int8_t direction = 1, previousStart = -1;
  };
  struct ControllerState {
    uint8_t selectedFx = STUT;
    RepeatState stutter, looper;
    uint8_t delay = 0, hpf = 0, lpf = 0, pumpAmount = 0;
    uint8_t decay = 64;
    bool reverseEnabled = false;
    uint8_t tailDelayAmount = 0;
    bool tailDelayEnabled = false;
    uint8_t bpfLayerCount = 0, editedBpfLayer = 0;
    uint8_t bpfFrequencyValue[3] = {35,65,95};
    uint8_t mackieAmount = 0, shermanAmount = 0;
    bool selectedCharacterModel = false;
    uint8_t kickShape = 64;
    bool pumpEnabled = false;
  };
  using SendCC = bool (*)(void*, uint8_t, uint8_t);
  void begin(SendCC send, void* context);
  void setActive(bool active);
  void sampleAngle(uint8_t knob, float angle); // Existing dual-track ADC angle.
  void buttonEdge(uint8_t button, bool pressed, uint32_t now);
  void service(uint32_t now);
  void render(Adafruit_SH1106G& overview, Adafruit_SH1106G* focus,
              uint32_t bpm, uint32_t now);
  const ControllerState& state() const { return state_; }
private:
  struct PhysicalPot {
    bool initialized = false;
    float angle = 0;
    float accumulated = 0; // Signed relative movement in MIDI-value units.
  };
  struct ButtonState { bool down = false, longFired = false; uint32_t pressedMs = 0; };
  struct FocusState {
    uint8_t knob = 0;
    bool resetOverlay = false;
    uint32_t resetMs = 0;
  };
  struct MidiState {
    SendCC send = nullptr;
    void* context = nullptr;
    bool initialized = false;
    bool pending[19] = {};
    uint8_t value[19] = {};
  };
  ControllerState state_;
  PhysicalPot physical_[6];
  ButtonState buttons_[6];
  FocusState focus_;
  MidiState midi_;
  bool active_ = false, dirty_ = true, rendered_ = false;
  uint32_t lastFrameMs_ = 0, renderedBpm_ = 0;

  Parameter assignment(uint8_t knob) const;
  uint8_t& position(Parameter parameter);
  const uint8_t& position(Parameter parameter) const;
  uint8_t outputValue(Parameter parameter) const;
  void queue(uint8_t cc, uint8_t value);
  void flushMidi();
  void snapshot();
  void focusControl(uint8_t knob);
  void reassign(uint8_t knob);
  void adjust(uint8_t knob, int delta);
  void updateRepeat(RepeatState& repeat, bool loop, uint8_t value);
  void resetFx(uint32_t now);
  static uint8_t randomStart(bool loop, int8_t previous);
  static uint8_t percent(uint8_t value);
  static float smoothstep(float value);
  static float frequency(Parameter parameter, uint8_t value);
  static float releaseMs(uint8_t value);
  static void shapeValues(uint8_t value, float& ratio, float& seconds);
  static void frequencyText(char* out, size_t size, float hz, bool compact);
  void valueText(Parameter parameter, char* out, size_t size, bool compact) const;
  void drawOverview(Adafruit_SH1106G& display);
  void drawFocus(Adafruit_SH1106G& display, uint32_t bpm);
  void drawVisualization(Adafruit_SH1106G& display, Parameter parameter);
};
#endif
