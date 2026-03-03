# seq-23: Sequencer Prototype (Work in Progress)

**Status:** WORK IN PROGRESS — ongoing prototype. Use at your own risk; expect frequent changes.
This repository contains firmware for a Teensy 4.1 based step sequencer prototype. It drives a 16-LED WS2812 chain, reads 16 buttons and 4 encoders, and outputs MIDI. The code was entirely vibe codeded since I have no clue how to write C and C++. All LLMs used were free versions. 

Key points:
- Prototype code: APIs, pin mappings and behaviors are experimental and subject to change.
- Pin mappings live in [include/SeqConfig.h](include/SeqConfig.h).

Quick start (PlatformIO):
```bash
cd Sequencer_prototybe
platformio run
platformio run --target upload
```

What to check on hardware:
- OLED UI responsiveness while turning encoders
- Encoder switch behavior and p-lock
- LED animations (boot, pause, fill, playhead)
- MIDI output on the configured `MIDI_TX_PIN` (see [include/SeqConfig.h](include/SeqConfig.h))

If upload fails:
- Ensure PlatformIO is installed and Teensy drivers are present.
- Verify wiring matches `include/SeqConfig.h` before connecting external signals.

Contributing / reporting issues:
- This is a private/solo prototype. To contribute or report bugs, open an issue or contact the author.

License:
- See project root for any license files. No formal release license is guaranteed for this prototype.

---

_This project is actively under development — expect breaking changes._
