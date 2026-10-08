# Held FX build and validation

See [controls, macros, files and tuning notes](../../../HELD_FX.md).
Both firmware images are built; this revision has **not been uploaded**.

[Validation manifest and image hashes](validation.json) records the current build.
The Seed image occupies 125,072 / 131,072 flash bytes; internal SRAM is
474,428 / 524,288 bytes. The new engine adds 266,728 bytes in SDRAM.

The controller, standalone engine and production MIDI/audio integration tests
pass with address/undefined-behavior sanitizers. All 16 gestures produce finite,
bounded audio in endpoint/control stress tests. Fourteen processor pads leave
the generated kick unchanged with silent external input. HAZARD deliberately
adds a procedural layer on the external bus, including quiet activity in silence.

DECAY preserves the detected leading transient exactly; release settles to
exact dry bypass. Clock/kick quantization, Stop, lost-controller lease and
last-held fallback are tested. Existing MIDI reception and GPIO pulse handling
remain active in the production integration test.

Host timing is not a measurement of Seed CPU load. Check audio_overruns during
hardware testing, especially with BANE/RADIO/HAZARD and heavy Mackie processing.
