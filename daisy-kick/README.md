# Daisy kick firmware

Daisy Seed kick/audio firmware originally imported from `DaisyExamples/files (1)/midi_oled_monitor.cpp`. This is separate firmware from the Teensy controller in the parent project's `src/` directory. Build it with Make, not PlatformIO.

## Build

Requires GNU Make, the ARM embedded GCC toolchain (`arm-none-eabi-g++`), and built libDaisy/DaisySP dependencies. Dependencies and generated binaries are not included in this folder.

The Makefile defaults to `../libDaisy` and `../DaisySP`, relative to this folder. Override these paths to use an existing DaisyExamples checkout:

```sh
make -C daisy-kick \
  LIBDAISY_DIR="/path/to/DaisyExamples/libDaisy" \
  DAISYSP_DIR="/path/to/DaisyExamples/DaisySP"
```

The local dependency revisions when this source was imported were:

- libDaisy: `9498417add4a4c76bc10737d5e9f43750a4da639`
- DaisySP: `a0494a3adb67f549e18dfd71a35fa656f65b38b6`

Initialize dependency submodules and build the libraries before building this application if using a fresh checkout. The outputs are generated under `daisy-kick/build/`, with target name `midi_oled_monitor`.

This folder now contains the maintained DSP and MIDI implementation. Consult its hardware and MIDI initialization sections before wiring or changing mappings. The Teensy controller's mapping is documented separately in [KICK_MENU2.md](../KICK_MENU2.md); this import does not change or reconcile the two implementations.


## FX regression checks

Run `python3 daisy-kick/test/fx_regression.py` from the repository root. It compiles the actual reverb/filter implementations and the kick FX routing function with a host C++ compiler and address/undefined-behavior sanitizers. Tests cover HPF retrigger continuity (including CC=1), reverb amount/trigger transitions, continuous muted delay memory, the kick LPF frequency response, and independent output-bus state. Pass an older source path as an argument to reproduce the original failures.

Reverb amount uses a 20 ms time constant. The tank continues running when muted and is ducked 95% on a hit rather than erased inside the audio callback, so existing tails decay naturally under the new hit. The HPF onset blend closes over 3 ms instead of jumping back to dry. CC34 now applies LPF to both kick and external outputs, using separate filter instances.

These are signal-level host regressions, not a measurement of the board's CPU headroom or analogue output. After flashing, check CC33/CC36 at values 0, 1 and 2 with repeated kicks and long decay, then sweep CC34 on both outputs. Confirm smooth enable/disable, sustained rapid retriggers, and the maximum LPF setting audibly removing high frequencies.
