# Daisy kick firmware

Daisy Seed kick/audio firmware, originally imported from `DaisyExamples/files (1)/midi_oled_monitor.cpp`. This is separate firmware from the Teensy controller in the parent project's `src/` directory. Build it with Make, not PlatformIO.

## Build

Requires GNU Make, the ARM embedded GCC toolchain (`arm-none-eabi-g++`), and built libDaisy/DaisySP dependencies. Dependencies and generated binaries are not included in this folder.

The Makefile defaults to `../libDaisy` and `../DaisySP`, relative to this folder. Override these paths to use an existing DaisyExamples checkout:

```sh
make -C daisy-kick \
  LIBDAISY_DIR="/path/to/DaisyExamples/libDaisy" \
  DAISYSP_DIR="/path/to/DaisyExamples/DaisySP"
```

To save those paths on your machine, create `daisy-kick/local.mk` (ignored by
Git):

```make
LIBDAISY_DIR = /path/to/DaisyExamples/libDaisy
DAISYSP_DIR = /path/to/DaisyExamples/DaisySP
```

Then, from the `seq--23` directory, run `make -C daisy-kick`.

## Upload over USB

Connect USB to the Daisy Seed itself. Hold BOOT, press and release RESET, then
release BOOT to enter the STM32 DFU bootloader. Check that `dfu-util -l` lists
the device, then run from `seq--23`:

```sh
make -C daisy-kick program-dfu
```

This builds the current source before uploading. PlatformIO's upload command
in this repository targets the Teensy 4.1 controller, not the Seed.

## Dependency versions

The local dependency revisions when this source was imported were:

- libDaisy: `9498417add4a4c76bc10737d5e9f43750a4da639`
- DaisySP: `a0494a3adb67f549e18dfd71a35fa656f65b38b6`

Initialize dependency submodules and build the libraries before building this application if using a fresh checkout. The outputs are generated under `daisy-kick/build/`, with target name `midi_oled_monitor`.

Consult the source's hardware and MIDI initialization sections before wiring or changing mappings. The Teensy controller's mapping is documented separately in [KICK_MENU2.md](../KICK_MENU2.md).
