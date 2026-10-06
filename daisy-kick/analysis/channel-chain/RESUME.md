> Upload update: Seed3 flashed with exact 109900-byte readback match; leave/reboot requested. Teensy upload confirmed by the user. See validation.json. The preparation notes below are historical.

# Resume / upload checkpoint — 2026-10-03

Latest instruction: finish the implementation/checks first; the user will then
connect boards to upload both. **No uploads during this revision yet.**

Current status and checksums: `validation.json`; measurements and limitations:
`README.md`. Changes are uncommitted; preserve unrelated `.DS_Store`.

Prepared Teensy: `.pio/build/teensy41/firmware.hex`. New FX BITCRUSH (CC37),
Menu 1 K3 SWEEP (CC78), K5 TMOD (CC79), K6 TUNE (velocity + CC77). Patch format
v19 saves bitcrush separately, retaining older EEPROM offsets. Use PlatformIO
`run -e teensy41 -t upload` once connected and authorized. Build cache/USB need
escalated access. Earlier serial port was `/dev/cu.usbmodem202508701`; re-detect.

Prepared Seed3: `daisy-kick/build/midi_oled_monitor.bin`, exact bytes/checksum
in manifest. USB identity last seen: STM32 `0483:df11`, serial `200364500000`.
DFU status must be checked with escalated `dfu-util -l`; sandbox listings can
hide a connected device. Upload alt0 at `0x08000000`, read back exact binary
length and compare SHA before issuing `0x08000000:leave`. The leave command
can report exit74 on disconnect; that alone does not invalidate a prior
successful readback. Do not assume the board is still in DFU.

Previously uploaded Seed binary SHA:
`bf32bcfcde9b27aa0142e3f91fb4c8ee9e7b9f355c9862e1516b26911c8f91aa`
(107000 bytes; shared-phase 180 Hz revision). Source backup used for the
aliasing comparison: `/tmp/seq-kick-channelchain/before.cpp`.

Current validation logs while this session exists:
- `/tmp/kick-verified-regressions.log`: ASAN/UBSAN + phase lock, MIDI parser /
  queue, 30-second automation, audio routing/headroom/retrigger tests.
- `/tmp/kick-ready-controller.log`: controller acceptance tests.
- `/tmp/kick-final-build.log`: Seed ARM build.
- `/tmp/kick-ready-teensy.log`: Teensy PlatformIO build.
- `/tmp/kick-final-phase.log`: 1080-case production DSP phase survey.
- `/tmp/kick-final-analysis.log`: mid-EQ, aliasing, pitch and glue probes.

The final host renderer and phase survey use 16-sample blocks like the Seed.
The isolated voice/crossover tests additionally cover other callback sizes.
`verify_seed_vector.py` checks the actual linked USART3 vector: the earlier
attempt using linker --wrap did not replace a startup-local weak reference;
the final --defsym alias does, and the binary vector was verified.

On hardware, verify MIDI reception and worst-case FX timing using a live soak;
host rendering cannot prove MCU deadline headroom. DWT/receiver debug counters
are retained as `audio_max_cycles`, `audio_overruns`, `midi_uart_errors`, and
`midi_rx.overflow`. No physical DAC/audio capture has been taken this revision.
