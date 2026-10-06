Current update: effect cycling preserves all amounts; knob 1 adds EXT-only
PITCH (±12 semitones, centre = zero shift). Built for manual upload; controller
checks passed. The validation notes below describe the earlier revision.

# Dedicated kick FX menu — validation

This revision, including **INT-only reverb**, is uploaded to both boards.
Teensy GUI uploader reported success and its USB serial device returned. Seed3's
113904-byte flash readback matches the compiled binary exactly. The DFU leave
request disconnected the device during the final status query; audio operation
has not been measured on the physical board.

- Controller ASan/UBSan suite: six assignments, short-click routes/cycles,
  one-second hold, multi-effect bar reset, cancellation, transport stop,
  queued MIDI backpressure at a bar boundary, Function + Erosion, saved slot
  state, ADC seams/clamping/jitter and all 128 display values pass.
- DSP ASan/UBSan suite and callback renders: five EXT-only effects leave the
  kick bit-identical; each switchable effect in EXT mode leaves it bit-identical;
  adding its internal route produces a measurable change. INT-only reverb leaves
  external audio bit-identical to dry and retains the same internal reverb as BOTH. With no kick, the
  external signal is bit-identical between route settings. Reverb tanks/filter
  histories are independent. MIDI Start/bar/stop/cancel reset checks pass.
- Existing phase, bass isolation, Mackie, reverb, Erosion, GPIO pulses,
  headroom and retrigger checks pass. The 54-case all-max-mixer regression
  peaks at 0.3583; that is the existing test grid, not an exhaustive bound on
  every new combination of external FX and input levels.
- ARM and Teensy builds pass. USART3 vector points to KickMidiRxIrq.
  Existing unused-function/display-format warnings remain.

Host tests do not measure Seed CPU deadlines, physical OLED appearance, or
end-to-end MIDI/OLED timing on hardware. Check audio_overruns on a physical
all-FX stress run. Reset masks are pre-armed at Seed's 96-clock boundary;
Teensy re-sends zeros as a fallback if congestion delayed the arm request.
That fallback can be one foreground UI pass late.

See [controls](../../../KICK_MENU2.md) and [build hashes](validation.json).

## Screen previews

Generated from the controller drawing functions, using the Adafruit bitmap font.
The values are illustrative control settings, not audio measurements.

![Permanent six-knob map](fx-overview.png)
![Function + Erosion frequency](fx-erosion-frequency.png)
![Five-second idle overview](fx-idle.png)

SVG originals are alongside these previews. The controller test binary's optional
`preview` argument writes them into its current directory.
