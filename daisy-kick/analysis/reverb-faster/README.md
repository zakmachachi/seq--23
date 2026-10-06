# Faster, louder filtered echo

Straight eighth-note echo replaces dotted eighth (167 ms versus 250 ms at
180 BPM). Return coefficient rises .45 -> .72 (+4.08 dB). Every repeat and
its feedback passes a 240 Hz second-order high-pass and 2.8 kHz second-order
low-pass; the shared 180 Hz return high-pass remains. Measured loop-filter
gain: -36.13 dB at 30 Hz, -0.08 dB at 1 kHz, -24.69 dB at 10 kHz.

Fixed a fractional read-index wrap bug caught by sanitizers: integer wrapping
prevents a tiny negative fractional position rounding to the buffer size.
Boundary regression, 30-second automation, and 72 full-bus headroom cases pass.
Maximum combined peak: 0.756761, below the 0.93 safety-ceiling knee.

This is a musical approximation, not a WAV measurement of Pitch-Hiker.
No reference WAV was available. The user subsequently reported uploading the
current code and liking it; that upload's exact hash was not independently
verified. Listening examples are the reverb-0/50/75/100.wav files here.

Current work adds GPIO outputs without changing audio; see
[trigger/clock wiring](../../EURORACK_OUTPUTS.md).
