# Rodox Hoover reference

Source: Rodox Trading, “That Feels So Good Now”, artist upload on SoundCloud.
The local reference WAV is an eight-second excerpt beginning around 136 seconds,
kept solely for listening and signal analysis during this implementation.

The stereo difference makes the Hoover's ensemble movement easier to inspect
through the full mix. The useful behavior is sustained harmonic energy with
continuous PWM/chorus movement. Its note onset is not substantially louder than
the held body. This agrees with the artist's programming description supplied
for this project: start from Alpha Juno “What The?”, raise the envelope levels
and intermediate times, shorten attack and release, then tune PWM and chorus.

The firmware response is therefore:

- fast but nonzero 4–10 ms attack before the engine's movement scaling,
  without a one-sample phase reset;
- high L1/L2/L3 values, removing the old transient-to-quiet-body collapse;
- 140–1000 ms internal transition times and 28–110 ms release before the
  engine's 1.4× movement scaling;
- pitch movement reduced to a supporting scoop instead of a kick-like attack;
- BODY/TAIL controls density and sustain while PWM/SWARM and CHORUS remain
  independent performance macros.
