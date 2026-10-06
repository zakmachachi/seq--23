# Experimental sub pitch fall — not uploaded

The current firmware retains a fixed octave sub. These auditions change only
the sub phase increment in a temporary firmware copy; the body, gain and
amplitude envelopes remain unchanged.

The prototype uses `f(t) = min(90 Hz, octave_hz * (1 + depth * exp(-t/tau)))`.
`tau` is a time constant: pitch error falls by 60 dB after `6.9078 * tau`.
The conservative candidate starts D2's sub at 47.72 Hz (1.3 times its target)
and settles to 36.708 Hz, with a 20 ms time constant.

| Measurement, D2 example | Fixed sub | Conservative pitch fall |
| --- | ---: | ---: |
| Interpeak valley, 1/6-octave average | −22.84 dB | −17.68 dB |
| Largest smoothed 10 Hz fall within 40–90 Hz | 14.58 dB | 11.00 dB |
| 30–40 Hz energy change | Reference | −0.48 dB |
| 40–90 Hz energy change | Reference | +0.70 dB |
| Maximum-Mackie sample peak | 0.645 | 0.694 |

[Clean prototype](PROTOTYPE_ratio1.3_tau20ms.wav) ·
[Mackie prototype](PROTOTYPE_ratio1.3_tau20ms_mackie.wav) ·
[All nine candidates and settings](metrics.json)

A 1.6-times start with the same 20 ms constant fills the valley further, but
reduces 30–40 Hz energy by 1.53 dB. Larger, slower falls increasingly trade the
deepest bass for upper bass: the smoothest-scoring candidate loses 13.63 dB in
30–40 Hz and is unsuitable when deep bass is the priority.

**None of the nine candidates meets an unsmoothed maximum fall of 12 dB per
10 Hz everywhere.** Raw maximum falls remain 12.71–14.27 dB, chiefly on the
upper skirt of the unchanged 73.4 Hz body peak. Smoother plots do not establish
better sound. These samples have not received the full parameter sweep or
hardware verification applied to the current firmware.

The spectral methods match the [current comparison](../README.md). Audio is
from temporary full-firmware host renders, not a recording of Seed3.
