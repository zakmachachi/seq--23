# ACID prototype scope

ACID is a separate machine from HOOVER. Seq23 sends pitch, gate, slide and
velocity; the Seed supplies the voice. The current model uses a free-running
BLEP saw/pulse oscillator, a driven four-stage feedback low-pass, triggered
filter decay, and a separate gate amplitude envelope. Accent raises filter
opening, feedback and amplitude; slide avoids retriggering the filter contour.
Only note-offs matching the active pitch release the voice.

This is a playable prototype, **not yet a validated accurate TB-303 model**.
The filter does not solve a transistor/diode ladder circuit, its envelope/accent
coupling is simplified, and there is no measured 303 calibration. Earlier claims
of circuit-level accuracy and shared envelope charge were too strong for this
implementation. Hoover development does not establish 303 accuracy.

Physical Menu 2: cutoff, resonance, envelope amount, decay, accent amount and
saw/square blend. CC26–28 additionally expose drive, glide time and level.
Cutoff is logarithmic 55–7200 Hz; decay 55–900 ms; glide 8–280 ms.
The host WAV now renders this actual C++ voice rather than a separate Python
approximation. Future filter calibration can replace `dsp/Acid.h` independently
of Hoover and without modifying the kick Seed.
