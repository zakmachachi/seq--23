# Sync-only MIDI input

Teensy consumes incoming Clock (F8), Start (FA), Continue (FB), and Stop (FC)
and forwards those four bytes to MIDI OUT before generating corresponding local
notes. Every other incoming byte is discarded, including channel messages,
SysEx payload and song-position messages. No external note/CC/slide/gate parser
exists on this path. The sequencer's own note and parameter output is retained.
External clock still forwards while stopped, but does not advance ratchets.

The 1 ms engine services at most eight RX bytes per invocation (8 kB/s service
capacity versus 3.125 kB/s wire capacity). Actual timing includes timer service
jitter and time behind any bytes already committed to the UART. This is not a
hardware THRU or a zero-latency clock regenerator.

Local MIDI channel messages are written atomically under saved/restored IRQ
state, only if the complete message fits. No writes wait for UART space.
The UART driver's interrupt-drained TX storage is extended from 40 to 128 bytes
to accommodate layered note bursts. The previous 1024-byte foreground-drained
queue is not used. Ordinary CC writes require 124 free bytes, preventing them
from filling the burst reserve; existing control mailboxes retry/coalesce the
latest values. Engine/slide controls share note priority. Channel messages leave
three byte slots for realtime traffic. Deferred note-offs are retained by note
and channel, retried by the timer, and sent before retriggering the same note.

There is a physical overload limit: sustained generated traffic plus sync cannot
exceed MIDI's wire bandwidth losslessly. `midiNoteOverflows` counts rejected whole
note-ons; `midiRealtimeOverflows` counts rejected realtime bytes if the reserved
space is exhausted. There are no partial note/CC writes, unbounded retries or
blocking writes to freeze the ISR. Incoming data deliberately ignored by the
sync-only policy is not forwarded. CC coalescing preserves latest control state,
not every intermediate automation value.

Startup restores EEPROM state before starting the engine ISR. Both Seeds keep
interrupt-fed RX queues with exact discontinuity markers and parser recovery.
Kick foreground parsing is bounded to 32 bytes/pass; synth parsing to eight
bytes/audio callback, still comfortably above wire rate. Synth FPU denormal
flushing is set for interrupt contexts as well as the main context.

Validation: `test/midi_wire/test.cpp` covers the sync-byte whitelist, CC reserve,
seven-channel note/engine/slide bursts, atomic refusal on full TX, and RX overflow
markers under address/undefined-behavior sanitizers. Teensy and both Seed builds
pass. Hardware clock jitter and worst-case audio CPU deadlines remain hardware
measurements; a desktop test cannot certify immunity to every runtime lockup.
