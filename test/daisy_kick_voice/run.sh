#!/usr/bin/env bash
# The retired extractor tested KickHitParams, five saws and the old phase
# handoff. Current coverage compiles the complete firmware instead, checking
# real MIDI controls, octave-sub tuning, onset, gates and deterministic resets.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$here/../daisy_kick_low_end/run.sh" "$@"
