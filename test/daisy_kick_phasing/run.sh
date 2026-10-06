#!/usr/bin/env bash
# WAVE now derives one saw from the body oscillator phase; the former five
# independent saws and their detune/spread ablation no longer exist.
# The replacement tests observe the real output: octave-sub invariance with
# WAVE, body/sub tuning, and exact fresh/retrigger agreement after the bridge.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$here/../daisy_kick_low_end/run.sh" "$@"
