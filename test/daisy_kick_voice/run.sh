#!/usr/bin/env bash
set -euo pipefail
voice_test_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
firmware="$voice_test_dir/../../daisy-kick/midi_oled_monitor.cpp"
work_dir="$(mktemp -d "${TMPDIR:-/tmp}/daisy-kick-voice.XXXXXX")"
trap 'rm -rf "$work_dir"' EXIT
# The voice block runs from its banner to just before TriggerKickVoice().
start=$(grep -n 'KICK VOICE — ONE SINE' "$firmware" | cut -d: -f1)
end=$(grep -n '^static void TriggerKickVoice' "$firmware" | cut -d: -f1)
sed -n "$((start - 1)),$((end - 1))p" "$firmware" > "$work_dir/voice_extract.inc"
"${CXX:-c++}" -std=c++14 -O2 -Wall -Wno-unused-function -fsanitize=address,undefined \
  -I"$work_dir" "$voice_test_dir/test.cpp" -o "$work_dir/voice_test"
"$work_dir/voice_test" "$@"
