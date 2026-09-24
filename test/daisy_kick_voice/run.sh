#!/usr/bin/env bash
set -euo pipefail
voice_test_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
firmware="$voice_test_dir/../../daisy-kick/midi_oled_monitor.cpp"
work_dir="$(mktemp -d "${TMPDIR:-/tmp}/daisy-kick-voice.XXXXXX")"
trap 'rm -rf "$work_dir"' EXIT
# The voice block runs from its banner to just before TriggerKickVoice().

end=$(grep -n '^static void TriggerKickVoice' "$firmware" | cut -d: -f1)
# The punch-profile switches sit just above the voice banner.
start=$(grep -n 'PUNCH PROFILE — ITERATIVE ABLATION' "$firmware" | cut -d: -f1)
sed -n "$((start - 1)),$((end - 1))p" "$firmware" > "$work_dir/voice_extract.inc"
built=$(grep -o 'KICK_PUNCH_PROFILE_STAGE = [0-9]*' "$work_dir/voice_extract.inc" | grep -o '[0-9]*$')

# Every stage is a build someone may flash, so every stage must pass. With
# "wav", only the stage the firmware is set to is rendered.
status=0
for stage in 0 1 2 3 4 5; do
  sed "s/KICK_PUNCH_PROFILE_STAGE = [0-9]*/KICK_PUNCH_PROFILE_STAGE = $stage/" \
    "$work_dir/voice_extract.inc" > "$work_dir/voice_stage.inc"
  cp "$work_dir/voice_stage.inc" "$work_dir/voice_extract_stage.inc"
  sed 's/#include "voice_extract.inc"/#include "voice_extract_stage.inc"/' \
    "$voice_test_dir/test.cpp" > "$work_dir/test_stage.cpp"
  "${CXX:-c++}" -std=c++14 -O2 -Wall -Wno-unused-function -fsanitize=address,undefined \
    -I"$work_dir" "$work_dir/test_stage.cpp" -o "$work_dir/voice_test_$stage"
  if [ "$stage" = "$built" ]; then args=("$@"); else args=(); fi
  "$work_dir/voice_test_$stage" "${args[@]+"${args[@]}"}" || status=1
done
exit $status
