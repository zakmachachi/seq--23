#!/usr/bin/env bash
# Supersaw phasing across the settings that matter, in the full firmware.
#
# Every setting renders twice at WAVE max: the firmware as it is, and the
# same firmware with the saws' spread zeroed, a perfectly static saw locked
# to the sub. The difference in each harmonic's swing is the phasing the
# supersaw itself adds; what the models, the ceiling or a decaying tail do
# to any waveform is in both and cancels. A control does the same for the
# Hz-detuned supersaw of fed0c2b and must catch it.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$here/../.."
firmware="${KICK_FIRMWARE:-$root/daisy-kick/midi_oled_monitor.cpp}"
work="$(mktemp -d "${TMPDIR:-/tmp}/daisy-kick-phasing.XXXXXX")"
trap 'rm -rf "$work"' EXIT

build() { # <source> <binary>
  "${CXX:-c++}" -std=c++17 -O2 -w -I "$root/daisy-kick/host/stubs" \
    -DFIRMWARE="\"$1\"" "$here/phasing.cpp" -o "$2"
}

sed 's/WAVE_SAW_SPREAD\[WAVE_SAWS\] = {[^}]*}/WAVE_SAW_SPREAD[WAVE_SAWS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}/' \
  "$firmware" > "$work/static.cpp"
grep -q 'WAVE_SAW_SPREAD\[WAVE_SAWS\] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}' "$work/static.cpp"
build "$firmware" "$work/now" & build "$work/static.cpp" "$work/static" & wait

control=0
if git -C "$root" show fed0c2b:daisy-kick/midi_oled_monitor.cpp > "$work/old.cpp" 2>/dev/null; then
  sed 's/WAVE_SAW_DETUNE_HZ\[WAVE_SAWS\] = {[^}]*}/WAVE_SAW_DETUNE_HZ[WAVE_SAWS] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}/' \
    "$work/old.cpp" > "$work/old_static.cpp"
  build "$work/old.cpp" "$work/old" & build "$work/old_static.cpp" "$work/old_static" & wait
  control=1
fi

# The reported setting, then one change at a time.
base="decay=1.0 shape=0.66 mackamt=0.87 mackie=1.0 sub=1.0 punch=1.0 line=0.33"
configs=(
  "baseline:"
  "LINE 10%:line=0.10"          "LINE 20%:line=0.20"         "LINE 60%:line=0.60"   "LINE 100%:line=1.0"
  "DECAY 0.5:decay=0.5"         "DECAY 0.7:decay=0.7"        "DECAY 0.85:decay=0.85"
  "dry, no model:mackamt=0"     "Mackie 35%:mackamt=0.35"
  "Tube 50%:model=1 tubeamt=0.5 mackamt=0"                   "Tube 100%:model=1 tubeamt=1.0 mackamt=0"
  "SHAPE 0:shape=0"             "SHAPE 33%:shape=0.33"       "SHAPE 100%:shape=1.0"
  "PITCH down:vel=1"            "PITCH up:vel=127"
  "TAIL MOD 50%:tailmod=0.5"    "TAIL MOD 100%:tailmod=1.0"
  "note 33 Hz:note=24"          "note 41 Hz:note=28"         "note 82 Hz:note=40"   "note 110 Hz:note=45"
  "BPF 2 layers:layers=0.67 bpf1=0.4"
  "WAVE 50%:wave=0.5"
  "TAIL DELAY:taildelay=0.6"
  "eighths:spacing_ms=230.75 hits=8"
  "one long hit:hits=1"
  "quiet dry:line=0.15 mackamt=0 punch=0.5 sub=0.6"
)

limit_body=1.5; limit_mid=3.0; limit_same=-80
fails=0
printf "%-18s %28s   %36s\n" "" "extra swing over a static saw" "retriggered kick vs the last one"
printf "%-18s %9s %9s %9s   %11s %11s %11s\n" "setting" "h2-8 dB" "h7-16 dB" "(1 harm.)" "after 90ms" "inside" "static saw"
for entry in "${configs[@]}"; do
  name="${entry%%:*}"; args="${entry#*:}"
  # shellcheck disable=SC2086
  read -r a1 a2 a3 same inside < <("$work/now" $base wave=1 $args)
  # shellcheck disable=SC2086
  read -r s1 s2 s3 _ s_inside < <("$work/static" $base wave=1 $args)
  e1=$(awk "BEGIN{print $a1-$s1}"); e2=$(awk "BEGIN{print $a2-$s2}"); e3=$(awk "BEGIN{print $a3-$s3}")
  ok=$(awk "BEGIN{print ($e1<=$limit_body && $e2<=$limit_mid && $same<=$limit_same)}")
  [ "$ok" = 1 ] || fails=$((fails + 1))
  printf "%-18s %9.2f %9.2f %9.2f   %8s dB %8s dB %8s dB  %s\n" "$name" "$e1" "$e2" "$e3" "$same" "$inside" "$s_inside" \
    "$([ "$ok" = 1 ] && echo PASS || echo FAIL)"
done

if [ "$control" = 1 ]; then
  worst=0
  for args in "" "mackamt=0" "decay=0.7"; do
    # shellcheck disable=SC2086
    read -r a1 a2 _ _ _ < <("$work/old" $base wave=1 $args)
    # shellcheck disable=SC2086
    read -r s1 s2 _ _ _ < <("$work/old_static" $base wave=1 $args)
    worst=$(awk "BEGIN{w=$worst; d=$a1-$s1; if(d>w)w=d; d=$a2-$s2; if(d>w)w=d; print w}")
  done
  caught=$(awk "BEGIN{print ($worst > 3*$limit_body)}")
  printf "control, Hz-detuned supersaw (fed0c2b): extra body swing up to %.1f dB  %s\n" "$worst" \
    "$([ "$caught" = 1 ] && echo "PASS (caught)" || echo "FAIL (missed)")"
  [ "$caught" = 1 ] || fails=$((fails + 1))
fi

echo "limits, band of 3 harmonics: body (h2-8) <= $limit_body dB, h7-16 <= $limit_mid dB; kicks identical after the handoff (<= $limit_same dB)"
[ "$fails" = 0 ] && echo "ALL CHECKS PASSED" || echo "$fails CHECK(S) FAILED"
exit $((fails > 0))
