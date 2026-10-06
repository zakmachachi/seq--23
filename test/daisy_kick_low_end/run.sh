#!/usr/bin/env bash
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd -- "$here/../.." && pwd)"
work="$(mktemp -d "${TMPDIR:-/tmp}/daisy-kick-low-end.XXXXXX")"
trap 'rm -rf "$work"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -w -fsanitize=address,undefined \
  -I "$root/daisy-kick/host/stubs" \
  "$root/daisy-kick/host/kick_host.cpp" -o "$work/render"
"${CXX:-c++}" -std=c++17 -O2 -w -fsanitize=address,undefined \
  -I "$root/daisy-kick/host/stubs" "$here/locked_core.cpp" -o "$work/locked_core"
"$work/locked_core"
"${CXX:-c++}" -std=c++17 -O2 -w -fsanitize=address,undefined \
  -I "$root/daisy-kick/host/stubs" "$here/reliability.cpp" -o "$work/reliability"
"$work/reliability"
"${CXX:-c++}" -std=c++17 -O2 -w -fsanitize=address,undefined \
  -I "$root/daisy-kick/host/stubs" "$here/bass_shelf.cpp" -o "$work/bass_shelf"
"$work/bass_shelf" > "$work/shelf.csv"
"${CXX:-c++}" -std=c++17 -O2 -w -fsanitize=address,undefined \
  -I "$root/daisy-kick/host/stubs" "$here/reverb.cpp" -o "$work/reverb"
"$work/reverb"
"${CXX:-c++}" -std=c++17 -O2 -w -fsanitize=address,undefined \
  -I "$root/daisy-kick/host/stubs" "$here/euro.cpp" -o "$work/euro"
"$work/euro"
"${CXX:-c++}" -std=c++17 -O2 -fsanitize=address,undefined "$here/erosion.cpp" -o "$work/erosion"
"$work/erosion"
"${CXX:-c++}" -std=c++17 -O2 -w -fsanitize=address,undefined \
  -I "$root/daisy-kick/host/stubs" "$here/fx_menu.cpp" -o "$work/fx_menu"
"$work/fx_menu"
python3 "$here/fx_routes.py" "$work/render" "$work"
python3 "$here/check.py" "$work/render" "$work"
