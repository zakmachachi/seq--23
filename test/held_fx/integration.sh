#!/usr/bin/env bash
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$here/../.."
work="$(mktemp -d "${TMPDIR:-/tmp}/held-integration.XXXXXX")"
trap 'rm -rf "$work"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -w -fsanitize=address,undefined -I "$root/daisy-kick/host/stubs" "$here/integration.cpp" -o "$work/integration"
"$work/integration"
"${CXX:-c++}" -std=c++17 -O2 -w -fsanitize=address,undefined -I "$root/daisy-kick/host/stubs" "$root/daisy-kick/host/kick_host.cpp" -o "$work/render"
python3 "$here/render_check.py" "$work/render" "$work"
