#!/usr/bin/env bash
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
binary="$(mktemp "${TMPDIR:-/tmp}/held-fx.XXXXXX")"
trap 'rm -f "$binary"' EXIT
"${CXX:-c++}" -std=c++17 -O2 -Wall -Wextra -fsanitize=address,undefined "$here/test.cpp" -o "$binary"
"$binary"
