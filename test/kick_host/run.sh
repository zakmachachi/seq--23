#!/usr/bin/env bash
set -euo pipefail
kick_test_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
kick_project_dir="$(cd -- "$kick_test_dir/../.." && pwd)"
kick_test_binary="$(mktemp "${TMPDIR:-/tmp}/kick-host.XXXXXX")"
trap 'rm -f "$kick_test_binary"' EXIT
"${CXX:-c++}" -std=c++17 -Wall -Wextra -fsanitize=address,undefined \
  -I"$kick_test_dir/stubs" -I"$kick_project_dir/include" \
  "$kick_test_dir/test.cpp" "$kick_project_dir/src/KickPerformance.cpp" \
  -o "$kick_test_binary"
"$kick_test_binary"
