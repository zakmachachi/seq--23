#!/usr/bin/env bash
# Renders screen 2's kick view to PNGs: bash test/kick_view/run.sh [out_dir]
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$here/../.."
out="${1:-.}"
font="$(find "$root/.pio/libdeps" -name glcdfont.c | head -1)"
bin="$(mktemp "${TMPDIR:-/tmp}/kick-view.XXXXXX")"
trap 'rm -f "$bin"' EXIT
"${CXX:-c++}" -std=c++17 -O1 -I"$root/include" -include cstdint -include algorithm \
  -DPROGMEM= -DFONT_SOURCE="\"$font\"" "$here/preview.cpp" -o "$bin"
"$bin" "$out" | while read -r pbm; do
  python3 -c "
from PIL import Image; import sys
im = Image.open(sys.argv[1]).convert('L').point(lambda v: 255 - v)
im.resize((512, 256), Image.NEAREST).save(sys.argv[1][:-4] + '.png')" "$pbm"
  rm -f "$pbm"; echo "${pbm%.pbm}.png"
done
