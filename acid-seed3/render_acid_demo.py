#!/usr/bin/env python3
"""Compatibility entry point: render both engines using the firmware C++ DSP."""
from pathlib import Path
import subprocess, tempfile
root = Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix="seq23-synth-") as tmp:
    exe = str(Path(tmp) / "render")
    subprocess.run(["c++", "-O2", "-std=c++14", str(root / "render_demo.cpp"), "-o", exe], check=True)
    subprocess.run([exe], cwd=root, check=True)
