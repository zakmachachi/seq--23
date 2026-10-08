#!/usr/bin/env python3
"""Rebuild the before/after reference study from the actual C++ voice."""
from pathlib import Path
import argparse
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--analysis-python', default='python3', help='Python with numpy, scipy and matplotlib')
args = parser.parse_args()
root = Path(__file__).resolve().parent
study = root / 'analysis' / 'hoover'
with tempfile.TemporaryDirectory(prefix='hoover-study-') as tmp:
    tmp = Path(tmp)
    old = (study / 'Hoover-before.h').read_text()
    old = old.replace('"Blocks.h"', '"' + str(root / 'dsp' / 'Blocks.h') + '"')
    old = old.replace('"../../include/SoftSynthProtocol.h"', '"' + str(root.parent / 'include' / 'SoftSynthProtocol.h') + '"')
    header = tmp / 'before.h'
    header.write_text(old)
    for version in ('before', 'after'):
        exe = tmp / version
        flags = ['-DSTUDY_OLD_HEADER="' + str(header) + '"'] if version == 'before' else []
        subprocess.run(['c++', '-O2', '-std=c++14'] + flags + [str(root / 'study_hoover.cpp'), '-o', str(exe)], check=True)
        subprocess.run([str(exe), str(study / (version + '-held.wav')), str(study / (version + '-control.csv'))], check=True)
subprocess.run([args.analysis_python, str(study / 'analyse.py')], check=True)
