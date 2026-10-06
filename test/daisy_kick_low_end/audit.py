"""Reproducible dry/wet phase survey of the firmware, with no Seed-side hooks.

Usage: python3 test/daisy_kick_low_end/audit.py OUTPUT_DIR [FIRMWARE.cpp]
Writes raw CSV/JSON and a self-contained interactive phase map.
"""
import concurrent.futures
import csv
import itertools
import hashlib
import json
from pathlib import Path
import random
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
OUTPUT = Path(sys.argv[1]).resolve()
SOURCE = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else ROOT / "daisy-kick/midi_oled_monitor.cpp"
OUTPUT.mkdir(parents=True, exist_ok=True)

cases = []
# Independent grid of tuning, character, waveform, sweep shape and decay.
for note, shape, wave, model, amount, decay in itertools.product(
        (28, 33, 36, 38, 41, 45), (0, .5, 1), (0, 1), (0, 1), (.2, .5, 1), (.5, 1)):
    cases.append(dict(note=note, shape=shape, wave=wave, model=model, amount=amount, decay=decay))
# Low BPF settings, including stacked bands close to the body.
for note, model, bands in itertools.product((28, 33, 38, 41), (0, 1),
                                           itertools.product((0, .15, .5), repeat=3)):
    cases.append(dict(note=note, model=model, amount=.7, wave=1, shape=.5,
                      layers=1, bpf1=bands[0], bpf2=bands[1], bpf3=bands[2]))
# Wet-only bitcrusher extremes with both models and low/high EQ boost centres.
for note, model, crush, band in itertools.product((28,33,38,41), (0,1), (.5,1), (0,.5,1)):
    cases.append(dict(note=note, model=model, amount=1, wave=1, shape=.5,
                      layers=1, bpf1=band, bpf2=band, bpf3=band, bitcrush=crush))
# Reproducible interacting settings. Very short decays have separate
# first-cycle/output regression checks; these windows inspect sustained bass.
rng = random.Random(2302)
for _ in range(384):
    cases.append(dict(note=rng.choice((28, 30, 33, 35, 38, 40, 41)),
                      model=rng.randrange(2), shape=rng.choice((0, .25, .5, .75, 1)),
                      wave=rng.choice((0, .5, 1)), amount=rng.choice((0, .2, .5, .8, 1)),
                      decay=rng.choice((.35, .5, .7, 1)), punch=rng.choice((0, .5, 1)),
                      tmod=rng.choice((0, 64 / 127, 1)), belly=rng.choice((0, .5, 1)),
                      vel=rng.choice((1, 64, 100, 127)), layers=rng.choice((0, 1 / 3, 2 / 3, 1)),
                      bpf1=rng.random(), bpf2=rng.random(), bpf3=rng.random(),
                      taildelay=rng.choice((0, 0, .3, .7)), hits=rng.choice((1, 4)),
                      line=rng.choice((.12, .33, .7))))

columns = ("window", "harmonic", "tracked_hz", "cycles", "dry_db", "wet_db",
           "phase_deg", "sum_vs_stronger_db", "output_harmonic_db", "output_rms_db", "peak")
with tempfile.TemporaryDirectory(prefix="kick-phase-") as tmp:
    tmp = Path(tmp)
    firmware = SOURCE.read_text()
    marker = "        float signal =\n            dry +\n            wet;"
    if firmware.count(marker) != 1:
        raise SystemExit("Cannot locate the dry/wet sum for the audit observer")
    firmware = firmware.replace(marker,
        "        ObserveKickMix(dry, wet, kick_voice.phase);\n" + marker)
    (tmp / "firmware.cpp").write_text(firmware)
    binary = tmp / "phase"
    subprocess.run(["c++", "-std=c++17", "-O2", "-w", "-I", str(ROOT / "daisy-kick/host/stubs"), "-I", str(ROOT / "daisy-kick"),
                    f'-DFIRMWARE="{tmp / "firmware.cpp"}"', str(HERE / "phase.cpp"), "-o", str(binary)], check=True)

    def measure(item):
        index, settings = item
        result = subprocess.run([str(binary)] + [f"{k}={v}" for k, v in settings.items()],
                                capture_output=True, text=True, check=True)
        metrics = [dict(zip(columns, map(float, row.split(",")))) for row in result.stdout.splitlines()]
        return dict(id=index, settings=settings, metrics=metrics)

    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        for result in pool.map(measure, enumerate(cases)):
            results.append(result)
            if len(results) % 100 == 0:
                print(f"Measured {len(results)}/{len(cases)} combinations", flush=True)

(OUTPUT / "phase-data.json").write_text(json.dumps(results, separators=(",", ":")))
with (OUTPUT / "phase-data.csv").open("w", newline="") as stream:
    writer = csv.writer(stream)
    writer.writerow(("case", "settings", *columns))
    for case in results:
        for metric in case["metrics"]:
            writer.writerow((case["id"], json.dumps(case["settings"], sort_keys=True),
                             *(metric[c] for c in columns)))

body = [(m["sum_vs_stronger_db"], c, m) for c in results for m in c["metrics"]
        if m["harmonic"] == 1 and 30 <= m["tracked_hz"] * m["harmonic"] <= 90 and m["cycles"] >= 2
        and max(m["dry_db"], m["wet_db"]) > -60]
body.sort(key=lambda entry: entry[0])
enabled = [(v,c,m) for v,c,m in body
           if c["settings"].get("punch", .5) > 0]
summary = dict(firmware_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
               combinations=len(results), body_windows=len(body),
               enabled_component_windows=len(enabled),
               enabled_component_worst_db=min((v for v,_,_ in enabled), default=0),
               enabled_component_windows_below_minus_1_db=sum(v < -1 for v,_,_ in enabled),
               component_scope="Body-clock fundamental projections, 30-90 Hz. Higher harmonics remain in raw data but are excluded from this summary because a clean sine has no second harmonic. Enabled components exclude PUNCH=0; finite windows and swept tones can still bias projections.",
               windows_below_minus_1_db=sum(v < -1 for v, _, _ in body),
               worst=[dict(loss_db=v, settings=c["settings"], metric=m) for v, c, m in body[:12]])
(OUTPUT / "summary.json").write_text(json.dumps(summary, indent=2))
print(json.dumps({k: v for k, v in summary.items() if k != "worst"}, indent=2), flush=True)
print("Worst bass window:", json.dumps(summary["worst"][0] if body else None), flush=True)

html = (HERE / "phase-map.html").read_text().replace("/* AUDIT_DATA */[]", json.dumps(results, separators=(",", ":")))
(OUTPUT / "phase-map.html").write_text(html)
