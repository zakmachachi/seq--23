#!/usr/bin/env python
"""Render actual firmware and compare with separately downloaded original TR-909 WAVs.

Python 2.7 + numpy/matplotlib (also Python 3 compatible). No reference audio
is copied into the repository. See --help for independently supplied paths.
"""
from __future__ import print_function
import argparse
import hashlib
import json
import math
import os
import subprocess
import tempfile
import wave

os.environ.setdefault('MPLCONFIGDIR', '/tmp/seq-kick-comparison-matplotlib')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
RATE = 48000
WINDOW = .600
NFFT = 262144
BANDS = ((20, 20000), (30, 40), (40, 90), (30, 90), (90, 500), (500, 20000))
COLORS = ['#2563eb', '#ef4444', '#059669', '#9333ea', '#d97706']


def sha(path):
    with open(path, 'rb') as stream:
        return hashlib.sha256(stream.read()).hexdigest()


def read_wav(path):
    stream = wave.open(path, 'rb')
    rate, channels, width = stream.getframerate(), stream.getnchannels(), stream.getsampwidth()
    raw = stream.readframes(stream.getnframes())
    stream.close()
    if width == 3:
        b = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3).astype(np.int32)
        x = b[:, 0] | (b[:, 1] << 8) | (b[:, 2] << 16)
        x = np.where(x & 0x800000, x - 0x1000000, x).astype(float) / 8388608.
    elif width == 2:
        x = np.frombuffer(raw, dtype='<i2').astype(float) / 32768.
    else:
        raise ValueError('Expected PCM16 or PCM24 WAV')
    return x.reshape(-1, channels).mean(axis=1), rate


def db(x):
    return 20 * np.log10(np.maximum(np.asarray(x), 1e-12))


def spectrum(x, rate):
    # Identical 600 ms window in seconds at native rates. 2 ms cosine edges
    # suppress crop discontinuities without a Hann window hiding the attack.
    n = int(WINDOW * rate)
    v = np.zeros(n)
    v[:min(n, len(x))] = x[:n]
    taper = np.ones(n)
    edge = int(.002 * rate)
    ramp = .5 - .5 * np.cos(np.linspace(0, np.pi, edge))
    taper[:edge], taper[-edge:] = ramp, ramp[::-1]
    energy = np.abs(np.fft.rfft(v * taper, NFFT)) ** 2 / float(rate * rate)
    freq = np.fft.rfftfreq(NFFT, 1. / rate)
    return freq, energy


def smooth_spectrum(freq, energy):
    centers = np.logspace(np.log10(20), np.log10(20000), 1100)
    # Full band width 1/6 octave, i.e. +/-1/12 octave. Average linear power.
    half = 2 ** (1. / 12)
    smoothed = []
    for center in centers:
        selected = energy[(freq >= center / half) & (freq <= center * half)]
        smoothed.append(selected.mean() if len(selected) else np.interp(center, freq, energy))
    return centers, np.array(smoothed)


def band_energy(freq, energy, lo, hi):
    mask = (freq >= lo) & (freq < hi)
    return float(energy[mask].sum() * (freq[1] - freq[0]))


def envelope(x, rate):
    # Forward 20 ms RMS windows, hop 1 ms; time labels at window centers.
    length, hop = int(.020 * rate), int(.001 * rate)
    cumulative = np.r_[0., np.cumsum(x * x)]
    starts = np.arange(0, min(len(x) - length, int(WINDOW * rate)), hop)
    rms = np.sqrt(np.maximum((cumulative[starts + length] - cumulative[starts]) / length, 0))
    return (starts + length / 2.) / rate, rms


def pitch(x, rate):
    # No filtering: interpolate same-polarity crossings, reject cycles below
    # -35 dB of peak or with periods outside 30..500 Hz. Attack is omitted.
    z = np.where((x[:-1] <= 0) & (x[1:] > 0))[0]
    crossing = (z - x[z] / (x[z + 1] - x[z])) / rate
    t, f = [], []
    threshold = max(abs(x)) * 10 ** (-35. / 20)
    for i in range(len(z) - 1):
        hz = 1. / (crossing[i + 1] - crossing[i])
        center = (crossing[i + 1] + crossing[i]) / 2
        if .008 < center < .32 and 30 < hz < 500 and np.max(np.abs(x[z[i]:z[i + 1] + 1])) > threshold:
            t.append(center)
            f.append(hz)
    return np.array(t), np.array(f)


def amplitude(x, rate, hz, start=.080, end=.280):
    v = x[int(start * rate):int(end * rate)]
    w = np.hanning(len(v))
    return float(2 * abs(np.sum(v * w * np.exp(-2j * np.pi * hz * np.arange(len(v)) / rate))) / w.sum())


def save_plot(fig, output, name):
    fig.tight_layout()
    fig.savefig(os.path.join(output, name + '.png'), dpi=160, bbox_inches='tight', pad_inches=.15)
    fig.savefig(os.path.join(output, name + '.svg'), bbox_inches='tight', pad_inches=.15)
    plt.close(fig)


def write_wav(path, x):
    values = np.asarray(np.round(np.clip(x, -1, 1) * 32767), dtype='<i2')
    stream = wave.open(path, 'wb')
    stream.setnchannels(1)
    stream.setsampwidth(2)
    stream.setframerate(RATE)
    stream.writeframes(values.tostring())
    stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-dir', default='/tmp/seq-kick-909-reference')
    parser.add_argument('--output', default=os.path.join(ROOT, 'daisy-kick/analysis/coherent'))
    parser.add_argument('--renderer', help='Optional already compiled current firmware host; default compiles it')
    args = parser.parse_args()
    if not os.path.isdir(args.output):
        os.makedirs(args.output)
    work = tempfile.mkdtemp(prefix='seq-kick-909-comparison-')
    source = os.path.join(ROOT, 'daisy-kick/midi_oled_monitor.cpp')
    source_sha = sha(source)
    binary = args.renderer or os.path.join(work, 'render')
    if not args.renderer:
        subprocess.check_call(['c++', '-std=c++17', '-O2', '-w', '-I', os.path.join(ROOT, 'daisy-kick/host/stubs'),
                               os.path.join(ROOT, 'daisy-kick/host/kick_host.cpp'), '-o', binary])
    base = dict(hits=1, spacing_ms=100, tail=1200, line=1, mackie=1, tube=1,
                mackamt=0, tubeamt=0, model=0, bpf=0, layers=0, hpf=0, lpf=0,
                reverb=0, sub=0, punch=1, wave=0, reverse=0, taildelay=0,
                sweeptime=.55, depthtrim=.55, curve=64./127, belly=64./127,
                gate=20, note=38, shape=.5, vel=100, decay=.5)
    cases = [
        ('firmware_g1_medium', 'Firmware G1, medium', dict(note=31, shape=.50, vel=70, decay=.30)),
        ('firmware_g1_long', 'Firmware G1, long', dict(note=31, shape=.50, vel=70, decay=.50)),
        ('d2_clean_body', 'D2 clean, SUB 0', {}),
        ('d2_clean_sub', 'D2 clean, SUB 1', dict(sub=1)),
        ('d2_mackie_max', 'D2 Mackie maximum', dict(sub=1, wave=1, mackamt=1, bpf=1, layers=1)),
        ('d2_tube_max', 'D2 Tube maximum', dict(sub=1, wave=1, tubeamt=1, model=1, bpf=1, layers=1)),
        ('d2_mackie_tailgate', 'D2 Mackie + tail gap', dict(sub=1, wave=1, mackamt=1, taildelay=.45)),
    ]
    series, metrics = {}, {}
    for key, label, changes in cases:
        settings = dict(base)
        settings.update(changes)
        raw = os.path.join(work, key + '.f32')
        subprocess.check_call([binary, raw] + ['%s=%s' % item for item in sorted(settings.items())])
        x = np.fromfile(raw, dtype='<f4').astype(float)[RATE // 4:]
        if not np.all(np.isfinite(x)):
            raise ValueError('Non-finite firmware render: ' + key)
        series[key] = (x, RATE, label)
        metrics[key] = dict(settings=settings, source='actual firmware AudioCallback', raw_sha256=sha(raw))
        if key in ('firmware_g1_medium', 'd2_clean_sub', 'd2_mackie_max', 'd2_mackie_tailgate'):
            # Firmware only. Preserve actual level and append a short silence.
            write_wav(os.path.join(args.output, key + '.wav'), np.r_[np.zeros(4800), x])
    references = [('reference_medium', 'TR-909 T50 A50 D50', 'BassDrum909-tune050-attack050-decay050.wav'),
                  ('reference_long', 'TR-909 T50 A0 D100', 'BassDrum909-tune050-attack000-decay100.wav')]
    for key, label, filename in references:
        path = os.path.join(args.reference_dir, filename)
        x, rate = read_wav(path)
        series[key] = (x, rate, label)
        metrics[key] = dict(source='AudioRealism original TR-909 recording', filename=filename, sha256=sha(path))
    for key, (x, rate, label) in series.items():
        freq, energy = spectrum(x, rate)
        total = band_energy(freq, energy, 20, 20000)
        peak = float(np.max(abs(x)))
        entry = metrics[key]
        entry.update(label=label, sample_rate=rate, sample_peak_dbfs=float(db(peak)),
                     peak=peak, capture_seconds=len(x) / float(rate),
                     band_energy_fraction_of_20_20000_hz={('%d_%d_hz' % (lo, hi)): band_energy(freq, energy, lo, hi) / total for lo, hi in BANDS},
                     raw_band_energy={('%d_%d_hz' % (lo, hi)): band_energy(freq, energy, lo, hi) for lo, hi in BANDS})
        root = 440 * 2 ** (((31 if ('g1' in key or 'reference' in key) else 38) - 69) / 12.)
        entry['body_root_hz'] = root
        entry['root_band_fraction_20_20000'] = band_energy(freq, energy, root * 2 ** (-1./12), root * 2 ** (1./12)) / total
        entry['body_projection_80_280_ms_dbfs'] = float(db(amplitude(x, rate, root)))
        entry['octave_projection_80_280_ms_dbfs'] = float(db(amplitude(x, rate, root / 2)))
        t, rms = envelope(x, rate)
        entry['rms20ms_relative_peak_db_at_ms'] = {str(ms): float(db(np.interp(ms / 1000., t, rms) / peak)) for ms in (20, 50, 100, 150, 200, 300, 400)}
        pt, pf = pitch(x, rate)
        if key.startswith(('reference', 'firmware_g1')):
            entry['body_pitch_cycles'] = [dict(time_ms=float(ti * 1000), hz=float(fi)) for ti, fi in zip(pt, pf)]
    # Time/pitch plots: no time warping, each waveform normalized to own peak.
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    for i, key in enumerate(('reference_medium', 'firmware_g1_medium')):
        x, rate, label = series[key]
        t = np.arange(len(x)) / float(rate)
        axes[0, 0].plot(t * 1000, x / max(abs(x)), label=label, color=COLORS[i], alpha=.8)
        axes[0, 1].plot(t * 1000, x / max(abs(x)), label=label, color=COLORS[i], alpha=.8)
    axes[0, 0].set_xlim(0, 250)
    axes[0, 1].set_xlim(0, 30)
    for key, color, style in [('reference_medium', COLORS[0], '-'), ('firmware_g1_medium', COLORS[1], '-'),
                              ('reference_long', COLORS[0], '--'), ('firmware_g1_long', COLORS[1], '--')]:
        x, rate, label = series[key]
        t, rms = envelope(x, rate)
        axes[1, 0].plot(t * 1000, db(rms / max(abs(x))), style, color=color, label=label)
    for i, key in enumerate(('reference_medium', 'firmware_g1_medium')):
        x, rate, label = series[key]
        t, f = pitch(x, rate)
        axes[1, 1].plot(t * 1000, f, 'o-', markersize=3, color=COLORS[i], label=label)
    for ax, title in zip(axes.flatten(), ('Medium decay: waveform', 'First 30 ms: attack and sweep', '20 ms RMS envelope / each sample peak', 'Measured cycle-average body pitch')):
        ax.set_title(title)
        ax.set_xlabel('Time from file onset / firmware trigger (ms)')
        ax.grid(True, alpha=.25)
        ax.legend(loc='best', fontsize=8)
    axes[0, 0].set_ylabel('Amplitude / own peak')
    axes[0, 1].set_ylabel('Amplitude / own peak')
    axes[1, 0].set_xlim(0, 550)
    axes[1, 0].set_ylim(-65, 0)
    axes[1, 0].set_ylabel('dB relative to sample peak')
    axes[1, 1].set_xlim(0, 220)
    axes[1, 1].set_ylim(30, 200)
    axes[1, 1].set_ylabel('Hz (no pitch estimate at noisy attack)')
    save_plot(fig, args.output, '909_time_pitch')
    # Each clean comparison normalized to its own raw 20..20k spectral peak.
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    for i, key in enumerate(('reference_medium', 'firmware_g1_medium')):
        x, rate, label = series[key]
        freq, energy = spectrum(x, rate)
        norm = energy[(freq >= 20) & (freq <= 20000)].max()
        sf, se = smooth_spectrum(freq, energy)
        for ax in axes:
            ax.semilogx(freq, 10 * np.log10(np.maximum(energy / norm, 1e-14)), color=COLORS[i], alpha=.23, linewidth=.7)
            ax.semilogx(sf, 10 * np.log10(np.maximum(se / norm, 1e-14)), color=COLORS[i], linewidth=1.8, label=label)
    for ax in axes:
        ax.set_ylim(-75, 3)
        ax.set_ylabel('dB / own raw spectral maximum')
        ax.grid(True, alpha=.25)
        ax.legend(loc='best')
    axes[0].set_xlim(20, 20000)
    axes[1].set_xlim(20, 200)
    axes[0].set_title('600 ms clean-kick spectra: faint = raw; thick = 1/6-octave power average')
    axes[1].set_title('Bass detail: same spectra and normalization; raw dips remain visible')
    axes[1].set_xticks([30, 36.7, 40, 50, 60, 73.4, 90, 120, 180])
    axes[1].set_xticklabels(['30', '36.7', '40', '50', '60', '73.4', '90', '120', '180'], fontsize=9)
    axes[1].set_xlabel('Frequency (Hz)')
    save_plot(fig, args.output, '909_spectrum')
    # Full-LINE D2 spectra use one common reference, never per-trace gain.
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    freq, clean_energy = spectrum(series['d2_clean_sub'][0], RATE)
    common = clean_energy[(freq >= 20) & (freq <= 20000)].max()
    for i, key in enumerate(('d2_clean_body', 'd2_clean_sub', 'd2_mackie_max', 'd2_tube_max')):
        x, rate, label = series[key]
        freq, energy = spectrum(x, rate)
        sf, se = smooth_spectrum(freq, energy)
        for ax in axes:
            ax.semilogx(freq, 10 * np.log10(np.maximum(energy / common, 1e-14)), color=COLORS[i], alpha=.20, linewidth=.65)
            ax.semilogx(sf, 10 * np.log10(np.maximum(se / common, 1e-14)), color=COLORS[i], linewidth=1.7, label=label)
    for ax in axes:
        ax.grid(True, alpha=.25)
        ax.legend(loc='best', fontsize=9)
        ax.set_ylabel('dB / clean SUB 1 raw spectral maximum')
        ax.set_ylim(-65, 8)
    axes[0].set_xlim(20, 20000)
    axes[1].set_xlim(20, 200)
    for hz in (36.7081, 73.4162):
        axes[1].axvline(hz, color='#777777', linestyle=':', alpha=.6)
    axes[0].set_title('D2 at full LINE: faint = raw; thick = 1/6-octave power average')
    axes[1].set_title('D2 bass detail: 36.7 Hz optional sub, 73.4 Hz body; one common gain reference')
    axes[1].set_xticks([30, 36.7, 40, 50, 60, 73.4, 90, 120, 180])
    axes[1].set_xticklabels(['30', '36.7', '40', '50', '60', '73.4', '90', '120', '180'], fontsize=9)
    axes[1].set_xlabel('Frequency (Hz)')
    save_plot(fig, args.output, 'd2_low_end')
    fig, axes = plt.subplots(2, 1, figsize=(12, 6))
    for i, key in enumerate(('d2_clean_sub', 'd2_mackie_max', 'd2_mackie_tailgate')):
        x, rate, label = series[key]
        t = np.arange(len(x)) / float(rate)
        axes[0].plot(t * 1000, x, color=COLORS[i], alpha=.6, label=label)
        t, rms = envelope(x, rate)
        axes[1].plot(t * 1000, db(rms), color=COLORS[i], label=label)
    for ax in axes:
        ax.set_xlim(0, 500)
        ax.grid(True, alpha=.25)
        ax.legend(loc='best', fontsize=8)
    axes[0].set_ylabel('Output amplitude (full LINE)')
    axes[0].set_title('Actual D2 output and intentional tail-gap example')
    axes[1].set_ylabel('20 ms RMS dBFS')
    axes[1].set_ylim(-70, 0)
    axes[1].set_xlabel('Time from trigger (ms)')
    save_plot(fig, args.output, 'd2_envelope')
    if sha(source) != source_sha:
        raise RuntimeError('Firmware source changed during render; rerun for consistent source provenance')
    valley_lines = []
    common = spectrum(series['d2_clean_sub'][0], RATE)[1]
    grid = spectrum(series['d2_clean_sub'][0], RATE)[0]
    common_peak = common[(grid >= 20) & (grid <= 20000)].max()
    for key in ('d2_clean_sub', 'd2_mackie_max', 'd2_tube_max'):
        freq, energy = spectrum(series[key][0], RATE)
        sf, se = smooth_spectrum(freq, energy)
        valley = {}
        for name, ff, ee in (('raw', freq, energy), ('sixth_octave', sf, se)):
            level = 10 * np.log10(np.maximum(ee / common_peak, 1e-14))
            selected = np.where((ff >= 40) & (ff <= 73.4162))[0]
            k = selected[np.argmin(level[selected])]
            starts = np.linspace(40, 80, 4001)
            falls = np.interp(starts, ff, level) - np.interp(starts + 10, ff, level)
            drop = int(np.argmax(falls))
            valley[name] = dict(valley_hz=float(ff[k]), valley_db_relative_common_raw_peak=float(level[k]),
                               largest_10hz_fall_db=float(falls[drop]), fall_start_hz=float(starts[drop]),
                               drop_40_to_50_hz_db=float(np.interp(40, ff, level) - np.interp(50, ff, level)))
        metrics[key]['interpeak_valley'] = valley
        raw, smooth = valley['raw'], valley['sixth_octave']
        valley_lines.append('| %s | %.2f Hz / %.2f dB | %.2f Hz / %.2f dB | %.2f dB |' % (metrics[key]['label'], raw['valley_hz'], raw['valley_db_relative_common_raw_peak'], smooth['valley_hz'], smooth['valley_db_relative_common_raw_peak'], smooth['drop_40_to_50_hz_db']))
    valley_report = '### Bass trough measurements\n\nThe octave pair settles at 36.7 and 73.4 Hz. The table measures the region between those tones against the common clean SUB 1 raw spectral maximum.\n\n| Signal | Raw valley | 1/6-octave valley | Smoothed 40-to-50 Hz fall |\n|---|---:|---:|---:|\n' + '\n'.join(valley_lines) + '\n\nThese pitched signals do not form a flat broadband bass shelf. A spectral valley is not itself evidence of destructive mixing: compare isolated paths and phase traces as well. Raw and smoothed results are both retained; no minima are interpolated away.'
    delta_lines = []
    for key in ('d2_mackie_max', 'd2_tube_max'):
        entry, clean = metrics[key], metrics['d2_clean_sub']
        changes = {band: 10 * math.log10(entry['raw_band_energy'][band] / clean['raw_band_energy'][band]) for band in ('30_40_hz', '40_90_hz', '30_90_hz')}
        entry['energy_change_vs_d2_clean_sub_db'] = changes
        entry['bpf_default_centers_hz'] = [330, 700, 1450]
        delta_lines.append('| %s | %+.4f dB | %+.4f dB | %+.4f dB |' % (entry['label'], changes['30_40_hz'], changes['40_90_hz'], changes['30_90_hz']))
    delta_report = 'Absolute energy change versus D2 clean SUB 1 at identical LINE gain (these values are not normalized per signal):\n\n| Signal | 30-40 Hz | 40-90 Hz | 30-90 Hz |\n|---|---:|---:|---:|\n' + '\n'.join(delta_lines) + '\n\nThese character examples also change WAVE to 1; the table measures the complete preset change, not distortion alone.'
    document = dict(firmware_sha256=source_sha, renderer_sha256=sha(binary),
                    methods=dict(window_seconds=WINDOW, edge_taper_ms=2, zero_padded_fft=NFFT,
                                 smoothing='linear-power average in full 1/6-octave bands',
                                 pitch='linearly interpolated positive crossings; cycle peak above -35dB; 30..500Hz',
                                 root_projection_window_ms=[80, 280], reference_audio_not_redistributed=True), cases=metrics)
    with open(os.path.join(args.output, 'metrics.json'), 'w') as stream:
        json.dump(document, stream, indent=2, sort_keys=True)
    lines = []
    for key in ('reference_medium', 'firmware_g1_medium', 'reference_long', 'firmware_g1_long', 'd2_clean_body', 'd2_clean_sub', 'd2_mackie_max', 'd2_tube_max'):
        m = metrics[key]
        frac = m['band_energy_fraction_of_20_20000_hz']
        lines.append('| %s | %.2f | %.2f%% | %.2f%% | %.2f%% | %.2f | %.2f |' %
                     (m['label'], m['sample_peak_dbfs'], 100*frac['30_40_hz'], 100*frac['40_90_hz'], 100*m['root_band_fraction_20_20000'], m['body_projection_80_280_ms_dbfs'], m['octave_projection_80_280_ms_dbfs']))
    report = '''# Clean analog-reference and D2 bass comparison

Actual firmware host renders are compared with separately downloaded, attributed original TR-909 recordings. This is an engineering comparison, not a claim of circuit-level emulation. Reference WAVs are **not included**. Only the four firmware audition WAVs are exported here.

## Results

![Time, envelope, pitch](909_time_pitch.png)
![Clean spectra](909_spectrum.png)
![D2 full-LINE low end](d2_low_end.png)
![D2 output and tail gap](d2_envelope.png)

Energy percentages use the same first 600 ms, integrated relative to each signal's 20 Hz-20 kHz energy. They describe energy distribution, not frequency-response flatness. Projection columns measure a Hann-weighted sinusoidal component over 80-280 ms; the reference's slightly drifting pitch is not an exact musical note. Raw dBFS across hardware recording and firmware have different gains and must not be read as absolute loudness comparisons.

| Signal | Peak dBFS | 30-40 Hz energy | 40-90 Hz energy | Body-root 1/6-octave energy | Body projection dBFS | Octave projection dBFS |
|---|---:|---:|---:|---:|---:|---:|
%s

%s

%s

D2 settles at 73.416 Hz, with its optional octave at 36.708 Hz. The octave and body now use one phase clock, so both sweep together. Clean/dirty lanes use matched 180 Hz fourth-order Linkwitz-Riley filters, with SUB excluded from distortion. PUNCH is the clean lane gain; mixer2 sums external FX and uses gentle continuous bus compression with 0.8 output trim. These two pitched components do not form a flat 30-90 Hz spectrum. SUB 0 leaves the body sounding; SUB 1 fills the octave region. The raw curves retain window-related interference and spectral minima. Strong distortion changes upper harmonics; its bass retention can be evaluated with the shared-gain D2 plots and the absolute metrics, rather than per-trace spectral normalization.

The 909 and firmware share a falling-pitch body, but their amplitude curves and harmonic distribution differ. The reference's late decay accelerates; the firmware holds through pitch settling plus one sub cycle, followed by exponential decay. The medium/long knob settings below are selected comparisons, not a numerical fit or a claim that knob percentages correspond across instruments. Observe the first 30 ms panel for transient differences and the pitch panel for the remaining trajectory mismatch. The audio reference is one producer-attributed original unit with undisclosed interface, serial number and accent setting.

## Exact settings

All renders: actual `daisy-kick/host/kick_host.cpp` -> firmware `AudioCallback`, 48 kHz; LINE=1, PUNCH=1, performance filters/reverb/reverse/tail-gap off (the fixed crossover remains active), CURVE=64/127, BELLY=64/127, sweep-time/depth trims=.55, gate=20 ms. These continuous controls are rounded to MIDI CC values by the host. The host's 250 ms control-settling silence is removed exactly; no per-file time warping or onset optimization is applied.

- G1 clean comparison: MIDI note 31 (48.999 Hz), SHAPE=.50 (~90 ms pitch-envelope time after MIDI quantization), velocity=70 (pitch depth), WAVE=0, SUB=0, both character amounts=0. DECAY=.30 for the medium reference and .50 for the long reference. The core's intentional body harmonics and short pulse/noise attack remain active.
- D2 clean: MIDI note 38 (73.416 Hz), SHAPE=.50, velocity=100, DECAY=.50, WAVE=0; SUB=0 or 1; character amounts=0.
- D2 maximum character: same D2, SUB=1, WAVE=1, selected Mackie/Tube amount=1, both character return faders=1, BPF blend=1/layers=1 (three layers, initialized centers 330/700/1450 Hz); all other effects off. Exact host settings appear in `metrics.json`.
- Tail-gap audition: D2, SUB=1, WAVE=1, Mackie amount=1, TAIL DELAY=.45. This intentionally removes and restores the body; it is shown separately from the maximum-character spectral comparison.

## Analysis and reproducibility

Run `%s test/daisy_kick_low_end/compare_909.py --reference-dir /path/to/AudioRealism/BassDrum`. Requires numpy and matplotlib; uses Python 2.7 or Python 3. It compiles a fresh host unless `--renderer` is supplied. Firmware SHA256 for this run: `%s`. Renderer SHA256 and reference file hashes are recorded in `metrics.json`.

The clean comparison uses each trace's own 20 Hz-20 kHz raw spectral maximum; the D2 plot uses the clean SUB 1 maximum for **every** trace, preserving level changes. Spectra cover 0-600 ms with 2 ms cosine edge tapers, calculated at native sample rates, zero-padded to 262144 points. Zero padding samples the transform densely; actual frequency resolution remains set by 600 ms. Faint curves are unsmoothed; thick curves average linear spectral power within full 1/6-octave bands. No EQ, gain matching by frequency, or interpolation over dips is applied. Envelopes use 20 ms RMS windows, plotted at window centers. Pitch uses linearly interpolated positive-going crossings, excludes the noisy attack and low-level tail, and measures cycle averages.

The firmware auditions retain original output levels, with 100 ms silence before the single hit. They are 16-bit mono WAV, 48 kHz. The rendered output is a desktop float model with hardware stubs; this does not measure the Seed codec, output electronics, listening room, speakers or perceived loudness.

## Recording attribution and primary references

- [AudioRealism original TR-909 sample pack](https://audiorealism.se/TR-909-SamplePack.html): individual outputs, part volume 50%%, no clipping, 96 kHz capture. `BassDrum909-tune050-attack050-decay050.wav` and `BassDrum909-tune050-attack000-decay100.wav`; verified mono PCM24/96 kHz. The pack's included Readme forbids redistribution of samples whole or in part without permission. Download independently; no analog-reference audio is exported here.
- [Original Roland TR-909 service notes](https://www.polynominal.com/site/studio/gear/drum/roland-tr909/roland-tr909-service-manual.pdf), pp6/9/11: triggered oscillator method, actual centered-knob scope traces, and separate tune/attack/decay paths. Page 7 documents a tune-range capacitor revision; one vintage unit is not a universal waveform target.
- [Official Jomox ModBase 09 MkII manual](https://www.jomox.de/upload/manuals/ModBase09Mk2_E.pdf), pp15-16/19: distinct base pitch and pitch-envelope amount, near-sine body with variable harmonics, pulse/noise attack, decay and smoothing filter. No calibrated ModBase recording was obtained; Jomox validation here is documentary, not quantitative.
''' % ('\n'.join(lines), delta_report, valley_report, '/usr/bin/python2.7', source_sha)
    with open(os.path.join(args.output, 'README.md'), 'w') as stream:
        stream.write(report)
    print('Wrote plots, metrics and firmware-only WAVs to', args.output)
    print('Raw render workspace:', work)


if __name__ == '__main__':
    main()
