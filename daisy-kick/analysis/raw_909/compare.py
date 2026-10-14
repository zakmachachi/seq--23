# -*- coding: utf-8 -*-
#!/usr/bin/env python
"""Plot a clean TR-909 reference against Seed raw and previous EXT KICK renders.

Python 2.7 + NumPy + Matplotlib compatible. Reference audio stays outside the
repository; the script accepts the local WAV and host-render paths explicitly.
"""
import argparse
import math
import wave

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


SR = 48000
HOST_PREROLL = int(.25 * SR)


def reference_wav(path):
    f = wave.open(path, 'rb')
    channels = f.getnchannels()
    width = f.getsampwidth()
    rate = f.getframerate()
    frames = f.readframes(f.getnframes())
    f.close()
    if width == 3:
        raw = np.frombuffer(frames, dtype=np.uint8).reshape(-1, channels, 3)
        samples = (raw[:, :, 0].astype(np.int32) |
                   (raw[:, :, 1].astype(np.int32) << 8) |
                   (raw[:, :, 2].astype(np.int32) << 16))
        samples[(samples & 0x800000) != 0] -= 0x1000000
        audio = samples.astype(np.float64) / 8388608.0
    elif width == 2:
        audio = np.frombuffer(frames, dtype='<i2').reshape(-1, channels).astype(np.float64) / 32768.0
    elif width == 4:
        audio = np.frombuffer(frames, dtype='<i4').reshape(-1, channels).astype(np.float64) / 2147483648.0
    else:
        raise ValueError('reference must be PCM 16, 24 or 32 bit')
    audio = np.mean(audio, axis=1)
    if rate != SR:
        if rate == 96000 and SR == 48000:
            audio = audio[::2]
        else:
            raise ValueError('expected 48 or 96 kHz reference WAV')
    return audio


def host_render(path):
    return np.fromfile(path, dtype=np.float32)[HOST_PREROLL:]


def hardcore_segment(path, start, duration=.25):
    """Read one PCM16 stereo mix excerpt and resample it to the host rate."""
    f = wave.open(path, 'rb')
    channels, width, rate = f.getnchannels(), f.getsampwidth(), f.getframerate()
    f.setpos(int(start * rate))
    frames = f.readframes(int(duration * rate))
    f.close()
    if width != 2:
        raise ValueError('hardcore comparison excerpt must be PCM16')
    audio = np.frombuffer(frames, dtype='<i2').reshape(-1, channels).astype(np.float64) / 32768.0
    audio = np.mean(audio, axis=1)
    if rate != SR:
        positions = np.arange(int(duration * SR), dtype=np.float64) * rate / SR
        audio = np.interp(positions, np.arange(len(audio)), audio)
    return audio


def spectrum(x, duration=.25):
    count = min(len(x), int(duration * SR))
    y = np.asarray(x[:count], dtype=np.float64)
    window = np.hanning(count)
    fft = np.abs(np.fft.rfft(y * window)) / max(np.sum(window), 1.0)
    freq = np.fft.rfftfreq(count, 1.0 / SR)
    body = (freq >= 30) & (freq < 60)
    ref_level = math.sqrt(np.sum(fft[body] ** 2))
    db = 20.0 * np.log10(np.maximum(fft / max(ref_level, 1e-12), 1e-8))
    return freq, db


def envelope_db(x, duration=.23):
    count = min(len(x), int(duration * SR))
    step = int(.010 * SR)
    peak = max(np.max(np.abs(x[:count])), 1e-12)
    times, values = [], []
    for start in range(0, count - step + 1, step):
        rms = math.sqrt(np.mean(np.asarray(x[start:start + step], dtype=np.float64) ** 2))
        times.append((start + step / 2.0) * 1000.0 / SR)
        values.append(20.0 * math.log10(max(rms / peak, 1e-8)))
    return times, values


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--reference', required=True)
    ap.add_argument('--old-processed', required=True)
    ap.add_argument('--old-passthrough', required=True)
    ap.add_argument('--raw-model', required=True)
    ap.add_argument('--raw-ext', required=True)
    ap.add_argument('--mackie', default=None)
    ap.add_argument('--tube', default=None)
    ap.add_argument('--hardcore', default=None)
    ap.add_argument('--hardcore-start', type=float, default=29.10)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()

    curves = [
        ('909 reference', reference_wav(args.reference)),
        ('old EXT KICK chain', host_render(args.old_processed)),
        ('old ext passthrough', host_render(args.old_passthrough)),
        ('raw oscillator', host_render(args.raw_model)),
        ('raw EXT KICK', host_render(args.raw_ext)),
    ]
    if args.mackie:
        curves.append(('Mackie + one BPF', host_render(args.mackie)))
    if args.tube:
        curves.append(('Tube + one BPF', host_render(args.tube)))
    if args.hardcore:
        curves.append(('TaMaHaTa mix excerpt', hardcore_segment(args.hardcore, args.hardcore_start)))
    colors = ['#111111', '#dc503c', '#888888', '#2171b5', '#31a354',
              '#9467bd', '#ff7f0e', '#8c564b']
    fig, axes = plt.subplots(2, 1, figsize=(11, 8))
    for (name, audio), color in zip(curves, colors):
        freq, db = spectrum(audio)
        mask = (freq >= 25) & (freq <= 18000)
        axes[0].semilogx(freq[mask], db[mask], color=color, linewidth=1.5, label=name)
        t, env = envelope_db(audio)
        axes[1].plot(t, env, color=color, linewidth=1.5, label=name)
    axes[0].set_xlim(25, 18000)
    axes[0].set_ylim(-100, 25)
    axes[0].set_ylabel('Spectrum (dB relative to 30-60 Hz energy)')
    axes[0].set_xlabel('Frequency (Hz)')
    axes[0].grid(True, which='both', alpha=.25)
    axes[0].legend(loc='lower left', fontsize=8)
    axes[1].set_xlim(0, 230)
    axes[1].set_ylim(-50, 2)
    axes[1].set_ylabel('10 ms RMS (dB relative to each peak)')
    axes[1].set_xlabel('Time after trigger (ms)')
    axes[1].grid(True, alpha=.25)
    axes[1].legend(loc='lower left', fontsize=8)
    fig.suptitle('TR-909 clean reference vs Seed kick paths')
    fig.tight_layout()
    fig.savefig(args.output, dpi=160)


if __name__ == '__main__':
    main()
