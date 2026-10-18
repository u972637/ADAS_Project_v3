#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
퇴치 음원(멧돼지·고라니 각 10개)의 음높이를 1.5~2kHz 쪽으로 올린 버전을 만든다.

tools/eq_filter.py 의 EQ 는 원래 있는 1.5~2kHz 성분의 크기만 바꿀 수 있어서,
에너지가 대부분 550~1000Hz 에 몰린 원본(개 짖음·늑대 울음)에는 효과가
약했다. 여기서는 소리 전체의 음높이를 올려 그 에너지 자체를 1.5~2kHz 로
옮긴다.

- 클립마다 0 ~ MAX_SEMITONES 반음을 SEMITONE_STEP 단위로 실제로 올려 보고,
  1.5~2kHz 비중이 가장 큰 값을 고른다 (클립마다 에너지가 몰린 위치가 달라서
  전부 같은 만큼 올리면 오히려 대역을 벗어나는 클립이 생김).
- 길이·리듬은 그대로 두고 음높이만 올린다 (phase vocoder 로 늘린 뒤 다시
  줄이는 방식). 많이 올릴수록 작은 개·장난감 소리처럼 변하고, 짖는 소리의
  시작 부분이 약간 뭉개지거나 울리는 듯한 잡음이 생길 수 있다.
- RMS 는 원본과 맞추고 최대값은 TARGET_PEAK_DBFS 이하 (eq_filter.py 와 동일).

출력 (animal/settings.py 의 AUDIO_VARIANT_DIRS, 원본과 같은 폴더 구조·이름):
  "pitch"    → assets/audio_pitch/     음높이만 올림
  "pitch_eq" → assets/audio_pitch_eq/  올린 뒤 eq_filter.py 의 EQ 까지
다시 실행하면 같은 결과로 덮어쓰고, 원본에서 빠진 클립의 생성물은 지운다.
재생에 쓰려면 animal/settings.py 의 AUDIO_VARIANT = "pitch" 또는 "pitch_eq".

실행:
  uv run python tools/pitch_shift.py                    # 생성 + 클립별 올린 양·비중 표
  uv run python tools/pitch_shift.py --compare boar_07  # 원본 → pitch → pitch+EQ 연속 재생
"""

import argparse
import sys
from fractions import Fraction
from pathlib import Path

import numpy as np
from scipy import signal

TOOLS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = TOOLS_DIR.parent
for path in (PROJECT_ROOT, TOOLS_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from animal import settings
import eq_filter
from eq_filter import (
    band_share, design_eq, find_source_clip, match_level, play_in_order,
    read_pcm16_mono, remove_stale, source_clips, write_pcm16_mono,
)

# ──────────────────────────────────────────────
# 설정 — 이 값들만 바꾸고 다시 실행하면 된다
# ──────────────────────────────────────────────
MAX_SEMITONES = 12.0   # 최대로 올릴 양(반음). 12 = 1옥타브 = 주파수 2배
SEMITONE_STEP = 0.5    # 자동 선택 때 훑는 간격(반음)
N_FFT = 2048           # phase vocoder 프레임 길이 — 48kHz 에서 약 43ms
HOP = N_FFT // 4


def phase_vocoder(spectrum: np.ndarray, rate: float) -> np.ndarray:
    """STFT 프레임을 rate 간격으로 다시 읽어 길이를 1/rate 배로 바꾼다
    (rate < 1 이면 길어짐). 크기는 이웃 프레임 사이를 보간하고, 위상은 각
    주파수 칸의 실제 회전 속도대로 이어 붙여서 음높이가 유지되게 한다."""
    n_bins, n_frames = spectrum.shape
    expected_advance = 2 * np.pi * HOP * np.arange(n_bins) / N_FFT
    padded = np.pad(spectrum, ((0, 0), (0, 2)))
    steps = np.arange(0, n_frames, rate)
    out = np.empty((n_bins, len(steps)), dtype=complex)
    phase = np.angle(padded[:, 0])
    for index, step in enumerate(steps):
        left, right = padded[:, int(step)], padded[:, int(step) + 1]
        frac = step - int(step)
        magnitude = (1 - frac) * np.abs(left) + frac * np.abs(right)
        out[:, index] = magnitude * np.exp(1j * phase)
        deviation = np.angle(right) - np.angle(left) - expected_advance
        deviation -= 2 * np.pi * np.round(deviation / (2 * np.pi))
        phase += expected_advance + deviation
    return out


def pitch_shift(samples: np.ndarray, semitones: float) -> np.ndarray:
    """길이는 그대로, 음높이만 semitones 반음 올린다."""
    if semitones == 0:
        return samples.copy()
    ratio = 2 ** (semitones / 12)
    stft_args = dict(window="hann", nperseg=N_FFT, noverlap=N_FFT - HOP)
    _, _, spectrum = signal.stft(samples, **stft_args)
    # 1) 음높이는 두고 길이만 ratio 배로 늘린다
    _, stretched = signal.istft(phase_vocoder(spectrum, 1 / ratio), **stft_args)
    # 2) 늘어난 걸 원래 길이로 압축 재생 → 음높이가 ratio 배로 올라감
    fraction = Fraction(ratio).limit_denominator(100)
    shifted = signal.resample_poly(stretched, fraction.denominator, fraction.numerator)
    out = np.zeros_like(samples)
    count = min(len(samples), len(shifted))
    out[:count] = shifted[:count]
    return out


def best_shift(samples: np.ndarray, sample_rate: int) -> tuple[float, np.ndarray]:
    """0 ~ MAX_SEMITONES 중 1.5~2kHz 비중이 가장 큰 올린 양과 그 결과."""
    candidates = np.arange(0, MAX_SEMITONES + SEMITONE_STEP / 2, SEMITONE_STEP)
    best = (-1.0, 0.0, samples)
    for semitones in candidates:
        shifted = pitch_shift(samples, float(semitones))
        share = band_share(shifted, sample_rate)
        if share > best[0]:
            best = (share, float(semitones), shifted)
    return best[1], best[2]


def generate() -> None:
    source = settings.AUDIO_VARIANT_DIRS["original"]
    pitch_dir = settings.AUDIO_VARIANT_DIRS["pitch"]
    pitch_eq_dir = settings.AUDIO_VARIANT_DIRS["pitch_eq"]
    clips = source_clips()
    if not clips:
        raise SystemExit(f"[ERROR] 원본 음원이 없음: {source}/<동물>/*.wav")
    print(f"[PITCH] 클립별 자동, 0~{MAX_SEMITONES:g}반음 ({SEMITONE_STEP:g}반음 간격) "
          f"중 {eq_filter.EQ_PASS_HZ[0]:.0f}~{eq_filter.EQ_PASS_HZ[1]:.0f}Hz 비중 최대")
    print(f"[EQ]    {eq_filter.EQ_DESCRIPTION}")
    remove_stale(pitch_dir, clips)
    remove_stale(pitch_eq_dir, clips)

    taps_by_rate: dict[int, np.ndarray] = {}
    for index, path in enumerate(clips):
        relative = path.relative_to(source)
        rate, original = read_pcm16_mono(path)
        if rate not in taps_by_rate:
            taps_by_rate[rate] = design_eq(rate)
            eq_filter.check_response(taps_by_rate[rate], rate)
        if index == 0:
            print(f"\n{'파일':28s} {'올린 양':>14s}   1.5~2kHz 비중: 원본 → pitch → pitch+EQ")
        semitones, shifted = best_shift(original, rate)
        pitch_out, pitch_limited = match_level(original, shifted)
        eq_out, eq_limited = match_level(
            original, signal.fftconvolve(shifted, taps_by_rate[rate], mode="same"))
        pitch_written = write_pcm16_mono(pitch_dir / relative, rate, pitch_out)
        eq_written = write_pcm16_mono(pitch_eq_dir / relative, rate, eq_out)
        flags = [name for name, hit in (("pitch", pitch_limited), ("pitch+EQ", eq_limited)) if hit]
        print(f"{str(relative):28s} {semitones:+5.1f}반음 (x{2 ** (semitones / 12):.2f})   "
              f"{band_share(original, rate) * 100:5.1f}% → "
              f"{band_share(pitch_written, rate) * 100:5.1f}% → "
              f"{band_share(eq_written, rate) * 100:5.1f}%"
              f"{'  (피크 제한: ' + ', '.join(flags) + ')' if flags else ''}")

    print(f"\n[OK] {len(clips)}개씩 생성 → {pitch_dir}, {pitch_eq_dir}")


def compare(name: str) -> None:
    original = find_source_clip(name)
    relative = original.relative_to(settings.AUDIO_VARIANT_DIRS["original"])
    play_in_order([("원본    ", original),
                   ("pitch   ", settings.AUDIO_VARIANT_DIRS["pitch"] / relative),
                   ("pitch+EQ", settings.AUDIO_VARIANT_DIRS["pitch_eq"] / relative)])


def main() -> None:
    parser = argparse.ArgumentParser(description="퇴치 음원 음높이를 1.5~2kHz 쪽으로 올림")
    parser.add_argument("--compare", metavar="CLIP",
                        help="원본 → pitch → pitch+EQ 순서로 재생 (예: boar_07)")
    args = parser.parse_args()
    if args.compare:
        compare(args.compare)
    else:
        generate()


if __name__ == "__main__":
    main()
