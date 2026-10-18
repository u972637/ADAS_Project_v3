#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
동물별 퇴치 음원(멧돼지·고라니 각 10개)에 1.5~2kHz 강조 EQ 를 씌운다.

EQ = 주파수별 볼륨 조절. 스피커 지향성이 잘 나타나는 1.5~2kHz 는 그대로
두고(0dB) 나머지 대역은 EQ_CUT_DB 만큼 줄여서, 상대적으로 그 대역이
강조되게 한다 (audio_test/with_reflector.csv 기준 2000Hz 는 45~90°에서
0° 대비 약 20~23dB 줄어 지향성이 가장 뚜렷한 축에 속한다).

   dB
    0 ┤             ┌─────┐
      │            ╱       ╲
  -12 ┤───────────╱         ╲──────────
      └──────────┬───┬─────┬───┬──────── Hz
               1.0k 1.5k  2.0k 3.0k

- 선형 위상 FIR(scipy firwin2) — 대역별 볼륨만 바뀌고 소리의 시간 모양은
  그대로다. 필터 지연은 잘라내서 원본과 길이·타이밍이 같다.
- 대역 밖을 줄인 만큼 전체가 작아지므로 RMS 를 원본과 맞춘다 (원본과 비교할
  때 "소리가 커서 좋게 들리는" 착각 방지). 그 결과 최대값이 TARGET_PEAK_DBFS
  를 넘으면 클립 전체를 그만큼 낮춰 클리핑을 막는다.
- EQ 는 이미 있는 소리의 크기만 바꾼다 — 원본에 1.5~2kHz 성분이 거의 없는
  클립은 강조 효과가 약하다. 생성 후 출력되는 표의 "대역 비중"으로 확인.

입력: animal/settings.py 의 AUDIO_VARIANT_DIRS["original"] (assets/audio/<동물>/*.wav)
출력: AUDIO_VARIANT_DIRS["eq"] (assets/audio_eq/<동물>/*.wav) — 같은 폴더
      구조·파일 이름. 다시 실행하면 같은 결과로 덮어쓰고, 원본에서 빠진
      클립의 EQ 파일은 지운다(재생은 폴더 안 WAV 를 전부 쓰므로 남아 있으면
      안 됨).
재생에 쓰려면 animal/settings.py 의 AUDIO_VARIANT = "eq".
(피치를 1.5~2kHz 쪽으로 올리는 버전은 tools/pitch_shift.py — 여기 함수들을
 그대로 가져다 쓴다.)

실행:
  uv run python tools/eq_filter.py                    # 20개 생성 + 비교표 출력
  uv run python tools/eq_filter.py --compare boar_07  # 원본 → 1초 → EQ 연속 재생
"""

import argparse
import platform
import subprocess
import sys
import time
import wave
from pathlib import Path

import numpy as np
from scipy import signal

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from animal import settings

# ──────────────────────────────────────────────
# EQ 설정 — 이 값들만 바꾸고 다시 실행하면 된다
# ──────────────────────────────────────────────
EQ_PASS_HZ = (1500.0, 2000.0)        # 그대로 둘(= 강조할) 대역
EQ_TRANSITION_HZ = (1000.0, 3000.0)  # 이 바깥은 EQ_CUT_DB, 안쪽으로 서서히 0dB 로 전환
EQ_CUT_DB = -12.0                    # 강조 대역 밖 감쇠량
NUMTAPS = 2001                       # FIR 길이(홀수) — 48kHz 에서 약 42ms
TARGET_PEAK_DBFS = -1.0              # 크기 보정 후 허용 최대값

PCM16_MAX = 32767
COMPARE_GAP_SECONDS = 1.0

EQ_DESCRIPTION = (
    f"EQ: {EQ_PASS_HZ[0] / 1000:g}-{EQ_PASS_HZ[1] / 1000:g} kHz 0 dB, "
    f"below {EQ_TRANSITION_HZ[0] / 1000:g} kHz / above {EQ_TRANSITION_HZ[1] / 1000:g} kHz "
    f"{EQ_CUT_DB:g} dB (linear-phase FIR, {NUMTAPS} taps), RMS matched to original, "
    f"peak <= {TARGET_PEAK_DBFS:g} dBFS"
)


def design_eq(sample_rate: int) -> np.ndarray:
    nyquist = sample_rate / 2
    knots_hz = [0.0, EQ_TRANSITION_HZ[0], EQ_PASS_HZ[0], EQ_PASS_HZ[1],
                EQ_TRANSITION_HZ[1], nyquist]
    knots_db = [EQ_CUT_DB, EQ_CUT_DB, 0.0, 0.0, EQ_CUT_DB, EQ_CUT_DB]
    # firwin2 는 넘겨준 점 사이를 진폭(선형)으로 보간하므로, 전환 구간이 dB
    # 기준으로 곧게 이어지도록 곡선을 촘촘히 찍어서 넘긴다.
    freq = np.linspace(0.0, nyquist, 4097)
    gain = 10 ** (np.interp(freq, knots_hz, knots_db) / 20)
    return signal.firwin2(NUMTAPS, freq, gain, fs=sample_rate)


def response_db(taps: np.ndarray, sample_rate: int, freqs_hz) -> np.ndarray:
    _, response = signal.freqz(taps, worN=np.asarray(freqs_hz, dtype=float), fs=sample_rate)
    return 20 * np.log10(np.abs(response))


def check_response(taps: np.ndarray, sample_rate: int, tolerance_db: float = 0.5) -> None:
    """NUMTAPS 등을 바꿨다가 설계가 목표 곡선을 못 따라가면 바로 멈춘다."""
    center = (EQ_PASS_HZ[0] + EQ_PASS_HZ[1]) / 2
    checks = {center: 0.0,
              EQ_TRANSITION_HZ[0] / 2: EQ_CUT_DB,
              EQ_TRANSITION_HZ[1] * 2: EQ_CUT_DB}
    actual = response_db(taps, sample_rate, list(checks))
    for (freq, expected), got in zip(checks.items(), actual):
        if abs(got - expected) > tolerance_db:
            raise RuntimeError(f"EQ 응답 {freq:.0f}Hz = {got:+.2f}dB "
                               f"(목표 {expected:+.1f}dB ±{tolerance_db}dB)")
    print("[EQ 응답] " + "  ".join(f"{freq:.0f}Hz {got:+.2f}dB"
                                    for freq, got in zip(checks, actual)))


def read_pcm16_mono(path: Path) -> tuple[int, np.ndarray]:
    with wave.open(str(path), "rb") as reader:
        if reader.getnchannels() != 1 or reader.getsampwidth() != 2:
            raise ValueError(f"16bit 모노 WAV 만 지원: {path}")
        raw = reader.readframes(reader.getnframes())
        return reader.getframerate(), np.frombuffer(raw, dtype="<i2").astype(np.float64)


def write_pcm16_mono(path: Path, sample_rate: int, samples: np.ndarray) -> np.ndarray:
    path.parent.mkdir(parents=True, exist_ok=True)
    pcm = np.clip(np.round(samples), -PCM16_MAX - 1, PCM16_MAX).astype("<i2")
    with wave.open(str(path), "wb") as writer:
        writer.setnchannels(1)
        writer.setsampwidth(2)
        writer.setframerate(sample_rate)
        writer.writeframes(pcm.tobytes())
    return pcm.astype(np.float64)


def rms(samples: np.ndarray) -> float:
    return float(np.sqrt(np.mean(samples ** 2)))


def band_share(samples: np.ndarray, sample_rate: int) -> float:
    """전체 에너지 중 EQ_PASS_HZ 대역이 차지하는 비율."""
    power = np.abs(np.fft.rfft(samples)) ** 2
    freqs = np.fft.rfftfreq(len(samples), 1 / sample_rate)
    in_band = (freqs >= EQ_PASS_HZ[0]) & (freqs <= EQ_PASS_HZ[1])
    return float(power[in_band].sum() / power.sum())


def match_level(reference: np.ndarray, processed: np.ndarray) -> tuple[np.ndarray, bool]:
    """processed 의 RMS 를 reference(원본)와 맞춘 결과, 그리고 피크 제한이
    걸렸는지 — 최대값이 TARGET_PEAK_DBFS 를 넘으면 그만큼 낮춘다."""
    processed_peak = float(np.abs(processed).max())
    if processed_peak == 0:
        return processed, False
    gain = rms(reference) / rms(processed)
    peak_limit = PCM16_MAX * 10 ** (TARGET_PEAK_DBFS / 20)
    limited = processed_peak * gain > peak_limit
    if limited:
        gain = peak_limit / processed_peak
    return processed * gain, limited


def apply_eq(samples: np.ndarray, taps: np.ndarray) -> tuple[np.ndarray, bool]:
    """EQ 를 걸고 RMS 를 원본과 맞춘 결과, 그리고 피크 제한이 걸렸는지."""
    # 홀수 길이 대칭 FIR 에 mode="same" 이면 지연(NUMTAPS//2 샘플)이 잘려나가
    # 원본과 길이·타이밍이 같다.
    return match_level(samples, signal.fftconvolve(samples, taps, mode="same"))


def source_clips() -> list[Path]:
    """원본 폴더의 <동물>/*.wav — 재생 쪽(AudioCatalog.from_directory)과 같은 규칙."""
    return sorted(settings.AUDIO_VARIANT_DIRS["original"].glob("*/*.wav"))


def remove_stale(target: Path, clips: list[Path]) -> None:
    """원본에서 빠진 클립의 생성물을 지운다 — 재생은 폴더 안 WAV 를 전부 쓴다."""
    source = settings.AUDIO_VARIANT_DIRS["original"]
    stale = ({path.relative_to(target) for path in target.glob("*/*.wav")}
             - {path.relative_to(source) for path in clips})
    for relative in sorted(stale):
        (target / relative).unlink()
        print(f"[삭제] 원본에 없는 생성 음원: {target / relative}")


def find_source_clip(name: str) -> Path:
    stem = Path(name).stem
    matches = [path for path in source_clips() if path.stem == stem]
    if not matches:
        raise SystemExit(f"[ERROR] 클립을 찾을 수 없음: {name} (예: boar_07, water_deer_03)")
    return matches[0]


def play_in_order(labeled_paths: list[tuple[str, Path]]) -> None:
    """(이름, 파일) 순서대로 COMPARE_GAP_SECONDS 씩 쉬며 재생."""
    for label, path in labeled_paths:
        if not path.is_file():
            raise SystemExit(f"[ERROR] 음원이 없음: {path} — 먼저 인자 없이 실행해 생성하세요.")
    command = settings.AUDIO_PLAYER_COMMAND or (
        "afplay" if platform.system() == "Darwin" else "aplay")
    for index, (label, path) in enumerate(labeled_paths):
        if index:
            time.sleep(COMPARE_GAP_SECONDS)
        print(f"[재생] {label}: {path}")
        subprocess.run([command, str(path)], check=True)


def generate() -> None:
    source, target = settings.AUDIO_VARIANT_DIRS["original"], settings.AUDIO_VARIANT_DIRS["eq"]
    clips = source_clips()
    if not clips:
        raise SystemExit(f"[ERROR] 원본 음원이 없음: {source}/<동물>/*.wav")
    print(f"[EQ] {EQ_DESCRIPTION}")
    remove_stale(target, clips)

    taps_by_rate: dict[int, np.ndarray] = {}
    for index, path in enumerate(clips):
        relative = path.relative_to(source)
        rate, original = read_pcm16_mono(path)
        if rate not in taps_by_rate:
            taps_by_rate[rate] = design_eq(rate)
            check_response(taps_by_rate[rate], rate)
        if index == 0:
            print(f"\n{'파일':28s} {'1.5~2kHz 비중':>15s} {'RMS':>13s} {'peak':>13s}")
        shaped, limited = apply_eq(original, taps_by_rate[rate])
        written = write_pcm16_mono(target / relative, rate, shaped)
        print(f"{str(relative):28s} "
              f"{band_share(original, rate) * 100:5.1f}% → {band_share(written, rate) * 100:5.1f}% "
              f"{rms(original):5.0f} → {rms(written):5.0f} "
              f"{np.abs(original).max():5.0f} → {np.abs(written).max():5.0f}"
              f"{'  (피크 제한)' if limited else ''}")

    print(f"\n[OK] {len(clips)}개 생성 → {target}")
    print("     (피크 제한) = RMS 를 원본만큼 올리면 클리핑돼서 그보다 작게 맞춘 클립")


def compare(name: str) -> None:
    original = find_source_clip(name)
    relative = original.relative_to(settings.AUDIO_VARIANT_DIRS["original"])
    play_in_order([("원본", original),
                   ("EQ  ", settings.AUDIO_VARIANT_DIRS["eq"] / relative)])


def main() -> None:
    parser = argparse.ArgumentParser(description="퇴치 음원에 1.5~2kHz 강조 EQ 적용")
    parser.add_argument("--compare", metavar="CLIP",
                        help="원본 → 1초 → EQ 순서로 재생 (예: boar_07)")
    args = parser.parse_args()
    if args.compare:
        compare(args.compare)
    else:
        generate()


if __name__ == "__main__":
    main()
