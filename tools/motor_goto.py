#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
지정한 픽셀 좌표 (x, y)로 Yaw/Pitch 짐벌을 이동시키고 그 자리를 유지한다.

main.py(카메라+YOLO) 없이, 좌표만 직접 줘서 실제 운영 Tracker(두 축 모두,
motor/dm4310_tracker.py)를 단독으로 구동해보고 싶을 때 쓴다.

Tracker.update(x, y) 를 한 번만 호출하면 내부 1000Hz 제어 스레드가 알아서
그 목표를 계속 유지하므로(MIT 임피던스가 버팀), 이 스크립트는 상태를
주기적으로 출력하며 대기하다가 Ctrl+C 로 종료한다(모터 비활성화+포트 닫힘).

좌표계: motor/dm4310_tracker.py 의 FRAME_W/FRAME_H(기본 2592x1944, 카메라
해상도) 기준 — 좌측 상단이 (0,0), 우측 하단이 (FRAME_W,FRAME_H).

실행:
  uv run python tools/motor_goto.py 1200 900   # 그 좌표로 이동 후 유지
  uv run python tools/motor_goto.py            # 대화형으로 좌표 입력
"""

import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from motor.dm4310_tracker import Tracker, FRAME_W, FRAME_H

STATUS_INTERVAL = 0.5  # 상태 출력 주기(초)


def read_target_xy():
    if len(sys.argv) >= 3:
        return int(sys.argv[1]), int(sys.argv[2])

    x = int(input(f"목표 x (0~{FRAME_W}, 좌측=0): ").strip())
    y = int(input(f"목표 y (0~{FRAME_H}, 상단=0): ").strip())
    return x, y


def main():
    x, y = read_target_xy()
    print(f"[목표 좌표] x={x}, y={y}  (프레임 기준 {FRAME_W}x{FRAME_H})")

    print("[..] 모터 연결 중...")
    tracker = Tracker()
    print("[OK] 연결 완료 — 목표 좌표로 이동 명령을 보냈습니다.")

    tracker.update(x, y)

    try:
        while True:
            time.sleep(STATUS_INTERVAL)
            freq = tracker.get_actual_freq()
            yaw_pos, yaw_vel, yaw_tau = tracker.get_state("yaw")
            pitch_pos, pitch_vel, pitch_tau = tracker.get_state("pitch")
            print(f"  [STATUS] freq={freq:6.1f}Hz  "
                  f"yaw={yaw_pos:+.3f}rad({yaw_pos * 180 / 3.141592653589793:+.1f}°)  "
                  f"pitch={pitch_pos:+.3f}rad({pitch_pos * 180 / 3.141592653589793:+.1f}°)")
    except (EOFError, KeyboardInterrupt):
        print("\n[중단] Ctrl+C — 모터 비활성화 중...")
    finally:
        tracker.close()
        print("[OK] 모터 비활성화 완료, 포트 닫힘")


if __name__ == "__main__":
    main()
