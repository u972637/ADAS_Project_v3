#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Yaw(CAN 1)/Pitch(CAN 2) 방향키 수동 위치 제어 테스트.

motor/dm4310_tracker.py 와 동일한 연결 정보·게인(KP/KD)을 그대로 가져와
두 축을 동시에 구동한다 — main.py 의 카메라/YOLO 파이프라인 없이, 키보드로
직접 각도를 조금씩 밀어보고 싶을 때 쓴다.

실행하면 메뉴를 고르기 전에 먼저 두 축을 중앙(0°, 0°)으로 램프 이동시킨다
(motor/dm4310_tracker.py 의 GO_CENTER_DURATION 과 동일한 지속시간 재사용).

메뉴:
  1. START (아직 구현되지 않음 — 추후 main.py 자동 추적(Tracker)과 연결 예정)
  2. 방향키로 위치 제어 (모터 각도를 직접 조그)
  3. 가상 코 좌표 테스트 (wasd로 "코 좌표"를 움직여 실제 Tracker로 추적)
  0. 종료

2번 진입 시 조작 — 모터 각도를 직접 명령:
  a : yaw   +STEP_DEG    d : yaw   -STEP_DEG
  w : pitch +STEP_DEG    s : pitch -STEP_DEG
  q : 메뉴로 복귀 (Ctrl+C 는 프로그램 전체 종료)

한 번 입력될 때마다 STEP_DEG 만큼 누적된다. 이 스크립트가 "키를 누르고
있음"을 직접 감지하는 게 아니라, 터미널/OS 가 키를 꾹 누르고 있을 때 반복
입력해주는 자동반복(auto-repeat) 이벤트를 매번 더하는 방식이라, 꾹 누르고
있으면 계속 쌓인다. STEP_DEG 는 보수적으로(작게) 잡혀 있다 — 필요하면 상수
값만 조정할 것.

안전 가동범위(도) — 나중에 이 값만 고치면 된다:
  YAW_MIN_DEG   / YAW_MAX_DEG   = -175 / 175
  PITCH_MIN_DEG / PITCH_MAX_DEG =  -30 / 180

3번 진입 시 조작 — main.py 가 YOLO 검출로 넘겨주는 "코 좌표"를 흉내낸다.
2번과 달리 모터 각도를 직접 보내는 게 아니라, 노란 점(가상 코 좌표)을
motor/dm4310_tracker.py 의 실제 운영 Tracker.update(x, y) 에 그대로
흘려보낸다 — 픽셀→각도 변환(RAD_PER_PIXEL_X/Y), 부호(SIGN_YAW/PITCH),
가동범위(YAW_RANGE_RAD/PITCH_RANGE_RAD) 등 실제 로직을 그대로 검증한다:
  a : 코 좌표 x -NOSE_STEP_PX (왼쪽)   d : 코 좌표 x +NOSE_STEP_PX (오른쪽)
  w : 코 좌표 y -NOSE_STEP_PX (위)     s : 코 좌표 y +NOSE_STEP_PX (아래)
  q 또는 ESC : 메뉴로 복귀
코 좌표는 항상 프레임 중앙(Tracker 기준 0°, 0°)에서 시작한다.

실행:
  uv run python test/motor_manual_control.py
"""

import math
import os
import select
import sys
import termios
import time
import tty

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import cv2
import numpy as np
import serial

from motor.DM_CAN import Motor, MotorControl, Control_Type
from motor.dm4310_tracker import (
    DEVICENAME, BAUDRATE, MOTOR_TYPE,
    CAN_ID_YAW, CAN_ID_PITCH, MASTER_ID_YAW, MASTER_ID_PITCH,
    KP, KD, GO_CENTER_DURATION,
    Tracker, FRAME_W, FRAME_H, CENTER_X, CENTER_Y,
)

AXES = ("yaw", "pitch")
CAN_ID = {"yaw": CAN_ID_YAW, "pitch": CAN_ID_PITCH}
MASTER_ID = {"yaw": MASTER_ID_YAW, "pitch": MASTER_ID_PITCH}
TAU_FF = 0.0

# 방향키 한 번(자동반복 포함) 입력당 움직일 각도(도). 보수적으로 작게 잡음.
STEP_DEG = 0.3

# 안전 가동범위(도) — 나중에 이 값만 고치면 됨
YAW_MIN_DEG,   YAW_MAX_DEG   = -175.0, 175.0
PITCH_MIN_DEG, PITCH_MAX_DEG =  -30.0, 180.0
_RANGE_DEG = {
    "yaw":   (YAW_MIN_DEG,   YAW_MAX_DEG),
    "pitch": (PITCH_MIN_DEG, PITCH_MAX_DEG),
}

# 제어 스트리밍 주파수(Hz) — 운영 코드(motor/dm4310_tracker.py)와 동일하게 1000Hz
CONTROL_FREQ = 1000
DT = 1.0 / CONTROL_FREQ

# ──────────────────────────────────────────────
# 3번(가상 코 좌표 테스트) 전용 설정
# ──────────────────────────────────────────────

# wasd 한 번 입력당 가상 코 좌표를 움직일 픽셀 수
NOSE_STEP_PX = 20

# 2592x1944 그대로 창을 띄우면 화면보다 커서 축소 표시만 한다
# (Tracker.update() 에는 항상 원본 해상도 좌표를 그대로 넘김).
DISPLAY_SCALE = 0.35


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ──────────────────────────────────────────────
# 터미널 raw 키 입력 (main.py 와 동일한 방식)
# ──────────────────────────────────────────────

def setup_terminal_keyboard():
    if not sys.stdin.isatty():
        return None
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    return old_settings


def restore_terminal_keyboard(old_settings):
    if old_settings is not None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def read_key_nonblocking():
    if not sys.stdin.isatty():
        return None
    readable, _, _ = select.select([sys.stdin], [], [], 0)
    if not readable:
        return None
    return sys.stdin.read(1).lower()


# ──────────────────────────────────────────────
# 모터 제어
# ──────────────────────────────────────────────

class ManualController:
    def __init__(self):
        self.ser = serial.Serial(DEVICENAME, BAUDRATE, timeout=0.001)
        self.ctrl = MotorControl(self.ser)
        self.motor = {}
        for axis in AXES:
            self.motor[axis] = Motor(MOTOR_TYPE, CAN_ID[axis], MASTER_ID[axis])
            self.ctrl.addMotor(self.motor[axis])
            if not self.ctrl.switchControlMode(self.motor[axis], Control_Type.MIT):
                print(f"[WARN] {axis} 축 MIT 모드 전환 확인 불가 — 반응이 없을 수 있음")
            self.ctrl.enable(self.motor[axis])

        self.target_deg = {}
        for axis in AXES:
            self.ctrl.refresh_motor_status(self.motor[axis])
            self.target_deg[axis] = math.degrees(self.motor[axis].getPosition())

    def current_deg(self, axis: str) -> float:
        return math.degrees(self.motor[axis].getPosition())

    def _send(self):
        for axis in AXES:
            q_des = math.radians(self.target_deg[axis])
            self.ctrl.controlMIT(self.motor[axis], KP, KD, q_des, 0.0, TAU_FF)

    def nudge(self, axis: str, delta_deg: float):
        lo, hi = _RANGE_DEG[axis]
        self.target_deg[axis] = clamp(self.target_deg[axis] + delta_deg, lo, hi)

    def ramp_to_center(self, duration: float = GO_CENTER_DURATION):
        print(f"[중앙 복귀] {duration:.1f}초 동안 0°, 0° 로 이동합니다...")
        start = dict(self.target_deg)
        steps = max(int(duration * CONTROL_FREQ), 1)
        dt = duration / steps
        for i in range(1, steps + 1):
            frac = i / steps
            for axis in AXES:
                self.target_deg[axis] = start[axis] + (0.0 - start[axis]) * frac
            self._send()
            time.sleep(dt)
        print(f"  완료: yaw={self.current_deg('yaw'):+.1f}°  "
              f"pitch={self.current_deg('pitch'):+.1f}°")

    def close(self):
        for axis in AXES:
            self.ctrl.disable(self.motor[axis])
        self.ser.close()


# ──────────────────────────────────────────────
# 메뉴 2: 방향키 제어
# ──────────────────────────────────────────────

def run_manual_jog(mc: ManualController):
    print("\n[방향키 제어] a/d=yaw +/-, w/s=pitch +/-, q=메뉴로 복귀 (Ctrl+C=완전 종료)")
    print(f"  스텝: {STEP_DEG}°/입력  범위: yaw[{YAW_MIN_DEG:.0f},{YAW_MAX_DEG:.0f}]° "
          f"pitch[{PITCH_MIN_DEG:.0f},{PITCH_MAX_DEG:.0f}]°\n")

    old_settings = setup_terminal_keyboard()
    next_t = time.time()
    next_print = time.time()
    try:
        while True:
            key = read_key_nonblocking()
            if key == 'q':
                print("\n[메뉴로 복귀]")
                break
            elif key == 'a':
                mc.nudge("yaw", +STEP_DEG)
            elif key == 'd':
                mc.nudge("yaw", -STEP_DEG)
            elif key == 'w':
                mc.nudge("pitch", +STEP_DEG)
            elif key == 's':
                mc.nudge("pitch", -STEP_DEG)

            mc._send()

            now = time.time()
            if now >= next_print:
                print(f"\r  yaw 목표={mc.target_deg['yaw']:+7.2f}°(실측{mc.current_deg('yaw'):+7.2f}°)  "
                      f"pitch 목표={mc.target_deg['pitch']:+7.2f}°(실측{mc.current_deg('pitch'):+7.2f}°)   ",
                      end="", flush=True)
                next_print = now + 0.2

            next_t += DT
            sleep_time = next_t - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_t = time.time()
    finally:
        restore_terminal_keyboard(old_settings)
        print()


# ──────────────────────────────────────────────
# 메뉴 3: 가상 코 좌표 테스트
# ──────────────────────────────────────────────

def run_virtual_nose_test():
    """
    wasd 로 "가상 코 좌표"(노란 점)를 움직여 실제 운영 Tracker.update(x, y)
    로 흘려보낸다 — 2번(모터 각도 직접 조그)과 달리 main.py 가 YOLO 검출로
    넘겨주는 입력을 그대로 흉내내는 것이라, 픽셀→각도 환산·부호·가동범위
    로직 전체(motor/dm4310_tracker.py)를 실제로 거친다.

    2번과 포트를 동시에 쓸 수 없으므로(같은 시리얼 장치), 이 함수 안에서
    자체적으로 Tracker 를 새로 열고 끝나면 닫는다 — 호출하는 쪽(main())이
    먼저 ManualController 를 닫아둔 상태여야 한다.
    """
    print("\n[가상 코 좌표 테스트] a/d=좌/우, w/s=상/하 로 노란 점(코 좌표) 이동")
    print(f"  스텝: {NOSE_STEP_PX}px/입력  시작 위치: 중앙({CENTER_X},{CENTER_Y}) "
          f"= Tracker 기준 0°,0°  — q 또는 ESC 로 메뉴 복귀\n")

    print("[..] Tracker 연결 중...")
    tracker = Tracker()
    print("[OK] 연결 완료 — 창이 뜨면 거기서 wasd 를 누르세요.")

    x, y = CENTER_X, CENTER_Y   # 코 좌표는 항상 중앙(=0°,0°)에서 시작
    disp_w, disp_h = int(FRAME_W * DISPLAY_SCALE), int(FRAME_H * DISPLAY_SCALE)
    win_name = "Virtual Nose (q/ESC: back)"
    cv2.namedWindow(win_name)

    try:
        while True:
            canvas = np.zeros((disp_h, disp_w, 3), dtype=np.uint8)
            cx, cy = disp_w // 2, disp_h // 2
            cv2.line(canvas, (cx - 15, cy), (cx + 15, cy), (0, 160, 0), 1)
            cv2.line(canvas, (cx, cy - 15), (cx, cy + 15), (0, 160, 0), 1)

            dot_x, dot_y = int(x * DISPLAY_SCALE), int(y * DISPLAY_SCALE)
            cv2.circle(canvas, (dot_x, dot_y), 8, (0, 255, 255), -1)  # 노란 점(BGR)
            cv2.putText(canvas, f"nose=({x},{y})", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow(win_name, canvas)
            key = cv2.waitKey(33) & 0xFF   # ~30Hz, main.py 검출 루프 주기와 유사

            if key in (ord('q'), 27):
                break
            elif key == ord('a'):
                x = max(0, x - NOSE_STEP_PX)
            elif key == ord('d'):
                x = min(FRAME_W, x + NOSE_STEP_PX)
            elif key == ord('w'):
                y = max(0, y - NOSE_STEP_PX)
            elif key == ord('s'):
                y = min(FRAME_H, y + NOSE_STEP_PX)

            tracker.update(x, y)
    finally:
        cv2.destroyWindow(win_name)
        tracker.close()
        print("\n[메뉴로 복귀] Tracker 연결 종료")


# ──────────────────────────────────────────────
# 메뉴
# ──────────────────────────────────────────────

def print_menu():
    print("\n======================================")
    print(" Yaw/Pitch 수동 제어")
    print("======================================")
    print(" 1. START (아직 구현되지 않음)")
    print(" 2. 방향키로 위치 제어 (모터 각도 직접 조그)")
    print(" 3. 가상 코 좌표 테스트 (wasd로 코 좌표 이동 → 실제 Tracker)")
    print(" 0. 종료")
    print("======================================")


def main():
    print("[..] 모터 연결 중...")
    mc = ManualController()
    print("[OK] 연결 완료")

    mc.ramp_to_center()

    try:
        while True:
            print_menu()
            try:
                choice = input("선택 > ").strip()
            except EOFError:
                break

            if choice == "1":
                print("[안내] 1번(START)은 아직 구현되지 않았습니다.")
            elif choice == "2":
                run_manual_jog(mc)
            elif choice == "3":
                # 3번은 같은 시리얼 포트를 Tracker 로 새로 열어야 해서,
                # ManualController 가 들고 있던 연결을 잠깐 반납했다가
                # 돌아오면 다시 연다.
                mc.close()
                run_virtual_nose_test()
                print("[..] 모터 재연결 중...")
                mc = ManualController()
                print("[OK] 재연결 완료")
            elif choice == "0":
                print("종료합니다.")
                break
            else:
                print("[!] 0, 1, 2, 3 중에서 선택하세요.")
    except KeyboardInterrupt:
        print("\n[중단]")
    finally:
        mc.close()
        print("[OK] 모터 비활성화, 포트 닫힘")


if __name__ == "__main__":
    main()
