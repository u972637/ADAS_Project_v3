#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Yaw(CAN 1)/Pitch(CAN 2) 모터 + 카메라 통합 테스트 메뉴.

motor/dm4310_tracker.py 와 동일한 연결 정보·게인을 그대로 가져와 두 축을
구동한다.

메뉴:
  1. 동물 추적 + 종별 음향 재생 (main.py 와 동일한 wildlife.py 파이프라인)
  2. YOLO 코 좌표 추적 (손들기 ARM_UP_THRESHOLD초로 추적 시작)
  3. 방향키로 위치 제어 (모터 각도를 직접 조그)
  4. 음향테스트용 위치 고정 (yaw=0°, pitch=0°로 이동 후 유지)
  0. 종료

1·2·3·4번 모두 **q 또는 ESC로 정상 종료하면** 공통으로 "숙인위치"
(BOW_YAW_DEG=0°, BOW_PITCH_DEG=-4°)로 BOW_DURATION(1초)에 걸쳐 복귀한 뒤
메뉴로 돌아온다 (Ctrl+C 로 강제중지한 경우는 복귀 없이 바로 모터를 끈다).

1번 진입 시 동작 — people/main_facetrack.py 가 아니라 main.py 가 쓰는
wildlife.py/camera.py/settings.py 조합을 그대로 재사용해 멧돼지/고라니를
탐지하고, 탐지된 bbox 중심으로 Tracker 를 조준한 뒤 종에 맞는 WAV를
무작위로 재생한다. main.py 자체를 그대로 실행하면 Tracker 를 내부에 숨겨
가지고 있어 종료 시 숙인위치로 보낼 수 없으므로, 이 함수가 Tracker 를
직접 들고 재구현한다.

2번에서 손들기로 추적이 확정되는 그 순간(락온)에 audio/track_start2.mp3 를
1회 재생해 추적 시작을 알린다. 화면에는 검출된 코 좌표를 항상 표시하지만, 실제로
motor/dm4310_tracker.py 의 Tracker.update(x, y) 를 호출해 모터를 움직이는
건 그 사람이 ARM_UP_THRESHOLD(2초) 이상 손을 들고 있어야 시작되고(=
"추적 확정"), ARM_DOWN_GRACE(프레임) 넘게 손을 내리면 멈춘다. "타겟
모드"(확정 상태)가 아니면 즉시 기본 자세(DEFAULT_YAW_DEG=0°,
DEFAULT_PITCH_DEG=30°)로 1회 복귀한다.

3번 진입 시 조작 — 모터 각도를 직접 명령:
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

4번 진입 시 동작 — 카메라/YOLO 없이 모터만 yaw=0°, pitch=0°(정면·수평)로
이동시켜 그 자리에 고정한다. audio_test/ 의 각도·거리별 dB 측정을 할 때,
스피커(짐벌) 조준은 항상 0°로 고정해두고 마이크를 각 각도·거리로 옮겨가며
재는 용도다. q 또는 ESC 로 메뉴 복귀.

실행:
  uv run python tools/motor_manual_control.py
"""

import math
import os
import select
import sys
import termios
import time
import tty

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(TEST_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)
if TEST_DIR not in sys.path:
    sys.path.insert(0, TEST_DIR)

import cv2
import serial
from ultralytics import YOLO

from motor.DM_CAN import Motor, MotorControl, Control_Type
from motor.dm4310_tracker import (
    DEVICENAME, BAUDRATE, MOTOR_TYPE,
    CAN_ID_YAW, CAN_ID_PITCH, MASTER_ID_YAW, MASTER_ID_PITCH,
    KP_YAW, KD_YAW, KP_PITCH, KD_PITCH, TAU_FF_YAW, TAU_FF_PITCH,
    Tracker, FRAME_W, FRAME_H,
)
# camera_test.py 는 같은 tools/ 폴더에 있는 가벼운 카메라 열기/프레임 변환
# 유틸리티 — people/main_facetrack.py 를 그대로 import하면 ultralytics/torch 까지
# 끌려와 무거워지므로, 카메라 관련 부분만 재사용한다.
from camera_test import DEFAULT_CAMERA_IDX, open_camera as cam_open_camera, read_frame as cam_read_frame
from detection.yolo_pose import YoloPoseDetector

# main.py(야생동물 음향 퇴치)가 쓰는 것과 동일한 모듈 — 1번 메뉴가 재사용한다.
from animal import settings
from animal.wildlife import (
    TrackerCameraController, WildlifeDeterrent, process_frame, validate_model_classes,
)
from animal.mp3_player import MP3LoopPlayer

AXES = ("yaw", "pitch")
CAN_ID = {"yaw": CAN_ID_YAW, "pitch": CAN_ID_PITCH}
MASTER_ID = {"yaw": MASTER_ID_YAW, "pitch": MASTER_ID_PITCH}
# motor/dm4310_tracker.py 와 동일한 축별 게인/피드포워드를 그대로 가져와 쓴다
# (Pitch 만 중력보상 TAU_FF_PITCH 를 가질 수 있고, Yaw 는 항상 0).
_KP     = {"yaw": KP_YAW,   "pitch": KP_PITCH}
_KD     = {"yaw": KD_YAW,   "pitch": KD_PITCH}
_TAU_FF = {"yaw": TAU_FF_YAW, "pitch": TAU_FF_PITCH}

# 방향키 한 번(자동반복 포함) 입력당 움직일 각도(도). 보수적으로 작게 잡음.
STEP_DEG = 1.0

# 테스트 시작 시 램프 이동할 기본 자세(도). 짐벌이 완전히 수평(0,0)이 아니라
# 보통 사람 얼굴 높이를 보도록 pitch 를 띄워둔 상태를 기본으로 쓴다.
# 나중에 바꾸고 싶으면 이 두 상수만 고치면 됨.
DEFAULT_YAW_DEG = 0.0
DEFAULT_PITCH_DEG = 30.0

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
# 공통: q 로 정상 종료할 때 돌아갈 "숙인위치"
# ──────────────────────────────────────────────
BOW_YAW_DEG   = 0.0
BOW_PITCH_DEG = -4.0
BOW_DURATION  = 1.0   # 초

# ──────────────────────────────────────────────
# 2번(YOLO 코 좌표 추적 테스트) 전용 설정
# ──────────────────────────────────────────────

# people/main_facetrack.py 와 동일한 모델/기준값 — 파인튜닝 모델로 바꿀 땐
# 이 경로만 바꾸면 됨.
MODEL_PATH = os.path.join(PROJECT_ROOT, "assets", "models", "yolov8n-pose.pt")
ARM_UP_THRESHOLD = 2.0    # 초 — 이만큼 손을 들고 있어야 추적 확정
ARM_DOWN_GRACE = 10       # 프레임 — 이만큼 넘게 손을 내리면 추적 해제

# 2번("인식모드") 진입 시 1회 재생할 안내음
TRACK_START2_PATH = os.path.join(PROJECT_ROOT, "audio", "track_start2.mp3")

# 2592x1944 그대로 창을 띄우면 화면보다 커서 축소 표시만 한다
# (Tracker.update() 에는 항상 원본 해상도 좌표를 그대로 넘김).
DISPLAY_SCALE = 0.30


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _bow_tracker(tracker: Tracker, duration: float = BOW_DURATION):
    """q 로 정상 종료할 때 공통으로 호출 — '숙인위치'로 duration초에 걸쳐
    램프 이동시키고, 그 이동이 끝날 때까지 기다린 뒤 돌아온다."""
    print(f"[숙인위치 복귀] {duration:.1f}초 동안 yaw={BOW_YAW_DEG:.0f}°, "
          f"pitch={BOW_PITCH_DEG:.0f}° 로 이동합니다...")
    tracker.go_to(yaw_rad=math.radians(BOW_YAW_DEG),
                  pitch_rad=math.radians(BOW_PITCH_DEG),
                  duration=duration)
    time.sleep(duration)


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
            self.ctrl.controlMIT(self.motor[axis], _KP[axis], _KD[axis], q_des, 0.0, _TAU_FF[axis])

    def nudge(self, axis: str, delta_deg: float):
        lo, hi = _RANGE_DEG[axis]
        self.target_deg[axis] = clamp(self.target_deg[axis] + delta_deg, lo, hi)

    def ramp_to(self, goal_deg: dict, duration: float):
        """임의의 목표 각도(도)로 duration초에 걸쳐 선형 램프 이동."""
        start = dict(self.target_deg)
        steps = max(int(duration * CONTROL_FREQ), 1)
        dt = duration / steps
        for i in range(1, steps + 1):
            frac = i / steps
            for axis in AXES:
                self.target_deg[axis] = start[axis] + (goal_deg[axis] - start[axis]) * frac
            self._send()
            time.sleep(dt)

    def close(self):
        # Ctrl+C 등으로 이미 한 번 닫힌 뒤 바깥 try/finally 가 다시 close() 를
        # 호출하는 경로가 있어, 이미 닫힌 포트에 재차 disable() 을 보내 예외가
        # 나는 걸 막기 위해 멱등하게 만든다.
        if getattr(self, "_closed", False):
            return
        self._closed = True
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
        # while 을 'q'(정상 종료)로 빠져나온 경우에만 실행된다 — 루프 중
        # 예외(Ctrl+C 등)가 나면 아래로 내려오지 않고 바로 finally 로 간다.
        print(f"[숙인위치 복귀] {BOW_DURATION:.1f}초 동안 yaw={BOW_YAW_DEG:.0f}°, "
              f"pitch={BOW_PITCH_DEG:.0f}° 로 이동합니다...")
        mc.ramp_to({"yaw": BOW_YAW_DEG, "pitch": BOW_PITCH_DEG}, BOW_DURATION)
    finally:
        restore_terminal_keyboard(old_settings)
        print()


# ──────────────────────────────────────────────
# 메뉴 4: 음향테스트용 위치 고정 (yaw=0°, pitch=0°)
# ──────────────────────────────────────────────

def run_audio_test_pose(mc: ManualController):
    """
    카메라/YOLO 없이 순수하게 모터만 yaw=0°, pitch=0°(정면·수평)로 이동시켜
    그 자리에 고정해둔다. audio_test/ 의 각도·거리별 dB 측정(스피커 조준은
    항상 0°로 고정해두고, 마이크를 각 각도·거리로 옮겨가며 측정)을 할 때
    쓴다. q 또는 ESC 로 메뉴 복귀 — 복귀 시 다른 메뉴와 동일하게 숙인위치로
    복귀한다.
    """
    print("\n[음향테스트 위치 고정] yaw=0°, pitch=0°로 이동 후 유지합니다 — q 로 메뉴 복귀")
    mc.ramp_to({"yaw": 0.0, "pitch": 0.0}, BOW_DURATION)
    print(f"  고정 완료: yaw={mc.current_deg('yaw'):+.1f}°  pitch={mc.current_deg('pitch'):+.1f}°")

    old_settings = setup_terminal_keyboard()
    next_t = time.time()
    try:
        while True:
            key = read_key_nonblocking()
            if key == 'q':
                print("\n[메뉴로 복귀]")
                break
            mc._send()   # 같은 목표(0,0)를 계속 스트리밍해 토크를 유지

            next_t += DT
            sleep_time = next_t - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_t = time.time()
        # while 을 'q'(정상 종료)로 빠져나온 경우에만 실행된다 — 루프 중
        # 예외(Ctrl+C 등)가 나면 아래로 내려오지 않고 바로 finally 로 간다.
        print(f"[숙인위치 복귀] {BOW_DURATION:.1f}초 동안 yaw={BOW_YAW_DEG:.0f}°, "
              f"pitch={BOW_PITCH_DEG:.0f}° 로 이동합니다...")
        mc.ramp_to({"yaw": BOW_YAW_DEG, "pitch": BOW_PITCH_DEG}, BOW_DURATION)
    finally:
        restore_terminal_keyboard(old_settings)
        print()


# ──────────────────────────────────────────────
# 메뉴 1: 동물 추적 + 종별 음향 재생
# ──────────────────────────────────────────────

def run_wildlife_track():
    """
    main.py(야생동물 음향 퇴치)와 동일한 wildlife.py/camera.py/settings.py
    조합을 그대로 재사용해 멧돼지/고라니를 탐지한다. 탐지 자체(화면에 bbox
    표시, 확정 전엔 노란 박스)는 시작하자마자 바로 되지만, 실제로
    motor/dm4310_tracker.py 의 Tracker 로 조준(= "타겟 모드", 박스가 초록
    으로 바뀜)하는 건 **w 키를 눌러야** 시작된다 — w 를 누르기 전까지는
    종에 맞는 WAV 재생/조준 없이 탐지 결과만 보여준다. **r 키**를 누르면
    타겟 모드를 해제하고(재추적) 다시 w 로 시작할 수 있는 대기 상태로
    돌아간다. (main.py 자체는 이 게이팅 없이 탐지되는 즉시 조준+재생한다 —
    이 테스트 메뉴에서만 수동으로 시작/재시작 시점을 통제할 수 있게 w/r 를
    추가함.)

    main.py 를 그대로 실행하면 Tracker 를 자기 안에 숨겨 갖고 있어 q 종료
    시 숙인위치로 보낼 수 없으므로, 이 함수가 Tracker 를 직접 들고 있는
    채로 main.py 의 run() 과 비슷한 루프를 재구현한다. q/ESC 로 정상
    종료하면 _bow_tracker() 로 숙인위치에 복귀한다.

    2번(YOLO 코 추적)과 포트를 동시에 쓸 수 없으므로(같은 시리얼 장치),
    이 함수 안에서 자체적으로 Tracker 를 새로 열고 끝나면 닫는다 — 호출하는
    쪽(main())이 먼저 ManualController 를 닫아둔 상태여야 한다.
    """
    print("\n[동물 추적 테스트] 멧돼지/고라니 탐지는 바로 시작하지만,")
    print("  w 를 눌러야 실제 조준(타겟 모드)이 시작됩니다 — q 또는 ESC 로 메뉴 복귀\n")

    model_path = settings.MODEL_PATH
    if not model_path.is_file():
        print(f"[ERROR] 동물 탐지 모델이 없습니다: {model_path} — 메뉴로 복귀합니다.")
        return

    print(f"[..] 동물 탐지 모델 로드 중... ({model_path})")
    model = YOLO(str(model_path))
    print("[OK] 모델 로드 완료")

    service = WildlifeDeterrent.from_settings()
    try:
        validate_model_classes(model.names, service.class_map, service.expected_names)
    except ValueError as e:
        print(f"[ERROR] 모델 클래스 불일치 — {e} — 메뉴로 복귀합니다.")
        return
    for class_id in sorted(service.class_map):
        print(f"[CLASS] model {class_id}={model.names[class_id]} "
              f"-> audio {service.class_map[class_id]}")

    print(f"[..] 카메라 index={settings.CAMERA_INDEX} 여는 중...")
    cap = cam_open_camera(settings.CAMERA_INDEX)
    if cap is None:
        print(f"[ERROR] 카메라(index={settings.CAMERA_INDEX})를 열 수 없습니다 — 메뉴로 복귀합니다.")
        return

    print("[..] Tracker 연결 중...")
    tracker = Tracker()
    camera_controller = TrackerCameraController(tracker, (FRAME_W, FRAME_H))
    service.camera = None   # "타겟 모드"가 아니면 탐지는 하되 조준은 안 함 — w 로 켬
    print("[OK] 연결 완료 — w 를 눌러야 실제로 조준을 시작합니다.")

    disp_w, disp_h = int(FRAME_W * DISPLAY_SCALE), int(FRAME_H * DISPLAY_SCALE)
    win_name = "Wildlife Tracking (w: start tracking, q/ESC: back)"
    cv2.namedWindow(win_name)
    next_hz_print = time.time()   # 1초마다 파이프라인 전체 실측 Hz 콘솔 출력
    target_mode = False   # w 를 누르면 True 로 전환되고 이후 계속 유지됨

    try:
        while True:
            ok, frame = cam_read_frame(cap)
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            event = process_frame(model, service, frame, time.monotonic())
            if event.audio_file is not None:
                print(f"[AUDIO] {event.reason}: {event.audio_file}")

            disp = cv2.resize(frame, (disp_w, disp_h))
            label = "No target"
            if event.target is not None:
                x1, y1, x2, y2 = (int(v * DISPLAY_SCALE) for v in event.target.xyxy)
                box_color = (0, 255, 0) if target_mode else (0, 255, 255)   # 확정전=노랑, 확정(타겟모드)=초록
                cv2.rectangle(disp, (x1, y1), (x2, y2), box_color, 2)
                label = f"{event.label or 'Unknown'} {event.target.confidence:.2f}"
            cv2.putText(disp, f"{label} | {event.reason}", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            mode_text = "TARGET MODE (tracking)" if target_mode else "WAITING (press w to start tracking)"
            mode_color = (0, 255, 0) if target_mode else (0, 255, 255)
            cv2.putText(disp, mode_text, (10, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2)

            cv2.putText(disp, "[W]Start [R]Reset [Q]Quit", (10, disp_h - 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            hz_text = (f"cam->target {tracker.get_update_freq():.1f}Hz | "
                       f"ctrl thread {tracker.get_actual_freq():.0f}Hz | "
                       f"CAN tx {tracker.get_can_tx_freq():.0f}Hz | "
                       f"rx yaw {tracker.get_rx_freq('yaw'):.0f}/"
                       f"pitch {tracker.get_rx_freq('pitch'):.0f}Hz")
            cv2.putText(disp, hz_text, (10, disp_h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 0), 1)

            now = time.time()
            if now >= next_hz_print:
                print(f"\r  [Hz] 카메라->목표좌표={tracker.get_update_freq():6.1f}  "
                      f"제어스레드(틱)={tracker.get_actual_freq():6.1f}  "
                      f"CAN-USB tx={tracker.get_can_tx_freq():6.1f}  "
                      f"액추에이터 rx yaw={tracker.get_rx_freq('yaw'):6.1f} "
                      f"pitch={tracker.get_rx_freq('pitch'):6.1f}   ",
                      end="", flush=True)
                next_hz_print = now + 1.0

            cv2.imshow(win_name, disp)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('w') and not target_mode:
                target_mode = True
                service.camera = camera_controller
                print("\n[타겟 모드 시작] w 입력 — 이제부터 탐지되면 실제로 조준합니다.")
            elif key == ord('r') and target_mode:
                target_mode = False
                service.camera = None
                print("\n[재추적] r 입력 — 타겟 모드 해제, w 로 다시 시작할 수 있습니다.")
        # while 을 'q'/ESC(정상 종료)로 빠져나온 경우에만 실행된다 — 루프 중
        # 예외(Ctrl+C 등)가 나면 아래로 내려오지 않고 바로 finally 로 간다.
        _bow_tracker(tracker)
    finally:
        cv2.destroyWindow(win_name)
        cap.release()
        service.player.stop()
        tracker.close()
        print("\n[메뉴로 복귀] 카메라/Tracker 연결 종료")


# ──────────────────────────────────────────────
# 메뉴 2: YOLO 코 좌표 추적 ("인식모드")
# ──────────────────────────────────────────────

def run_auto_track():
    """
    실제 카메라 화면 위에서 YoloPoseDetector 로 매 프레임 코 좌표·팔든상태를
    인식한다. people/main_facetrack.py 와 같은 규칙으로, 화면 중 몸 박스가 가장 큰
    사람을 대상으로 보고 코 좌표는 검출될 때마다 항상 화면에 표시하지만,
    실제 motor/dm4310_tracker.py 의 Tracker.update(x, y) 호출(= 모터 추적)은
    그 사람이 ARM_UP_THRESHOLD(초) 이상 손을 들고 있어야 시작되고,
    ARM_DOWN_GRACE(프레임) 넘게 손을 내리면 멈춘다.

    "타겟 모드"(= confirmed, 손들기로 추적이 확정된 상태)가 아니면 — 코가
    보이든 안 보이든, 손을 드는 중(ARM_UP_THRESHOLD 못 채움)이든 — 그 즉시
    기본 자세(DEFAULT_YAW_DEG, DEFAULT_PITCH_DEG = 0°, 30°)로 Tracker.go_to()
    를 1회 호출해 복귀시킨다 — 매 프레임 다시 호출하지 않도록(램프 시작점이
    계속 "지금 위치"로 리셋돼 버려 실제로는 못 움직이게 되므로) 상태가
    "타겟 모드 → 아님"으로 바뀌는 순간에만 트리거한다.

    손을 들어 ARM_UP_THRESHOLD초가 "막" 지나서 추적이 확정되는 그 순간에만
    audio/track_start2.mp3 를 1회 재생한다(추적 중엔 다시 재생하지 않음).
    화면에는 대상 사람의 몸 박스를 항상 표시하며, 확정 전엔 노란색, 확정
    (타겟 모드)되면 초록색으로 바뀐다. **r 키**를 누르면 확정 상태를 즉시
    해제하고(재추적) 다시 손들기 대기 상태로 돌아간다. q/ESC 로 정상
    종료하면 _bow_tracker() 로 숙인위치에 복귀한다.

    카메라는 people/main_facetrack.py 와 같은 설정(DEFAULT_CAMERA_IDX=4, 실측
    확인됨)으로 camera_test.py 의 open_camera()/read_frame() 을 재사용한다.

    3번(수동 조그)과 포트를 동시에 쓸 수 없으므로(같은 시리얼 장치), 이
    함수 안에서 자체적으로 Tracker 를 새로 열고 끝나면 닫는다 — 호출하는
    쪽(main())이 먼저 ManualController 를 닫아둔 상태여야 한다.
    """
    print("\n[인식모드] 손을 들어 ARM_UP_THRESHOLD초 이상 유지하면 추적 시작")
    print(f"  ARM_UP_THRESHOLD={ARM_UP_THRESHOLD:.0f}s  ARM_DOWN_GRACE={ARM_DOWN_GRACE}frame"
          f"  — q 또는 ESC 로 메뉴 복귀\n")

    print(f"[..] YOLO pose 모델 로드 중... ({MODEL_PATH})")
    detector = YoloPoseDetector(MODEL_PATH)
    print(f"[OK] 모델 로드 완료 (device={detector.device})")

    print(f"[..] 카메라 index={DEFAULT_CAMERA_IDX} 여는 중...")
    cap = cam_open_camera(DEFAULT_CAMERA_IDX)
    if cap is None:
        print(f"[ERROR] 카메라(index={DEFAULT_CAMERA_IDX})를 열 수 없습니다 — 메뉴로 복귀합니다.")
        return

    print("[..] Tracker 연결 중...")
    tracker = Tracker()
    print("[OK] 연결 완료 — 창이 뜨면 손을 들어보세요.")

    disp_w, disp_h = int(FRAME_W * DISPLAY_SCALE), int(FRAME_H * DISPLAY_SCALE)
    win_name = "YOLO Nose Tracking (arm-up gated, q/ESC: back)"
    cv2.namedWindow(win_name)

    arm_up_start = None    # 현재 대상이 손을 들기 시작한 시각 (None=안 듦)
    confirmed = False      # ARM_UP_THRESHOLD 넘겨서 "추적 확정" 상태인지
    arm_down_count = 0
    went_default = False   # 코 좌표 소실로 기본 자세 복귀를 이미 보냈는지
    next_hz_print = time.time()   # 1초마다 파이프라인 전체 실측 Hz 콘솔 출력

    try:
        while True:
            ok, frame = cam_read_frame(cap)
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            people = detector.detect(frame)

            # 몸 박스가 가장 큰 사람을 이번 프레임의 대상으로 본다
            primary = None
            best_area = 0.0
            for person in people:
                x1, y1, x2, y2 = person.box
                area = (x2 - x1) * (y2 - y1)
                if area > best_area:
                    best_area, primary = area, person

            nose_xy = primary.nose if primary is not None else None
            arm_now = bool(primary is not None and primary.arm_up)
            elapsed = 0.0

            if arm_now:
                arm_down_count = 0
                if arm_up_start is None:
                    arm_up_start = time.time()
                elapsed = time.time() - arm_up_start
                if elapsed >= ARM_UP_THRESHOLD and not confirmed:
                    confirmed = True
                    # 손들기로 추적이 "막" 확정되는 순간에만 1회 재생
                    MP3LoopPlayer(TRACK_START2_PATH, loop=False).play_once()
            else:
                arm_down_count += 1
                if confirmed and arm_down_count > ARM_DOWN_GRACE:
                    confirmed = False
                if arm_up_start is not None and not confirmed:
                    arm_up_start = None

            sent = False
            if confirmed and nose_xy is not None:
                went_default = False   # 추적 중엔 복귀 트리거 재무장
                tracker.update(nose_xy[0], nose_xy[1])
                sent = True
            elif not went_default:
                # confirmed(추적 확정) 가 아니면 — 코가 보이든 안 보이든 —
                # 기본 자세로 1회 복귀. 다시 추적이 확정되면 재무장된다.
                tracker.go_to(yaw_rad=math.radians(DEFAULT_YAW_DEG),
                              pitch_rad=math.radians(DEFAULT_PITCH_DEG))
                went_default = True

            disp = cv2.resize(frame, (disp_w, disp_h))
            cx, cy = disp_w // 2, disp_h // 2
            cv2.line(disp, (cx - 15, cy), (cx + 15, cy), (0, 160, 0), 1)
            cv2.line(disp, (cx, cy - 15), (cx, cy + 15), (0, 160, 0), 1)

            if primary is not None:
                bx1, by1, bx2, by2 = (int(v * DISPLAY_SCALE) for v in primary.box)
                box_color = (0, 255, 0) if confirmed else (0, 255, 255)   # 확정전=노랑, 확정=초록
                cv2.rectangle(disp, (bx1, by1), (bx2, by2), box_color, 2)

            if nose_xy is not None:
                nx, ny = int(nose_xy[0] * DISPLAY_SCALE), int(nose_xy[1] * DISPLAY_SCALE)
                color = (0, 0, 255) if sent else (0, 255, 255)   # 전송중=빨강, 대기=노랑
                cv2.circle(disp, (nx, ny), 7, color, -1)

            if sent:
                status = f"TRACKING  nose={nose_xy}"
                color = (0, 0, 255)
            elif arm_now and not confirmed:
                status = f"ARM UP {elapsed:.1f}/{ARM_UP_THRESHOLD:.0f}s"
                color = (0, 255, 255)
            else:
                status = f"NOT TRACKING -> default pose ({DEFAULT_YAW_DEG:.0f}, {DEFAULT_PITCH_DEG:.0f})"
                color = (255, 150, 0)
            cv2.putText(disp, status, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            cv2.putText(disp, "[R]Reset [Q]Quit", (10, disp_h - 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            hz_text = (f"cam->target {tracker.get_update_freq():.1f}Hz | "
                       f"ctrl thread {tracker.get_actual_freq():.0f}Hz | "
                       f"CAN tx {tracker.get_can_tx_freq():.0f}Hz | "
                       f"rx yaw {tracker.get_rx_freq('yaw'):.0f}/"
                       f"pitch {tracker.get_rx_freq('pitch'):.0f}Hz")
            cv2.putText(disp, hz_text, (10, disp_h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 0), 1)

            now = time.time()
            if now >= next_hz_print:
                print(f"\r  [Hz] 카메라->목표좌표={tracker.get_update_freq():6.1f}  "
                      f"제어스레드(틱)={tracker.get_actual_freq():6.1f}  "
                      f"CAN-USB tx={tracker.get_can_tx_freq():6.1f}  "
                      f"액추에이터 rx yaw={tracker.get_rx_freq('yaw'):6.1f} "
                      f"pitch={tracker.get_rx_freq('pitch'):6.1f}   ",
                      end="", flush=True)
                next_hz_print = now + 1.0

            cv2.imshow(win_name, disp)
            key = cv2.waitKey(1) & 0xFF   # 카메라 자체 fps(~25)로 자연스럽게 제한됨
            if key in (ord('q'), 27):
                break
            elif key == ord('r'):
                # 재추적 — 확정 상태를 전부 지우고 손들기 대기로 복귀.
                # went_default 도 같이 지워서 다음 프레임에 기본 자세 복귀가
                # 한 번 더 트리거되게 한다(이미 확정 해제 상태라도 재확인용).
                arm_up_start = None
                confirmed = False
                arm_down_count = 0
                went_default = False
                print("\n[재추적] r 입력 — 추적 확정 해제, 기본 자세로 복귀합니다.")
        # while 을 'q'/ESC(정상 종료)로 빠져나온 경우에만 실행된다 — 루프 중
        # 예외(Ctrl+C 등)가 나면 아래로 내려오지 않고 바로 finally 로 간다.
        _bow_tracker(tracker)
    finally:
        cv2.destroyWindow(win_name)
        cap.release()
        tracker.close()
        print("\n[메뉴로 복귀] 카메라/Tracker 연결 종료")


# ──────────────────────────────────────────────
# 메뉴
# ──────────────────────────────────────────────

def print_menu():
    print("\n======================================")
    print(" Yaw/Pitch 수동 제어")
    print("======================================")
    print(" 1. 동물 추적 + 종별 음향 재생 (멧돼지/고라니 탐지)")
    print(" 2. YOLO 코 좌표 추적 (손들기로 시작, 인식모드 진입 시 안내음)")
    print(" 3. 방향키로 위치 제어 (모터 각도 직접 조그)")
    print(" 4. 음향테스트용 위치 고정 (yaw=0°, pitch=0°)")
    print(" 0. 종료")
    print("======================================")


def main():
    print("[..] 모터 연결 중...")
    mc = ManualController()
    print("[OK] 연결 완료")

    try:
        while True:
            print_menu()
            try:
                choice = input("선택 > ").strip()
            except EOFError:
                break

            if choice == "1":
                # 1·2번은 같은 시리얼 포트를 Tracker 로 새로 열어야 해서,
                # ManualController 가 들고 있던 연결을 잠깐 반납했다가
                # 돌아오면 다시 연다.
                mc.close()
                run_wildlife_track()
                print("[..] 모터 재연결 중...")
                mc = ManualController()
                print("[OK] 재연결 완료")
            elif choice == "2":
                # 기본 자세로의 복귀는 따로 먼저 하지 않는다 — YOLO 모델
                # 로드가 끝나고 추적 루프가 돌기 시작하면 (아직 타겟 모드가
                # 아닌) 첫 프레임부터 run_auto_track() 내부의 "타겟 모드
                # 아님 → 기본 자세" 로직이 자연스럽게 같은 동작을 한다.
                mc.close()
                run_auto_track()
                print("[..] 모터 재연결 중...")
                mc = ManualController()
                print("[OK] 재연결 완료")
            elif choice == "3":
                run_manual_jog(mc)
            elif choice == "4":
                run_audio_test_pose(mc)
            elif choice == "0":
                print("종료합니다.")
                break
            else:
                print("[!] 0, 1, 2, 3, 4 중에서 선택하세요.")
    except KeyboardInterrupt:
        print("\n[중단]")
    finally:
        mc.close()
        print("[OK] 모터 비활성화, 포트 닫힘")


if __name__ == "__main__":
    main()
