#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main.py — Arducam B0538C + YOLOv8 pose + Damiao DM-J4310-2EC(CAN, 1000Hz) 통합

동작 흐름:
  1. Arducam B0538C(video4, 2592x1944 @ 25fps, UVC YUY2)로 영상 수신
  2. PoseDetector(기본: YOLOv8 pose)로 사람 감지 + 관절 추출 (~25fps, 카메라 fps 한도)
  3. 손을 3초 이상 든 사람을 트래킹 대상으로 확정
  4. 확정된 사람의 코 좌표 → Tracker.update() (검출 fps 만큼 목표 좌표 입력)
     → 1000Hz CAN MIT 스트리밍 스레드가 모터 실측 위치를 읽어 목표를 갱신하고
       실제 모터 제어 (motor/dm4310_tracker.py 참고)

검출기는 detection.base.PoseDetector 인터페이스만 지키면 어떤 모델로도
교체 가능하도록 분리돼 있다 (detection/yolo_pose.py 참고, 파인튜닝 모델은
MODEL_PATH 만 바꾸면 됨).

실행:
  python3 main.py           # 모터 없이 화면만
  python3 main.py --motor   # 모터 제어 포함
"""

import argparse
import math
import os
import select
import shutil
import sys
import termios
import threading
import time
import tty
import warnings
from collections import defaultdict

# Arducam B0538C(OG05B1B)는 UVC 표준 YUY2로 출력하는 카메라라 v4l2convert.so
# 소프트웨어 디모자이킹이 필요 없음 (이전 oCam의 raw Bayer(GRBG) 대응용이었음).
USE_V4L2_CONVERT_SO = False
V4L2_CONVERT_SO = "/usr/lib/x86_64-linux-gnu/libv4l/v4l2convert.so"


def enable_v4l2_convert_so_if_needed():
    """v4l2convert.so 방식 사용 시 LD_PRELOAD 적용 후 프로세스를 재시작."""
    if not USE_V4L2_CONVERT_SO:
        return
    if not os.path.exists(V4L2_CONVERT_SO):
        print(f"[WARN] v4l2convert.so 파일이 없습니다: {V4L2_CONVERT_SO}")
        return
    preload = os.environ.get("LD_PRELOAD", "")
    if V4L2_CONVERT_SO in preload.split(":"):
        return
    os.environ["LD_PRELOAD"] = f"{V4L2_CONVERT_SO}:{preload}" if preload else V4L2_CONVERT_SO
    os.execv(sys.executable, [sys.executable] + sys.argv)


enable_v4l2_convert_so_if_needed()
os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")
warnings.filterwarnings("ignore", message="CUDA initialization:.*")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

import cv2
import numpy as np
from mp3_player import MP3LoopPlayer
from detection.yolo_pose import YoloPoseDetector

# ──────────────────────────────────────────────
# 설정
# ──────────────────────────────────────────────
CAMERA_IDX   = 4                     # Arducam B0538C 카메라 인덱스 — 임시값, 미검증.
                                      # 아직 카메라를 연결 전이라 실제 몇 번 /dev/video*
                                      # 로 잡힐지 모른다(USB-CAN 동글도 예상과 달리
                                      # /dev/ttyUSB0 아닌 /dev/ttyACM0 로 잡혔던 전례
                                      # 있음). 연결 후 `ls /dev/video*`/`v4l2-ctl --list-devices`
                                      # 로 반드시 재확인할 것.
MODEL_PATH   = "yolov8n-pose.pt"     # YOLO pose 모델
CAMERA_FOURCC = "YUY2"               # B0538C(OG05B1B) 네이티브 출력 포맷 (UVC 표준, Bayer 아님)
FRAME_W, FRAME_H = 2592, 1944        # B0538C 최대 해상도 @ 25fps (USB 3.0 기준)
BAYER_TO_BGR = cv2.COLOR_BayerGB2BGR # (구) oCam GRBG raw → OpenCV BGR, B0538C는 사용 안 함
YUYV_TO_BGR = cv2.COLOR_YUV2BGR_YUY2 # B0538C YUY2 raw → OpenCV BGR

# raw Bayer 변환 후 guvcview 기준 색감에 가깝게 맞추던 후처리값 (구 oCam 컬러 센서용).
# B0538C는 흑백(Mono) 글로벌 셔터 센서라 보정할 색 정보 자체가 없으므로 중립값으로 둔다.
# 필요 시 밝기(brightness_gain)만 조정해서 쓸 것.
COLOR_CORRECTION = {
    "blue_gain": 1.0,
    "green_gain": 1.0,
    "red_gain": 1.0,
    "brightness_gain": 1.0,
}

AUDIO_DIR = os.path.join(BASE_DIR, "audio")
TRACK_START_MP3 = os.path.join(AUDIO_DIR, "track_start.mp3")
SAMPLE_MP3      = os.path.join(AUDIO_DIR, "sample.mp3")
ROUTE_EXIT_MP3  = os.path.join(AUDIO_DIR, "route_exit.mp3")
RIGHT_EXIT_MP3  = os.path.join(AUDIO_DIR, "right_exit.mp3")
LEFT_EXIT_MP3   = os.path.join(AUDIO_DIR, "left_exit.mp3")
GREENLIGHT_MP3  = os.path.join(AUDIO_DIR, "greenlight.mp3")
FINISH_MP3      = os.path.join(AUDIO_DIR, "finish.mp3")

# 손 든 상태를 몇 초 유지해야 트래킹 확정할지
ARM_UP_THRESHOLD = 3.0               # 초

# 손이 내려간 것으로 판정하기까지 허용할 연속 프레임 수
# (YOLO 키포인트 노이즈로 인한 순간 끊김 방지)
ARM_DOWN_GRACE = 10                  # 프레임

ROUTE_EXIT_COOLDOWN = 3.0            # 초

# 볼륨 제어 (+/-/0 키)
VOLUME_DEFAULT = 0.7   # 0 키로 복귀할 기준 볼륨 (0.0 = 묵음 / 1.0 = 최대)
VOLUME_STEP    = 0.1   # +/- 한 번당 변화량 (0.0 ~ 1.0 범위 내에서 클램프)

# 구간 이탈 판단: 모터 각도(yaw, rad 기준 중앙=0) × YOLO 박스 크기
ROUTE_EXIT_SCORE_THRESHOLD = 0.85
ROUTE_EXIT_LOG_INTERVAL = 1.0
# 모터 미사용 시 카메라 화각으로 폴백.
# Arducam B0538C 렌즈 스펙: DFOV 95° / HFOV 82° / VFOV 66°, EFL 3.6mm, F2.8.
# 화면 중앙 기준 각도 ≈ (화면 중심 대비 픽셀 오차 비율) × (HFOV/2) 로 선형 근사
# (motor/dm4310_tracker.py 의 RAD_PER_PIXEL_X/Y 유도와 동일한 이론적 근거).
CAMERA_HORIZONTAL_FOV_DEG = 82.0


# ──────────────────────────────────────────────
# OpenCV Qt 폰트 설정
# ──────────────────────────────────────────────

def ensure_opencv_qt_fonts():
    """OpenCV Qt backend가 찾는 cv2/qt/fonts 디렉터리에 시스템 폰트를 연결."""
    cv2_dir = os.path.dirname(cv2.__file__)
    font_dir = os.path.join(cv2_dir, "qt", "fonts")
    source_dir = "/usr/share/fonts/truetype/dejavu"
    font_names = ["DejaVuSans.ttf", "DejaVuSans-Bold.ttf", "DejaVuSansMono.ttf"]
    if all(os.path.exists(os.path.join(font_dir, n)) for n in font_names):
        return
    try:
        os.makedirs(font_dir, exist_ok=True)
        for name in font_names:
            src = os.path.join(source_dir, name)
            dst = os.path.join(font_dir, name)
            if not os.path.exists(src) or os.path.exists(dst):
                continue
            try:
                os.symlink(src, dst)
            except OSError:
                shutil.copy2(src, dst)
    except OSError as e:
        print(f"[WARN] OpenCV Qt 폰트 설정 실패: {e}")


# ──────────────────────────────────────────────
# 얼굴 박스 유틸
# (얼굴 박스 추정 자체는 검출기가 PersonPose.face_box 로 제공)
# ──────────────────────────────────────────────

def in_face_box(x, y, box):
    """좌표가 얼굴 박스 안에 있으면 True."""
    if box is None:
        return True  # 박스 없으면 통과
    x1, y1, x2, y2 = box
    return x1 <= x <= x2 and y1 <= y <= y2


# ──────────────────────────────────────────────
# 화면 오버레이
# ──────────────────────────────────────────────

def draw_info(frame, nose_x, nose_y, arm_up, confirmed, elapsed, box, fw, fh):
    cx, cy = fw // 2, fh // 2

    # 중앙 십자선
    cv2.line(frame, (cx - 30, cy), (cx + 30, cy), (0, 255, 0), 1)
    cv2.line(frame, (cx, cy - 30), (cx, cy + 30), (0, 255, 0), 1)

    # ── 상태 1: 아무도 없음 ──
    if not arm_up and not confirmed:
        cv2.putText(frame, "No target", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        return

    # ── 상태 2: 손 올림 대기 중 ──
    if arm_up and not confirmed:
        if box is not None:
            x1, y1, x2, y2 = box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(frame, "ARM UP!", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.putText(frame, "ARM UP! Holding...", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        bar_x, bar_y, bar_w, bar_h = 20, 75, 300, 20
        ratio = min(elapsed / ARM_UP_THRESHOLD, 1.0)
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (80, 80, 80), -1)
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + int(bar_w * ratio), bar_y + bar_h), (0, 255, 255), -1)
        cv2.putText(frame, f"{elapsed:.1f}s / {ARM_UP_THRESHOLD:.0f}s", (bar_x + bar_w + 10, bar_y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        return

    # ── 상태 3: 트래킹 확정 ──
    if box is not None:
        x1, y1, x2, y2 = box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, "TARGET", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    if nose_x is not None:
        cv2.circle(frame, (nose_x, nose_y), 7, (0, 0, 255), -1)
        cv2.line(frame, (nose_x, nose_y), (cx, cy), (255, 100, 0), 1)
        cv2.putText(frame, f"TRACKING  Nose:({nose_x},{nose_y})", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)


# ──────────────────────────────────────────────
# 구간 이탈 판단 (사용자 각도 × YOLO 박스 크기)
# ──────────────────────────────────────────────

def calculate_route_exit_score(nose_x, box, yaw_rad=None):
    """이탈 각도(모터 또는 카메라) × YOLO 박스 크기로 이탈 점수 계산.

    yaw_rad 제공 시: Yaw 모터의 현재 목표각(rad, 중앙=0) 사용 (더 정확)
    yaw_rad=None 시: 카메라 화면 기준 각도로 폴백
    """
    if box is None:
        return 0.0, 0.0, 0.0

    if yaw_rad is not None:
        angle_deg = math.degrees(abs(float(yaw_rad)))
    elif nose_x is not None:
        # 카메라 기준 폴백
        center_offset = abs(float(nose_x) - (FRAME_W / 2.0)) / (FRAME_W / 2.0)
        angle_deg = center_offset * (CAMERA_HORIZONTAL_FOV_DEG / 2.0)
    else:
        return 0.0, 0.0, 0.0

    x1, y1, x2, y2 = box
    box_area = max(float(x2 - x1), 0.0) * max(float(y2 - y1), 0.0)
    size_percent = (box_area / float(FRAME_W * FRAME_H)) * 100.0
    score = angle_deg / size_percent if size_percent > 0.0 else 0.0
    return angle_deg, size_percent, score


def is_route_exit_by_angle_size(nose_x, box, yaw_rad=None):
    """각도 × 사용자 크기 점수가 기준 이상이면 구간 이탈로 판단."""
    _, _, score = calculate_route_exit_score(nose_x, box, yaw_rad=yaw_rad)
    return score >= ROUTE_EXIT_SCORE_THRESHOLD


def get_exit_direction(yaw_rad=None, nose_x=None):
    """이탈 방향 반환: 'right' | 'left' | None.

    yaw_rad 우선 사용, 없으면 nose_x(카메라)로 폴백.
    yaw_rad > 0 → right, yaw_rad < 0 → left (motor/dm4310_tracker.py SIGN_YAW 기준).
    """
    if yaw_rad is not None:
        if yaw_rad > 0:
            return 'right'
        if yaw_rad < 0:
            return 'left'
    elif nose_x is not None:
        if nose_x > FRAME_W / 2:
            return 'right'
        if nose_x < FRAME_W / 2:
            return 'left'
    return None


def draw_route_exit_status(frame, angle_deg, size_percent, score, is_route_exit, direction=None):
    """각도/크기 기반 이탈 상태를 화면에 표시."""
    color = (0, 0, 255) if is_route_exit else (0, 255, 0)
    if is_route_exit and direction:
        label = f"Route: OUT ({direction.upper()})"
    elif is_route_exit:
        label = "Route: OUT"
    else:
        label = "Route: inside"
    cv2.putText(frame, label, (20, FRAME_H - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    cv2.putText(frame, f"angle={angle_deg:.1f} size={size_percent:.1f}% score={score:.1f}/{ROUTE_EXIT_SCORE_THRESHOLD:.0f}",
                (20, FRAME_H - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


# ──────────────────────────────────────────────
# 카메라
# ──────────────────────────────────────────────

def open_camera(frame_w, frame_h):
    """카메라를 열고 기본 해상도를 적용."""
    cap = cv2.VideoCapture(CAMERA_IDX, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    if USE_V4L2_CONVERT_SO:
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
    else:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*CAMERA_FOURCC))
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_h)
    cap.set(cv2.CAP_PROP_FPS, 25)   # B0538C 스펙상 2592x1944 최대 fps (USB 3.0 기준)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def read_camera_frame(cap):
    """OpenCV V4L2 오류가 나도 프로그램 전체가 죽지 않게 한 프레임 읽기.
    V4L2 버퍼 assertion 오류는 pipeline이 즉시 망가지므로 False를 반환해 재연결을 유도.
    """
    try:
        ret, frame = cap.read()
    except (cv2.error, Exception) as e:
        message = str(e).splitlines()[-1] if str(e) else "unknown error"
        print(f"[WARN] 카메라 프레임 읽기 실패: {message}")
        return False, None

    if not ret or frame is None:
        return ret, frame

    if not USE_V4L2_CONVERT_SO and frame.size == FRAME_W * FRAME_H:
        frame = frame.reshape(FRAME_H, FRAME_W)

    if USE_V4L2_CONVERT_SO:
        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    elif len(frame.shape) == 2:
        frame = cv2.cvtColor(frame, BAYER_TO_BGR)
    elif len(frame.shape) == 3 and frame.shape[2] == 1:
        frame = cv2.cvtColor(frame[:, :, 0], BAYER_TO_BGR)
    elif len(frame.shape) == 3 and frame.shape[2] == 2:
        frame = cv2.cvtColor(frame, YUYV_TO_BGR)

    if frame.shape[0] <= 1 or frame.shape[1] <= 1:
        print(f"[WARN] 잘못된 카메라 프레임 크기: {frame.shape}")
        return False, None
    if len(frame.shape) != 3 or frame.shape[2] != 3:
        print(f"[WARN] 지원하지 않는 카메라 프레임 형식: {frame.shape}")
        return False, None

    frame = apply_color_correction(frame)
    return True, frame


def apply_color_correction(frame):
    """BGR 프레임에 guvcview 느낌의 색감 보정을 후처리로 적용."""
    channel_gains = get_color_correction_gains(COLOR_CORRECTION)
    corrected = frame.astype("float32")
    corrected *= channel_gains
    return corrected.clip(0, 255).astype("uint8")


def get_color_correction_gains(config):
    """guvcview식 보정값을 BGR 후처리용 채널 gain으로 변환."""
    color_gains = np.sqrt(np.array([
        config["blue_gain"],
        config["green_gain"],
        config["red_gain"],
    ], dtype=np.float32))
    brightness_gain = np.sqrt(np.float32(config["brightness_gain"]))
    return color_gains * brightness_gain


class CameraCapture:
    """카메라 캡처를 전용 데몬 스레드에서 실행.
    메인 루프(오디오·YOLO)와 완전히 분리되어 V4L2 select() 타임아웃을 방지.
    """

    def __init__(self, cap):
        self._cap   = cap
        self._frame = None
        self._ok    = False
        self._lock  = threading.Lock()
        self._stop  = threading.Event()
        self._fail  = 0
        t = threading.Thread(target=self._run, daemon=True)
        t.start()

    def _run(self):
        while not self._stop.is_set():
            ret, frame = read_camera_frame(self._cap)
            if ret:
                self._fail = 0
                with self._lock:
                    self._ok    = True
                    self._frame = frame
            else:
                self._fail += 1
                if self._fail >= 2:
                    print(f"[WARN] 카메라 오류 {self._fail}회 — 재연결 시도")
                    self._cap.release()
                    time.sleep(1.5)
                    for _ in range(3):
                        new_cap = open_camera(FRAME_W, FRAME_H)
                        if new_cap is not None:
                            self._cap = new_cap
                            for _ in range(10):
                                self._cap.read()
                            print("[OK] 카메라 재연결 성공")
                            break
                        time.sleep(1.0)
                    self._fail = 0
                else:
                    time.sleep(0.01)

    def read(self):
        """최신 프레임을 반환. 아직 없으면 (False, None)."""
        with self._lock:
            return self._ok, self._frame

    def release(self):
        self._stop.set()
        self._cap.release()


def setup_terminal_keyboard():
    """터미널에 포커스가 있어도 q/s 단일 키를 받을 수 있게 설정."""
    if not sys.stdin.isatty():
        return None
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    return old_settings


def restore_terminal_keyboard(old_settings):
    if old_settings is not None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def read_terminal_key():
    if not sys.stdin.isatty():
        return None
    readable, _, _ = select.select([sys.stdin], [], [], 0)
    if not readable:
        return None
    return sys.stdin.read(1).lower()


def handle_key(key, sample_player):
    """q는 종료, s는 sample 음성 재생."""
    if key == "q":
        return False
    if key == "s":
        sample_player.play_once()
    return True


# ──────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────

def main(use_motor: bool):
    ensure_opencv_qt_fonts()
    track_start_player = MP3LoopPlayer(TRACK_START_MP3, loop=False)
    sample_player      = MP3LoopPlayer(SAMPLE_MP3,      loop=False)
    route_exit_player  = MP3LoopPlayer(ROUTE_EXIT_MP3,  loop=False)
    right_exit_player  = MP3LoopPlayer(RIGHT_EXIT_MP3,  loop=False)
    left_exit_player   = MP3LoopPlayer(LEFT_EXIT_MP3,   loop=False)
    greenlight_player  = MP3LoopPlayer(GREENLIGHT_MP3,  loop=False)
    finish_player      = MP3LoopPlayer(FINISH_MP3,      loop=False)

    # 모터 초기화 (Damiao DM-J4310-2EC, CAN, 1000Hz MIT 스트리밍)
    tracker = None
    if use_motor:
        try:
            from motor.dm4310_tracker import Tracker
            tracker = Tracker()
            print("[OK] Damiao DM-J4310-2EC 모터 연결 완료")
        except Exception as e:
            print(f"[WARN] 모터 연결 실패 → DRY_RUN: {e}")

    # 검출기 로드 (기본: YOLOv8 pose. 파인튜닝 모델 교체 시 MODEL_PATH만 변경)
    detector = YoloPoseDetector(MODEL_PATH)
    print(f"[OK] 검출기 로드 완료 device={detector.device}")
    print(f"[ROUTE] angle-size detector enabled threshold={ROUTE_EXIT_SCORE_THRESHOLD:.0f}")

    cap = open_camera(FRAME_W, FRAME_H)
    if cap is None:
        print("[ERROR] 카메라를 열 수 없습니다.")
        sys.exit(1)

    # 카메라 워밍업 후 캡처 스레드 시작
    print("[..] 카메라 워밍업 중...")
    for _ in range(20):
        read_camera_frame(cap)
    print(f"[OK] 카메라 시작: {FRAME_W}x{FRAME_H}")
    camera = CameraCapture(cap)   # 전용 스레드에서 캡처 시작

    # 트래킹 상태
    arm_up_start   = defaultdict(lambda: None)
    arm_confirmed  = defaultdict(lambda: False)
    arm_down_count = defaultdict(lambda: 0)
    tracked_id     = None
    last_nose_x, last_nose_y = None, None
    last_face_box  = None
    was_tracking = False
    was_route_exit = False
    last_right_exit_time  = 0.0
    last_left_exit_time   = 0.0
    last_route_exit_log_time = 0.0
    terminal_settings = setup_terminal_keyboard()

    def _reset_tracking():
        """'w' 키 — 현재 트래킹을 즉시 해제하고 손 든 사람 대기 상태로 복귀."""
        nonlocal tracked_id, last_nose_x, last_nose_y, last_face_box, was_tracking
        if tracked_id is not None:
            arm_up_start[tracked_id]   = None
            arm_confirmed[tracked_id]  = False
            arm_down_count[tracked_id] = 0
        tracked_id = None
        last_nose_x, last_nose_y = None, None
        last_face_box = None
        was_tracking  = False

    def _set_volume(val: float):
        v = max(0.0, min(1.0, val))
        sample_player.set_volume(v)
        print(f"[VOL] {v:.0%}")

    try:
        while True:
            _key = read_terminal_key()
            if _key == 'w':
                _reset_tracking()
            elif _key == 'e':
                _reset_tracking()
                if tracker is not None:
                    tracker.go_center()
            elif _key == '+':
                _set_volume(sample_player.get_volume() + VOLUME_STEP)
            elif _key == '-':
                _set_volume(sample_player.get_volume() - VOLUME_STEP)
            elif _key == '0':
                _set_volume(VOLUME_DEFAULT)
            elif _key == 'o':
                greenlight_player.play_once()
            elif _key == 'p':
                finish_player.play_once()
            elif not handle_key(_key, sample_player):
                break

            ret, frame = camera.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            frame = cv2.flip(frame, 1)

            _t0 = time.time()
            people = detector.detect(frame)          # ~25fps 검출 (카메라 fps 한도, PersonPose 목록)
            _yolo_ms = (time.time() - _t0) * 1000

            nose_x, nose_y = None, None
            elapsed = 0.0
            current_face_box = None
            tracked_box = None

            n_person = len(people)

            # 유효하지 않은 tracked_id 초기화
            if tracked_id is not None and tracked_id >= n_person:
                tracked_id = None

            # 트래킹 대상 없으면 손 든 사람 중 가장 큰 박스 선택
            if tracked_id is None:
                best_area, best_id = 0, None
                for i, person in enumerate(people):
                    if not person.arm_up:
                        continue
                    x1, y1, x2, y2 = person.box
                    area = (x2 - x1) * (y2 - y1)
                    if area > best_area:
                        best_area, best_id = area, i
                if best_id is not None:
                    tracked_id = best_id

            # 트래킹 대상 처리
            if tracked_id is not None:
                person = people[tracked_id]
                arm_now = person.arm_up
                current_face_box = person.face_box  # 대기 중 표시용

                if arm_now:
                    arm_down_count[tracked_id] = 0
                    if arm_up_start[tracked_id] is None:
                        arm_up_start[tracked_id] = time.time()
                    elapsed = time.time() - arm_up_start[tracked_id]
                    if elapsed >= ARM_UP_THRESHOLD:
                        arm_confirmed[tracked_id] = True
                else:
                    arm_down_count[tracked_id] += 1
                    # grace period 내에는 트래킹 유지, 초과 시에만 해제
                    if arm_down_count[tracked_id] > ARM_DOWN_GRACE:
                        arm_up_start[tracked_id]  = None
                        arm_confirmed[tracked_id] = False
                        arm_down_count[tracked_id] = 0
                        tracked_id = None
                        last_nose_x, last_nose_y = None, None
                        last_face_box = None

                if tracked_id is not None and arm_confirmed[tracked_id] and person.nose is not None:
                    raw_x, raw_y = person.nose
                    tracked_box = person.box

                    # 얼굴 박스 필터: 이전 얼굴 박스 안에 있는 좌표만 수락
                    if last_nose_x is None:
                        # 첫 확정 → 그대로 사용하고 얼굴 박스 초기화
                        last_nose_x, last_nose_y = raw_x, raw_y
                        last_face_box = person.face_box
                    elif in_face_box(raw_x, raw_y, last_face_box):
                        # 얼굴 박스 안 → 수락 후 박스 갱신
                        last_nose_x, last_nose_y = raw_x, raw_y
                        last_face_box = person.face_box
                    # 얼굴 박스 밖 → 무시, 이전 좌표 유지

                    # motor/dm4310_tracker.py 의 FRAME_W/H 가 카메라 해상도와
                    # 동일하므로 스케일 없이 카메라 픽셀 좌표를 그대로 전달
                    # (화면 표시에도 동일한 좌표를 그대로 사용)
                    nose_x, nose_y = last_nose_x, last_nose_y

                    if tracker is not None:
                        tracker.update(nose_x, nose_y)

            is_tracking   = tracked_id is not None and arm_confirmed[tracked_id]
            is_arm_raised = tracked_id is not None and not is_tracking
            display_box   = last_face_box if is_tracking else current_face_box
            if is_tracking and not was_tracking:
                track_start_player.play_once()
            was_tracking = is_tracking

            yaw_rad = tracker.get_yaw_angle_rad() if tracker is not None else None
            angle_deg, size_percent, route_exit_score = calculate_route_exit_score(
                nose_x, tracked_box, yaw_rad=yaw_rad)
            is_route_exit = False
            exit_direction = None
            if is_tracking and tracked_box is not None:
                is_route_exit  = is_route_exit_by_angle_size(
                    nose_x, tracked_box, yaw_rad=yaw_rad)
                exit_direction = get_exit_direction(yaw_rad=yaw_rad, nose_x=nose_x)
                now = time.time()
                if now - last_route_exit_log_time >= ROUTE_EXIT_LOG_INTERVAL:
                    print(
                        f"[ROUTE] angle={angle_deg:.1f} size={size_percent:.1f}% "
                        f"score={route_exit_score:.1f}/{ROUTE_EXIT_SCORE_THRESHOLD:.0f} "
                        f"exit={is_route_exit} dir={exit_direction}"
                    )
                    last_route_exit_log_time = now
                if is_route_exit and not was_route_exit:
                    if exit_direction == 'right' and now - last_right_exit_time >= ROUTE_EXIT_COOLDOWN:
                        right_exit_player.play_once()
                        last_right_exit_time = now
                    elif exit_direction == 'left' and now - last_left_exit_time >= ROUTE_EXIT_COOLDOWN:
                        left_exit_player.play_once()
                        last_left_exit_time = now
                was_route_exit = is_route_exit
            elif not is_tracking:
                was_route_exit = False

            draw_route_exit_status(frame, angle_deg, size_percent, route_exit_score, is_route_exit, exit_direction)
            draw_info(frame, nose_x, nose_y, is_arm_raised, is_tracking, elapsed, display_box, FRAME_W, FRAME_H)
            cv2.putText(frame, f"YOLO {_yolo_ms:.1f}ms  ({1000/_yolo_ms:.0f}fps)",
                        (FRAME_W - 280, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)
            cv2.putText(frame, "[W]Retrack [E]Center [S]Sound [+/-]Vol [0]VolReset [O]Green [P]Finish [Q]Quit",
                        (20, FRAME_H - 85), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
            cv2.imshow("Face Tracker", frame)
            cv_key = cv2.waitKey(1) & 0xFF
            if cv_key != 255:
                ch = chr(cv_key).lower()
                if ch == 'w':
                    _reset_tracking()
                elif ch == 'e':
                    _reset_tracking()
                    if tracker is not None:
                        tracker.go_center()
                elif ch in ('+', '='):
                    _set_volume(sample_player.get_volume() + VOLUME_STEP)
                elif ch == '-':
                    _set_volume(sample_player.get_volume() - VOLUME_STEP)
                elif ch == '0':
                    _set_volume(VOLUME_DEFAULT)
                elif ch == 'o':
                    greenlight_player.play_once()
                elif ch == 'p':
                    finish_player.play_once()
                elif not handle_key(ch, sample_player):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        restore_terminal_keyboard(terminal_settings)
        track_start_player.close()
        sample_player.close()
        route_exit_player.close()
        right_exit_player.close()
        left_exit_player.close()
        greenlight_player.close()
        finish_player.close()
        camera.release()
        cv2.destroyAllWindows()
        if tracker is not None:
            tracker.close()
            print("[OK] 모터 토크 OFF, 포트 닫힘")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--motor", action="store_true", help="Dynamixel 모터 제어 활성화")
    args = parser.parse_args()
    main(use_motor=args.motor)
