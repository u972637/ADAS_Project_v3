#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Optional experiment menu; production main.py does not depend on this file.
1/2 rehearse the shared pipeline; 3 retains the legacy hand-raise demo.
4/5 use the same Tracker, logical right/up +, limits and 500 Hz loop as runtime.
"""

import math
import os
import platform
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

from motor.config import FRAME_W, FRAME_H
# camera_test.py 는 같은 tools/ 폴더에 있는 가벼운 카메라 열기/프레임 변환
# 유틸리티 — people/main_facetrack.py 를 그대로 import하면 ultralytics/torch 까지
# 끌려와 무거워지므로, 카메라 관련 부분만 재사용한다.
from camera_test import DEFAULT_CAMERA_IDX, open_camera as cam_open_camera, read_frame as cam_read_frame
from animal.camera import camera_hint
from detection.yolo_pose import YoloPoseDetector

from animal.mp3_player import MP3LoopPlayer
from animal.audio import AudioCatalog, WavPlayer

AXES = ("yaw", "pitch")
# 방향키 한 번(자동반복 포함) 입력당 움직일 각도(도). 보수적으로 작게 잡음.
STEP_DEG = 1.0

from motor.config import CONTROL_FREQ, YAW_RANGE_RAD, PITCH_RANGE_RAD
YAW_MIN_DEG, YAW_MAX_DEG = -math.degrees(YAW_RANGE_RAD), math.degrees(YAW_RANGE_RAD)
PITCH_MIN_DEG, PITCH_MAX_DEG = -math.degrees(PITCH_RANGE_RAD), math.degrees(PITCH_RANGE_RAD)
_RANGE_DEG = {'yaw': (YAW_MIN_DEG, YAW_MAX_DEG), 'pitch': (PITCH_MIN_DEG, PITCH_MAX_DEG)}
DT = 1.0 / CONTROL_FREQ

# ──────────────────────────────────────────────
# 공통: q 로 정상 종료할 때 돌아갈 "숙인위치"
# ──────────────────────────────────────────────
BOW_YAW_DEG   = 0.0
BOW_PITCH_DEG = -4.0
BOW_DURATION  = 1.0   # 초

# ──────────────────────────────────────────────
# 3번(YOLO 코 좌표 추적 테스트) 전용 설정
# ──────────────────────────────────────────────

# people/main_facetrack.py 와 동일한 모델/기준값 — 파인튜닝 모델로 바꿀 땐
# 이 경로만 바꾸면 됨.
MODEL_PATH = os.path.join(PROJECT_ROOT, "assets", "models", "yolov8n-pose.pt")
ARM_UP_THRESHOLD = 2.0    # 초 — 이만큼 손을 들고 있어야 추적 확정
ARM_DOWN_GRACE = 10       # 프레임 — 이만큼 넘게 손을 내리면 추적 해제

# 3번("인식모드") 진입 시 1회 재생할 안내음
TRACK_START2_PATH = os.path.join(PROJECT_ROOT, "assets", "voice", "track_start2.mp3")

# 3번은 1/2(assets/warnings*, 중앙 근접도 기반)와 트리거 방식은 다르지만
# 같은 음원 폴더(settings.WARNING_AUDIO_DIR, 현재 warnings_pitch_eq)를 쓴다.
# A=멧돼지(boar), S=고라니(water_deer) 재생, D=중지(재생 중이든 아니든).
MENU3_ALARM_LABELS = ("boar", "water_deer")

# 3번에서 "추적 확정 아님" → 기본자세 복귀는 CONTROL_MODE별 기본 길이(현재
# step모드 3초)와 무관하게 1초 고정 — 1/2번의 종료조건 복귀(8초, settings.
# TERMINATION_RETURN_SECONDS)보다 훨씬 빠르게, 시작하자마자도 바로 반응하도록.
MENU3_DEFAULT_RETURN_SECONDS = 1.0

# 3번만 쓰는 기본자세 — 1/2번 공유 기본자세(motor/config.py의
# DEFAULT_YAW_DEG/DEFAULT_PITCH_DEG, 현재 0°/15°)와 분리.
MENU3_DEFAULT_YAW_DEG = 0.0
MENU3_DEFAULT_PITCH_DEG = 30.0

# 2592x1944 그대로 창을 띄우면 화면보다 커서 축소 표시만 한다
# (Tracker.update() 에는 항상 원본 해상도 좌표를 그대로 넘김).
DISPLAY_SCALE = 0.30


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


from animal.overlay import put_text_kr as put_text_kr


def _bow_tracker(tracker, duration: float = BOW_DURATION):
    """q 로 정상 종료할 때 공통으로 호출 — '숙인위치'로 duration초에 걸쳐
    램프 이동시키고, 그 이동이 끝날 때까지 기다린 뒤 돌아온다."""
    print(f"[숙인위치 복귀] {duration:.1f}초 동안 yaw={BOW_YAW_DEG:.0f}°, "
          f"pitch={BOW_PITCH_DEG:.0f}° 로 이동합니다...")
    tracker.go_to(yaw_rad=math.radians(BOW_YAW_DEG),
                  pitch_rad=math.radians(BOW_PITCH_DEG),
                  duration=duration)
    if not tracker.wait_until_complete():
        raise RuntimeError("기본 자세 이동 시간 초과")


# ──────────────────────────────────────────────
# 실험 메뉴의 터미널 raw 키 입력 (정식 main은 Enter로 명령을 제출)
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
    """Manual goals use the common controller instead of a second CAN implementation."""
    def __init__(self):
        from animal import settings as _settings

        if _settings.MOTOR_PROCESS:
            from motor.ipc_tracker import RemoteTracker

            self.tracker = RemoteTracker()
        else:
            from motor.factory import create_tracker

            self.tracker = create_tracker()
        self.target_deg = {a: self.current_deg(a) for a in AXES}

    def current_deg(self, axis):
        return math.degrees(self.tracker.get_state(axis)[0])

    def _send(self):
        self.tracker.assert_healthy()

    def nudge(self, axis, delta_deg):
        lo, hi = _RANGE_DEG[axis]
        self.target_deg[axis] = clamp(self.target_deg[axis]+delta_deg, lo, hi)
        self.tracker.go_to(*(math.radians(self.target_deg[a]) for a in AXES), duration=0)

    def ramp_to(self, goal_deg, duration):
        self.target_deg = {a: clamp(goal_deg[a], *_RANGE_DEG[a]) for a in AXES}
        self.tracker.go_to(*(math.radians(self.target_deg[a]) for a in AXES), duration=duration)
        if not self.tracker.wait_until_complete():
            raise RuntimeError("실험 자세 이동 시간 초과")

    def close(self):
        self.tracker.close()


# ──────────────────────────────────────────────
# 메뉴 4: 방향키 제어
# ──────────────────────────────────────────────

def run_manual_jog(mc: ManualController):
    print("\n[방향키 제어] a/d=yaw -/+, w/s=pitch +/-, q=메뉴로 복귀 (Ctrl+C=완전 종료)")
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
                mc.nudge("yaw", -STEP_DEG)
            elif key == 'd':
                mc.nudge("yaw", +STEP_DEG)
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
# 메뉴 5: 음향테스트용 위치 고정 (yaw=0°, pitch=0°)
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
# 메뉴 2: 사람을 동물로 간주해 추적 (person_proxy=True)
# ──────────────────────────────────────────────

def run_wildlife_track(person_proxy=False):
    if person_proxy:
        from tools.person_proxy import run_person_test
        run_person_test()
    else:
        from animal.runtime import run
        run(enable_motor=True, start_paused=True)



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
    기본 자세(MENU3_DEFAULT_YAW_DEG, MENU3_DEFAULT_PITCH_DEG = 0°, 30° — 1/2번과 분리)로 Tracker.go_to()
    를 1회 호출해 복귀시킨다 — 매 프레임 다시 호출하지 않도록(램프 시작점이
    계속 "지금 위치"로 리셋돼 버려 실제로는 못 움직이게 되므로) 상태가
    "타겟 모드 → 아님"으로 바뀌는 순간에만 트리거한다.

    손을 들어 ARM_UP_THRESHOLD초가 "막" 지나서 추적이 확정되는 그 순간에만
    assets/voice/track_start2.mp3 를 1회 재생한다(추적 중엔 다시 재생하지 않음).
    화면에는 대상 사람의 몸 박스를 항상 표시하며, 확정 전엔 노란색, 확정
    (타겟 모드)되면 초록색으로 바뀐다. **r 키**를 누르면 확정 상태를 즉시
    해제하고(재추적) 다시 손들기 대기 상태로 돌아간다. q/ESC 로 정상
    종료하면 _bow_tracker() 로 숙인위치에 복귀한다.

    카메라는 animal/settings.py 의 CAMERA_INDEX(/dev/videoN, 또는 설정 시 CAMERA_DEVICE)로
    camera_test.py 의 open_camera()/read_frame() 을 재사용한다.

    4번(수동 조그)과 포트를 동시에 쓸 수 없으므로(같은 시리얼 장치), 이
    함수 안에서 자체적으로 Tracker 를 새로 열고 끝나면 닫는다 — 호출하는
    쪽(main())이 먼저 ManualController 를 닫아둔 상태여야 한다.
    """
    print("\n[인식모드] 손을 들어 ARM_UP_THRESHOLD초 이상 유지하면 추적 시작")
    print(f"  ARM_UP_THRESHOLD={ARM_UP_THRESHOLD:.0f}s  ARM_DOWN_GRACE={ARM_DOWN_GRACE}frame"
          f"  — q 또는 ESC 로 메뉴 복귀\n")

    print(f"[..] YOLO pose 모델 로드 중... ({MODEL_PATH})")
    detector = YoloPoseDetector(MODEL_PATH)
    print(f"[OK] 모델 로드 완료 (device={detector.device})")

    print(f"[..] 카메라 {DEFAULT_CAMERA_IDX} 여는 중...")
    cap = cam_open_camera(DEFAULT_CAMERA_IDX)
    if cap is None:
        print(f"[ERROR] 카메라 {DEFAULT_CAMERA_IDX} 를 열 수 없습니다 — 메뉴로 복귀합니다.")
        print(f"        {camera_hint()}")
        return

    print("[..] Tracker 연결 중...")
    from animal import settings as _settings

    if _settings.MOTOR_PROCESS:
        from motor.ipc_tracker import RemoteTracker

        tracker = RemoteTracker()
        print("[OK] 연결 완료(GIL 분리 프로세스) — 창이 뜨면 손을 들어보세요.")
    else:
        from motor.factory import create_tracker

        tracker = create_tracker()
        print("[OK] 연결 완료 — 창이 뜨면 손을 들어보세요.")

    disp_w, disp_h = int(FRAME_W * DISPLAY_SCALE), int(FRAME_H * DISPLAY_SCALE)
    win_name = "YOLO Nose Tracking (arm-up gated, q/ESC: back)"
    cv2.namedWindow(win_name)

    alarm_catalogs = {}
    alarm_player = None
    try:
        command = _settings.AUDIO_PLAYER_COMMAND or (
            "afplay" if platform.system() == "Darwin" else "aplay"
        )
        for label in MENU3_ALARM_LABELS:
            folder = _settings.WARNING_AUDIO_DIR / label
            if list(folder.glob("*.wav")):
                alarm_catalogs[label] = AudioCatalog.from_directory(
                    _settings.WARNING_AUDIO_DIR, {label}, 1
                )
            else:
                print(f"[WARN] 경계음 없음: {folder}/*.wav — 해당 키 비활성")
        if alarm_catalogs:
            alarm_player = WavPlayer(command)
    except Exception as error:
        print(f"[WARN] 경계음 재생기 초기화 실패: {error} — A/S 키 비활성")
        alarm_catalogs = {}

    def play_alarm(label):
        if alarm_player is None or label not in alarm_catalogs:
            print(f"\n[WARN] {label} 경계음 파일이 없어 재생할 수 없습니다.")
            return
        try:
            alarm_player.play(alarm_catalogs[label].next(label))
        except Exception as error:
            print(f"\n[WARN] 경계음 재생 실패: {error}")

    def stop_alarm():
        if alarm_player is not None:
            alarm_player.stop()

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
                    stop_alarm()  # 추적 종료 — 재생 중이든 아니든 소리 끔
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
                tracker.go_to(yaw_rad=math.radians(MENU3_DEFAULT_YAW_DEG),
                              pitch_rad=math.radians(MENU3_DEFAULT_PITCH_DEG),
                              duration=MENU3_DEFAULT_RETURN_SECONDS)
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
                status = f"NOT TRACKING -> default pose ({MENU3_DEFAULT_YAW_DEG:.0f}, {MENU3_DEFAULT_PITCH_DEG:.0f})"
                color = (255, 150, 0)
            cv2.putText(disp, status, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            cv2.putText(disp, "[A]Boar [S]Deer [D]Stop [R]Reset [Q]Quit", (10, disp_h - 35),
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
                stop_alarm()
                print("\n[재추적] r 입력 — 추적 확정 해제, 기본 자세로 복귀합니다.")
            elif key == ord('a'):
                play_alarm("boar")
            elif key == ord('s'):
                play_alarm("water_deer")
            elif key == ord('d'):
                stop_alarm()
        # while 을 'q'/ESC(정상 종료)로 빠져나온 경우에만 실행된다 — 루프 중
        # 예외(Ctrl+C 등)가 나면 아래로 내려오지 않고 바로 finally 로 간다.
        _bow_tracker(tracker)
    finally:
        stop_alarm()
        cv2.destroyWindow(win_name)
        cap.release()
        tracker.close()
        print("\n[메뉴로 복귀] 카메라/Tracker 연결 종료")


# ──────────────────────────────────────────────
# 메뉴
# ──────────────────────────────────────────────

def print_menu():
    print("\n======================================")
    print(" 실험 메뉴 (정식 운용: python main.py)")
    print("======================================")
    print(" 1. 동물 모델 통합 실험 (조준·음향 포함, W로 시작)")
    print(" 2. 사람 대역 통합 실험 (pose 모델, W로 시작)")
    print(" 3. YOLO 코 좌표 추적 (손들기로 시작, 인식모드 진입 시 안내음)")
    print(" 4. 방향키로 위치 제어 (모터 각도 직접 조그)")
    print(" 5. 음향테스트용 위치 고정 (yaw=0°, pitch=0°)")
    print(" 0. 종료")
    print("======================================")


def main():
    mc = None  # Open a motor connection only for the selected menu.

    try:
        while True:
            print_menu()
            try:
                choice = input("선택 > ").strip()
            except EOFError:
                break

            if choice in ("1", "2"):
                # 1·2·3번은 같은 시리얼 포트를 Tracker 로 새로 열어야 해서,
                # ManualController 가 들고 있던 연결을 잠깐 반납했다가
                # 돌아오면 다시 연다.
                if mc is not None:
                    mc.close()
                    mc = None
                run_wildlife_track(person_proxy=(choice == "2"))
            elif choice == "3":
                # 기본 자세로의 복귀는 따로 먼저 하지 않는다 — YOLO 모델
                # 로드가 끝나고 추적 루프가 돌기 시작하면 (아직 타겟 모드가
                # 아닌) 첫 프레임부터 run_auto_track() 내부의 "타겟 모드
                # 아님 → 기본 자세" 로직이 자연스럽게 같은 동작을 한다.
                if mc is not None:
                    mc.close()
                    mc = None
                run_auto_track()
            elif choice == "4":
                if mc is None:
                    mc = ManualController()
                run_manual_jog(mc)
            elif choice == "5":
                if mc is None:
                    mc = ManualController()
                run_audio_test_pose(mc)
            elif choice == "0":
                print("종료합니다.")
                break
            else:
                print("[!] 0, 1, 2, 3, 4, 5 중에서 선택하세요.")
    except KeyboardInterrupt:
        print("\n[중단]")
    finally:
        if mc is not None:
            mc.close()
        print("[OK] 모터 비활성화, 포트 닫힘")


if __name__ == "__main__":
    main()
