#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DM-J4310-2EC 2축(Yaw-Pitch) CAN MIT모드 대상 추적 모터 제어 모듈.

카메라/동물 검출은 외부 코드가 담당하고, 이 모듈은 대상의 화면 좌표를
모터 명령으로 바꾼다. tracker.update(x, y)를 검출된 프레임마다 호출한다.

클로즈드 루프 설계 (호스트 측 피드백)
  이전 버전은 호스트가 내부적으로 시뮬레이션한(quintic spline) 위치를 기준으로
  다음 목표를 계산했다 — 모터가 실제로 그 위치에 있는지는 확인하지 않는 사실상
  피드포워드 스트리밍이었다.

  이 버전은 매 CAN 사이클마다 모터가 응답으로 돌려주는 실제 위치/속도/토크
  (motor/DM_CAN.py 의 controlMIT() 가 전송 직후 자동으로 recv() 해서 파싱)를
  self._current 로 읽어와서 다음 목표 계산의 "현재값"으로 사용한다.
  목표값(self._target)은 검출 루프가 준 좌표를 그대로(증분 스텝만큼) 반영하며,
  더 이상 호스트 쪽에서 quintic spline 등으로 궤적을 깎지 않는다 — 부드러움은
  MIT 모드의 Kp/Kd 임피던스(스프링-댐퍼)가 물리적으로 담당한다.

한 사이클(1000Hz) 동작
  1) self._target 을 그대로 controlMIT(q_des=target, dq_des=0, Kp, Kd, tau_ff) 로 전송
  2) 전송 직후 모터 응답으로 갱신된 실제 위치/속도/토크를 self._current 에 저장
  3) go_center() 로 인한 중앙 복귀 램프가 진행 중이면 target 을 선형 보간

update() 동작 (검출 루프, ~30Hz)
  픽셀 오차 → 각도 스텝(rad) 환산 → target = 실제 현재 위치(self._current) + 스텝
  (스프링-댐퍼 특성상 target 이 계단형으로 튀어도 실제 움직임은 부드럽게 수렴함)

준비물
  - Damiao 전용 USB-CAN 동글, /dev/ttyUSBx 로 연결 (motor/DM_CAN.py 참고, 921600 baud)
  - 두 모터의 CAN ID가 Yaw=0x01, Pitch=0x02(MasterID 0x11/0x12)로 설정돼
    있어야 함 — tools/actuator_conn_test.py 의 "CAN ID 찾기/할당"(--step id)로
    Windows 전용 소프트웨어 없이 CAN 통신만으로 확인/변경 가능
  - MIT 모드는 __init__() 에서 매번 자동으로 전환(switchControlMode)하므로
    별도 사전 설정 불필요 (새 모터는 기본값이 MIT이지만, 혹시 다른 모드로
    저장된 적이 있어도 이 초기화 과정에서 MIT로 맞춰짐)

주의 (실제 하드웨어 투입 전 반드시 확인)
  - YAW_RANGE_RAD / PITCH_RANGE_RAD 는 기구적으로 안전한 실제 가동범위로 조정할 것
  - RAD_PER_PIXEL_X/Y 는 Arducam B0538C 렌즈 스펙(HFOV/VFOV)으로 이론 유도한
    값이며(아래 상수 정의부 주석 참고), 짐벌 기어비(모터-카메라 간 감속비)는
    반영되어 있지 않다. 기어비가 1:1이 아니면 그 비율만큼 곱해서 보정할 것,
    실측 후 미세조정 필요
  - KP/KD 는 낮은 값부터 시작해서 진동 없이 안정적으로 추종할 때까지 올릴 것
  - 시리얼 왕복 특성상 self._current 는 최대 한두 사이클(<2ms) 지연될 수 있음
"""

import math
import time
import threading

import serial

from motor.DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type

# ============================================================
# 1) 연결 설정
# ============================================================

# 실측(lsusb/ls /dev/serial/by-id): Damiao 동글이 "HDSC CDC Device"로 USB
# CDC-ACM 클래스로 잡혀 /dev/ttyACM0 로 인식됨 (USB-Serial 칩 종류에 따라
# ttyUSB 대신 ttyACM 으로 뜨는 경우) — 이 머신 기준값이니 다른 PC에 꽂으면
# ls /dev/ttyUSB* /dev/ttyACM* 로 다시 확인할 것.
DEVICENAME = "/dev/ttyACM0"
BAUDRATE   = 921600

MOTOR_TYPE = DM_Motor_Type.DM4310

CAN_ID_YAW      = 0x01
CAN_ID_PITCH    = 0x02
MASTER_ID_YAW   = 0x11
MASTER_ID_PITCH = 0x12

AXES = ("yaw", "pitch")

# ============================================================
# 2) 화면 / 추적 파라미터
# ============================================================

# 검출기가 넘겨주는 좌표가 기준으로 삼는 해상도.
# Arducam B0538C 목표 캡처 해상도(2592x1944)와 동일하다. 다른 해상도로
# 검출한 경우 호출자가 이 좌표계로 스케일해야 한다.
FRAME_W, FRAME_H   = 2592, 1944
CENTER_X, CENTER_Y = FRAME_W // 2, FRAME_H // 2

# ------------------------------------------------------------
# 픽셀 오차 → 목표 각도 스텝(rad) 환산계수 — 이론적 유도 (Arducam B0538C 렌즈 스펙 기준)
# ------------------------------------------------------------
# 렌즈 스펙 (핀홀/레티리니어 모델의 왜곡을 무시한 근사에 사용):
#   대각 화각 DFOV = 95°, 수평 화각 HFOV = 82°, 수직 화각 VFOV = 66°
#   초점거리 EFL = 3.6mm, F수 = F2.8, 픽셀 피치 = 2.2µm (센서: OG05B1B)
#
# 레티리니어 렌즈는 광축에서 x만큼 떨어진 점의 각도가 atan(x/f) 이고,
# 제어 목표가 대상을 화면 중앙 근처로 유지하는 것이라 x가 작을 때
# atan(x/f) ≈ x/f 로 선형 근사가 잘 맞는다. 화면 전체 화각(FOV)이 스펙으로
# 주어졌으므로, 화면 전체에 각도가 픽셀에 선형 비례한다고 가정하면:
#
#   RAD_PER_PIXEL_X = radians(HFOV) / FRAME_W   (가로 → Yaw)
#   RAD_PER_PIXEL_Y = radians(VFOV) / FRAME_H   (세로 → Pitch)
#
# 교차검증(초점거리·픽셀피치 기반 광축 중심 국소 민감도 dθ/dx = 1/f):
#   pixel_pitch(0.0022mm) / EFL(3.6mm) ≈ 0.000611 rad/px
#   → 아래 두 값과 10~20% 이내로 일치해 FOV 기반 근사가 타당함을 확인.
#   (완전히 같지 않은 건 렌즈 왜곡·FOV 정의 차이 때문 — 실제 튜닝 시 미세조정 필요)
HFOV_DEG = 82.0
VFOV_DEG = 66.0

RAD_PER_PIXEL_X = math.radians(HFOV_DEG) / FRAME_W   # ≈ 0.000552 rad/px, err_x → Yaw
RAD_PER_PIXEL_Y = math.radians(VFOV_DEG) / FRAME_H   # ≈ 0.000593 rad/px, err_y → Pitch

# 실측 확인된 하드웨어 동작: yaw/pitch 모두 "+각도 명령 → 추적 대상의 픽셀
# 좌표가 (0,0)(좌상단) 쪽으로 가까워짐"(yaw+ → 좌측, pitch+ → 위쪽).
# 즉 대상 픽셀이 CENTER 보다 작은 쪽(= err 가 양수)에 있을 때는 그쪽으로
# "더" 보내면 안 되고(이미 0쪽에 가까운데 더 밀면 더 벗어남), 반대 부호로
# 보정해야 중앙으로 수렴한다 → 두 축 모두 SIGN = -1.
SIGN_YAW   = -1
SIGN_PITCH = -1

DEADZONE_PX  = 0
MAX_STEP_RAD = 0.6   # update() 한 번에 허용할 최대 목표각 이동량 — 튀는 검출값 방어

# 짐벌 가동범위 (중앙 0 rad 기준, 실제 기구 한계에 맞춰 조정할 것)
YAW_RANGE_RAD   = 1.2
PITCH_RANGE_RAD = 0.6

_RANGE_RAD = {"yaw": YAW_RANGE_RAD, "pitch": PITCH_RANGE_RAD}

# ============================================================
# 3) MIT 모드 임피던스 게인
# ============================================================
#   torque = Kp*(q_des - q) + Kd*(dq_des - dq) + tau_ff
#   낮은 값에서 시작해 진동 없이 잘 따라올 때까지 올릴 것 (DM4310 Kp 0~500, Kd 0~5)
#   호스트 쪽 궤적 스무딩이 없어졌으므로, 부드러움은 전적으로 이 두 값이 담당한다.

KP     = 500.0
KD     = 5.0
TAU_FF = 0.0

# go_center() 전용 중앙 복귀 램프 지속 시간(초) — 선형 보간
GO_CENTER_DURATION = 1.5

# 궤적 스트리밍 스레드 목표 주파수(Hz). 실제 달성 주파수는 USB-CAN 동글의
# 시리얼 처리량에 따라 달라짐 — get_actual_freq() 로 실측 가능.
CONTROL_FREQ = 1000


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Tracker:
    def __init__(self, devicename: str = DEVICENAME, baudrate: int = BAUDRATE):
        self._serial = serial.Serial(devicename, baudrate, timeout=0.001)
        self._ctrl = MotorControl(self._serial)

        self._motor = {
            "yaw":   Motor(MOTOR_TYPE, CAN_ID_YAW,   MASTER_ID_YAW),
            "pitch": Motor(MOTOR_TYPE, CAN_ID_PITCH, MASTER_ID_PITCH),
        }
        for axis in AXES:
            self._ctrl.addMotor(self._motor[axis])

        # CTRL_MODE(RID=10) 레지스터가 MIT(1)가 아닌 다른 모드로 저장돼
        # 있으면 controlMIT() 프레임을 보내도 모터가 반응하지 않는다.
        # 새 모터는 기본값이 MIT이지만, 이전에 다른 모드로 쓰고 저장된 적이
        # 있을 수 있으니 매번 명시적으로 MIT로 전환한다 (세션 한정 전환 —
        # save_motor_param() 을 호출하지 않으므로 플래시에는 저장되지 않고,
        # 이 초기화 시점에만 확실히 MIT로 맞춰두는 안전장치).
        for axis in AXES:
            if not self._ctrl.switchControlMode(self._motor[axis], Control_Type.MIT):
                print(f"[WARN] {axis} 축을 MIT 모드로 전환했는지 확인할 수 없음 "
                      f"— controlMIT() 명령에 반응이 없을 수 있음")

        # 모터 활성화
        for axis in AXES:
            self._ctrl.enable(self._motor[axis])

        # 실제 현재 각도를 읽어 초기 목표/현재값으로 사용 (급격한 점프 방지)
        init_pos = {}
        for axis in AXES:
            self._ctrl.refresh_motor_status(self._motor[axis])
            init_pos[axis] = _clamp(
                self._motor[axis].getPosition(), -_RANGE_RAD[axis], _RANGE_RAD[axis])

        self._lock = threading.Lock()
        # 실측값 (모터 응답으로 매 사이클 갱신): {axis: (pos, vel, tau)}
        self._current = {axis: (init_pos[axis], 0.0, 0.0) for axis in AXES}
        # 목표값 (update() / go_center() 가 갱신): {axis: pos}
        self._target = dict(init_pos)
        # 중앙 복귀 램프 진행 상태 (None 이면 비활성)
        self._ramp = None

        # 실측 제어주파수 (디버깅/모니터링용)
        self._actual_freq = 0.0
        self._freq_lock = threading.Lock()

        # 제어 스트리밍 스레드 시작
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._control_thread, daemon=True)
        self._thread.start()

    # --------------------------------------------------------
    # 제어 스트리밍 스레드 (1000Hz 목표)
    # --------------------------------------------------------

    def _control_thread(self):
        """
        CONTROL_FREQ Hz 로 self._target 을 그대로 MIT 프레임에 실어 보내고,
        전송 직후 모터 응답(실제 위치/속도/토크)을 self._current 에 반영한다.

        고정 dt 로 sleep 하지 않고 다음 목표 시각(next_t)을 누적해 두어
        한 사이클이 늦어져도 위상이 계속 밀리지 않도록 함(잉크리멘탈 스케줄링).
        """
        dt = 1.0 / CONTROL_FREQ
        next_t = time.time()
        tick_count = 0
        window_t0 = next_t

        while not self._stop_event.is_set():
            t_now = time.time()

            with self._lock:
                if self._ramp is not None:
                    tau = min((t_now - self._ramp['t0']) / self._ramp['T'], 1.0)
                    for axis in AXES:
                        start = self._ramp['start'][axis]
                        self._target[axis] = start + (0.0 - start) * tau
                    if tau >= 1.0:
                        self._ramp = None
                q_des = dict(self._target)

            for axis in AXES:
                goal = _clamp(q_des[axis], -_RANGE_RAD[axis], _RANGE_RAD[axis])
                motor = self._motor[axis]

                self._ctrl.controlMIT(motor, KP, KD, goal, 0.0, TAU_FF)

                # controlMIT() 이 전송 직후 recv() 해서 갱신해 둔 실측값을 반영
                with self._lock:
                    self._current[axis] = (
                        motor.getPosition(), motor.getVelocity(), motor.getTorque())

            tick_count += 1
            if tick_count >= CONTROL_FREQ:
                now = time.time()
                with self._freq_lock:
                    self._actual_freq = tick_count / max(now - window_t0, 1e-9)
                tick_count = 0
                window_t0 = now

            next_t += dt
            sleep_time = next_t - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # 이번 사이클이 늦었으면 기준 시각을 현재로 재설정해
                # 이후 프레임이 연달아 몰아치지 않게 함
                next_t = time.time()

    def get_actual_freq(self) -> float:
        """직전 1초(대략) 동안 실측된 제어 스트리밍 주파수(Hz)."""
        with self._freq_lock:
            return self._actual_freq

    # --------------------------------------------------------
    # 핵심 제어 루프: 검출 루프에서 매 프레임 호출
    # --------------------------------------------------------

    def update(self, target_x, target_y):
        """
        대상 픽셀 좌표(검출 프레임마다) → 목표 각도 스텝 계산 → target 갱신.

        target 은 항상 '모터가 실제로 보고한 현재 위치'(self._current)를
        기준으로 계산한다. 호스트가 시뮬레이션한 값이 아니라 실측값을 쓰므로
        모터가 밀리거나 막혀도 다음 보정이 실제 상태 기준으로 이루어진다.
        실제 모터 전송은 _control_thread(1000Hz) 가 담당하므로 이 함수는
        블로킹 없음.
        """
        if target_x is None or target_y is None:
            return

        err_x = CENTER_X - target_x
        err_y = CENTER_Y - target_y

        if abs(err_x) < DEADZONE_PX:
            err_x = 0
        if abs(err_y) < DEADZONE_PX:
            err_y = 0

        if err_x == 0 and err_y == 0:
            return

        step_yaw   = _clamp(SIGN_YAW   * err_x * RAD_PER_PIXEL_X, -MAX_STEP_RAD, MAX_STEP_RAD)
        step_pitch = _clamp(SIGN_PITCH * err_y * RAD_PER_PIXEL_Y, -MAX_STEP_RAD, MAX_STEP_RAD)

        with self._lock:
            self._ramp = None   # 새 검출 좌표가 들어오면 중앙 복귀 램프는 취소

            cur_yaw,   _, _ = self._current["yaw"]
            cur_pitch, _, _ = self._current["pitch"]

            self._target["yaw"]   = _clamp(cur_yaw   + step_yaw,   -YAW_RANGE_RAD,   YAW_RANGE_RAD)
            self._target["pitch"] = _clamp(cur_pitch + step_pitch, -PITCH_RANGE_RAD, PITCH_RANGE_RAD)

    def get_state(self, axis: str):
        """axis('yaw'|'pitch') 의 실측 (pos_rad, vel_rad_s, torque_nm) 반환."""
        with self._lock:
            return self._current[axis]

    def get_yaw_angle_rad(self) -> float:
        """Yaw 모터의 실측 현재 각도(rad, 중앙=0)를 반환 (목표값이 아닌 실제값)."""
        with self._lock:
            return self._current["yaw"][0]

    def go_center(self):
        """두 모터를 선형 램프로 중앙(0 rad)까지 천천히 복귀."""
        t_now = time.time()
        with self._lock:
            start = {axis: self._current[axis][0] for axis in AXES}
            self._ramp = {'t0': t_now, 'T': GO_CENTER_DURATION, 'start': start}

    def close(self):
        """종료 시 스레드 정지 후 모터 비활성화, 시리얼 포트 닫기."""
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        for axis in AXES:
            self._ctrl.disable(self._motor[axis])
        self._serial.close()
