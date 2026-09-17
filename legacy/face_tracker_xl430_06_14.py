#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XL430-W250-T 2축(Yaw-Pitch) 얼굴(코) 추적 - 모터 제어 모듈

역할 분담
  - 카메라/코 검출(좌표 계산)은 "외부 코드"가 담당
  - 이 파일은 "코 좌표 (x, y) -> 모터 명령" 부분만 담당
  - 사용법: tracker.update(nose_x, nose_y) 를 매 프레임 호출

한 사이클 동작
  1) 픽셀 오차 → 환산계수 → 목표 펄스 계산
  2) 현재 궤적 상태(위치·속도·가속도)를 시작점으로 quintic spline 재계획
  3) 별도 스레드(200 Hz)가 중간 위치를 GroupSyncWrite 로 모터에 스트리밍
  4) 모터 내장 PROFILE_VELOCITY = 0 (속도 무제한): 부드러움은 궤적이 담당

준비물
  - Dynamixel SDK  : pip install dynamixel-sdk
  - Dynamixel Wizard 2.0 으로 Yaw=ID 1, Pitch=ID 2 미리 설정
  - 모터 전원(배터리/SMPS 12V) 연결, U2D2 USB 연결
"""

import os
import time
import threading
from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    GroupSyncRead,
    DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD,
    COMM_SUCCESS,
)

# ============================================================
# 1) 연결 설정
# ============================================================

DEVICENAME   = "/dev/ttyUSB0"
BAUDRATE     = 1000000
PROTOCOL_VER = 2.0
DXL_ID_YAW   = 1
DXL_ID_PITCH = 2
DXL_IDS      = [DXL_ID_YAW, DXL_ID_PITCH]

# ============================================================
# 2) XL430 컨트롤 테이블 주소 및 크기
# ============================================================

ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
LEN_POSITION          = 4

OP_MODE_POSITION = 3
TORQUE_ENABLE    = 1
TORQUE_DISABLE   = 0

POS_MIN    = 1024
POS_MAX    = 3072
POS_CENTER = 2048

# ============================================================
# 3) 화면 / 추적 파라미터
# ============================================================

FRAME_W, FRAME_H   = 1280, 720
CENTER_X, CENTER_Y = FRAME_W // 2, FRAME_H // 2

# 픽셀 오차 → 펄스 이동량 환산계수
CONVERSION  = 0.25


SIGN_YAW    = -1
SIGN_PITCH  = +1

DEADZONE_PX = 0
MAX_STEP    = 700

# 초기 정면 이동용 속도 제한 (이후 궤적 스트리밍으로 전환 시 0 으로 바꿈)
PROFILE_VELOCITY_INIT = 50

# ============================================================
# 4) Quintic spline 파라미터
# ============================================================

# 한 구간의 지속 시간 (초).
#   작게(0.05~0.10): 응답 빠르지만 움직임이 다소 거칠어짐
#   크게(0.20~0.30): 매우 부드럽지만 응답이 느려짐
TRAJ_DURATION = 0.2

# go_center() 전용 궤적 지속 시간 (초).
#   크게 할수록 중앙 복귀가 천천히 부드럽게 됨.
GO_CENTER_DURATION = 1.5

# 궤적 스트리밍 스레드 주파수 (Hz).
#   높을수록 모터가 궤적을 정밀하게 따라가며, 버스 부하도 증가.
#   200 Hz 는 XL430 + U2D2 환경에서 안정적으로 동작.
TRAJ_FREQ = 200


# ============================================================
# 유틸 함수
# ============================================================

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _to_4byte(value):
    return [
        DXL_LOBYTE(DXL_LOWORD(value)),
        DXL_HIBYTE(DXL_LOWORD(value)),
        DXL_LOBYTE(DXL_HIWORD(value)),
        DXL_HIBYTE(DXL_HIWORD(value)),
    ]


def _quintic_coeffs(q0, v0, a0, qf, vf, af, T):
    """
    경계 조건 6개로 quintic polynomial θ(τ) = Σ c_i·τ^i 계수 계산.

      τ ∈ [0, T]
      θ(0)=q0,  θ'(0)=v0,  θ''(0)=a0
      θ(T)=qf,  θ'(T)=vf,  θ''(T)=af

    vf=0, af=0 으로 주면 목표에서 부드럽게 정지.
    v0, a0 에 직전 구간의 끝 속도·가속도를 넣으면 구간 간 연속성 확보.
    """
    h  = qf - q0
    T2 = T * T;  T3 = T2 * T;  T4 = T3 * T;  T5 = T4 * T
    c0 = q0
    c1 = v0
    c2 = a0 / 2.0
    c3 = ( 20*h - (8*vf + 12*v0)*T - (3*a0 - af)*T2) / (2.0 * T3)
    c4 = (-30*h + (14*vf + 16*v0)*T + (3*a0 - 2*af)*T2) / (2.0 * T4)
    c5 = ( 12*h -  6*(vf + v0)*T  - (a0 - af)*T2) / (2.0 * T5)
    return (c0, c1, c2, c3, c4, c5)


def _eval_quintic(c, tau):
    """quintic polynomial 의 위치·속도·가속도를 동시에 반환."""
    t2 = tau * tau;  t3 = t2 * tau;  t4 = t3 * tau;  t5 = t4 * tau
    pos = c[0] + c[1]*tau  + c[2]*t2  + c[3]*t3  + c[4]*t4  + c[5]*t5
    vel = c[1] + 2*c[2]*tau + 3*c[3]*t2 + 4*c[4]*t3 + 5*c[5]*t4
    acc = 2*c[2] + 6*c[3]*tau + 12*c[4]*t2 + 20*c[5]*t3
    return pos, vel, acc


# ============================================================
# Tracker 클래스
# ============================================================

class Tracker:
    def __init__(self):
        self.port   = PortHandler(DEVICENAME)
        self.packet = PacketHandler(PROTOCOL_VER)

        if not self.port.openPort():
            raise IOError("포트 열기 실패 — DEVICENAME 또는 USB 연결 확인")
        if not self.port.setBaudRate(BAUDRATE):
            raise IOError("보율 설정 실패 — BAUDRATE 값 확인")

        # USB-시리얼 latency timer 확인 및 설정 (목표: 1ms).
        # udev 규칙이 적용되어 있으면 이미 1이므로 쓰기를 건너뜀.
        _lat = f"/sys/bus/usb-serial/devices/{os.path.basename(DEVICENAME)}/latency_timer"
        try:
            with open(_lat, "r") as _f:
                _cur = int(_f.read().strip())
            if _cur != 1:
                with open(_lat, "w") as _f:
                    _f.write("1\n")
        except OSError:
            pass   # udev 규칙으로 이미 1ms 인 경우 정상 (쓰기 권한 불필요)

        # 두 모터 초기화
        for dxl_id in DXL_IDS:
            self._w1(dxl_id, ADDR_TORQUE_ENABLE,    TORQUE_DISABLE)
            self._w1(dxl_id, ADDR_OPERATING_MODE,   OP_MODE_POSITION)
            self._w4(dxl_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY_INIT)
            self._w1(dxl_id, ADDR_TORQUE_ENABLE,    TORQUE_ENABLE)

        # GroupSyncWrite / GroupSyncRead 초기화
        self.sync_write = GroupSyncWrite(
            self.port, self.packet, ADDR_GOAL_POSITION, LEN_POSITION)
        self.sync_read  = GroupSyncRead(
            self.port, self.packet, ADDR_PRESENT_POSITION, LEN_POSITION)
        for dxl_id in DXL_IDS:
            if not self.sync_read.addParam(dxl_id):
                raise RuntimeError(f"SyncRead addParam 실패 id={dxl_id}")

        # 정면으로 이동 후 안착 대기
        self._write_goals(POS_CENTER, POS_CENTER)
        time.sleep(0.5)

        # 궤적 스트리밍 모드: 모터 내장 속도 프로파일을 끔
        # (부드러움은 이제 quintic 궤적이 담당)
        for dxl_id in DXL_IDS:
            self._w4(dxl_id, ADDR_PROFILE_VELOCITY, 0)

        # 실제 현재 위치를 읽어 궤적 초기 상태로 사용
        present  = self._read_present()
        init_y   = float(present[DXL_ID_YAW]   if present else POS_CENTER)
        init_p   = float(present[DXL_ID_PITCH] if present else POS_CENTER)

        # 궤적 상태 (lock 으로 보호): {axis_id: [pos, vel, acc]}
        self._lock  = threading.Lock()
        self._state = {
            DXL_ID_YAW:   [init_y, 0.0, 0.0],
            DXL_ID_PITCH: [init_p, 0.0, 0.0],
        }

        # 초기 궤적: 제자리 정지 (trivial, T=1 s 로 충분히 길게)
        t0 = time.time()
        self._traj = {
            DXL_ID_YAW: {
                't0':     t0,
                'T':      1.0,
                'coeffs': _quintic_coeffs(init_y, 0.0, 0.0, init_y, 0.0, 0.0, 1.0),
            },
            DXL_ID_PITCH: {
                't0':     t0,
                'T':      1.0,
                'coeffs': _quintic_coeffs(init_p, 0.0, 0.0, init_p, 0.0, 0.0, 1.0),
            },
        }

        # 궤적 스트리밍 스레드 시작
        self._stop_event = threading.Event()
        self._thread     = threading.Thread(target=self._traj_thread, daemon=True)
        self._thread.start()

    # --------------------------------------------------------
    # 저수준 래퍼
    # --------------------------------------------------------

    def _w1(self, dxl_id, addr, val):
        self.packet.write1ByteTxRx(self.port, dxl_id, addr, val)

    def _w4(self, dxl_id, addr, val):
        self.packet.write4ByteTxRx(self.port, dxl_id, addr, val)

    def _read_present(self):
        if self.sync_read.txRxPacket() != COMM_SUCCESS:
            return None
        out = {}
        for dxl_id in DXL_IDS:
            if not self.sync_read.isAvailable(
                    dxl_id, ADDR_PRESENT_POSITION, LEN_POSITION):
                return None
            out[dxl_id] = self.sync_read.getData(
                dxl_id, ADDR_PRESENT_POSITION, LEN_POSITION)
        return out

    def _write_goals(self, goal_yaw, goal_pitch):
        self.sync_write.clearParam()
        self.sync_write.addParam(DXL_ID_YAW,   bytes(_to_4byte(goal_yaw)))
        self.sync_write.addParam(DXL_ID_PITCH, bytes(_to_4byte(goal_pitch)))
        self.sync_write.txPacket()

    # --------------------------------------------------------
    # 궤적 스트리밍 스레드
    # --------------------------------------------------------

    def _traj_thread(self):
        """
        TRAJ_FREQ Hz 로 quintic 궤적을 평가하고, 중간 위치를 모터에 전송.
        _write_goals 는 lock 밖에서 호출해 I/O 중 lock 을 점유하지 않도록 함.
        """
        dt = 1.0 / TRAJ_FREQ
        while not self._stop_event.is_set():
            t_now = time.time()

            with self._lock:
                for ax_id in DXL_IDS:
                    traj        = self._traj[ax_id]
                    tau         = min(t_now - traj['t0'], traj['T'])
                    pos, vel, acc = _eval_quintic(traj['coeffs'], tau)
                    self._state[ax_id] = [pos, vel, acc]
                g_yaw   = _clamp(int(round(self._state[DXL_ID_YAW][0])),   POS_MIN, POS_MAX)
                g_pitch = _clamp(int(round(self._state[DXL_ID_PITCH][0])), POS_MIN, POS_MAX)

            self._write_goals(g_yaw, g_pitch)

            elapsed = time.time() - t_now
            time.sleep(max(0.0, dt - elapsed))

    # --------------------------------------------------------
    # 핵심 제어 루프: 매 프레임 외부에서 호출
    # --------------------------------------------------------

    def update(self, nose_x, nose_y):
        """
        픽셀 좌표 → 목표 펄스 계산 → quintic 궤적 재계획.
        실제 모터 전송은 _traj_thread 가 담당 (이 함수는 블로킹 없음).

        재계획 시 현재 궤적의 (pos, vel, acc) 를 시작 조건으로 사용하므로
        매 프레임 갱신해도 가속도 연속성이 유지됨.
        """
        if nose_x is None or nose_y is None:
            return

        err_x = CENTER_X - nose_x
        err_y = CENTER_Y - nose_y

        if abs(err_x) < DEADZONE_PX:
            err_x = 0
        if abs(err_y) < DEADZONE_PX:
            err_y = 0

        if err_x == 0 and err_y == 0:
            return

        step_yaw   = _clamp(SIGN_YAW   * err_x * CONVERSION, -MAX_STEP, MAX_STEP)
        step_pitch = _clamp(SIGN_PITCH * err_y * CONVERSION, -MAX_STEP, MAX_STEP)

        with self._lock:
            q0_y, v0_y, a0_y = self._state[DXL_ID_YAW]
            q0_p, v0_p, a0_p = self._state[DXL_ID_PITCH]

            # 현재 궤적 위치 기준으로 새 목표 계산
            qf_y = float(_clamp(q0_y + step_yaw,   POS_MIN, POS_MAX))
            qf_p = float(_clamp(q0_p + step_pitch, POS_MIN, POS_MAX))

            t_now = time.time()
            self._traj[DXL_ID_YAW] = {
                't0':     t_now,
                'T':      TRAJ_DURATION,
                'coeffs': _quintic_coeffs(q0_y, v0_y, a0_y, qf_y, 0.0, 0.0, TRAJ_DURATION),
            }
            self._traj[DXL_ID_PITCH] = {
                't0':     t_now,
                'T':      TRAJ_DURATION,
                'coeffs': _quintic_coeffs(q0_p, v0_p, a0_p, qf_p, 0.0, 0.0, TRAJ_DURATION),
            }

    def get_yaw_pulse(self) -> float:
        """현재 yaw 모터의 궤적 위치(펄스)를 반환."""
        with self._lock:
            return self._state[DXL_ID_YAW][0]

    def go_center(self):
        """두 모터를 quintic 궤적으로 중앙(2048)으로 천천히 복귀."""
        t_now = time.time()
        with self._lock:
            for ax_id, center in [(DXL_ID_YAW, float(POS_CENTER)),
                                  (DXL_ID_PITCH, float(POS_CENTER))]:
                q0, v0, a0 = self._state[ax_id]
                self._traj[ax_id] = {
                    't0':     t_now,
                    'T':      GO_CENTER_DURATION,
                    'coeffs': _quintic_coeffs(q0, v0, a0, center, 0.0, 0.0, GO_CENTER_DURATION),
                }

    def close(self):
        """종료 시 스레드 정지 후 토크 OFF."""
        self._stop_event.set()
        self._thread.join(timeout=1.0)
        for dxl_id in DXL_IDS:
            self._w1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.port.closePort()


# ============================================================
# 사용 예시
# ============================================================
# def get_nose_from_your_detector():
#     raise NotImplementedError("여기에 코 좌표 검출 코드를 연결하세요")

if __name__ == "__main__":
    tracker = Tracker()
    try:
        while True:
            x, y = get_nose_from_your_detector()
            tracker.update(x, y)
            time.sleep(0.025)   # 약 33 Hz (검출 루프)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.close()
