#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DM-J4310-2EC 영점(0도) 설정 유틸리티

"0도를 어디로 할지"는 숫자로 알려주는 게 아니라, 모터를 손으로 원하는 자세로
돌려놓은 뒤 영점 설정 명령을 보내면 모터가 "지금 내가 있는 그 자리"를 스스로
읽어서 0으로 저장하는 방식이다. 모터 내부 비휘발성 메모리에 저장되므로 전원을
꺼도 유지된다 — 조립 시(또는 출력 디스크를 다시 조립했을 때) 한 번만 하면 된다.

중요: 모터를 활성화(enable)하지 않은 상태로 진행한다. 이미 활성화돼 있으면
모터가 MIT 임피던스로 그 순간 위치를 스프링처럼 붙잡고 있어서 손으로 돌리기
어렵다 — 그래서 이 스크립트는 motor.dm4310_tracker.Tracker(생성 시 자동 enable)
대신 motor.DM_CAN 을 직접 사용해 enable()을 호출하지 않는다.

실행:
  uv run python test/zero_actuator.py
"""

import math
import os
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import serial

from motor.DM_CAN import Motor, MotorControl
from motor.dm4310_tracker import (
    DEVICENAME, BAUDRATE, MOTOR_TYPE, AXES,
    CAN_ID_YAW, CAN_ID_PITCH, MASTER_ID_YAW, MASTER_ID_PITCH,
)

CAN_ID = {"yaw": CAN_ID_YAW, "pitch": CAN_ID_PITCH}
MASTER_ID = {"yaw": MASTER_ID_YAW, "pitch": MASTER_ID_PITCH}
AXIS_LABEL = {"yaw": "Yaw(좌우)", "pitch": "Pitch(상하)"}


def read_position(ctrl: MotorControl, motor: Motor) -> float:
    """토크 걸지 않고 현재 실제 각도(rad)만 읽어온다."""
    ctrl.refresh_motor_status(motor)
    return motor.getPosition()


def fmt_rad(rad: float) -> str:
    """라디안 값을 'rad(도)' 형태로 함께 보여주는 포맷터.

    CAN 프로토콜/모터 내부는 전부 라디안 기준이라 계산은 라디안으로 하지만,
    사람이 읽을 땐 도(degree) 가 더 직관적이라 항상 같이 표시한다.
    """
    return f"{rad:+.4f}rad ({math.degrees(rad):+.1f}°)"


def main():
    print(f"[..] {DEVICENAME} 연결 중 (baud={BAUDRATE})...")
    ser = serial.Serial(DEVICENAME, BAUDRATE, timeout=0.001)
    ctrl = MotorControl(ser)

    motors = {}
    for axis in AXES:
        motors[axis] = Motor(MOTOR_TYPE, CAN_ID[axis], MASTER_ID[axis])
        ctrl.addMotor(motors[axis])
    print("[OK] 연결 완료 — 모터는 활성화(enable)하지 않았으므로 손으로 자유롭게 돌아갑니다.\n")

    try:
        for axis in AXES:
            motor = motors[axis]
            label = AXIS_LABEL[axis]

            before = read_position(ctrl, motor)
            print(f"[{label}] 현재 각도 = {fmt_rad(before)}")

            input(f"  → 짐벌을 {label} 원하는 0도 위치(정면/수평)로 손으로 맞춘 뒤 Enter 를 누르세요...")

            ctrl.set_zero_position(motor)
            time.sleep(0.1)

            # set_zero_position() 만으로는 플래시에 저장되지 않고 전원이
            # 꺼지면 되돌아간다(공식 문서 기준) — save_motor_param() 으로
            # 반드시 플래시에 써야 영구적으로 유지된다.
            ctrl.save_motor_param(motor)
            time.sleep(0.1)

            after = read_position(ctrl, motor)
            print(f"  [DONE] 영점 설정 완료 → 새 각도 = {fmt_rad(after)} (0에 가까워야 정상)\n")

        print("모든 축 영점 설정 완료. 전원을 꺼도 유지됩니다 — 이제 main.py/motor_test.py를")
        print("정상적으로 실행하면 이 자세가 0 rad(중앙) 기준이 됩니다.")

    except (EOFError, KeyboardInterrupt):
        print("\n[중단] 영점 설정을 완료하지 못했습니다.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
